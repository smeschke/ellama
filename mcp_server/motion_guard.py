"""Motion guard: stops the robot when the sensors stop agreeing with what it was told to do.

Pure logic like tilt_guard.py -- RobotLink feeds it the commands it sends, every encoder
sample and every IMU sample, and acts on the fault it returns, so it can be unit-tested and
replayed against recordings.

Four detectors, each tuned against the 47 recorded runs in recordings/ (no trips on normal
driving; see test_motion_guard.py) and each with a number that is physically explained:

stall      A wheel commanded above the motor deadband for >= 1.0 s (ramp + spin-up are done)
           turns < 0.5 in/s for 0.4 s. Then the IMU decides WHAT it is:
             - the chassis is pivoting the way the encoders say (gyro agrees), or both
               wheels are stopped  ->  "stall": the robot is stuck on something
             - one encoder says stopped but the gyro says the robot is going straight
               ->  "encoder_fault": the wheel is turning, the sensor isn't producing it
           The recordings show why this split matters: in the 10-03 follow runs the right
           encoder reads ~0 for 25-58% of "commanded" samples, and the robot was fine.
direction  A wheel moves >= 1.5 in/s the OPPOSITE way to its command for 0.5 s ->
           "encoder_fault" (sign/magnet problem) or something is dragging it.
slip       Over 1 s the wheels report a rotation >= 20 deg but the gyro saw less than 1/3.5 of
           it. An in-place spin scrubs, so the encoders legitimately read ~1.6-2x the gyro
           (measured); a robot held by a foot read 5x. ->  "slip": wheels turning, chassis not.
bump       Hard: horizontal acceleration spike >= 0.6 g (normal driving peaks at 0.43 g).
           Corroborated: a >= 0.35 g spike within 0.15 s of the wheels' mean speed collapsing
           to < 40% of what it was 0.3 s earlier (from >= 3 in/s) with the command held ->
           "bump". A speed drop alone is NOT enough -- ordinary braking and turning do that.

Faults latch like the tilt guard's (stop -> check -> decide -> move): nothing drives until
the operator clears it. Two exceptions are deliberate. An "encoder_fault" only latches when
the running tool depends on the encoders (encoders_required); a camera-only run
(perform) sets it False, because a dead encoder must not stop a run that doesn't use them, and the event
is just logged. And mode "warn" logs every event without ever stopping, for tuning on a robot.

What it cannot see: wheels spinning on the spot with the robot not moving at all and the IMU
quiet looks like a stall (good), but a robot that is moving fine yet *should* have stopped (a
person in front of it) is the camera's job. This is the layer that notices "the data stopped
matching" -- it is not an obstacle sensor.
"""

import collections
import math
import threading

# --- thresholds (see the module docstring for where each number comes from) -----------------
STRAIGHT_PWM_MIN = 50    # both wheels the same direction: forward deadband is ~45-50
TURN_PWM_MIN = 65        # opposed wheels (spin in place): turn deadband measured 50-60
STALL_GRACE_S = 1.0
STALL_HOLD_S = 0.4
STALL_SPEED_IN_S = 0.5
ONE_SIDED_MOVING_IN_S = 1.5
YAW_TOL_ABS_DPS = 8.0
YAW_TOL_REL = 0.5
DIRECTION_SPEED_IN_S = 1.5
DIRECTION_HOLD_S = 0.5
SLIP_WINDOW_S = 1.0
SLIP_MIN_ENC_DEG = 20.0
SLIP_RATIO = 3.5
SLIP_MIN_WHEEL_IN_S = 1.5   # each wheel's mean |speed| over the window
BUMP_HARD_G = 0.6
BUMP_SOFT_G = 0.35
BUMP_WINDOW_S = 0.15
BUMP_LOOKBACK_S = 0.3
BUMP_SPEED_FRAC = 0.4
BUMP_MIN_SPEED_IN_S = 3.0
BUMP_CMD_HELD_FRAC = 0.7
ACC_BASELINE_ALPHA = 0.04   # ~0.5 s at 50 Hz: slow gravity/tilt baseline the spike is measured from
GZ_ALPHA = 0.2


class MotionGuard:
    def __init__(self, forward_pwm_sign=-1, track_width_in=20.0, mode="on"):
        self.forward_pwm_sign = forward_pwm_sign
        self.track_width_in = float(track_width_in)
        self.mode = mode                    # "on" | "warn" | "off"
        self.encoders_required = True       # tools that don't use the encoders set this False
        self._lock = threading.RLock()
        self.tripped = False
        self.fault = None
        self.events = collections.deque(maxlen=12)   # everything noticed, tripped or not
        self.reset_state()

    # -- bookkeeping ---------------------------------------------------------------------

    def reset_state(self):
        with self._lock:
            self.cmd = (0, 0)
            self.cmd_t = None
            self._above = [None, None]      # since when each wheel has been commanded >= min, same sign
            self._cmd_sign = [0, 0]
            self._below = [None, None]      # since when each wheel has read < STALL_SPEED
            self._wrong = [None, None]      # since when each wheel has moved against its command
            self.speed = (0.0, 0.0)
            self._speed_hist = collections.deque()      # (t, mean |speed|, mean |cmd|)
            self._slip = collections.deque()            # (t, enc_rate_dps, gz_dps, dt, |vl|, |vr|, vl, vr)
            self._slip_speed = (0.0, 0.0)
            self._last_imu_t = None
            self.gz = 0.0
            self._acc_base = None
            self._spike = (None, 0.0)       # (time, g) of the last >= BUMP_SOFT_G spike
            self.last_hp_g = 0.0

    def clear(self):
        with self._lock:
            if not self.tripped:
                return True, "no motion fault latched"
            self.tripped = False
            self.fault = None
            self.reset_state()
            return True, "motion fault cleared"

    def _min_pwm(self, l, r):
        return TURN_PWM_MIN if (l * r) < 0 else STRAIGHT_PWM_MIN

    def _fire(self, kind, detail, t, **extra):
        """Record an event; latch and return a fault dict only if it should stop the robot."""
        ev = dict(kind=kind, reason="%s: %s" % (kind, detail), t=round(t, 3), **extra)
        self.events.append(ev)
        if self.mode != "on":
            return None
        if kind == "encoder_fault" and not self.encoders_required:
            ev["note"] = "logged only: this run does not depend on the encoders"
            return None
        if self.tripped:
            return None
        self.tripped = True
        self.fault = ev
        return ev

    # -- inputs ------------------------------------------------------------------------

    def on_command(self, left, right, t):
        """Called with every nonzero-or-zero drive command RobotLink sends (raw PWM)."""
        with self._lock:
            self.cmd, self.cmd_t = (int(left), int(right)), t
            m = self._min_pwm(left, right)
            for w, c in enumerate((left, right)):
                s = (c > 0) - (c < 0)
                if abs(c) >= m:
                    if self._above[w] is None or s != self._cmd_sign[w]:
                        self._above[w] = t
                        self._below[w] = None
                        self._wrong[w] = None
                else:
                    self._above[w] = None
                    self._below[w] = None
                    self._wrong[w] = None
                self._cmd_sign[w] = s

    def on_enc(self, speed_l, speed_r, t):
        """One encoder sample: signed wheel speeds in in/s, forward positive. Returns a
        latched fault dict the moment one trips, else None."""
        if self.mode == "off":
            return None
        with self._lock:
            self.speed = (speed_l, speed_r)
            dt = None
            if self._speed_hist:
                dt = t - self._speed_hist[-1][0]
            mean_abs = (abs(speed_l) + abs(speed_r)) / 2.0
            mean_cmd = (abs(self.cmd[0]) + abs(self.cmd[1])) / 2.0
            self._speed_hist.append((t, mean_abs, mean_cmd))
            while self._speed_hist and t - self._speed_hist[0][0] > 1.0:
                self._speed_hist.popleft()

            enc_rate = math.degrees((speed_r - speed_l) / self.track_width_in)  # CCW positive
            self._slip.append((t, enc_rate, self.gz, dt or 0.0, abs(speed_l), abs(speed_r), speed_l, speed_r))
            while self._slip and t - self._slip[0][0] > SLIP_WINDOW_S:
                self._slip.popleft()
            n = len(self._slip)
            self._slip_speed = (sum(x[4] for x in self._slip) / n, sum(x[5] for x in self._slip) / n)

            f = self._check_stall(t) or self._check_direction(t) or self._check_slip(t) \
                or self._check_bump_soft(t, mean_abs)
            return f

    def on_imu(self, gz_dps, ax_g, ay_g, t):
        """One IMU sample: yaw rate (deg/s, CCW positive) and body-frame accel in g."""
        if self.mode == "off":
            return None
        with self._lock:
            self.gz += GZ_ALPHA * (gz_dps - self.gz)
            if self._acc_base is None:
                self._acc_base = [ax_g, ay_g]
            hp = math.hypot(ax_g - self._acc_base[0], ay_g - self._acc_base[1])
            self._acc_base[0] += ACC_BASELINE_ALPHA * (ax_g - self._acc_base[0])
            self._acc_base[1] += ACC_BASELINE_ALPHA * (ay_g - self._acc_base[1])
            self.last_hp_g = hp
            if hp >= BUMP_SOFT_G:
                self._spike = (t, hp)
            moving = self.cmd != (0, 0) and self.cmd_t is not None and t - self.cmd_t < 0.5
            if hp >= BUMP_HARD_G and moving:
                return self._fire("bump", "%.2f g impact spike while driving (hard-hit limit %.1f g)"
                                  % (hp, BUMP_HARD_G), t, peak_g=round(hp, 2))
            return None

    # -- detectors ----------------------------------------------------------------------

    def _commanded_long_enough(self, w, t, secs):
        a = self._above[w]
        return a is not None and (t - a) >= secs

    def _check_stall(self, t):
        for w in (0, 1):
            if not self._commanded_long_enough(w, t, STALL_GRACE_S):
                self._below[w] = None
                continue
            if abs(self.speed[w]) < STALL_SPEED_IN_S:
                if self._below[w] is None:
                    self._below[w] = t
                if t - self._below[w] >= STALL_HOLD_S:
                    return self._classify_stall(w, t)
            else:
                self._below[w] = None
        return None

    def _classify_stall(self, w, t):
        """Wheel w reads stopped though commanded. Is it stuck, or is its encoder lying? Ask the
        gyro which hypothesis explains the rotation it actually saw:

          stuck  -> the chassis pivots about the held wheel, so the gyro matches the rotation the
                    encoders imply, and is NOT bigger than the driving wheel alone can produce
          dead   -> the wheel is really turning, so the robot rotates MORE than the encoders say
                    (a spin with one dead encoder) or not at all while they say it should
                    (straight drive with one dead encoder)
        Nothing rotating and nothing implied -> both wheels are stuck (or both encoders are dead:
        indistinguishable, and stopping is the safe answer)."""
        name = ("left", "right")[w]
        enc_yaw = math.degrees((self.speed[1] - self.speed[0]) / self.track_width_in)
        gz = self.gz
        if abs(enc_yaw) < 5.0 and abs(gz) < 5.0:
            return self._fire(
                "stall", "%s wheel commanded %d has not turned for %.1fs (%.1f in/s) and nothing "
                "is rotating -- stuck, or no traction" % (name, self.cmd[w], STALL_HOLD_S,
                                                          self.speed[w]), t, wheel=name)
        if (self.cmd[0] * self.cmd[1]) < 0:
            # In a spin, "this wheel is held and the other pivots the robot" and "this encoder is
            # dead and both wheels turn with scrub" give the SAME gyro rate (~20 deg/s at 6 in/s:
            # 6.7/20 rad/s pivoting, or 2*6.7/20 rad/s divided by the ~1.7x scrub). The IMU cannot
            # tell them apart, so don't pretend to: report the encoder as unverifiable.
            return self._fire(
                "encoder_fault",
                "%s wheel commanded %d reads %.1f in/s during a spin while the other runs %.1f "
                "in/s (gyro %.0f deg/s) -- either this wheel is held or its encoder is dead; the "
                "IMU cannot tell which in a spin" % (name, self.cmd[w], self.speed[w],
                                                     self.speed[1 - w], gz), t, wheel=name,
                ambiguous=True)
        cap = 1.15 * abs(enc_yaw) + 2.0
        consistent = (abs(gz - enc_yaw) <= max(YAW_TOL_ABS_DPS, YAW_TOL_REL * abs(enc_yaw))
                      and abs(gz) <= cap)
        if consistent:
            other = 1 - w
            return self._fire(
                "stall", "%s wheel commanded %d is stopped (%.1f in/s) while the other runs %.1f "
                "in/s and the chassis pivots (%.0f deg/s, encoders imply %.0f) -- snagged on "
                "something" % (name, self.cmd[w], self.speed[w], self.speed[other], gz, enc_yaw),
                t, wheel=name)
        return self._fire(
            "encoder_fault",
            "%s wheel commanded %d reads %.1f in/s but the gyro sees %.0f deg/s where the encoders "
            "imply %.0f -- the wheel is turning, the sensor is not producing it"
            % (name, self.cmd[w], self.speed[w], gz, enc_yaw), t, wheel=name)

    def _check_direction(self, t):
        for w in (0, 1):
            if not self._commanded_long_enough(w, t, STALL_GRACE_S):
                self._wrong[w] = None
                continue
            c = self.cmd[w]
            want = 1 if ((c > 0) - (c < 0)) == self.forward_pwm_sign else -1
            if self.speed[w] * want <= -DIRECTION_SPEED_IN_S:
                if self._wrong[w] is None:
                    self._wrong[w] = t
                if t - self._wrong[w] >= DIRECTION_HOLD_S:
                    name = ("left", "right")[w]
                    return self._fire(
                        "encoder_fault", "%s wheel commanded %d (%s) but moves %.1f in/s the other "
                        "way for %.1fs" % (name, c, "forward" if want > 0 else "reverse",
                                           self.speed[w], DIRECTION_HOLD_S), t, wheel=name)
            else:
                self._wrong[w] = None
        return None

    def _check_slip(self, t):
        if len(self._slip) < 10 or t - self._slip[0][0] < 0.8 * SLIP_WINDOW_S:
            return None
        # A wheel that reads ~0 while commanded would fake a rotation that never happened (a dead
        # encoder looked exactly like this in the 10-03 runs). Slip is only judged when BOTH
        # wheels are commanded and measurably turning for the whole window; the stall/encoder
        # checks own the "a wheel reads nothing" case.
        if not (self._commanded_long_enough(0, t, SLIP_WINDOW_S)
                and self._commanded_long_enough(1, t, SLIP_WINDOW_S)):
            return None
        if min(self._slip_speed[0], self._slip_speed[1]) < SLIP_MIN_WHEEL_IN_S:
            return None
        # ...and the freshest samples must look like two working encoders: each wheel turning
        # (an encoder that just dropped to 0 mid-window is a glitch, not slip) and the way it
        # was told to (a wheel reading backwards against its command is a sign/magnet problem).
        for w in (0, 1):
            c = self.cmd[w]
            want = 1 if ((c > 0) - (c < 0)) == self.forward_pwm_sign else -1
            recent = list(self._slip)[-10:]
            if min(x[4 + w] for x in recent) < 1.0 or min(want * x[6 + w] for x in recent) <= 0:
                return None
        enc = sum(x[1] * x[3] for x in self._slip)
        gyr = sum(x[2] * x[3] for x in self._slip)
        if abs(enc) >= SLIP_MIN_ENC_DEG and abs(gyr) * SLIP_RATIO < abs(enc):
            return self._fire(
                "slip", "wheels turned the robot %.0f deg in %.1fs but the gyro saw %.0f deg "
                "(%.1fx; a spin on a floor reads ~1.6-2x) -- the chassis is not following the "
                "wheels (held, wedged, or no traction)" % (enc, SLIP_WINDOW_S, gyr,
                                                          abs(enc) / max(abs(gyr), 0.1)), t,
                enc_deg=round(enc, 1), gyro_deg=round(gyr, 1))
        return None

    def _check_bump_soft(self, t, mean_abs):
        st, g = self._spike
        if st is None or t - st > BUMP_WINDOW_S or len(self._speed_hist) < 3:
            return None
        before = [h for h in self._speed_hist if BUMP_LOOKBACK_S - 0.06 <= t - h[0] <= BUMP_LOOKBACK_S + 0.06]
        if not before:
            return None
        sp0 = sum(h[1] for h in before) / len(before)
        cmd0 = sum(h[2] for h in before) / len(before)
        cmd_now = (abs(self.cmd[0]) + abs(self.cmd[1])) / 2.0
        if sp0 >= BUMP_MIN_SPEED_IN_S and mean_abs < BUMP_SPEED_FRAC * sp0 \
                and cmd_now >= BUMP_CMD_HELD_FRAC * cmd0:
            return self._fire(
                "bump", "%.2f g spike and wheel speed fell %.1f -> %.1f in/s in %.1fs with the "
                "command held -- hit something" % (g, sp0, mean_abs, BUMP_LOOKBACK_S), t,
                peak_g=round(g, 2))
        return None

    def snapshot(self):
        with self._lock:
            return dict(
                mode=self.mode,
                tripped=self.tripped,
                fault=self.fault,
                encoders_required=self.encoders_required,
                recent_events=list(self.events)[-5:],
                accel_spike_g=round(self.last_hp_g, 2),
            )
