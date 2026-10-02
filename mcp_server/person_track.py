"""Turn-to-me controller: person bearings in, spin-in-place wheel commands out. Pure logic --
no serial, camera or MCP -- so it can be simulated offline (see
person_track_sim.py) before it ever drives the robot.

Conventions (same as line_follow.py): bearing + = person is to the RIGHT of where the robot
points; turn effort t + = turn RIGHT (left wheel faster); IMU yaw + = turned LEFT (CCW), so a
person at bearing b is at world yaw (yaw_at_frame - b). Only spins (f = 0), never drives.

How it steers: each camera frame gives a *world-frame* target yaw = (yaw when the frame was
captured) - bearing. The camera runs ~0.25 s behind the IMU, so the yaw is looked up from a
short history at that earlier time; this keeps the stale-image lag out of the control loop.
The spin itself is closed on the IMU yaw (no lag): error = target - yaw. Spin plant numbers
are the line follower's (deadband ~28, ~0.5 deg/s per unit above it) -- starting points, to
be tuned on the first bench test.

It HOLDS: once facing the person (within DONE_DEG) it stops spinning but keeps watching, and
spins again if they move more than REENGAGE_DEG away. It ends only on: lost (no person for
LOST_TIMEOUT_S), stale (no camera frame), timeout, stopped (stop() called).
With several people in frame it follows whoever is nearest the current target in world yaw,
so it doesn't hop between people; the tool refuses to *start* if it sees more than one.
"""

import bisect
import math
from collections import deque

from line_follow import Command, OUTER_MAX, PLANT_G_SPIN, T_DEADBAND

CAM_LAG_S = 0.25        # camera frame arrives this long after it was captured. GUESS.
FULL_SPEED_ERR_DEG = 15.0  # error at which the spin reaches its max speed (so gain = u_max / this)
SPIN_U_MAX = 18.0       # fastest spin we ask for (deg/s)
U_MIN = 0.4             # below this don't spin at all
DONE_DEG = 3.0          # stop spinning inside this...
REENGAGE_DEG = 6.0      # ...and start again beyond this (hysteresis)
LOST_TIMEOUT_S = 1.0    # no person for this long = stop
STALE_FRAME_S = 0.5     # no new frame for this long = stop
SMOOTH = 0.6            # EMA weight on a new target measurement (1.0 = none)
HISTORY_S = 2.0
STALL_RATE_DPS = 2.0    # told to spin but turning slower than this = stuck on the deadband
STALL_AFTER_S = 0.3     # ...for this long: raise the effort floor
BUMP_RATE = 20.0        # effort units per second to add while stalled
BUMP_MAX = 40.0         # most we'll add on top of the nominal curve (per direction)


def spin_effort(err_deg, u_max=SPIN_U_MAX):
    """Turn effort t (+ = right) to rotate by err_deg (+ = needs a LEFT turn, IMU sign).
    u_max is the fastest spin asked for, deg/s."""
    u = min(u_max, u_max / FULL_SPEED_ERR_DEG * abs(err_deg))
    if u < U_MIN:
        return 0.0
    t = min(T_DEADBAND + u / PLANT_G_SPIN, OUTER_MAX)
    return -math.copysign(t, err_deg)       # needs left (+err) -> turn left -> t negative


class PersonTracker:
    def __init__(self, max_run_s=60.0, forward_pwm_sign=-1, spin_u_max=SPIN_U_MAX,
                 lost_timeout_s=LOST_TIMEOUT_S):
        # These three can be changed while running (see robot_server.turn_to_me_set).
        # max_run_s / lost_timeout_s of 0 or None mean "never".
        self.max_run_s = max_run_s
        self.spin_u_max = spin_u_max
        self.lost_timeout_s = lost_timeout_s
        self.sign = forward_pwm_sign
        self.status = "idle"
        self.target = None            # world yaw of the person, degrees (IMU sign)
        self.spinning = False
        self.frames = 0
        self._hist = deque()          # (t, yaw)

    # -- lifecycle ---------------------------------------------------------------------
    def start(self, now, yaw):
        self.status = "running"
        self.t_start = self.last_frame_t = self.last_seen_t = now
        self.target = None
        self.spinning = False
        self.engaged = False          # has it started its first turn yet
        self.bump = {1: 0.0, -1: 0.0}  # learned extra effort to beat static friction, per turn direction
        self.spin_since = None        # when the current spin started / last made progress
        self.last_step = None
        self.frames = 0
        self._hist.clear()
        self.note_yaw(now, yaw)

    def stop(self):
        if self.status == "running":
            self.status = "stopped"
        return self._zero()

    # -- inputs ------------------------------------------------------------------------
    def note_yaw(self, now, yaw):
        """Call every loop tick with the current IMU yaw (keeps the history for lag lookup)."""
        self._hist.append((now, yaw))
        while self._hist and now - self._hist[0][0] > HISTORY_S:
            self._hist.popleft()

    def yaw_at(self, t):
        h = self._hist
        if not h:
            return None
        ts = [x[0] for x in h]
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return h[0][1]
        if i >= len(h):
            return h[-1][1]
        (t0, y0), (t1, y1) = h[i - 1], h[i]
        return y0 + (y1 - y0) * (t - t0) / (t1 - t0) if t1 > t0 else y1

    def on_frame(self, res, frame_t):
        """One person_vision.analyze() result per camera frame (frame_t = arrival time)."""
        if self.status != "running":
            return
        self.last_frame_t = frame_t
        self.frames += 1
        people = res.get("people") or []
        y_cap = self.yaw_at(frame_t - CAM_LAG_S)
        if not people or y_cap is None:
            return
        worlds = [y_cap - p["bearing_deg"] for p in people]
        w = worlds[0] if self.target is None else min(worlds, key=lambda x: abs(x - self.target))
        self.last_seen_t = frame_t
        self.target = w if self.target is None else SMOOTH * w + (1 - SMOOTH) * self.target

    # -- per tick ----------------------------------------------------------------------
    def step(self, now, yaw):
        """Wheel command to hold until the next tick, given the current IMU yaw."""
        self.note_yaw(now, yaw)
        if self.status != "running":
            return self._zero()
        if self.max_run_s and now - self.t_start > self.max_run_s:
            return self._end("timeout")
        if now - self.last_frame_t > STALE_FRAME_S:
            return self._end("stale")
        if self.lost_timeout_s and now - self.last_seen_t > self.lost_timeout_s:
            return self._end("lost")
        if self.target is None:
            return self._cmd(0.0, dict(note="waiting for a person"))
        err = self.target - yaw
        if self.spinning and abs(err) < DONE_DEG:
            self.spinning = False
        elif not self.spinning and abs(err) > (REENGAGE_DEG if self.engaged else DONE_DEG):
            self.spinning = self.engaged = True
        t = spin_effort(err, self.spin_u_max) if self.spinning else 0.0
        if t == 0.0:
            self.spinning = False
        if self.spinning:
            d = 1 if t > 0 else -1
            dt = 0.0 if self.last_step is None else min(now - self.last_step, 0.1)
            old_yaw = self.yaw_at(now - STALL_AFTER_S)
            rate = 0.0 if old_yaw is None else abs(yaw - old_yaw) / STALL_AFTER_S
            if self.spin_since is None:
                self.spin_since = now
            if now - self.spin_since >= STALL_AFTER_S and rate < STALL_RATE_DPS:
                self.bump[d] = min(BUMP_MAX, self.bump[d] + BUMP_RATE * dt)     # stuck: push harder
            t = math.copysign(min(abs(t) + self.bump[d], OUTER_MAX), t)
        else:
            self.spin_since = None
        self.last_step = now
        return self._cmd(t, dict(err_deg=round(err, 1), target_yaw=round(self.target, 1),
                                 t=round(t), spin=self.spinning, spin_dps=self.spin_u_max, lost_s=self.lost_timeout_s,
                                 max_s=self.max_run_s, facing=abs(err) < DONE_DEG,
                                 bump=round(self.bump[1]), bump_neg=round(self.bump[-1])))

    # -- helpers -----------------------------------------------------------------------
    def _cmd(self, t, info):
        return Command(int(round(self.sign * t)), int(round(self.sign * -t)), self.status, info)

    def _zero(self):
        return Command(0, 0, self.status, {})

    def _end(self, why):
        self.status = why
        return self._zero()
