"""Line-following controller: line detections in, wheel commands out. Pure logic -- no
serial, camera or MCP -- so it can be replayed and simulated offline (line_follow_sim.py)
before it ever drives the robot.

Frame of reference used throughout ("real" units): f = forward effort (+ forward), t = turn
effort (+ turn RIGHT, i.e. left wheel faster). left_real = f + t, right_real = f - t. The
robot's raw PWM sign is applied last (`forward_pwm_sign`; robot_server.FORWARD_PWM_SIGN is -1,
so raw = -real there, which is also why the stick sends negative numbers to go forward).

What the numbers are built on (fit from tape_manual_run1/2, 2026-10-02, one unit):
  * forward: ~0.095 in/s per unit of f, with a dead zone below f ~ 45-50.
  * turning has TWO regimes (fit from the manual runs):
      driving (f > ~40): linear, no dead zone, ~0.2 deg/s of yaw per unit of t (t=25 -> ~5 deg/s)
      spinning (f ~ 0):  dead zone to |t| ~ 28, then ~0.5 deg/s per unit
                         (t=46 -> ~9 deg/s, t=64 -> ~17, t=85 -> ~30)
    Cruise turn authority is low: with the outer wheel capped at OUTER_MAX, cruise can only
    steer ~6 deg/s. So bends are taken the way the human drove them: when the target is far
    off to the side, stop and spin toward the line, then resume driving.
  * lag: ~0.2 s command -> wheel speed, ~0.15 s command -> yaw rate, camera ~0.25 s behind
    the IMU. All guesses until the first bench test on blocks; every number here is a
    starting point, not a measurement of the finished system.

Terminal states, after which update() returns zero commands for good:
  finished  the cross strip at the end of the course reached FINISH_Y_FRAC of the way up
            the image (and stayed for FINISH_FRAMES frames)
  lost      no line in the image for LOST_TIMEOUT_S
  stale     no new camera frame for STALE_FRAME_S
  timeout   max_run_s elapsed
  stopped   stop() was called
"""

import math
from dataclasses import dataclass, field

import line_vision

# ---- tuning ----------------------------------------------------------------------------
F_CRUISE = 58          # forward effort on a straight; deadband is ~45-50, so don't go lower
F_MOVE_MIN = 50        # whenever moving forward at all, at least this (else it's in the dead zone)
OUTER_MAX = 90         # no wheel above this, even in a corner (policy ceiling is 125)
T_DEADBAND = 28        # spin-in-place: turn effort below this produces ~no yaw
PLANT_G_SPIN = 0.5     # spin: deg/s of yaw per unit of turn effort above the deadband
PLANT_G_DRIVE = 0.2    # driving: deg/s of yaw per unit of turn effort (linear, no deadband)
SPIN_U_MAX = 18.0      # fastest spin we ask for (deg/s). The camera is ~0.4 s behind, so a
                       # faster spin overshoots the line by more than a few degrees.
U_MIN = 0.4            # below this aim error (deg/s) don't turn at all
K_PHI = 1.4            # deg/s of turn per degree between where we're pointing and the target point
PHI_SLOW = 8.0         # while driving, slow down from F_CRUISE to F_MOVE_MIN between these aim angles
PHI_SPIN = 16.0        # beyond this aim angle, stop and spin...
PHI_SPIN_EXIT = 7.0    # ...until the target is within this, then drive again (hysteresis)
AXLE_TO_NEAR_BAND_IN = 7.5   # from the axle (where the robot pivots) to the nearest band. GUESS:
                             # measure with a ruler on the real mount.
SMOOTH = 0.6           # EMA weight on the newest measurement (1.0 = no smoothing)
LOOKAHEAD_IN = 5.0     # where along the fitted line (in from the near band) to read the offset
FIT_SPAN_IN = 8.0      # fit the line over only this much of the nearest line. Fitting the whole
                       # visible span made it spin for a corner a foot away, before reaching it.
ASPECT = 1.0           # ground inches per pixel vertically / horizontally. UNCALIBRATED.

# ---- safety ----------------------------------------------------------------------------
LOST_TIMEOUT_S = 0.5   # no line for this long = stop. (~8 frames at 16 fps)
STALE_FRAME_S = 0.5    # no new frame for this long = stop
FINISH_Y_FRAC = 0.35   # cross strip this far down from the top of the image = we're there
FINISH_FRAMES = 3      # ...for this many consecutive frames (one stray frame fired at 32.8s)
MIN_BANDS_TO_START = 3


@dataclass
class Command:
    left_raw: int = 0
    right_raw: int = 0
    status: str = "running"
    info: dict = field(default_factory=dict)


def measure(res):
    """Boil a line_vision.analyze() result down to a dict (e_near, e_look, psi_deg, phi_deg,
    span_in), or None if there aren't two usable bands. + = to the right: the line's offset
    e, its lean psi going away, and phi, the angle from our heading to a target point on it."""
    bands = res.get("bands") or []
    if len(bands) < 2:
        return None
    # ground distance of each band from the nearest one, integrating the per-row scale
    # (the tape is a known width, so each band's width in pixels gives its inches/pixel)
    s = [0.0]
    for a, b in zip(bands, bands[1:]):
        ipp = 0.5 * (line_vision.TAPE_WIDTH_IN / a["width_px"] + line_vision.TAPE_WIDTH_IN / b["width_px"])
        s.append(s[-1] + abs(a["y"] - b["y"]) * ipp * ASPECT)
    e = [b["lateral_in"] for b in bands]
    keep = max(2, sum(1 for x in s if x <= FIT_SPAN_IN))
    s, e = s[:keep], e[:keep]
    n = len(s)
    mean_s, mean_e = sum(s) / n, sum(e) / n
    var = sum((x - mean_s) ** 2 for x in s)
    slope = 0.0 if var == 0 else sum((x - mean_s) * (y - mean_e) for x, y in zip(s, e)) / var
    icpt = mean_e - slope * mean_s
    look = min(LOOKAHEAD_IN, s[-1])
    y_t = icpt + slope * look                      # lateral position of the target point
    phi = math.degrees(math.atan2(y_t, AXLE_TO_NEAR_BAND_IN + look))   # aim angle from the axle
    return dict(e_near=icpt, e_look=y_t, psi_deg=math.degrees(math.atan(slope)), phi_deg=phi,
                span_in=s[-1])


def wheel_efforts(phi_deg, spinning):
    """The control law (pure pursuit). `phi_deg` is the angle from the robot's heading to a
    target point a little way up the line (+ = right of us). Returns (f, t, spinning) in
    real units; `spinning` is the mode to pass back in next time (it has hysteresis)."""
    if spinning:
        if abs(phi_deg) < PHI_SPIN_EXIT:
            spinning = False
    elif abs(phi_deg) > PHI_SPIN:
        spinning = True

    if spinning:
        u = max(-SPIN_U_MAX, min(SPIN_U_MAX, K_PHI * phi_deg))
        t = math.copysign(T_DEADBAND + abs(u) / PLANT_G_SPIN, u) if abs(u) >= U_MIN else 0.0
        return 0.0, min(abs(t), OUTER_MAX) * (1 if t >= 0 else -1), True

    # driving: slow down as the target gets further off to the side
    slow = max(0.0, min(1.0, (abs(phi_deg) - PHI_SLOW) / (PHI_SPIN - PHI_SLOW)))
    f = F_CRUISE - slow * (F_CRUISE - F_MOVE_MIN)
    u = K_PHI * phi_deg
    t = 0.0 if abs(u) < U_MIN else u / PLANT_G_DRIVE
    room = OUTER_MAX - f                     # nothing above OUTER_MAX on the outer wheel
    return f, max(-room, min(room, t)), False


class LineFollower:
    """Feed it one analyze() result per camera frame via update(); it returns the wheel
    command to hold until the next frame. Call check(now) between frames (the loop should
    run it at ~20 Hz) so a dead camera is noticed even though no frames arrive."""

    def __init__(self, max_run_s=120.0, forward_pwm_sign=-1, f_cruise=None):
        self.max_run_s = max_run_s
        self.sign = forward_pwm_sign
        self.f_cruise = f_cruise
        self.status = "idle"
        self.t_start = None
        self.last_frame_t = None
        self.last_seen_t = None
        self.e = self.psi = None
        self.spinning = False
        self.finish_count = 0
        self.frames = 0

    # -- lifecycle ---------------------------------------------------------------------
    def can_start(self, res):
        """(ok, why). The line must already be in view and well tracked."""
        if not res.get("found") or res.get("n_bands", 0) < MIN_BANDS_TO_START:
            return False, f"line not clearly visible ({res.get('n_bands', 0)} bands, need {MIN_BANDS_TO_START})"
        if res.get("cross_strip"):
            return False, "the finish strip is already in view"
        return True, ""

    def start(self, now):
        self.status = "running"
        self.t_start = self.last_frame_t = self.last_seen_t = now
        self.e = self.psi = None
        self.spinning = False
        self.finish_count = self.frames = 0

    def stop(self):
        if self.status == "running":
            self.status = "stopped"
        return self._zero()

    # -- per frame ---------------------------------------------------------------------
    def update(self, res, now):
        if self.status != "running":
            return self._zero()
        self.last_frame_t = now
        self.frames += 1
        if now - self.t_start > self.max_run_s:
            return self._end("timeout")

        # finish strip: sustained, and close enough
        if res.get("cross_strip") and (res.get("cross_y_frac") or 0) >= FINISH_Y_FRAC:
            self.finish_count += 1
            if self.finish_count >= FINISH_FRAMES:
                return self._end("finished")
        else:
            self.finish_count = 0

        m = measure(res) if res.get("found") else None
        if m is None:
            if now - self.last_seen_t > LOST_TIMEOUT_S:
                return self._end("lost")
            return self._hold(extra=dict(note="no line this frame"))   # coast on last command
        self.last_seen_t = now

        a = SMOOTH
        self.e = m["e_near"] if self.e is None else a * m["e_near"] + (1 - a) * self.e
        self.psi = m["phi_deg"] if self.psi is None else a * m["phi_deg"] + (1 - a) * self.psi
        f, t, self.spinning = wheel_efforts(self.psi, self.spinning)   # self.psi: smoothed aim angle
        if self.f_cruise is not None and f > 0:
            f = min(f, max(F_MOVE_MIN, self.f_cruise))
        self._last = (f, t)
        return self._cmd(f, t, dict(e_in=round(self.e, 2), phi_deg=round(self.psi, 1),
                                    f=round(f), t=round(t), spin=self.spinning, bands=res.get("n_bands")))

    def check(self, now):
        """Between frames: end the run if the camera has gone quiet."""
        if self.status != "running":
            return self._zero()
        if now - self.last_frame_t > STALE_FRAME_S:
            return self._end("stale")
        if now - self.t_start > self.max_run_s:
            return self._end("timeout")
        return self._hold()

    # -- helpers -----------------------------------------------------------------------
    def _cmd(self, f, t, info):
        left_real, right_real = f + t, f - t
        return Command(int(round(self.sign * left_real)), int(round(self.sign * right_real)),
                       self.status, info)

    def _hold(self, extra=None):
        f, t = getattr(self, "_last", (0.0, 0.0))
        c = self._cmd(f, t, extra or {})
        return c

    def _zero(self):
        return Command(0, 0, self.status, {})

    def _end(self, why):
        self.status = why
        return self._zero()
