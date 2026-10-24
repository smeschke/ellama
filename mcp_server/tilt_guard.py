"""IMU tip guard: decides, sample by sample, whether the robot is about to tip over.

Pure logic, no serial and no threads -- RobotLink feeds it every IMU sample and acts on
the result, so it can be unit-tested and replayed against recordings.

How it measures tilt. The IMU is not mounted level (it rests at a few degrees of
pitch/roll on a flat floor), so absolute pitch/roll are meaningless as a limit. The guard
captures a LEVEL REFERENCE -- the filtered orientation while the robot sits still -- and
measures the angle between the current "up" vector and the reference one. That is the
real lean of the chassis from where it was level, in any direction, so a diagonal bank
counts in full (pitch and roll are never checked separately, and never added up wrongly).

When it trips. A tip is a rotation, so the guard also looks ahead: it trips when

    tilt + min(max(0, tilt_rate) * lookahead_s, 0.3 * limit_deg)  >=  limit_deg

for `confirm_samples` samples in a row. `lookahead_s` is roughly how long the robot takes
to actually stop after a "stop" is sent (serial + bridge + the motor board's PWM ramp), so
a fast lean is caught before it reaches the limit, not after. Once tripped it LATCHES:
nothing drives until an operator confirms the robot is upright (clear()).

The tilt comes from bridge.OrientationState's complementary filter, so brief
accelerations from starting and braking (which fool a bare accelerometer by 10-15 deg) are
mostly rejected by the gyro term, and a real lean is picked up immediately by the gyro.

Modes and retuning. This guard exists only because there is an IMU. With no IMU it has
nothing to measure and reports itself inactive -- it does not refuse to let the robot move;
whether a missing IMU should stop a run is the required-sensor policy's call, in
sensors.py, not this file's. `mode` is "on" (trips and latches), "warn" (records what it
would have tripped on and never stops the robot) or "off" (does not evaluate at all, while
still reporting the lean). Mode and thresholds are meant to be changed while running, via
configure(): a limit that is right on a flat floor is wrong on a ramp the robot is
deliberately climbing, and restarting the server to say so would throw away the level
reference too.
"""

import collections
import math
from typing import Optional

DEFAULT_LIMIT_DEG = 25.0
# Stop latency: ~20 ms IMU period + serial/bridge + the motor board ramping PWM to 0 at
# 1 count / 3 ms (0.2 s from PWM 70, 0.4 s from the 125 ceiling).
DEFAULT_LOOKAHEAD_S = 0.3
# The look-ahead can add at most this fraction of the limit, so a quick jolt at small tilt
# (a 90 deg/s pitch bump at 3 deg would "predict" 30) can't trip it: with the default 25 deg
# limit a fast lean trips at 17.5 deg, a slow one at 25.
MAX_LOOKAHEAD_FRAC = 0.3
DEFAULT_CONFIRM_SAMPLES = 2      # at 50 Hz: a single glitched sample can't trip it
DEFAULT_CLEAR_MARGIN_DEG = 5.0   # must be this far under the limit to clear a fault
RATE_ALPHA = 0.3                 # smoothing of the tilt-rate estimate
REF_SAMPLES = 25                 # ~0.5 s of stillness to capture the level reference
REF_MAX_RATE_DPS = 10.0          # "still" = tilt-rate and gyro below this (the gyro rests at ~2.6)
CLEAR_MAX_RATE_DPS = 20.0


def _up(pitch_deg, roll_deg):
    """Unit gravity vector in the body frame for bridge.OrientationState's pitch/roll
    (pitch = atan2(-ax, hypot(ay, az)), roll = atan2(ay, az))."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return (-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))


def _angle_deg(a, b):
    """Angle between two unit vectors; atan2 form so it stays accurate near 0."""
    cross = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    return math.degrees(math.atan2(math.sqrt(sum(c * c for c in cross)), dot))


class TiltGuard:
    #: settable at runtime through configure(); everything else is structural
    TUNABLE = ("limit_deg", "lookahead_s", "confirm_samples", "clear_margin_deg")

    def __init__(self, limit_deg=DEFAULT_LIMIT_DEG, lookahead_s=DEFAULT_LOOKAHEAD_S,
                 confirm_samples=DEFAULT_CONFIRM_SAMPLES, clear_margin_deg=DEFAULT_CLEAR_MARGIN_DEG,
                 enabled=True, mode=None):
        self.limit_deg = float(limit_deg)
        self.lookahead_s = float(lookahead_s)
        self.confirm_samples = int(confirm_samples)
        self.clear_margin_deg = float(clear_margin_deg)
        # `enabled=False` is the older spelling of mode="off" and several callers still use
        # it; an explicit mode wins.
        self.mode = mode if mode is not None else ("on" if enabled else "off")
        self.events = collections.deque(maxlen=12)   # trips, including warn-mode ones
        self.reset_reference()
        self.tripped = False
        self.fault = None          # dict describing the trip while latched
        self.tilt_deg = 0.0
        self.max_tilt_deg = 0.0   # peak lean seen since start or the last cleared fault
        self.rate_dps = 0.0
        self.pitch = self.roll = 0.0
        self.samples = 0
        self._last_ms = None
        self._last_tilt = None
        self._over = 0

    @property
    def enabled(self):
        """True unless the guard is switched off. Kept as a property so the older
        `enabled=` spelling and `mode` can never disagree."""
        return self.mode != "off"

    @enabled.setter
    def enabled(self, value):
        self.mode = "on" if value else "off"

    def configure(self, mode=None, **thresholds):
        """Change the mode and/or thresholds while running. Returns (ok, message).

        Deliberately does NOT clear a latched fault: switching a guard off suspends its
        enforcement (robot_server.py stops consulting it), but what it saw stays on the
        record and comes back if it is switched on again. clear() is the only way to say
        the robot has been looked at.
        """
        if mode is not None:
            if mode not in ("on", "warn", "off"):
                return False, "mode must be on, warn or off"
            self.mode = mode
        for k, v in thresholds.items():
            if k not in self.TUNABLE:
                return False, "unknown tilt setting %r; settable: %s" % (k, ", ".join(self.TUNABLE))
            if v is None:
                continue
            try:
                setattr(self, k, int(v) if k == "confirm_samples" else float(v))
            except (TypeError, ValueError):
                return False, "%s must be a number, got %r" % (k, v)
        if self.limit_deg <= 0:
            return False, "limit_deg must be > 0"
        # A limit raised above the current lean un-trips nothing by itself, but it does mean
        # the robot is no longer over the line -- say so rather than leaving it a mystery.
        note = ""
        if self.tripped and self.tilt_deg < self.limit_deg - self.clear_margin_deg:
            note = " (still latched from an earlier trip; clear_tilt_fault releases it)"
        return True, "tilt guard: mode=%s limit=%.0f deg lookahead=%.2fs%s" % (
            self.mode, self.limit_deg, self.lookahead_s, note)

    # -- reference ---------------------------------------------------------------------

    def reset_reference(self):
        """Forget the level reference; the next ~0.5 s of stillness captures a new one."""
        self._ref = None           # unit "up" vector at level
        self._ref_pr = None        # (pitch, roll) it was taken at, for reporting
        self._ref_source = None    # "auto" | "operator"
        self._acc = []             # (pitch, roll) samples while collecting

    def set_reference(self, pitch_deg, roll_deg, source="operator"):
        self._ref = _up(pitch_deg, roll_deg)
        self._ref_pr = (pitch_deg, roll_deg)
        self._ref_source = source
        self._acc = []
        self._last_tilt = None
        self.tilt_deg = 0.0
        self.rate_dps = 0.0
        self._over = 0

    @property
    def ready(self):
        return self._ref is not None

    # -- per IMU sample ------------------------------------------------------------------

    def update(self, pitch_deg, roll_deg, ms, gyro_dps=0.0) -> Optional[dict]:
        """Feed one IMU sample. Returns the fault dict on the sample that trips (once), else
        None. `ms` is the sender's millis(); `gyro_dps` is the horizontal-axes gyro
        magnitude, used only to decide the robot is "still" while capturing the reference."""
        self.samples += 1
        self.pitch, self.roll = pitch_deg, roll_deg
        dt = None
        if self._last_ms is not None and ms > self._last_ms:
            dt = (ms - self._last_ms) / 1000.0
        self._last_ms = ms

        if self._ref is None:
            # Capture the level reference from REF_SAMPLES consecutive still samples.
            if gyro_dps > REF_MAX_RATE_DPS:
                self._acc = []
            else:
                self._acc.append((pitch_deg, roll_deg))
                if len(self._acc) >= REF_SAMPLES:
                    n = len(self._acc)
                    self.set_reference(sum(p for p, _ in self._acc) / n,
                                       sum(r for _, r in self._acc) / n, source="auto")
            return None

        tilt = _angle_deg(_up(pitch_deg, roll_deg), self._ref)
        if self._last_tilt is not None and dt:
            raw_rate = (tilt - self._last_tilt) / dt
            self.rate_dps += RATE_ALPHA * (raw_rate - self.rate_dps)
        self._last_tilt = tilt
        self.tilt_deg = tilt
        if tilt > self.max_tilt_deg:
            self.max_tilt_deg = tilt

        if self.mode == "off" or self.tripped:
            return None
        predicted = tilt + min(max(0.0, self.rate_dps) * self.lookahead_s,
                               MAX_LOOKAHEAD_FRAC * self.limit_deg)
        self._over = self._over + 1 if predicted >= self.limit_deg else 0
        if self._over >= self.confirm_samples:
            fault = dict(
                reason=("tilt %.1f deg (rate %+.0f deg/s, predicted %.1f) reached the %.0f deg limit"
                        % (tilt, self.rate_dps, predicted, self.limit_deg)),
                tilt_deg=round(tilt, 1), rate_dps=round(self.rate_dps, 1),
                predicted_deg=round(predicted, 1), limit_deg=self.limit_deg,
                pitch_deg=round(pitch_deg, 1), roll_deg=round(roll_deg, 1), ms=ms)
            self.events.append(fault)
            if self.mode == "warn":
                # Noticed and recorded, but the robot keeps going. _over is reset so the next
                # sustained lean is reported again instead of firing every sample.
                self._over = 0
                return None
            self.tripped = True
            self.fault = fault
            return fault
        return None

    # -- operator actions ----------------------------------------------------------------

    def clear(self):
        """Release a latched fault. Only allowed when the robot is back well under the
        limit and not still moving; returns (ok, message)."""
        if not self.tripped:
            return True, "no fault latched"
        if self.tilt_deg > self.limit_deg - self.clear_margin_deg:
            return False, ("still leaning %.1f deg; needs to be under %.0f deg (limit %.0f minus "
                           "%.0f margin) before the fault can clear"
                           % (self.tilt_deg, self.limit_deg - self.clear_margin_deg,
                              self.limit_deg, self.clear_margin_deg))
        if abs(self.rate_dps) > CLEAR_MAX_RATE_DPS:
            return False, "robot is still moving (tilt rate %+.0f deg/s); let it settle" % self.rate_dps
        self.tripped = False
        self.fault = None
        self._over = 0
        self.max_tilt_deg = self.tilt_deg
        return True, "fault cleared"

    def snapshot(self):
        return dict(
            mode=self.mode,
            enabled=self.enabled,
            ready=self.ready,
            tilt_deg=round(self.tilt_deg, 1),
            max_tilt_deg=round(self.max_tilt_deg, 1),
            tilt_rate_dps=round(self.rate_dps, 1),
            limit_deg=self.limit_deg,
            lookahead_s=self.lookahead_s,
            tripped=self.tripped,
            fault=self.fault,
            reference_source=self._ref_source,
            reference_pitch_roll_deg=None if self._ref_pr is None else
            (round(self._ref_pr[0], 2), round(self._ref_pr[1], 2)),
            recent_events=list(self.events)[-3:],
            tunable=list(self.TUNABLE),
        )
