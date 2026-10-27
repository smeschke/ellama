#!/usr/bin/env python3
"""MCP server that lets an AI agent drive eLlama over computer_bridge.ino, closing the
loop on the wheel encoders instead of driving open-loop for a fixed duration.

Talks over the same USB-serial link as calibration/live_view.py and toandfro.py did --
reuses bridge/ for serial I/O, line parsing and the distance/heading math,
so there is exactly one place that logic lives.

Nothing here assumes a fixed set of sensors. The firmware's SensorPacket carries the
sender's own name, so a sensor appears when it starts transmitting and disappears when it
stops, and both are normal: an encoder next to a wheel gets damaged, a ToF board gets
added, and a robot in a walled-off lab under a ceiling camera may carry no sensors at all
and still be fully instrumented. So this server REPORTS what it has -- sensors() -- and
enforces what it was TOLD to need -- set_required_sensors(). That decision belongs to the
operator or to the model driving, made for the situation at hand, not to a constant in
this file. Guards follow from the same place: every guard reads a sensor, so a guard whose
sensor is absent reports itself inactive instead of refusing to let the robot move, and
guards() / set_guard() turn them off and retune them while running (pushing something
needs the bump detector off; a ramp needs a higher tilt limit).

What has NOT changed: Read DRIVING_POLICY.md before calling anything here. Every tool that
can move the robot requires confirmed_safe=true, set only after asking the human operator
in the moment. The robot now has forward range sensors (read_range()), but range alone is
not clearance -- it sees two narrow cones straight ahead, not the sides, not the floor and
not behind. Per the operator's stated policy a sensor-verified small bump forward could one
day skip the ask; that is not wired in, so until it is: ask first, every time, regardless
of move size. First-time-testing-a-new-motor and full-power moves require asking even then.

Setup:
    pip install mcp pyserial
    python3 mcp_server/robot_server.py            # run directly, or register with an
                                                    # MCP client (see mcp_server/README.md)

Env (all of these are startup defaults; the sensor and guard ones are settable at runtime):
    ELLAMA_SERIAL_PORT       explicit serial port (e.g. /dev/ttyUSB0); default auto-detects
                             via bridge/'s port picker.
    ELLAMA_REQUIRED_SENSORS  comma/space separated sensors the drive tools insist on;
                             default "enc,imu". Set it empty to require none.
"""

import array
import atexit
import bisect
import functools
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import tempfile
import time
import urllib.request
import wave
from pathlib import Path
from typing import Literal, Optional

import serial

# bridge/ is the PC-side client library for computer_bridge.ino: the serial reader,
# parse_line(), and the EncoderState/OrientationState/TofState math. It is imported
# rather than copied so this server and calibration/live_view.py compute distance, heading
# and range from the same code.
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
import bridge  # noqa: E402  (bridge/__init__.py)
import sensors as sensorlib  # noqa: E402  (mcp_server/sensors.py; `sensors` is a tool name)
import recording  # noqa: E402  (mcp_server/recording.py)
import hearing  # noqa: E402  (mcp_server/hearing.py)
import tilt_guard  # noqa: E402  (mcp_server/tilt_guard.py)

from mcp.server.mcpserver import MCPServer as FastMCP  # mcp>=2.0 renamed FastMCP -> MCPServer

# ---------------------------------------------------------------------------
# Safety constants -- sourced from DRIVING_POLICY.md, not guessed. If the policy
# document changes, it is right and these are stale; check it before trusting these.
# ---------------------------------------------------------------------------

# "Scripted ground-truth-run ceiling" in DRIVING_POLICY.md -- same value the deleted
# toandfro.py used for encoder-verified runs where the robot actually rolls.
MAX_PWM_CEILING = 125

# True gear ratio confirmed by calibration/30_inches.csv (a real tape-measured 30in
# straight run) -- see calibration/analyze_run.py's TRUE_GEAR_RATIO and its docstring.
# calibration/config.json still says 6.33 and is being left as-is for now, so this
# overrides it in memory only; it does not touch the file.
TRUE_GEAR_RATIO = 4.5

# A stream with no packet for this long is treated as silent. Everything reports at 5 Hz
# (ToF) or 50 Hz (encoders, IMU), so this is many missed packets for any of them.
TELEMETRY_STALE_S = 2.0

# ---------------------------------------------------------------------------
# Which sensors this robot is required to have before the drive tools will move it.
# A policy, not a constant: see the module docstring. The default is the pair the
# closed-loop tools were written against, so nothing loosens until somebody says so.
# ---------------------------------------------------------------------------
_required_env = os.environ.get("ELLAMA_REQUIRED_SENSORS")
POLICY = sensorlib.SensorPolicy(
    [n for n in re.split(r"[,\s]+", _required_env) if n] if _required_env is not None
    else sensorlib.SensorPolicy.DEFAULT)
if _required_env is not None:
    POLICY.set_by = "ELLAMA_REQUIRED_SENSORS"

# Measured PWM -> steady forward speed, ROBOT_CHARACTERIZATION.md section 1 (sweep of
# 2026-09-27, reproduced within ~4% on 09-29). Used ONLY to time an open-loop move when the
# encoders are absent and the operator has said that is acceptable -- it is a prediction
# from a curve, never a measurement, and every result that uses it says so.
PWM_SPEED_IN_S = ((50, 4.11), (60, 6.13), (80, 9.36), (100, 12.03), (125, 14.46))

# No TURN_SCRUB_FACTOR here any more. It existed to rescue an encoder-measured in-place
# turn by dividing out ~2x of wheel scrub, but scrub varies with the surface, so the
# divisor was only ever right on the floor it was calibrated on. turn_degrees now requires
# the IMU rather than correcting a measurement it should not be taking: operator policy,
# 2026-10-24 -- encoders measure arcs and straight lines on this robot, never spins.
# simulation/ keeps its own TURN_SCRUB_FACTOR; that one predicts open-loop turn rate and
# is a different quantity (an effective-track-width multiplier).
# Below this the robot did not move at all in either direction (pwm 40: 0.02 in/s).
PWM_DEADBAND = 45

# Which wheels a distance is measured by. "both" averages them, which is the distance the
# chassis covered; a single wheel is the same number on a straight line and the only one
# available when the other channel is dead. Offered on the straight and arc tools only --
# during a spin in place the wheels turn opposite ways and scrub, so neither one wheel nor
# their average is a distance the robot travelled (see turn_degrees, which uses the IMU).
ENCODER_SELECTIONS = ("both", "left", "right")
WHEEL_NAMES = ("left", "right")

# A channel that was already frozen when a move started is not a dropout: last_counts()
# keeps handing back the same number, so the loop would drive to its timeout chasing a
# target it can never measure (encoder="left"/"right"), or to twice the distance asked
# because the average of a moving wheel and a dead one rises at half rate
# (encoder="both"). Real motion ticks fast -- ~587 ticks/in, so even 1 in/s is hundreds of
# counts a second -- so anything that has not moved by now is not moving.
ENCODER_FIRST_TICK_S = 0.6

# The timer above is the backstop, not the main check: a short move measured by an average
# that includes a dead channel reaches its doubled target in a fraction of a second, so the
# fault has to be caught the moment the two wheels disagree instead. One selected wheel
# having covered this much while another has not ticked at all is that disagreement.
FROZEN_COMPARE_IN = 0.25

# --- range-terminated approach (drive_until_range) -------------------------------------
# Measured 5.3-5.8 Hz on TOF_FRONT; the slow end is the safe one to plan against.
RANGE_SAMPLE_HZ = 5.0
# Brake and settle after the last command the loop sends.
RANGE_STOP_LEAD_S = 0.2
# Steering reads the MEDIAN of the last few samples (see TofState.median_mm), which is the
# right call for noise but lags while the robot is moving: half the window, by definition.
# Observed 2026-10-22 backing up at ~170 mm/s, median 355 mm against a raw 257 mm.
RANGE_MEDIAN_LAG_SAMPLES = bridge.TofState.HISTORY // 2
# How out of date the number the loop is steering by can be once the wheels have stopped.
RANGE_LATENCY_S = (1.0 + RANGE_MEDIAN_LAG_SAMPLES) / RANGE_SAMPLE_HZ + RANGE_STOP_LEAD_S
# speed_for_pwm is a prediction off a curve, not a measurement, so leave headroom rather
# than driving at exactly the speed the arithmetic says is stoppable.
# Raised 1.25 -> 5.0 on 2026-10-24. At 1.25 the taper started far too late to ever finish:
# four approaches to a wall (targets 250/100/150/150, pwm 60-110) all ended by the tyres
# touching it at ~102mm, and every one reported pulses: 0 -- the continuous phase never
# wound down far enough to hand over to the pulse endgame, so the careful part of this
# tool had never once run. pwm_used.min bottomed out at 66 on the last of them, well above
# the 60 floor it was given. The arithmetic said 1.25 was enough; the robot disagreed.
RANGE_APPROACH_MARGIN = 5.0
# After a pulse, the median needs a full window of new samples before it means anything.
RANGE_MEDIAN_REFRESH_S = (bridge.TofState.HISTORY + 1) / RANGE_SAMPLE_HZ
# Long enough that a slow creep still registers as progress, short enough that driving
# into something is caught in about the time a watching operator would call it.
RANGE_NO_PROGRESS_S = 2.0
RANGE_MIN_PULSE_S = 0.2
RANGE_MAX_PULSE_S = 1.0
RANGE_MAX_PULSES = 8
# A ToF this close is inside the robot's own bumper on most mountings, and VL53L0X
# accuracy falls apart in the last few cm.
RANGE_MIN_TARGET_MM = 50.0


def speed_for_pwm(pwm):
    """Predicted steady speed in in/s for a PWM magnitude, linearly interpolated between
    the measured points, or None below the deadband where the robot does not move."""
    pwm = abs(pwm)
    if pwm < PWM_DEADBAND:
        return None
    if pwm <= PWM_SPEED_IN_S[0][0]:
        return PWM_SPEED_IN_S[0][1] * pwm / PWM_SPEED_IN_S[0][0]
    for (p0, v0), (p1, v1) in zip(PWM_SPEED_IN_S, PWM_SPEED_IN_S[1:]):
        if pwm <= p1:
            return v0 + (v1 - v0) * (pwm - p0) / (p1 - p0)
    return PWM_SPEED_IN_S[-1][1]

def pwm_for_speed(speed_in_s):
    """The highest PWM whose predicted speed does not exceed `speed_in_s`, or None when
    even PWM_DEADBAND -- the slowest the robot moves at all -- is faster than that.

    The inverse of speed_for_pwm over the same measured curve, so the two agree by
    construction. None is the useful answer, not a failure: it means "you cannot creep
    this slowly", which is what tells an approach loop to stop and pulse instead of
    trying to drive continuously."""
    if speed_in_s <= 0:
        return None
    if speed_in_s < speed_for_pwm(PWM_DEADBAND):
        return None
    best = PWM_DEADBAND
    for p in range(PWM_DEADBAND, MAX_PWM_CEILING + 1):
        v = speed_for_pwm(p)
        if v is None or v > speed_in_s:
            break
        best = p
    return best


def selected_wheels(encoder):
    """Indices into an (left, right) pair that `encoder` names."""
    if encoder == "left":
        return (0,)
    if encoder == "right":
        return (1,)
    return (0, 1)


def traveled_inches(start, cur, encoder, in_per_tick):
    """Distance covered between two cumulative count pairs, by the selected encoder(s).

    Unsigned: every caller here already knows which way it commanded the wheels, and a
    reversed encoder would otherwise count a move backwards (see ENCODER_FORWARD_SIGN,
    which has flipped on this robot before)."""
    idx = selected_wheels(encoder)
    return sum(abs(cur[i] - start[i]) for i in idx) / len(idx) * in_per_tick


def frozen_wheels(start, cur, encoder):
    """Selected channels whose count has not changed at all since `start`."""
    return [WHEEL_NAMES[i] for i in selected_wheels(encoder) if cur[i] == start[i]]


def frozen_wheel_fault(start, cur, encoder, in_per_tick, elapsed_s):
    """Frozen selected channels, once that is a fault rather than a move still getting
    going. Either another selected wheel has already covered FROZEN_COMPARE_IN -- so the
    two disagree and one of them is wrong -- or nothing has ticked for
    ENCODER_FIRST_TICK_S, so nothing is measuring this move at all.

    Waiting only on the timer would miss the case that matters most: with encoder="both"
    and one channel dead, the average rises at half rate and a short move sails past twice
    the distance asked before the timer ever fires."""
    frozen = frozen_wheels(start, cur, encoder)
    if not frozen:
        return []
    moved_in = max(abs(cur[i] - start[i]) for i in selected_wheels(encoder)) * in_per_tick
    if moved_in >= FROZEN_COMPARE_IN or elapsed_s > ENCODER_FIRST_TICK_S:
        return frozen
    return []


def approach_pwm(remaining_mm, pwm_max, pwm_min):
    """The fastest PWM that still stops inside `remaining_mm`, or None to stop and pulse.

    The loop can only ever act on a number that is RANGE_LATENCY_S out of date, so the
    distance still to go has to cover the travel in that blind window: at PWM 80 (measured
    288 mm in a 1 s burst on 2026-10-22) that window is around 230 mm of unseen motion.
    Driving no faster than remaining/latency is what keeps the overshoot inside the gap
    rather than inside the wall."""
    allowed_in_s = (remaining_mm / bridge.MM_PER_IN) / (RANGE_LATENCY_S * RANGE_APPROACH_MARGIN)
    p = pwm_for_speed(allowed_in_s)
    if p is None or p < pwm_min:
        return None
    return min(pwm_max, p)


# Pause after every stop before starting a new move, satisfying "no sign reversal without
# a stop in between" unconditionally rather than trying to detect reversals.
STOP_SETTLE_S = 0.25

# computer_bridge.ino drops to listen-only if no serial line arrives for 500ms
# (SERIAL_IDLE_TIMEOUT_MS), so every drive loop re-sends its command at least this often.
# Verified 2026-09-28: without this, drive_distance(6in, pwm=50) stopped after ~3in.
DRIVE_KEEPALIVE_S = 0.1

# drive_arc() caps. A circle is one continuous closed-loop motion, so it gets its own
# path-length cap; it still aborts the moment telemetry goes stale. Radius floor keeps it
# from degenerating into an in-place spin (use turn_degrees for that).
MAX_SINGLE_ARC_DEG = 360.0
MAX_SINGLE_ARC_IN = 160.0
MIN_ARC_RADIUS_IN = 6.0
# How often drive_arc re-measures curvature and corrects the wheel-speed split, and how
# hard it corrects (fraction of pwm per unit of relative curvature error).
ARC_CONTROL_DT_S = 0.1
ARC_GAIN = 0.15
# With the inner wheel stopped this robot can't curve tighter than ~12.5in radius
# (measured 2026-09-28, pwm=60), so drive_arc lets the inner wheel run backwards, down to
# this PWM, for tighter circles. The wheel split changes by at most ARC_MAX_STEP_PWM per
# control step, so the inner wheel eases through zero into reverse instead of jumping.
ARC_MAX_INNER_REVERSE_PWM = int(0.9 * MAX_PWM_CEILING)  # 112; at -60 it bottomed out at ~7.9in radius
ARC_MAX_STEP_PWM = 5

# Encoder count direction when that wheel rolls forward. (-1, -1) since 2026-10-24.
# Every earlier value here -- (1, -1) on 2026-09-28, (1, 1) on 2026-09-29 -- was read off a
# drive_distance(direction="forward") move, which applies FORWARD_PWM_SIGN. That sign was
# itself wrong until 2026-10-24, so those tests recorded the tick direction of a BACKWARD
# move and labelled it forward. The two constants then agreed with each other no matter
# which way the robot really went, and no encoder-based check could catch it.
# Settled by watching the robot instead: with FORWARD_PWM_SIGN = 1, a jog(+50, +50, 1s)
# moved it physically forward (operator confirmed by eye; there is now an arrow on the
# chassis, since a symmetrical robot gives you nothing else to go on) while the right
# encoder ran -2247 ticks / -3.83in. So forward counts DOWN on this robot.
# Only the right channel was verified -- the left was dead (reporting null) at the time,
# and its -1 is assumed by symmetry. Confirm it once that encoder reports again.
# Only drive_arc uses this; re-check it after any encoder wiring or firmware change, and
# check it by where the robot ends up, never by a "forward" command's own ticks.
ENCODER_FORWARD_SIGN = (-1, -1)

# Re-verified 2026-10-24, robot on the ground and driving: with this at -1, repeated
# direction="forward" moves at pwm=60 carried the robot visibly backwards, confirmed by
# the operator watching it. +1 makes direction="forward" match real forward travel. This
# supersedes the 2026-09-27 on-blocks check, which read the wheel rotation by eye with
# the robot jacked up and concluded the opposite; trust the on-the-ground result.
# Re-check this after any change to wiring, motor board firmware, or which physical
# wheel is "left" vs "right".
FORWARD_PWM_SIGN = 1

# ---------------------------------------------------------------------------
# Tilt guard (mcp_server/tilt_guard.py). The robot is low and can run banks up to ~25 deg;
# past that it tips. Every drive command goes through RobotLink.send_drive, which refuses
# (and sends "stop") once the guard has tripped, or while there is no live IMU to judge by.
# This only covers commands that come through THIS server -- the handheld stick is not
# guarded. Env knobs are the operator's, deliberately not tool arguments:
#   ELLAMA_TILT_LIMIT_DEG     trip angle from the level reference (default 25)
#   ELLAMA_TILT_LOOKAHEAD_S   stop-latency look-ahead on the tilt rate (default 0.3)
#   ELLAMA_TILT_GUARD=off     bench use only: never trips and never refuses (still reports tilt)
# ---------------------------------------------------------------------------
TILT_LIMIT_DEG = float(os.environ.get("ELLAMA_TILT_LIMIT_DEG", tilt_guard.DEFAULT_LIMIT_DEG))
TILT_LOOKAHEAD_S = float(os.environ.get("ELLAMA_TILT_LOOKAHEAD_S", tilt_guard.DEFAULT_LOOKAHEAD_S))
TILT_GUARD_ENABLED = os.environ.get("ELLAMA_TILT_GUARD", "on").lower() not in ("off", "0", "false", "no")
# Moving with the IMU silent means flying blind on the one thing this guard watches.
IMU_STALE_FAULT_S = 0.5


# One guard for the life of the server process, shared by every RobotLink. connect() (and a
# robot power cycle) must not be a way to wipe a latched fault, or to re-learn "level" from a
# robot that is leaning -- only restarting the server, which the operator does, resets it.
TILT = tilt_guard.TiltGuard(limit_deg=TILT_LIMIT_DEG, lookahead_s=TILT_LOOKAHEAD_S,
                            enabled=TILT_GUARD_ENABLED)


class GuardFault(Exception):
    """Raised by RobotLink.send_drive when the tilt guard refuses a drive command.
    Drive loops let it propagate: the tool's own `finally` sends the stop, and the tool returns
    the fault (see _tilt_guarded); autonomous threads end with reason "error: ... guard ...".
    `info` carries a snapshot of the guard."""

    def __init__(self, message, info=None):
        super().__init__(message)
        self.info = info or {}


TiltFault = GuardFault  # the original name; tests and callers still use it


def _pick_port(explicit):
    """Like bridge.pick_port, but raises instead of calling sys.exit() -- this runs
    inside a long-lived server, not a one-shot script, so nothing here may exit the
    process."""
    if explicit:
        return explicit
    candidates = bridge.list_candidate_ports()
    if not candidates:
        raise RuntimeError(
            "no serial ports found -- plug in the bridge ESP32, or pass an explicit port"
        )
    return candidates[0]


class RobotLink:
    """Owns the serial connection, the background telemetry pump, and drive commands."""

    def __init__(self, port=None):
        self.cfg = bridge.load_config()
        self.track_width_calibrated = (
            self.cfg["track_width_in"] != bridge.DEFAULT_CONFIG["track_width_in"]
        )
        self.cfg["gear_ratio"] = TRUE_GEAR_RATIO

        port = _pick_port(port)
        # exclusive=True so a second process opening this port fails loudly instead of
        # silently stealing bytes. Nothing locks a tty by default: two readers each consume
        # from the same buffer and both get torn lines, and opening the port a second time
        # resets the ESP32 (see bridge.RESET_SETTLE_S) -- mid-drive, if the robot is moving.
        self.ser = serial.Serial(port, bridge.BAUD_RATE, timeout=0.05, exclusive=True)
        time.sleep(bridge.RESET_SETTLE_S)  # opening the port resets the ESP32
        self.ser.reset_input_buffer()

        self.reader = bridge.SerialReader(self.ser)
        self.reader.start()

        self.enc = bridge.EncoderState(self.cfg)
        self.orient = bridge.OrientationState()
        self.tof = bridge.TofState()
        # Every stream heard from, keyed by the name inside the packet. Nothing registers
        # sensors in advance -- they appear when they transmit.
        self.sensors = sensorlib.SensorRegistry(TELEMETRY_STALE_S)
        # The tilt guard judges lean sample by sample, so a half-second gap in the IMU
        # already means it is guessing. Hold the IMU to a tighter deadline than the rest.
        self.sensors.set_stale_after("imu", IMU_STALE_FAULT_S)
        self.policy = POLICY   # module-level: survives connect(), like the guards
        self.lock = threading.Lock()
        self.recorder = recording.Recorder(REPO_ROOT / "recordings")
        self.last_imu = None

        # Serial writes come from tool threads AND the pump (the tilt trip sends "stop" from
        # there, ~20 ms after the offending IMU sample), so a line must never interleave.
        self._wlock = threading.RLock()
        self.tilt = TILT  # module-level: a latched fault and the level reference survive connect()

        self._stop_pump = threading.Event()
        self._pump_thread = threading.Thread(target=self._pump, daemon=True)
        self._pump_thread.start()

    # last_*_wall read through the registry so there is one answer to "when did we last
    # hear from this sensor", rather than a field here that could disagree with it.
    @property
    def last_enc_wall(self):
        s = self.sensors.get("enc")
        return None if s is None else s.last_wall

    @property
    def last_imu_wall(self):
        s = self.sensors.get("imu")
        return None if s is None else s.last_wall

    def _pump(self):
        """Drains the serial reader queue into the sensor registry and the enc/orient/tof
        state, forever. This is the only thread that mutates them -- tools only read."""
        while not self._stop_pump.is_set():
            try:
                raw = self.reader.q.get(timeout=0.2)
            except Exception:
                continue
            kind, payload = bridge.parse_line(raw)
            tripped = None
            now = time.monotonic()
            with self.lock:
                if kind == "ENC":
                    left, right, ms = payload
                    # Either count may be None: that encoder is not responding, which is a
                    # routine state next to a wheel, not an error. EncoderState holds that
                    # wheel's distance rather than reading it as a stationary wheel.
                    sl, sr = self.enc.update(left, right, ms)
                    self.sensors.observe("enc", (left, right), ms,
                                         available=list(self.enc.available), when=now)
                    self.recorder.on_enc(self.enc, self.orient, sl, sr)
                elif kind == "CMD":
                    self.recorder.on_cmd(*payload)  # the stick's command, overheard by the bridge
                elif kind == "IMU":
                    ax, ay, az, gx, gy, gz, ms = payload
                    agx, agy, agz, dx, dy, dz = self.orient.update(ax, ay, az, gx, gy, gz, ms)
                    self.sensors.observe("imu", (ax, ay, az, gx, gy, gz), ms, when=now)
                    self.recorder.on_imu(self.enc, self.orient, (agx, agy, agz))
                    self.last_imu = dict(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, ms=ms)
                    tripped = self.tilt.update(self.orient.pitch, self.orient.roll, ms,
                                               gyro_dps=math.hypot(dx, dy))
                elif kind == "SENSOR":
                    # Any SensorPacket sender. Generic on the name on purpose: the name
                    # travels in the packet, so flashing a second ToF board as "TOF_REAR"
                    # needs no change here, in bridge.py, or in the bridge firmware.
                    name, values, ms = payload
                    available = None
                    if sensorlib.canon(name).startswith("tof"):
                        # Range sensors are the ones whose -1 means "could not measure";
                        # TofState applies that reading and keeps the median history.
                        self.tof.update(name, values, ms)
                        available = [v != bridge.SENSOR_VALUE_INVALID for v in values]
                        self.recorder.on_range(self.tof.closest_mm())
                    self.sensors.observe(name, values, ms, available=available, when=now)
                    self.recorder.on_sensor(name, values, self.enc, self.orient)
                elif kind is None:
                    # computer_bridge.ino's own counters. DROPPED is its receive queue
                    # overrunning, which is what used to look like a radio fault.
                    parts = payload.split()
                    if len(parts) == 2 and parts[0] in ("DROPPED", "FOREIGN"):
                        try:
                            self.sensors.note_bridge(parts[0], int(parts[1]))
                        except ValueError:
                            pass
            if tripped is not None:
                # Outside the state lock: the guard already latched, so send_drive refuses
                # from here on. A blocking tool notices on its next send_drive, which
                # raises TiltFault.
                self.emergency_stop()

    def send_line(self, text):
        with self._wlock:
            self.ser.write((text + "\n").encode())
            self.ser.flush()

    def missing_required(self):
        """Required sensors that are not reporting right now (canonical names)."""
        return self.policy.missing(self.sensors)

    def _missing_detail(self, name):
        age = self.sensors.age_s(name)
        stream = self.sensors.get(name)
        if stream is not None and not stream.trusted:
            # One packet is not a sensor: the serial link tears and leaves plausible-looking
            # names behind (see sensors.py). Say so rather than reporting it as silent.
            return ("%s seen only once, not yet confirmed -- one packet can be a garbled "
                    "line rather than a sensor" % name)
        when = "never seen" if age is None else "last sample %.1fs ago" % age
        if name == "imu":
            return "no live IMU (%s)" % when
        if name == "enc":
            return "no live encoder telemetry (%s)" % when
        return "%s not reporting (%s)" % (name, when)

    def _drive_refusal(self):
        """Why a nonzero drive command must not go out right now, or None.

        Two different questions, asked in this order:

        1. Is a sensor this run was DECLARED to need missing? That is the policy's call
           (set_required_sensors), not this file's opinion. A sensor that is absent but not
           required is reported in the result and never blocks.
        2. Has a guard that is switched on actually tripped? A trip is a measurement that
           something went wrong, so it stays latched until an operator looks at the robot
           and calls clear_tilt_fault.

        A guard whose sensor is absent is NOT a refusal. With no IMU there is no tilt
        guard, which sensors() states as a fact; whether to drive anyway was already
        settled by question 1.
        """
        missing = self.missing_required()
        if missing:
            return ("%s -- required by the current sensor policy. Fix the link, or call "
                    "set_required_sensors() to declare this run does not need %s (see what "
                    "that costs in sensors())"
                    % ("; ".join(self._missing_detail(n) for n in missing),
                       " / ".join(missing)))
        t = self.tilt
        if t.mode == "off":
            return None
        if t.tripped:
            return "tilt guard tripped: " + t.fault["reason"]
        # The guard needs a level reference before it can judge anything, and it can only
        # capture one from a live IMU. If the IMU is required, step 1 already passed, so a
        # missing reference is a real "wait a moment". If it is NOT required, the operator
        # has said to drive without an IMU and an unready guard is not news.
        if "imu" in self.policy.required and not t.ready:
            return "tilt guard has no level reference yet (needs ~0.5 s of a still, live IMU)"
        return None

    #: the original name; test_tilt_guard.py and older callers still use it
    _tilt_refusal = _drive_refusal

    def send_drive(self, left, right):
        left = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(left)))
        right = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(right)))
        with self._wlock:  # check and write atomically vs. the trip's "stop" (also under it)
            if left or right:  # a zero command is always allowed -- it's how loops stop
                why = self._drive_refusal()
                if why is not None:
                    self.emergency_stop()
                    raise GuardFault(why, self._guard_info())
            self.send_line(f"{left} {right}")

    def _guard_info(self):
        """The tilt guard, annotated with whether the sensor it reads is even present --
        an "off" guard and one with no IMU look identical without this."""
        tilt = self.tilt.snapshot()
        imu_live = self.sensors.trusted_live("imu")
        tilt["sensor"] = "imu"
        tilt["sensor_live"] = imu_live
        tilt["active"] = tilt["mode"] == "on" and imu_live and tilt["ready"]
        if not imu_live:
            tilt["inactive_because"] = "no live IMU: nothing measures lean"
        elif not tilt["ready"]:
            tilt["inactive_because"] = "no level reference captured yet"
        return {"tilt": tilt, "policy": self.policy.snapshot(self.sensors)}

    def send_stop(self):
        self.send_line("stop")

    def send_listen(self):
        self.send_line("listen")

    def emergency_stop(self):
        """Best-effort stop for atexit/signal handlers. Never raises."""
        try:
            self.send_line("stop")
            self.ser.flush()
        except Exception:
            pass

    def last_counts(self):
        """(left, right) cumulative ticks, or None unless BOTH wheels reported.

        EncoderState.last holds a per-wheel value that is None while that encoder is not
        responding. Callers difference these to measure a move, and one None would make
        that arithmetic raise mid-move with the motors running -- so a partial reading is
        reported as no reading, and the drive tools decide what to do about it."""
        with self.lock:
            left, right = self.enc.last
            return None if (left is None or right is None) else (left, right)

    def wheel_counts(self, encoder="both"):
        """(left, right) cumulative ticks, or None unless the channels `encoder` needs
        are both reporting.

        last_counts() insists on both wheels because its callers difference both. A move
        measured by one nominated encoder needs only that one, so this is the same reading
        with a narrower requirement: the unselected slot may come back None, and nothing
        is allowed to difference it."""
        with self.lock:
            counts = tuple(self.enc.last)
        if any(counts[i] is None for i in selected_wheels(encoder)):
            return None
        return counts

    def range_snapshot(self, now=None):
        """Every ToF channel, or None if no range sensor has ever reported.

        `mm` is the median of the last few samples, which is what anything steering should
        use: the firmware deliberately sends any distance inside the sensor's physical
        range without consulting ST's RangeStatus, so occasional short samples arrive and
        the median is what removes them. `latest_mm` is the raw last sample.

        in_range describes `mm`, the median: false means there is no recent usable reading
        on that channel. latest_in_range describes the last raw ping on its own, which flaps
        near the sensor's limit -- a channel can read in_range=true with
        latest_in_range=false, meaning "something is there, the last ping just missed it".

        Neither is the same as live=false, which means the board has gone quiet. A sensor
        that reports "nothing in reach" and a sensor that reports nothing at all call for
        opposite actions, so they are never collapsed.
        """
        now = now or time.monotonic()
        if not self.tof.channels:
            return None
        channels = {}
        for (name, index) in sorted(self.tof.channels):
            entry = self.tof.channels[(name, index)]
            median = self.tof.median_mm(name, index)
            stream = self.sensors.get(name)
            channels[bridge.TofState.label(name, index)] = dict(
                sensor=name,
                channel=index,
                mm=median,
                inches=None if median is None else round(median / bridge.MM_PER_IN, 1),
                latest_mm=entry["mm"],
                # About `mm` (the median), not the last ping -- so a channel never reports
                # a distance and "nothing in range" in the same breath.
                in_range=median is not None,
                latest_in_range=not entry["invalid"],
                live=bool(stream and stream.live(now)),
            )
        closest = self.tof.closest_mm()
        live_any = any(c["live"] for c in channels.values())
        return dict(
            channels=channels,
            closest_mm=closest,
            closest_in=None if closest is None else round(closest / bridge.MM_PER_IN, 1),
            live=live_any,
            # Spelled out because "closest_mm: null" is ambiguous on its own, and the two
            # readings call for opposite actions.
            note=("no range sensor is reporting -- the robot is blind, not clear" if not live_any
                  else "nothing within range of any sensor" if closest is None
                  else "closest obstacle %.0f mm (%.1f in) -- forward cones only; this says "
                       "nothing about the sides, the floor or behind"
                       % (closest, closest / bridge.MM_PER_IN)),
        )

    def telemetry_snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = None if self.last_enc_wall is None else now - self.last_enc_wall
            imu_age = None if self.last_imu_wall is None else now - self.last_imu_wall
            left_ticks, right_ticks = self.enc.last
            heading = self.enc.heading_deg   # None while either encoder is missing
            return dict(
                left_ticks=left_ticks,
                right_ticks=right_ticks,
                # [left, right]: which encoder answered in the last packet. A damaged one
                # is routine, so this is reported, not treated as a fault.
                encoders_available=list(self.enc.available),
                dist_l_in=round(self.enc.dist_l, 3),
                dist_r_in=round(self.enc.dist_r, 3),
                dist_avg_in=round(self.enc.dist_avg, 3),
                # None rather than a number computed from one live wheel and one frozen
                # one, which would look plausible and drift without bound.
                heading_deg=None if heading is None else round(heading, 2),
                imu_last=self.last_imu,
                pitch_deg=round(self.orient.pitch, 2),
                roll_deg=round(self.orient.roll, 2),
                yaw_deg=round(self.orient.yaw, 2),
                tilt=self.tilt.snapshot(),
                telemetry_age_s=None if age is None else round(age, 2),
                telemetry_live=age is not None and age < TELEMETRY_STALE_S,
                imu_age_s=None if imu_age is None else round(imu_age, 2),
                imu_live=imu_age is not None and imu_age < TELEMETRY_STALE_S,
                range=self.range_snapshot(now),
                sensors=self.sensors.summary(now),
                bridge=dict(dropped_frames=self.sensors.dropped,
                            foreign_frames=self.sensors.foreign),
                required_sensors=list(self.policy.required),
                missing_sensors=self.policy.missing(self.sensors, now),
                gear_ratio_used=self.cfg["gear_ratio"],
                track_width_in=self.cfg["track_width_in"],
                track_width_calibrated=self.track_width_calibrated,
            )

    def in_per_tick(self):
        return bridge.in_per_tick(self.cfg)

    def close(self):
        self.emergency_stop()
        self.recorder.stop()
        self._stop_pump.set()
        self.reader.stop()  # joins bridge.py's own reader thread before the port closes
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Connection lifecycle -- lazy/reconnectable so the server can start before the ESP32
# is plugged in, and so a power-cycle doesn't require restarting the whole server.
# ---------------------------------------------------------------------------

link: Optional[RobotLink] = None
_connect_error: Optional[str] = None


def _connect(port=None):
    global link, _connect_error
    if link is not None:
        link.close()
        link = None
    try:
        link = RobotLink(port)
        _connect_error = None
    except Exception as e:
        _connect_error = str(e)
        raise
    return link


def _shutdown():
    if link is not None:
        link.emergency_stop()


atexit.register(_shutdown)
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, lambda signum, frame: (_shutdown(), sys.exit(1)))

try:
    _connect(os.environ.get("ELLAMA_SERIAL_PORT"))
except Exception:
    pass  # tools report the connection error; the server itself still starts




mcp = FastMCP("ellama-robot")


def _not_connected():
    return {"ok": False, "error": f"not connected to the robot bridge: {_connect_error}"}


def _tilt_gate():
    """Early, readable refusal for any motion tool; send_drive enforces the same thing as the
    last line of defence."""
    why = link._drive_refusal() if link is not None else None
    if why is None:
        return None
    missing = link.missing_required()
    if missing:
        hint = ("either bring %s back (check the board's power and link, then connect()), or "
                "decide this run does not need it: set_required_sensors() lists what each one "
                "costs. sensors() shows everything that IS reporting."
                % " / ".join(missing))
    elif link.tilt.tripped:
        hint = ("if the robot is upright and still: clear_tilt_fault(confirmed_safe=true) after "
                "asking the operator. On a slope the robot is meant to climb, "
                "set_guard('tilt', limit_deg=...) raises the limit instead.")
    else:
        hint = ("no level reference yet: wait a second with the robot still and the IMU live, "
                "or call set_level_reference")
    return {"ok": False, "error": why, "guards": link._guard_info(), "hint": hint}


def _tilt_guarded(fn):
    """Turn a TiltFault raised mid-move into a normal tool result. The tool's own `finally`
    has already sent the stop by the time this sees it."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except GuardFault as e:
            return {"ok": False, "error": "guard: %s" % e, "guards": e.info,
                    "telemetry": link.telemetry_snapshot() if link is not None else None}
    return wrapper


def _confirm_gate(confirmed_safe: bool):
    tilt_block = _tilt_gate()
    if tilt_block:
        return tilt_block
    if confirmed_safe:
        return None
    rng = link.range_snapshot() if link is not None else None
    return {
        "ok": False,
        "error": (
            "confirmed_safe must be true, and only after asking the human operator in "
            "this specific moment. The robot's range sensors cover two narrow cones "
            "straight ahead -- they are not clearance: they cannot see the sides, the "
            "floor, a drop, or anything behind. See DRIVING_POLICY.md. State the exact "
            "move (distance/degrees, direction, PWM) and wait for a go-ahead before "
            "calling again with confirmed_safe=true."
        ),
        "range": rng,
    }


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


@mcp.tool()
def connect(port: Optional[str] = None) -> dict:
    """(Re)connect to the bridge ESP32 over serial. Call this if the server started
    before the ESP32 was plugged in, or after a power cycle / reconnect. `port` defaults
    to the ELLAMA_SERIAL_PORT env var or auto-detection (e.g. /dev/ttyUSB0)."""
    try:
        l = _connect(port)
        return {"ok": True, "port": l.ser.port}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@mcp.tool()
def read_telemetry() -> dict:
    """Read-only. Current cumulative encoder counts, distance/heading since the last
    zero_odometry(), latest IMU sample and complementary-filter pitch/roll/yaw, and
    whether encoder telemetry is currently live (packets seen within the last couple of
    seconds). Always safe to call -- never sends a drive command. Check telemetry_live
    before trusting any distance/heading number here."""
    if link is None:
        return _not_connected()
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def sensors() -> dict:
    """Read-only. Every sensor stream this robot is actually producing, what each guard can
    therefore do, and which sensors the drive tools are currently required to have.

    This is the call to make before deciding whether the robot can run in a given
    situation. Nothing here is a fixed list: a stream appears the first time a packet with
    its name arrives, so a ToF board flashed as "TOF_REAR" shows up on its own.

    Read `streams` carefully -- three states that must not be confused:
      live=true, measuring=true    working
      live=true, measuring=false   reporting, but with nothing to report: a range sensor
                                   with nothing in its cone, or an encoder that is not
                                   responding. `available` says which channels.
      live=false                   silent: board dead, unpowered, unflashed or absent.

    "Nothing is in front of the robot" and "the robot cannot see" are different answers.

    A name that has been seen only once lands in `provisional`, not `streams`: the serial
    link tears, and a torn "IMU" line leaves "MU" looking like a sensor. A provisional name
    never satisfies a required sensor, and is dropped if it stays quiet. A board you have
    just switched on appears there for one packet and then moves to `streams`.

    `guards` says, per guard, whether its sensor is even present -- a guard with no sensor
    reports inactive and does NOT stop the robot; whether to drive in that state is the
    policy's call, not the guard's. `required` is that policy; `missing` is what it asks
    for and is not getting, and `costs` spells out what each absence means.
    """
    if link is None:
        return _not_connected()
    now = time.monotonic()
    snap = link.sensors.snapshot(now)
    missing = link.policy.missing(link.sensors, now)
    return {
        "ok": True,
        "streams": snap["streams"],
        "provisional": snap["provisional"],
        "provisional_note":
            "names seen only once. Either a board you just switched on, or a garbled serial "
            "line -- the link tears occasionally and a torn \"IMU\" leaves \"MU\" behind. "
            "They never satisfy a required sensor, and are forgotten if they stay quiet."
            if snap["provisional"] else None,
        "bridge": snap["bridge"],
        "range": link.range_snapshot(now),
        "guards": link._guard_info(),
        "required": list(link.policy.required),
        "required_set_by": link.policy.set_by,
        "missing": missing,
        "costs": sensorlib.capability_cost(missing),
        "can_drive": link._drive_refusal() is None,
        "why_not": link._drive_refusal(),
    }


@mcp.tool()
def read_range() -> dict:
    """Read-only. Distance to the closest thing the forward ToF sensors can see.

    `closest_mm` is a median of recent samples, which is what to steer by; each channel
    also carries `latest_mm`, the raw last reading. Never sends a drive command.

    Two things this does NOT mean. `closest_mm: null` with live=true means every sensor
    reported and found nothing in range -- clear as far as they reach. `live: false` means
    they are not reporting at all, which is blind, not clear; the `note` field says which.
    And range is not clearance either way: these are two narrow cones pointing forward.
    They cannot see the sides, the floor, a drop, or anything behind the robot, so they do
    not replace the operator's confirmed_safe.
    """
    if link is None:
        return _not_connected()
    rng = link.range_snapshot()
    if rng is None:
        return {"ok": True, "range": None,
                "note": "no range sensor has ever reported on this link -- none fitted, or "
                        "none powered. Check sensors() for what is reporting."}
    return {"ok": True, **rng}


@mcp.tool()
def set_required_sensors(names: list[str], confirmed_safe: bool) -> dict:
    """Declare which sensors the drive tools must have live before they will move the robot.
    Does not move the robot. Takes effect immediately and lasts until changed or the server
    restarts.

    This is the judgement call the robot cannot make for itself: whether the situation is
    instrumented well enough for what is about to happen. Examples:
      ["enc", "imu"]   the default -- the robot's own closed-loop feedback
      ["enc"]          the IMU board is dead; drive by encoders, no tilt guard
      []               nothing on the robot is required, because something outside it is
                       watching: a ceiling camera, a marker on the robot, an operator with
                       a hand on the stick. Everything becomes open-loop and says so.
      ["enc", "tof"]   also insist on a range sensor before driving

    "tof" (or "range") matches any ToF board, so it keeps working when TOF_FRONT is joined
    or replaced by another. Unknown names are accepted -- a sensor you are about to flash
    is simply "missing" until it transmits.

    confirmed_safe is required because loosening this widens what the robot will do without
    anything checking it; set it only after the operator has agreed to drive this way. The
    result lists what the new set costs you, and the change is written into any active
    recording.
    """
    if link is None:
        return _not_connected()
    if not confirmed_safe:
        return {"ok": False, "error":
                "confirmed_safe must be true: changing the required sensors changes what the "
                "robot will do with no feedback. Tell the operator which sensors you are "
                "dropping and what that costs (see sensors()), and wait for a go-ahead."}
    if not isinstance(names, (list, tuple)):
        return {"ok": False, "error": "names must be a list of sensor names, e.g. [\"enc\"]"}
    before = list(link.policy.required)
    required = link.policy.set(names)
    missing = link.policy.missing(link.sensors)
    link.recorder.on_config("required_sensors %s -> %s" % (before, required))
    return {
        "ok": True,
        "required": required,
        "was": before,
        "missing": missing,
        "costs": sensorlib.capability_cost(
            [n for n in sensorlib.SensorPolicy.DEFAULT if n not in required] + missing),
        "can_drive": link._drive_refusal() is None,
        "why_not": link._drive_refusal(),
        "guards": link._guard_info(),
    }


@mcp.tool()
def guards() -> dict:
    """Read-only. The tilt guard: mode, thresholds, whether the IMU it reads is live,
    whether it has tripped and why. Does not move the robot.

    A guard reporting active=false with inactive_because set is not broken -- it has no
    sensor to judge by. That on its own never stops the robot; set_required_sensors()
    decides whether driving without that sensor is acceptable.
    """
    if link is None:
        return _not_connected()
    return {"ok": True, **link._guard_info()}


@mcp.tool()
def set_guard(
    name: Literal["tilt"] = "tilt",
    mode: Optional[Literal["on", "warn", "off"]] = None,
    limit_deg: Optional[float] = None,
    lookahead_s: Optional[float] = None,
) -> dict:
    """Turn the tilt guard on or off and retune it, while running. Does not move the robot.

    `mode` is "on" (trips and latches), "warn" (records what it would have tripped on and
    never stops the robot -- use this to tune on a real robot) or "off".

    limit_deg (default 25) and lookahead_s are the thresholds. Raise limit_deg for a ramp
    the robot is meant to climb; the guard measures lean from where it was level, so a ramp
    is a real lean to it.

    What this deliberately does NOT do: clear a latched fault. Switching the guard off
    suspends its enforcement, but what it saw stays on the record and comes back if you
    switch it on again. Only clear_tilt_fault releases a latch, and it requires the
    operator to have actually looked at the robot.

    Changes are written into any active recording, so a run's CSV shows the guard config it
    was driven under.
    """
    if link is None:
        return _not_connected()
    ok, msg = link.tilt.configure(mode=mode, limit_deg=limit_deg, lookahead_s=lookahead_s)
    if not ok:
        return {"ok": False, "error": msg, "guards": link._guard_info()}
    link.recorder.on_config("%s guard: %s" % (name, msg))
    return {"ok": True, "message": msg, "guards": link._guard_info()}


@mcp.tool()
def zero_odometry() -> dict:
    """Make the robot's current position the origin for distance_*_in and heading_deg in
    read_telemetry(). Does not move the robot -- always safe to call, no confirmation
    needed."""
    if link is None:
        return _not_connected()
    with link.lock:
        link.enc.zero()
        link.orient.zero_yaw()
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def stop() -> dict:
    """Immediately command zero PWM (active braking, not coasting -- see
    DRIVING_POLICY.md). Always safe to call with no confirmation, any time, including
    when nothing is currently moving."""
    if link is None:
        return _not_connected()
    link.send_stop()
    time.sleep(STOP_SETTLE_S)
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def clear_tilt_fault(confirmed_safe: bool) -> dict:
    """Release the tilt guard after it tripped (the robot leaned past the limit and every
    drive tool is refused until this is called). Does not move the robot. Set
    confirmed_safe=true only after the human operator has looked at the robot and said it is
    upright and clear of whatever it was climbing. Refused while the IMU still reads the
    robot leaning within 5 deg of the limit or still rocking, so it cannot be used to drive
    out of a lean -- pick the robot up or level it by hand first."""
    if link is None:
        return _not_connected()
    if not confirmed_safe:
        return {"ok": False, "error": "confirmed_safe must be true, set only after the operator "
                "confirms the robot is upright and clear",
                "tilt_guard": link.tilt.snapshot()}
    with link.lock:
        ok, msg = link.tilt.clear()
    return {"ok": ok, "message": msg, "tilt_guard": link.tilt.snapshot()}


@mcp.tool()
def set_level_reference(confirmed_level: bool) -> dict:
    """Tell the tilt guard "this is level": record the robot's current orientation as the
    zero for tilt. It is captured automatically from the first ~0.5 s of a still, live IMU
    after connect(), so call this only if the robot was NOT on level ground then (e.g. the
    server connected while it sat on a ramp), or after the IMU board was remounted. Does not
    move the robot. confirmed_level=true only after the operator says the robot is on level
    ground. Refused if the robot is moving, or if it would move the reference by more than
    half the tilt limit from the one in use (a real mounting change is a few degrees; a
    bigger jump is the robot leaning, and this must not be a way to hide that)."""
    if link is None:
        return _not_connected()
    if not confirmed_level:
        return {"ok": False, "error": "confirmed_level must be true, set only after the operator "
                "confirms the robot is sitting on level ground"}
    snap = link.telemetry_snapshot()
    if not snap["imu_live"]:
        return {"ok": False, "error": "no live IMU", "telemetry": snap}
    with link.lock:
        t = link.tilt
        if abs(t.rate_dps) > tilt_guard.REF_MAX_RATE_DPS:
            return {"ok": False, "error": "robot is moving; hold it still first",
                    "tilt_guard": t.snapshot()}
        if t.ready and t.tilt_deg > t.limit_deg / 2.0:
            return {"ok": False, "error": "robot reads %.1f deg off the current reference, more than "
                    "half the %.0f deg limit -- that is a lean, not a mounting change. Level it "
                    "by hand, or restart the server to re-capture from scratch."
                    % (t.tilt_deg, t.limit_deg), "tilt_guard": t.snapshot()}
        t.set_reference(link.orient.pitch, link.orient.roll, source="operator")
    return {"ok": True, "tilt_guard": link.tilt.snapshot()}


@mcp.tool()
def listen() -> dict:
    """Stop and release control back to the handheld stick controller (sends 'listen',
    matching computer_bridge.ino's serial protocol). Use this when done driving so a
    human can pick up the stick without the two fighting on the same ESP-NOW channel.
    Always safe to call, no confirmation needed."""
    if link is None:
        return _not_connected()
    link.send_listen()
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def start_recording(name: Optional[str] = None) -> dict:
    """Read-only. Start recording telemetry to recordings/<name>/<name>.csv (name defaults
    to run_<timestamp>): one row per encoder/IMU sample, in the same format as
    calibration/live_view.py --csv, so analyze_csv.py reads it. Works while the stick
    controller drives -- this server sends nothing to the robot unless a drive tool is
    called. Never moves the robot, no confirmation needed. Use add_marker() during the
    recording, stop_recording() when done.

    Also saves commands.csv: every drive command the stick sent (time_s,left,right), if
    the bridge has the firmware that relays them."""
    if link is None:
        return _not_connected()
    try:
        path = link.recorder.start(name)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return {"ok": True, "csv": str(path), **link.telemetry_snapshot()}


@mcp.tool()
def add_marker(text: str) -> dict:
    """Read-only. While recording, write a MARK row (with the current distance, heading
    and orientation, and `text` in the note column) into the CSV, e.g. "start of ramp" or
    "wheel slipped". Never moves the robot."""
    if link is None:
        return _not_connected()
    with link.lock:
        t = link.recorder.mark(link.enc, link.orient, "MARK", text)
    if t is None:
        return {"ok": False, "error": "not recording; call start_recording() first"}
    return {"ok": True, "time_s": round(t, 3), "note": text}


@mcp.tool()
def stop_recording() -> dict:
    """Read-only. Stop recording and close the CSV. Returns its path, duration and how
    many samples and markers were saved. Never moves the robot."""
    if link is None:
        return _not_connected()
    summary = link.recorder.stop()
    if summary is None:
        return {"ok": False, "error": "not recording"}
    out = {"ok": True, **summary}
    if summary["commands"] == 0:
        out["note"] = ("no stick commands were captured: either the stick wasn't driven, or "
                       "the bridge still has firmware that doesn't relay them (reflash "
                       "firmware/computer_bridge/computer_bridge.ino)")
    return out


@mcp.tool()
@_tilt_guarded
def drive_distance(
    distance_inches: float,
    direction: Literal["forward", "reverse"],
    pwm: int,
    confirmed_safe: bool,
    encoder: Literal["both", "left", "right"] = "both",
    warn_lights: bool = True,
) -> dict:
    """Closed-loop straight-line move: drives at `pwm` (0-255, clamped to
    MAX_PWM_CEILING=125) until the wheel encoders report `distance_inches` traveled, then
    stops.

    `encoder` is which wheel measures the move: "both" (the default) averages them, which
    is the chassis distance; "left" or "right" measures by that wheel alone, which is the
    same distance on a straight line and the only honest answer when the other channel is
    dead. Pick one deliberately -- an average that includes a dead channel rises at half
    rate and drives twice as far as asked, which is why a selected channel that has not
    ticked within ENCODER_FIRST_TICK_S=0.6s aborts the move and says which wheel. With
    encoder="both" that check applies to each wheel, so a frozen channel now refuses
    instead of silently overshooting. Requires a live encoder feed (refuses otherwise) and confirmed_safe=true, set
    only after asking the human operator about THIS specific move, right before making
    it -- see DRIVING_POLICY.md and this file's module docstring for why that gate can't
    be skipped yet. Always stops first and settles before moving, so you never need to
    worry about reversing direction without a stop in between.

    Pick `pwm` deliberately, don't default to a "clearly visible" value: on any setup you
    haven't personally verified, DRIVING_POLICY.md's floor is 5-10% duty (13-26). There
    is no per-call distance cap: a single call will drive as far as it is asked to, so
    the stop-check-decide-move loop is the operator's to keep, not this tool's.

    direction is which way the robot travels; PWM sign is handled internally
    (FORWARD_PWM_SIGN in this file) but hasn't been verified against a live test yet --
    the first real call on new hardware should be a small one specifically to check the
    robot actually moves the way it was asked.

    By default (warn_lights=True) the WLED strip does a 1.5 s warning pattern (on, off, on,
    off, on, solid green at 50%) right before the motors start, then is restored when the
    move ends; pass warn_lights=False to skip it. A strip that can't be reached never
    blocks the move.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if distance_inches <= 0:
        return {"ok": False, "error": "distance_inches must be positive"}
    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    if encoder not in ENCODER_SELECTIONS:
        return {"ok": False, "error": "encoder must be one of %s"
                % ", ".join(ENCODER_SELECTIONS)}

    snap = link.telemetry_snapshot()
    start = link.wheel_counts(encoder)
    # Closed loop needs whichever channels `encoder` named to be reporting. Anything less
    # and this becomes a timed move, which the result says plainly.
    closed_loop = snap["telemetry_live"] and start is not None
    predicted_speed = open_loop_s = None
    if not closed_loop:
        predicted_speed = speed_for_pwm(pwm)
        if predicted_speed is None:
            return {"ok": False, "error":
                    "no encoder feedback, and pwm %d is below the measured deadband of %d -- "
                    "the robot would not move and nothing here could tell. Raise pwm, or "
                    "bring the encoders back." % (pwm, PWM_DEADBAND),
                    "telemetry": snap}
        open_loop_s = distance_inches / predicted_speed

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = link.in_per_tick()
    signed_pwm = pwm * (FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN)
    timeout_s = max(4.0, distance_inches * 0.5 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    try:
        link.send_drive(signed_pwm, signed_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                link.send_drive(signed_pwm, signed_pwm)
                last_send = time.monotonic()
            if closed_loop:
                cur = link.wheel_counts(encoder)
                if cur is None:
                    # An encoder dropped out with the wheels turning. Stop rather than
                    # carry on blind towards a target that can no longer be measured.
                    abort_reason = ("an encoder stopped reporting mid-move -- distance can "
                                    "no longer be measured, so the move was cut short")
                    break
                if traveled_inches(start, cur, encoder, in_per_tick) >= distance_inches:
                    break
                frozen = frozen_wheel_fault(start, cur, encoder, in_per_tick,
                                            time.monotonic() - t_start)
                if frozen:
                    # Only suggest the other wheel when there IS one still reporting --
                    # advising encoder="left" when the left channel is the dead one would
                    # send the next call straight back into this fault.
                    other = {"left": "right", "right": "left"}
                    hint = ((" Or pass encoder=\"%s\" to measure by the wheel that is "
                             "reporting." % other[frozen[0]]) if len(frozen) == 1 else "")
                    abort_reason = (
                        "the %s encoder has not ticked once while driving: it is dead, or "
                        "that wheel is held. Either way this move cannot be measured by "
                        "it, so it was cut short rather than driven to a target nothing "
                        "was measuring. Check the encoder (magnet gap, wiring).%s"
                        % (" and ".join(frozen), hint))
                    break
            elif time.monotonic() - t_start >= open_loop_s:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            time.sleep(0.03)
    finally:
        link.send_stop()
        _lights_restore(prior_lights)
        time.sleep(STOP_SETTLE_S)

    elapsed = time.monotonic() - t_start
    end = link.wheel_counts(encoder)
    measured = start is not None and end is not None
    actual_inches = (traveled_inches(start, end, encoder, in_per_tick)
                     if measured else None)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_inches": distance_inches,
        "direction": direction,
        "pwm_used": pwm,
        "closed_loop": closed_loop,
        "actual_inches_traveled": None if actual_inches is None else round(actual_inches, 2),
        "estimated_inches_traveled": (None if closed_loop or predicted_speed is None
                                      else round(elapsed * predicted_speed, 2)),
        "measured_by": encoder,
        "distance_source": (("both wheel encoders, averaged" if encoder == "both"
                             else "the %s wheel encoder only" % encoder) if measured else
                            "PREDICTED from the measured PWM-speed curve, not measured -- "
                            "no encoder feedback on this move"),
        "elapsed_s": round(elapsed, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def turn_degrees(
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
    warn_lights: bool = True,
) -> dict:
    """Closed-loop spin-in-place: drives the wheels in opposite directions at `pwm`
    (0-255, clamped to MAX_PWM_CEILING=125) until the IMU's gyro-integrated yaw reports
    `degrees` turned, then stops. Same confirmed_safe gate and stop-before-moving
    behavior as drive_distance -- see its docstring and DRIVING_POLICY.md.

    The IMU is the ONLY thing this tool will turn by, and it refuses outright without a
    live one -- there is deliberately no encoder fallback. Verified 2026-09-27 (robot on
    the ground, four 90deg turns each way): the encoder-differential estimate (dr-dl over
    track_width_in) read ~92 deg per commanded 90deg turn while the robot visually rotated
    only ~47 deg -- wheel scrub during an in-place spin makes the wheels turn much more
    than the chassis does. A 2026-09-29 re-run on the same floor measured ~1.7x rather
    than ~2x, which is the point: the error moves with the surface, so no fixed correction
    recovers it. The IMU yaw delta over those turns averaged ~49 deg, matching the ~47 deg
    observed by eye. On this robot the encoders measure arcs and straight lines; they do
    not measure spins. The response still includes encoder_degrees_estimate for reference,
    explicitly labeled unreliable, but nothing reads it.

    There is no per-call cap on how far it will turn. direction "left" is
    counter-clockwise viewed from above, matching bridge/'s heading
    convention.

    By default (warn_lights=True) the WLED strip does a 1.5 s warning pattern (on, off, on,
    off, on, solid green at 50%) right before the motors start, then is restored when the
    move ends; pass warn_lights=False to skip it. A strip that can't be reached never
    blocks the move.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if degrees <= 0:
        return {"ok": False, "error": "degrees must be positive"}
    snap = link.telemetry_snapshot()
    start = link.last_counts()
    # The IMU is the only thing allowed to measure a spin in place. The encoders are not a
    # fallback for this: wheel scrub makes them over-read an in-place turn by ~2x, and by a
    # ratio that moves with the surface, so there is no fixed divisor that fixes it.
    if not snap["imu_live"]:
        return {
            "ok": False,
            "error": "no live IMU, so nothing can measure this turn. The wheel encoders are "
            "not a substitute for a spin in place -- scrub makes them over-read it by ~2x, "
            "by a factor that changes with the surface. Bring the IMU back, or use "
            "jog(left_pwm, right_pwm, duration_s) for a timed turn you watch yourself.",
            "telemetry": snap,
        }
    feedback_source = "imu"

    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    start = link.last_counts()
    in_per_tick = link.in_per_tick()
    track_width = link.cfg["track_width_in"]
    with link.lock:
        start_yaw = link.orient.yaw

    # "left" (CCW, nose swings left) needs the right wheel driving real-forward and the
    # left wheel driving real-backward -- both relative to FORWARD_PWM_SIGN, not raw PWM
    # sign, the same correction drive_distance applies.
    sign = 1 if direction == "left" else -1
    left_pwm = -sign * FORWARD_PWM_SIGN * pwm
    right_pwm = sign * FORWARD_PWM_SIGN * pwm
    timeout_s = max(4.0, degrees * 0.1 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    try:
        link.send_drive(left_pwm, right_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                link.send_drive(left_pwm, right_pwm)
                last_send = time.monotonic()
            with link.lock:
                current_yaw = link.orient.yaw
            turned_so_far = abs(current_yaw - start_yaw)
            if turned_so_far >= degrees:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            time.sleep(0.03)
    finally:
        link.send_stop()
        _lights_restore(prior_lights)
        time.sleep(STOP_SETTLE_S)

    end = link.last_counts()
    with link.lock:
        end_yaw = link.orient.yaw
    if end is not None and start is not None:
        dl = (end[0] - start[0]) * in_per_tick
        dr = (end[1] - start[1]) * in_per_tick
        enc_deg = math.degrees((dr - dl) / track_width)
    else:
        dl = dr = enc_deg = None

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_degrees": degrees,
        "direction": direction,
        "pwm_used": pwm,
        "feedback_source": feedback_source,
        "actual_degrees_turned": round(end_yaw - start_yaw, 1),
        "measurement_note": "IMU gyro-integrated yaw -- the only source this tool will "
                            "turn by; the encoder figure below is reported for reference "
                            "and never steers the move",
        "imu_degrees": round(end_yaw - start_yaw, 1),
        "encoder_degrees_estimate": None if enc_deg is None else round(enc_deg, 1),
        "encoder_estimate_unreliable": "wheel scrub during in-place turns inflates this "
        "~2x vs. the IMU/real rotation -- see this tool's docstring",
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "track_width_calibrated": link.track_width_calibrated,
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def drive_arc(
    radius_inches: float,
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
    max_wheel_pwm: int = MAX_PWM_CEILING,
    travel: Literal["forward", "reverse"] = "forward",
    encoder: Literal["both", "left", "right"] = "both",
    warn_lights: bool = True,
) -> dict:
    """Closed-loop curved drive: both wheels roll forward (or backward, see `travel`), the
    outer one faster, so the robot drives an arc of `radius_inches` (measured to the robot's center) until the
    IMU's gyro-integrated yaw reports `degrees` turned, then stops. degrees=360 drives a
    full circle. Same confirmed_safe gate, live-telemetry requirement and
    stop-before-moving behavior as drive_distance -- see DRIVING_POLICY.md.

    `pwm` is the average wheel command; the outer wheel runs at pwm + d (never above
    MAX_PWM_CEILING) and the inner at pwm - d. d starts from the nominal track-width
    geometry, capped so the inner wheel starts at or above zero, then every
    ARC_CONTROL_DT_S the measured curvature (IMU yaw change / signed encoder path length)
    is compared with 1/radius and d is nudged to match, by at most ARC_MAX_STEP_PWM per
    step -- skid-steer wheel scrub makes the geometric split alone wrong, so the loop,
    not the geometry, sets the real radius. For radii tighter than the robot manages with
    the inner wheel stopped (~12.5in), the loop eases the inner wheel through zero into
    reverse, down to -ARC_MAX_INNER_REVERSE_PWM. Aborts immediately if encoder or IMU
    telemetry goes stale mid-move.

    direction uses the same wheel mapping as turn_degrees. Capped at
    MAX_SINGLE_ARC_DEG=360 and MAX_SINGLE_ARC_IN=160 of path per call, radius at least
    MIN_ARC_RADIUS_IN=6.

    `max_wheel_pwm` (default MAX_PWM_CEILING) caps the OUTER wheel's PWM for the whole
    move, including anything the curvature loop asks for -- use it to honor an operator's
    "never above N PWM" limit, since `pwm` is only the average and the outer wheel runs
    above it. The average is reduced if needed so the geometric starting split fits
    under the cap.

    `encoder` is which wheel the curvature loop measures its path with, as in
    drive_distance. One wheel is enough here, but it is not on the arc you asked for: the
    outer wheel runs a radius half a track wider than the robot's centre and the inner one
    half a track tighter, so the loop compares the measured curvature against 1/(radius +-
    half track) for that wheel instead of 1/radius, and the reported path is scaled back to
    the centre. That needs the selected wheel's own radius to be a real arc, so a single
    encoder is refused when the inner wheel would be pivoting inside MIN_ARC_RADIUS_IN.

    `travel="reverse"` drives the same arc backing up: both wheels roll backward and the
    robot's heading still turns `direction` (left = counter-clockwise seen from above), so
    the outer wheel is the LEFT one for "left" (the opposite of a forward arc) and the
    rear sweeps out along the path. Nothing on the robot senses what is behind it, so only
    back up into space the operator has said is clear. Forward then reverse arcs of the same
    direction trace an S-curve: the robot ends facing the opposite way, displaced along its
    original axis by twice the radius and back on the same line laterally.

    By default (warn_lights=True) the WLED strip does a 1.5 s warning pattern (on, off, on,
    off, on, solid green at 50%) right before the motors start, then is restored when the
    move ends; pass warn_lights=False to skip it. A strip that can't be reached never
    blocks the move.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if radius_inches < MIN_ARC_RADIUS_IN:
        return {"ok": False, "error": f"radius_inches must be >= {MIN_ARC_RADIUS_IN} -- "
                "use turn_degrees to spin in place"}
    if not (0 < degrees <= MAX_SINGLE_ARC_DEG):
        return {"ok": False, "error": f"degrees must be > 0 and <= {MAX_SINGLE_ARC_DEG}"}
    arc_len = radius_inches * math.radians(degrees)
    if arc_len > MAX_SINGLE_ARC_IN:
        return {"ok": False, "error": f"arc length {arc_len:.1f}in exceeds the "
                f"{MAX_SINGLE_ARC_IN}in per-call cap -- use a smaller radius or fewer degrees"}

    snap = link.telemetry_snapshot()
    if not snap["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry -- drive_arc steers by "
                "comparing wheel path against yaw, and without the encoders there is no "
                "path to compare. Use jog(left_pwm, right_pwm, duration_s) for a timed "
                "curve you watch yourself.", "telemetry": snap}
    if not snap["imu_live"]:
        # Not a degraded mode: the whole tool is "curve until the yaw says `degrees`".
        # Without yaw there is no target to drive to and no curvature loop to run, and a
        # fixed wheel split for a fixed time is exactly what jog already does.
        return {"ok": False, "error": "no live IMU -- drive_arc has no way to tell how far "
                "around the arc it has gone, and its curvature loop needs yaw to correct "
                "for wheel scrub. Use jog(left_pwm, right_pwm, duration_s) for a timed "
                "curve, or turn_degrees for a spin in place.", "telemetry": snap}

    wheel_cap = min(abs(int(max_wheel_pwm)), MAX_PWM_CEILING)
    pwm = min(abs(int(pwm)), wheel_cap)
    if pwm == 0:
        return {"ok": False, "error": "pwm and max_wheel_pwm must be > 0"}
    if encoder not in ENCODER_SELECTIONS:
        return {"ok": False, "error": "encoder must be one of %s"
                % ", ".join(ENCODER_SELECTIONS)}

    half_track = link.cfg["track_width_in"] / 2.0
    travel_sign = 1 if travel == "forward" else -1
    # Forward: "left" = right wheel on the outside, matching turn_degrees' mapping.
    # Reverse: heading turns the same way, so the outside wheel swaps sides.
    outer_is_right = (direction == "left") == (travel == "forward")

    # The wheel doing the measuring runs its own arc, concentric with the robot's but
    # wider (outer) or tighter (inner) by half a track. Measuring one wheel against
    # 1/radius would make the loop chase a curvature that wheel never drives.
    if encoder == "both":
        sel_radius = radius_inches
    else:
        sel_is_outer = (encoder == "right") == outer_is_right
        sel_radius = radius_inches + (half_track if sel_is_outer else -half_track)
        if sel_radius < MIN_ARC_RADIUS_IN:
            return {"ok": False, "error":
                    "radius_inches %.1f puts the %s wheel on a %.1fin arc, inside the "
                    "%.0fin minimum -- that wheel is pivoting, not tracing the curve, so "
                    "its path cannot measure one. Use encoder=\"%s\" (the outer wheel), "
                    "a wider radius, or encoder=\"both\"."
                    % (radius_inches, encoder, sel_radius, MIN_ARC_RADIUS_IN,
                       "right" if encoder == "left" else "left"),
                    "telemetry": snap}

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = link.in_per_tick()
    diff = pwm * min(1.0, half_track / radius_inches)
    target_curv = 1.0 / sel_radius  # rad per inch of the measuring wheel's path

    def forward_path_in(a, b):
        # Signed along the direction of travel, so a wheel running against it (the inner
        # wheel of a tight arc) subtracts from the path instead of adding.
        idx = selected_wheels(encoder)
        fwd = [(b[i] - a[i]) * ENCODER_FORWARD_SIGN[i] for i in idx]
        return travel_sign * sum(fwd) / len(fwd) * in_per_tick

    # Path of the measuring wheel, scaled to the robot's centre for reporting.
    center_scale = radius_inches / sel_radius

    max_diff = pwm + ARC_MAX_INNER_REVERSE_PWM

    def wheel_cmd(d):
        outer = min(wheel_cap, pwm + d)
        inner = max(-ARC_MAX_INNER_REVERSE_PWM, pwm - d)
        l_real, r_real = (inner, outer) if outer_is_right else (outer, inner)
        return (FORWARD_PWM_SIGN * travel_sign * l_real,
                FORWARD_PWM_SIGN * travel_sign * r_real)

    start = link.wheel_counts(encoder)
    if start is None:
        # telemetry_live only says a packet arrived; the channel this move measures by can
        # still be unavailable.
        return {"ok": False, "error":
                ("the %s encoder is not reporting -- drive_arc measures its path with it. "
                 "Check encoders_available in read_telemetry(), or pass encoder=\"left\" "
                 "/ \"right\" to measure with the wheel that is."
                 % ("left or right" if encoder == "both" else encoder)),
                "telemetry": snap}
    with link.lock:
        start_yaw = link.orient.yaw
    timeout_s = arc_len / 2.0 + 5.0  # generous: ~2 in/s average, well under PWM-50 speed

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    win_counts, win_yaw = start, start_yaw
    last_ctrl = t_start
    try:
        link.send_drive(*wheel_cmd(diff))
        while True:
            now = time.monotonic()
            with link.lock:
                cur_yaw = link.orient.yaw
                enc_age = None if link.last_enc_wall is None else now - link.last_enc_wall
                imu_age = None if link.last_imu_wall is None else now - link.last_imu_wall
            if abs(cur_yaw - start_yaw) >= degrees:
                break
            if enc_age is None or enc_age > TELEMETRY_STALE_S or imu_age is None \
                    or imu_age > TELEMETRY_STALE_S:
                abort_reason = "telemetry went stale mid-arc"
                break
            if now - t_start > timeout_s:
                hit_timeout = True
                break
            if now - last_ctrl >= ARC_CONTROL_DT_S:
                cur = link.wheel_counts(encoder)
                if cur is None:
                    abort_reason = "an encoder stopped reporting mid-arc"
                    break
                path = forward_path_in(win_counts, cur)
                if path > 0.3:  # enough travel for a meaningful curvature estimate
                    curv = math.radians(abs(cur_yaw - win_yaw)) / path
                    step = ARC_GAIN * pwm * (target_curv - curv) / target_curv
                    step = max(-ARC_MAX_STEP_PWM, min(ARC_MAX_STEP_PWM, step))
                    diff = max(0.0, min(float(max_diff), diff + step))
                    win_counts, win_yaw = cur, cur_yaw
                link.send_drive(*wheel_cmd(diff))  # also the bridge keepalive
                last_ctrl = now
            time.sleep(0.03)
    finally:
        link.send_stop()
        _lights_restore(prior_lights)
        time.sleep(STOP_SETTLE_S)

    end = link.wheel_counts(encoder)
    with link.lock:
        end_yaw = link.orient.yaw
    wheel_path_in = None if (start is None or end is None) else forward_path_in(start, end)
    path_in = None if wheel_path_in is None else wheel_path_in * center_scale
    turned = end_yaw - start_yaw
    eff_radius = (path_in / math.radians(abs(turned))
                  if path_in is not None and abs(turned) > 1 else None)
    final_l, final_r = wheel_cmd(diff)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_radius_in": radius_inches,
        "requested_degrees": degrees,
        "direction": direction,
        "travel": travel,
        "pwm_used": pwm,
        "actual_degrees_turned": round(turned, 1),
        "path_length_in": None if path_in is None else round(path_in, 2),
        "measured_by": encoder,
        "measuring_wheel_radius_in": round(sel_radius, 2),
        "effective_radius_in": None if eff_radius is None else round(eff_radius, 2),
        "final_wheel_pwm": {"left": round(final_l), "right": round(final_r)},
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def drive_until_range(target_mm: float, pwm: int, confirmed_safe: bool,
                      facing: Literal["auto", "forward", "reverse"] = "auto",
                      sensor: Optional[str] = None, tolerance_mm: float = 15.0,
                      min_pwm: int = PWM_DEADBAND + 5, warn_lights: bool = True) -> dict:
    """Closed-loop approach: drive straight toward whatever a ToF sensor is looking at
    until its median range reads `target_mm`, then stop. The loop closes on the range
    sensor, not the encoders, so it works with the wheel encoders dead.

    It drives in the direction the sensor FACES, because that is the only direction it can
    measure. With facing="auto" that is read off the sensor's name -- TOF_REAR means the
    robot backs up, anything else means it goes forward -- so a board mounted on the back
    but flashed as "TOF_FRONT" will send the robot the wrong way. Pass
    facing="forward"/"reverse" to say which way the board really points; the result always
    reports what it decided and why.

    Two phases, because the sensor is slow relative to the robot:

      1. Continuous, tapering. The number the loop steers by is a median over
         TofState.HISTORY samples at ~5 Hz, so it lags; RANGE_LATENCY_S folds that lag,
         one sample interval and the braking lead together, and the taper never commands
         more speed than stops inside the gap that is left (see approach_pwm).
      2. Pulses, once even min_pwm is too fast to stop in time. Short bursts at min_pwm,
         each followed by a full stop and RANGE_MEDIAN_REFRESH_S of new samples so the
         median is actually about where the robot now is. This is the stop-check-decide
         loop from DRIVING_POLICY.md section 5, which is what the last few centimetres
         need: the shortest burst this drivetrain can make still moves it ~20mm.

    It stops short rather than overshoot. `tolerance_mm` (default 15) is close enough, and
    if the shortest possible pulse would carry it past the target it stops where it is and
    says so in `stopped_short`. Two narrow cones at a fixed height are not clearance --
    they cannot see the floor, a drop, the sides, or anything below the beam -- so
    confirmed_safe still means the operator looked.

    Aborts, and says which: the sensor stops reporting, its median goes out of range (the
    target was lost -- that is blind, not clear), nothing gets closer for a few seconds
    (something is in the way, or the beam is skimming a surface at too shallow an angle),
    or RANGE_MAX_PULSES is reached.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if target_mm < RANGE_MIN_TARGET_MM:
        return {"ok": False, "error":
                "target_mm must be at least %.0f -- closer than that is inside the robot's "
                "own bumper on most mountings, and VL53L0X accuracy falls apart in the "
                "last few centimetres." % RANGE_MIN_TARGET_MM}
    if tolerance_mm < 5:
        return {"ok": False, "error": "tolerance_mm must be at least 5 -- the channels "
                "routinely disagree by more than that about where a surface is"}
    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    min_pwm = min(abs(int(min_pwm)), MAX_PWM_CEILING)
    if min_pwm < PWM_DEADBAND:
        return {"ok": False, "error":
                "min_pwm %d is below the measured deadband of %d, where the robot does not "
                "move at all -- the approach would stall out short of the target and look "
                "like an obstruction." % (min_pwm, PWM_DEADBAND)}
    if pwm < min_pwm:
        return {"ok": False, "error": "pwm must be >= min_pwm"}

    rng = link.range_snapshot()
    if rng is None:
        return {"ok": False, "error": "no range sensor has ever reported -- there is "
                "nothing to close the loop on. sensors() shows what is transmitting.",
                "telemetry": link.telemetry_snapshot()}

    names = sorted({c["sensor"] for c in rng["channels"].values()})
    if sensor is not None:
        if sensor not in names:
            return {"ok": False, "error": "no range sensor named %r is reporting (have: %s)"
                    % (sensor, ", ".join(names)), "range": rng}
        sensor_name = sensor
    elif len(names) == 1:
        sensor_name = names[0]
    else:
        return {"ok": False, "error":
                "several range sensors are reporting (%s) and they point different ways -- "
                "pass sensor= to say which one this move is closing on." % ", ".join(names),
                "range": rng}

    if facing == "auto":
        direction = "reverse" if sensor_name.upper().endswith("REAR") else "forward"
        facing_source = ("the sensor's name (%s), not a measurement -- a board mounted the "
                         "other way round will drive the robot the wrong way"
                         % sensor_name)
    else:
        direction = facing
        facing_source = "the caller said so"

    def read_gap():
        """(closest median mm on this sensor, is it live) -- the number to steer by."""
        snap = link.range_snapshot()
        chans = [c for c in snap["channels"].values() if c["sensor"] == sensor_name]
        medians = [c["mm"] for c in chans if c["mm"] is not None]
        return (min(medians) if medians else None), any(c["live"] for c in chans)

    gap, live = read_gap()
    if not live:
        return {"ok": False, "error": "%s is not reporting right now -- that is blind, not "
                "clear." % sensor_name, "range": rng}
    if gap is None:
        return {"ok": False, "error":
                "%s is reporting but has nothing in range, so there is nothing to approach. "
                "Point the robot at the target first (this is 'nothing within reach', not "
                "'the way is clear')." % sensor_name, "range": rng}
    if gap - target_mm <= tolerance_mm:
        return {"ok": True, "moved": False, "target_mm": target_mm, "final_mm": gap,
                "final_in": round(gap / bridge.MM_PER_IN, 1),
                "note": "already within tolerance of the target -- nothing to do",
                "range": link.range_snapshot(), "telemetry": link.telemetry_snapshot()}

    sign = FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN
    start_gap = gap
    # Generous: the continuous phase at min_pwm plus every pulse and its settle.
    timeout_s = ((start_gap - target_mm) / bridge.MM_PER_IN / speed_for_pwm(min_pwm)
                 + RANGE_MAX_PULSES * (RANGE_MAX_PULSE_S + RANGE_MEDIAN_REFRESH_S) + 5.0)

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    start_counts = link.wheel_counts("both") or link.wheel_counts("left") \
        or link.wheel_counts("right")
    in_per_tick = link.in_per_tick()
    t_start = time.monotonic()
    abort_reason = None
    hit_timeout = stopped_short = False
    pulses = 0
    best_gap = gap
    best_t = t_start
    pwm_log = []

    try:
        # --- phase 1: continuous, tapering as the gap closes ---------------------------
        while True:
            now = time.monotonic()
            gap, live = read_gap()
            if not live:
                abort_reason = "%s stopped reporting mid-approach" % sensor_name
                break
            if gap is None:
                abort_reason = ("%s lost the target mid-approach -- its median went out of "
                                "range, which is blind, not clear" % sensor_name)
                break
            if gap - target_mm <= tolerance_mm:
                break
            if now - t_start > timeout_s:
                hit_timeout = True
                break
            if gap < best_gap - 2:
                best_gap, best_t = gap, now
            elif now - best_t > RANGE_NO_PROGRESS_S:
                abort_reason = ("nothing got closer for %.1fs while driving: something is "
                                "in the way, a wheel is not turning, or the beam is "
                                "skimming the surface too obliquely to read"
                                % RANGE_NO_PROGRESS_S)
                break
            want = approach_pwm(gap - target_mm, pwm, min_pwm)
            if want is None:
                break  # too close to drive continuously; finish in pulses
            pwm_log.append(want)
            link.send_drive(sign * want, sign * want)
            time.sleep(DRIVE_KEEPALIVE_S)
        link.send_stop()
        time.sleep(STOP_SETTLE_S)

        # --- phase 2: pulse, stop, let the median catch up, look again -----------------
        while abort_reason is None and not hit_timeout:
            time.sleep(RANGE_MEDIAN_REFRESH_S)
            gap, live = read_gap()
            if not live:
                abort_reason = "%s stopped reporting between pulses" % sensor_name
                break
            if gap is None:
                abort_reason = ("%s lost the target between pulses -- its median went out "
                                "of range" % sensor_name)
                break
            remaining = gap - target_mm
            if remaining <= tolerance_mm:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            if pulses >= RANGE_MAX_PULSES:
                abort_reason = ("still %.0fmm short of the target after %d pulses -- "
                                "stopping rather than nibbling at it indefinitely"
                                % (remaining, pulses))
                break
            creep_in_s = speed_for_pwm(min_pwm)
            burst_s = (remaining / bridge.MM_PER_IN) / creep_in_s
            if burst_s < RANGE_MIN_PULSE_S:
                # The shortest burst this drivetrain makes would carry it past the target.
                # Stopping short is the right error to make.
                stopped_short = True
                break
            pulses += 1
            pwm_log.append(min_pwm)
            burst_s = min(RANGE_MAX_PULSE_S, burst_s)
            burst_start = time.monotonic()
            while time.monotonic() - burst_start < burst_s:
                link.send_drive(sign * min_pwm, sign * min_pwm)
                time.sleep(min(DRIVE_KEEPALIVE_S,
                               max(0.0, burst_s - (time.monotonic() - burst_start))))
            link.send_stop()
            time.sleep(STOP_SETTLE_S)
    finally:
        link.send_stop()
        _lights_restore(prior_lights)
        time.sleep(STOP_SETTLE_S)

    time.sleep(RANGE_MEDIAN_REFRESH_S)  # so final_mm is a settled median, not a lagging one
    final_gap, _ = read_gap()
    end_counts = link.wheel_counts("both") or link.wheel_counts("left") \
        or link.wheel_counts("right")
    traveled = None
    if start_counts is not None and end_counts is not None:
        moved = [abs(e - s) for s, e in zip(start_counts, end_counts)
                 if s is not None and e is not None]
        if moved:
            traveled = round(max(moved) * in_per_tick, 2)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "moved": True,
        "target_mm": target_mm,
        "tolerance_mm": tolerance_mm,
        "sensor": sensor_name,
        "direction": direction,
        "facing_from": facing_source,
        "start_mm": start_gap,
        "final_mm": final_gap,
        "final_in": None if final_gap is None else round(final_gap / bridge.MM_PER_IN, 1),
        "within_tolerance": (final_gap is not None
                             and final_gap - target_mm <= tolerance_mm),
        "stopped_short": stopped_short,
        "stopped_short_why": ("the shortest burst this drivetrain can make (%.2fs at pwm "
                              "%d) would have carried it past the target"
                              % (RANGE_MIN_PULSE_S, min_pwm)) if stopped_short else None,
        "pulses": pulses,
        "pwm_used": {"max": max(pwm_log) if pwm_log else None,
                     "min": min(pwm_log) if pwm_log else None},
        "wheel_travel_in": traveled,
        "range_closed_mm": None if final_gap is None else round(start_gap - final_gap, 1),
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "range": link.range_snapshot(),
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def jog(left_pwm: int, right_pwm: int, duration_s: float, confirmed_safe: bool,
        warn_lights: bool = True, require_telemetry: Optional[bool] = None) -> dict:
    """Raw open-loop command: drive at exactly (left_pwm, right_pwm) for duration_s
    seconds, then stop. No encoder target and no direction-aware sign correction (unlike
    drive_distance/turn_degrees) -- positive does not necessarily mean forward, see
    FORWARD_PWM_SIGN. This is for bench/commissioning tests where a fixed, observable
    duration matters more than hitting a distance or angle (checking direction, deadband,
    turn convention by eye). Same confirmed_safe gate as the other drive tools. Capped at
    MAX_PWM_CEILING=125 per side; duration_s has no upper bound, so an open-loop call runs
    for exactly as long as it is given.

    By default (warn_lights=True) the WLED strip does a 1.5 s warning pattern (on, off, on,
    off, on, solid green at 50%) right before the motors start, then is restored when the
    move ends; pass warn_lights=False to skip it. A strip that can't be reached never
    blocks the move.

    require_telemetry defaults to the server's sensor policy (set_required_sensors): it
    insists on live encoders only if "enc" is in the required set, which it is by default.
    Pass True or False to override for one call -- False ONLY for a bench test the operator
    is watching (e.g. robot on blocks while the encoder link is down): it still needs
    confirmed_safe, and the distances come back as null.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if duration_s <= 0:
        return {"ok": False, "error": "duration_s must be > 0"}

    snap = link.telemetry_snapshot()
    if require_telemetry is None:
        require_telemetry = "enc" in link.policy.required
    if require_telemetry and not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback. "
                     "Pass require_telemetry=false for a watched bench test, or call "
                     "set_required_sensors() to drop \"enc\" for this run.",
            "telemetry": snap,
        }

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    start = link.last_counts()
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < duration_s:
            link.send_drive(left_pwm, right_pwm)
            time.sleep(min(DRIVE_KEEPALIVE_S, max(0.0, duration_s - (time.monotonic() - t_start))))
    finally:
        link.send_stop()
        _lights_restore(prior_lights)
        time.sleep(STOP_SETTLE_S)

    end = link.last_counts()
    in_per_tick = link.in_per_tick()
    have_counts = start is not None and end is not None
    return {
        "ok": True,
        "left_pwm": left_pwm,
        "right_pwm": right_pwm,
        "duration_s": duration_s,
        "dist_l_in": round((end[0] - start[0]) * in_per_tick, 2) if have_counts else None,
        "dist_r_in": round((end[1] - start[1]) * in_per_tick, 2) if have_counts else None,
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def speak_then_drive(text: str, pwm: int, duration_s: float, confirmed_safe: bool,
                     direction: str = "forward", speed: float = 1.3,
                     warn_lights: bool = True, require_telemetry: Optional[bool] = None) -> dict:
    """speak(text), then the instant the last word and the closed-mouth settle are done,
    drive straight (open-loop, like jog) at `pwm` for duration_s -- one call, so there is no
    round trip to the model between the voice and the wheels. Both wheels get the same PWM
    (FORWARD_PWM_SIGN is applied for you: direction is forward or reverse), so a wheel or
    encoder fault will curve the path. Same confirmed_safe gate as the other drive tools
    (checked BEFORE speaking, so a refused move says nothing). warn_lights=True (default)
    keeps the pre-move blink (1.25 s, fading on off on off on, solid green while moving) between
    the speech and the motors, sent over the realtime channel so it shows right after the
    mouth closes; False starts the motors straight away. Capped at MAX_PWM_CEILING; duration_s has no upper bound."""
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if direction not in ("forward", "reverse"):
        return {"ok": False, "error": "direction must be 'forward' or 'reverse'"}
    if not (0 < pwm <= MAX_PWM_CEILING):
        return {"ok": False, "error": f"pwm must be 1-{MAX_PWM_CEILING}"}
    if duration_s <= 0:
        return {"ok": False, "error": "duration_s must be > 0"}
    if require_telemetry is None:
        require_telemetry = "enc" in link.policy.required
    if require_telemetry and not link.telemetry_snapshot()["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry -- refusing to drive without "
                "feedback. Pass require_telemetry=false for a watched bench test, or call "
                "set_required_sensors() to drop \"enc\" for this run."}
    said = speak(text, speed=speed)  # ends on the closed mouth, ~0.4 s after the last word
    if not said.get("ok"):
        return {"ok": False, "error": f"speech failed, did not drive: {said.get('error')}"}
    signed = pwm * (FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN)

    # speech -> closed mouth -> blink -> drive, back to back. The strip is still in WLED
    # realtime mode from the mouth (HTTP state changes are ignored until it times out), so the
    # blink and the solid-green "moving" light go out over the same UDP channel as the mouth.
    # Keep the mouth's pending restore from firing mid-drive; release it when the move ends.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def paint(level):
        v = int(255 * max(0.0, min(1.0, level)))
        try:
            sock.sendto(bytes([2, 2]) + bytes([0, v, 0]) * _Mouth.N, (WLED_HOST, 21324))
        except OSError:
            pass

    if warn_lights:
        _mouth.acquire(127)  # cancels the restore timer speak() left; keeps the pre-speech state
        phase_s, level = 0.25, 0.0
        for target in (1, 0, 1, 0, 1):  # on off on off on, fading, ends solid green
            t_phase = time.monotonic()
            while (f := (time.monotonic() - t_phase) / phase_s) < 1.0:
                paint(level + (target - level) * f)
                time.sleep(1 / 30)
            level = target
        paint(1.0)
    link.send_stop()
    start = link.last_counts()
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < duration_s:
            link.send_drive(signed, signed)
            if warn_lights:
                paint(1.0)
            time.sleep(min(DRIVE_KEEPALIVE_S, max(0.0, duration_s - (time.monotonic() - t_start))))
    finally:
        link.send_stop()
        if warn_lights:
            _mouth.release(2.6)
        time.sleep(STOP_SETTLE_S)
    end = link.last_counts()
    in_per_tick = link.in_per_tick()
    have_counts = start is not None and end is not None
    return {
        "ok": True, "left_pwm": signed, "right_pwm": signed, "duration_s": duration_s,
        "dist_l_in": round((end[0] - start[0]) * in_per_tick, 2) if have_counts else None,
        "dist_r_in": round((end[1] - start[1]) * in_per_tick, 2) if have_counts else None,
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "telemetry": link.telemetry_snapshot(),
        "spoke_seconds": said.get("seconds"), "direction": direction, "pwm": pwm,
    }


WLED_HOST = os.environ.get("ELLAMA_WLED_HOST", "192.168.0.57")


def _wled(path, body=None):
    req = urllib.request.Request(
        f"http://{WLED_HOST}{path}",
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=4) as r:
        return json.load(r)


def _wled_snapshot():
    """The strip's current on/bri/effect/speed/intensity/color, in a form _wled() can re-post."""
    st = _wled("/json/state")
    sg = st["seg"][0]
    return {"on": st["on"], "bri": st["bri"], "tt": 0,
            "seg": [{"fx": sg["fx"], "sx": sg["sx"], "ix": sg["ix"], "col": sg["col"]}]}


def _lights_restore(prior):
    if prior:
        try:
            _wled("/json/state", prior)
        except Exception:
            pass


def _blink(blinks=2, phase_s=0.3, color=(0, 255, 0), brightness=127, fade=True):
    """on, then `blinks` x (off, on), each phase phase_s long, leaving the strip solid `color`
    when it returns (so 2 blinks at 0.3 s = on off on off on = 1.5 s). With fade=True each
    phase is a WLED transition of the same length (tt is in 100 ms units), so it ramps up and
    down instead of switching; fade=False switches instantly. Phases are scheduled from one
    start time so the HTTP round-trips don't stretch the pattern."""
    tt = max(1, round(phase_s * 10)) if fade else 0
    on = {"on": True, "bri": max(0, min(255, brightness)), "tt": tt,
          "seg": [{"fx": 0, "col": [list(color)]}]}
    off = {"on": False, "tt": tt}
    t0 = time.monotonic()
    for k in range(1 + 2 * blinks):
        _wled("/json/state", on if k % 2 == 0 else off)
        time.sleep(max(0.0, t0 + (k + 1) * phase_s - time.monotonic()))


def _warn_blink(enabled):
    """Pre-move warning for the drive tools. Returns the strip's prior state for
    _lights_restore (None if skipped or the strip is unreachable -- never blocks a move)."""
    if not enabled:
        return None
    try:
        prior = _wled_snapshot()
        _blink()
        return prior
    except Exception:
        return None


@mcp.tool()
def lights_blink(blinks: int = 2, phase_s: float = 0.3, color: str = "00ff00",
                 brightness: int = 127, restore: bool = True, fade: bool = True) -> dict:
    """Blink the WLED strip: on, then `blinks` x (off, on), each phase phase_s seconds, ending
    solid on. Defaults are the pre-move warning (on off on off on = 1.5 s, green, 50%, each
    phase fading in or out; fade=False for hard switching). With restore=True the strip goes
    back to how it was afterwards. Safe, no confirmation needed."""
    blinks = max(1, min(10, blinks))
    h = color.lstrip("#")
    try:
        prior = _wled_snapshot()
        _blink(blinks, max(0.05, phase_s), tuple(int(h[i:i + 2], 16) for i in (0, 2, 4)), brightness, fade)
        if restore:
            _lights_restore(prior)
    except Exception as e:
        return {"ok": False, "error": f"WLED at {WLED_HOST} unreachable: {e}"}
    return {"ok": True, "seconds": round((1 + 2 * blinks) * max(0.05, phase_s), 2), "restored": restore}


@mcp.tool()
def lights(on: bool = True, brightness: Optional[int] = None, color: Optional[str] = None,
           effect: Optional[str] = None, speed: Optional[int] = None,
           intensity: Optional[int] = None) -> dict:
    """Control the WLED lights (default 192.168.0.57, override with ELLAMA_WLED_HOST).
    on=False turns them off. Optional: brightness 0-255, color as hex like "ff8800",
    effect by name (e.g. "Rainbow", "Breathe", "Solid"; see list_light_effects), speed and
    intensity 0-255. Safe, no confirmation needed."""
    seg = {}
    if color:
        h = color.lstrip("#")
        seg["col"] = [[int(h[i:i + 2], 16) for i in (0, 2, 4)]]
    if effect:
        names = _wled("/json/eff")
        match = [i for i, n in enumerate(names) if n.lower() == effect.lower()]
        if not match:
            return {"ok": False, "error": f"unknown effect {effect!r}; see list_light_effects"}
        seg["fx"] = match[0]
    if speed is not None:
        seg["sx"] = max(0, min(255, speed))
    if intensity is not None:
        seg["ix"] = max(0, min(255, intensity))
    body = {"on": on}
    if brightness is not None:
        body["bri"] = max(0, min(255, brightness))
    if seg:
        body["seg"] = [seg]
    try:
        _wled("/json/state", body)
        st = _wled("/json/state")
    except Exception as e:
        return {"ok": False, "error": f"WLED at {WLED_HOST} unreachable: {e}"}
    s0 = st["seg"][0]
    return {"ok": True, "on": st["on"], "brightness": st["bri"], "effect_id": s0["fx"],
            "color": s0["col"][0]}


@mcp.tool()
def list_light_effects() -> dict:
    """List the WLED effect names that lights(effect=...) accepts."""
    try:
        return {"ok": True, "effects": _wled("/json/eff")}
    except Exception as e:
        return {"ok": False, "error": f"WLED at {WLED_HOST} unreachable: {e}"}


PIPER_BIN = os.environ.get("ELLAMA_PIPER", str(Path.home() / ".local/share/piper-venv/bin/piper"))
PIPER_VOICE = os.environ.get("ELLAMA_PIPER_VOICE", str(Path.home() / "piper-voices/en_US-ryan-high.onnx"))
_speak_lock = threading.Lock()
_MOUTH_REST = [1.0, 0.8, 0.55, 0.35, 0.18, 0.07]  # closed mouth: center LED, brightness trails off


class _Mouth:
    """Expressions on the 27-LED mouth bar while the robot is NOT talking: a blue breath while
    it waits for you, a blue mouth that widens with your voice while it hears you, an amber dot
    sweeping side to side while it thinks. speak() draws its own mouth while talking. Owns the
    strip's pre-conversation state and restores it a moment after the last user."""
    N, C = 27, 13
    THINK_TIMEOUT_S = 45.0

    def __init__(self):
        self._lock = threading.RLock()
        self._thread = None
        self._stop = threading.Event()
        self._timer = None          # pending restore of the strip's prior state
        self._think_timer = None    # gives up thinking if nobody speaks
        self.prior = None
        self.mode = "waiting"
        self.level = 0.0
        self._t_mode = time.time()

    def acquire(self, bri=100):
        """Turn the strip on and remember how it was (once), cancelling any pending restore."""
        with self._lock:
            if self._timer:
                self._timer.cancel()
                self._timer = None
            try:
                if self.prior is None:
                    self.prior = _wled_snapshot()
                _wled("/json/state", {"on": True, "bri": bri, "tt": 0})
            except Exception:
                pass  # lights unreachable: carry on without a mouth

    def start(self, mode):
        with self._lock:
            self.stop_anim()
            self.level = 0.0
            self.set_mode(mode)
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def set_mode(self, mode):
        self.mode = mode
        self._t_mode = time.time()
        if mode == "thinking":
            if self._think_timer:
                self._think_timer.cancel()
            self._think_timer = threading.Timer(self.THINK_TIMEOUT_S, self.release)
            self._think_timer.daemon = True
            self._think_timer.start()

    def stop_anim(self):
        with self._lock:
            if self._think_timer:
                self._think_timer.cancel()
                self._think_timer = None
            self._stop.set()
            th, self._thread = self._thread, None
        if th and th is not threading.current_thread():
            th.join(1.0)

    def release(self, delay=0.6):
        """Stop animating and put the strip back as it was after delay seconds (long enough for
        WLED's realtime mode to time out, or another acquire() to take over)."""
        self.stop_anim()
        with self._lock:
            if self.prior is None:
                return
            if self._timer:
                self._timer.cancel()
            self._timer = threading.Timer(delay, self._restore)
            self._timer.daemon = True
            self._timer.start()

    def _restore(self):
        with self._lock:
            prior, self.prior, self._timer = self.prior, None, None
        if prior:
            try:
                _wled("/json/state", prior)
            except Exception:
                pass

    def _run(self):
        n, c = self.N, self.C
        rest = [_MOUTH_REST[abs(i - c)] if abs(i - c) < len(_MOUTH_REST) else 0.0 for i in range(n)]
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        glow, env, t0 = [0.0] * n, 0.0, time.time()
        while not self._stop.is_set():
            now = time.time()
            mode = self.mode
            if mode == "waiting":
                b = 0.30 + 0.20 * math.sin((now - t0) * 2 * math.pi / 2.4)
                glow = [r_ * b for r_ in rest]
                color = (0.1, 0.5, 1.0)
            elif mode == "hearing":
                env += (self.level - env) * 0.5
                length = env * (c + 1)
                glow = [max(min(1.0, max(0.0, length - abs(i - c))), glow[i] * 0.8, rest[i] * 0.5) for i in range(n)]
                color = (0.1, 0.5, 1.0)
            else:  # thinking
                pos = c + 11 * math.sin((now - self._t_mode) * 2.2)
                glow = [max(max(0.0, 1.0 - abs(i - pos) / 2.0), glow[i] * 0.88, rest[i] * 0.25) for i in range(n)]
                color = (1.0, 0.55, 0.05)
            pkt = bytearray([2, 2])
            for g in glow:
                v = 255 * g ** 2.2
                pkt += bytes([int(v * color[0]), int(v * color[1]), int(v * color[2])])
            try:
                sock.sendto(pkt, (WLED_HOST, 21324))
            except OSError:
                pass
            time.sleep(1 / 30)


_mouth = _Mouth()


def _beep(freq=1100, ms=70, vol=0.2):
    """Soft blip through the computer speakers: 'I heard you'. Fire and forget."""
    sr, n = 16000, 16000 * ms // 1000
    pcm = array.array("h", [int(32767 * vol * math.sin(2 * math.pi * freq * i / sr) * min(1.0, i / 80, (n - i) / 80))
                            for i in range(n)])
    hearing.mute_for(ms / 1000 + 0.4)
    threading.Thread(target=lambda: subprocess.run(
        ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(sr), "-c", "1"],
        input=pcm.tobytes(), capture_output=True), daemon=True).start()


@mcp.tool()
def speak(text: str, speed: float = 1.3, max_brightness: int = 127) -> dict:
    """Say text out loud on THIS computer's speakers (the robot has none) with Piper TTS,
    while the WLED strip shows a mouth: a closed-mouth glow that opens outward from the
    center with the real loudness of the audio. Blocks until done, then restores the lights
    to how they were. speed is Piper's length-scale (1.0 normal, 1.3 = 30% slower);
    max_brightness caps the strip during speech (0-255, default 127 = 50%). No confirmation
    needed."""
    if not _speak_lock.acquire(blocking=False):
        return {"ok": False, "error": "already speaking"}
    _mouth.stop_anim()
    spoke = False
    wav = tempfile.NamedTemporaryFile(suffix=".wav", delete=False).name
    try:
        r = subprocess.run([PIPER_BIN, "-m", PIPER_VOICE, "--length-scale", str(speed), "-f", wav],
                           input=text.encode(), capture_output=True, timeout=60)
        if r.returncode:
            return {"ok": False, "error": f"piper failed: {r.stderr.decode()[-300:]}"}
        w = wave.open(wav)
        sr, pcm = w.getframerate(), array.array("h", w.readframes(w.getnframes()))
        w.close()
        fps, hop, n = 30, sr // 30, 27
        rms = [math.sqrt(sum(x * x for x in pcm[i:i + hop]) / hop) for i in range(0, len(pcm) - hop, hop)]
        if not rms:
            return {"ok": False, "error": "no audio generated"}
        peak = sorted(rms)[int(len(rms) * 0.97)] or 1
        c = n // 2
        rest = [_MOUTH_REST[abs(i - c)] if abs(i - c) < len(_MOUTH_REST) else 0.0 for i in range(n)]
        _mouth.acquire(max(0, min(255, max_brightness)))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        glow, env = rest[:], 0.0

        def frame(target):
            nonlocal env
            env += (target - env) * (0.6 if target > env else 0.25)  # fast attack, slow release
            length = env * (c + 1)
            pkt = bytearray([2, 2])
            for i in range(n):
                glow[i] = max(min(1.0, max(0.0, length - abs(i - c))), glow[i] * 0.86, rest[i])
                pkt += bytes([0, int(255 * glow[i] ** 2.2), 0])
            try:
                sock.sendto(pkt, (WLED_HOST, 21324))
            except OSError:
                pass

        hearing.speaking(True)
        spoke = True
        player = subprocess.Popen(["aplay", "-q", wav])
        t0 = time.time()
        for k, r_ in enumerate(rms):
            while time.time() - t0 < k / fps:
                time.sleep(0.002)
            frame(min(1.0, r_ / peak))
        player.wait()
        hearing.speaking(False)
        spoke = False
        for _ in range(int(0.4 * fps)):  # settle to closed mouth briefly
            frame(0.0)
            time.sleep(1 / fps)
        return {"ok": True, "seconds": round(len(pcm) / sr, 1)}
    finally:
        if spoke:
            hearing.speaking(False)
        _mouth.release(2.6)  # in the background: after WLED's realtime timeout, unless hear() takes over
        try:
            os.unlink(wav)
        except OSError:
            pass
        _speak_lock.release()


_hear_lock = threading.Lock()


@mcp.tool()
def hear(wait_seconds: float = 10.0, max_seconds: float = 20.0, pause_seconds: float = 0.8) -> dict:
    """Listen on the webcam microphone plugged into THIS computer and return what the person
    said, as text. Waits up to wait_seconds for them to start talking, records until they
    pause for pause_seconds (or max_seconds of speech), then transcribes with Whisper. Blocks.
    For a conversation alternate hear() and speak(). Listening starts from the moment the
    last speak() finished, so anything the person said while you were deciding what to do
    next is not lost. The mouth bar shows the person it is listening (blue breath), hearing
    them (blue mouth widening with their voice) and thinking (amber dot, with a soft beep when
    they stop talking); speak() takes the lights over when you reply. Returns text="" if
    nobody spoke. No confirmation needed."""
    if not _hear_lock.acquire(blocking=False):
        return {"ok": False, "error": "already listening"}
    try:
        _mouth.acquire(100)
        _mouth.start("waiting")

        def on_event(e):
            if e == "speech_start":
                _mouth.set_mode("hearing")
            elif e == "speech_end":
                _mouth.set_mode("thinking")
                _beep()

        def on_level(lvl, thr):
            _mouth.level = min(1.0, lvl / (thr * 5))

        try:
            r = hearing.capture(max_seconds=max_seconds, wait_seconds=wait_seconds, silence_seconds=pause_seconds,
                                on_event=on_event, on_level=on_level)
        except Exception as e:
            _mouth.release()
            return {"ok": False, "error": f"mic capture failed: {e}"}
        info = {"noise_floor": round(r["floor"]), "threshold": round(r["threshold"]), "peak": round(r["peak"])}
        if r["pcm"] is None:
            _mouth.release()
            return {"ok": True, "text": "", "reason": "nobody spoke", **info}
        t0 = time.time()
        text = hearing.transcribe(r["pcm"])
        if not text:
            _mouth.release()
        return {"ok": True, "text": text, "heard_seconds": round(len(r["pcm"]) / hearing.RATE, 1),
                "transcribe_seconds": round(time.time() - t0, 1), "ended_by": r["reason"], **info}
    finally:
        _hear_lock.release()




if __name__ == "__main__":
    mcp.run()
