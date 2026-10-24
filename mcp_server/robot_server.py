#!/usr/bin/env python3
"""MCP server that lets an AI agent drive eLlama over computer_bridge.ino, closing the
loop on the wheel encoders instead of driving open-loop for a fixed duration.

Talks over the same USB-serial link as calibration/live_view.py and toandfro.py did --
reuses calibration/bridge.py for serial I/O, line parsing and the distance/heading math,
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
                             via calibration/bridge.py's port picker.
    ELLAMA_REQUIRED_SENSORS  comma/space separated sensors the drive tools insist on;
                             default "enc,imu". Set it empty to require none.
    ELLAMA_CAM_URL           explicit IP Webcam base URL (e.g. https://192.168.0.50:4444);
                             default scans the local /24 subnet for the phone.
"""

import array
import base64
import atexit
import bisect
import functools
import json
import math
import os
import re
import signal
import socket
import ssl
import subprocess
import sys
import threading
import tempfile
import time
import urllib.request
import wave
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
from pathlib import Path
from typing import Literal, Optional

import serial

# calibration/bridge.py is the PC-side client library for computer_bridge.ino: the serial
# reader, parse_line(), and the EncoderState/OrientationState/TofState math. It is imported
# rather than copied so this server and calibration/live_view.py compute distance, heading
# and range from the same code.
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "calibration"))
import bridge  # noqa: E402  (calibration/bridge.py)
import sensors as sensorlib  # noqa: E402  (mcp_server/sensors.py; `sensors` is a tool name)
import recording  # noqa: E402  (mcp_server/recording.py)
import hearing  # noqa: E402  (mcp_server/hearing.py)
import perform as perform_show  # noqa: E402  (mcp_server/perform.py)
import tilt_guard  # noqa: E402  (mcp_server/tilt_guard.py)
import motion_guard  # noqa: E402  (mcp_server/motion_guard.py)

from mcp.server.mcpserver import MCPServer as FastMCP  # mcp>=2.0 renamed FastMCP -> MCPServer
from mcp.server.mcpserver import Image
from PIL import Image as PILImage

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

# Wheel scrub during an in-place spin: the encoder differential read ~92 deg per real ~47
# deg turn (ROBOT_CHARACTERIZATION.md section 4 and turn_degrees' docstring), so the
# encoder estimate is divided by this when there is no IMU to measure rotation properly.
# A fallback, not an equal: scrub varies with the surface, which is why the IMU is used
# whenever it is there.
TURN_SCRUB_FACTOR = 1.96
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
RANGE_APPROACH_MARGIN = 1.25
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

# Per-call caps so one tool call can't send the robot further/further-turned than a
# human watching the chat can react to. Chain multiple confirmed calls for more --
# that is the stop-check-decide-move loop DRIVING_POLICY.md asks for, not a limitation
# to work around.
MAX_SINGLE_MOVE_IN = 60.0
MAX_SINGLE_TURN_DEG = 180.0

# jog() is open-loop and time-based, not encoder-target-based -- cap how long one call
# can run so a bad (left_pwm, right_pwm) can't run away unattended.
MAX_JOG_DURATION_S = 10.0

# computer_bridge.ino drops to listen-only if no serial line arrives for 500ms
# (SERIAL_IDLE_TIMEOUT_MS), so every drive loop re-sends its command at least this often.
# Verified 2026-09-28: without this, drive_distance(6in, pwm=50) stopped after ~3in.
DRIVE_KEEPALIVE_S = 0.1

# drive_arc() caps. A circle is one continuous closed-loop motion, so it gets its own
# path-length cap instead of MAX_SINGLE_MOVE_IN; it still aborts the moment telemetry
# goes stale. Radius floor keeps it from degenerating into an in-place spin (use
# turn_degrees for that).
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

# Encoder count direction when that wheel rolls forward. On 2026-09-28 this read (1, -1):
# a 6in forward drive_distance gave left +6.77in, right -6.66in. Re-verified 2026-09-29 and
# it is now (1, 1): a 22in forward drive_distance moved left +14617 ticks and right +14922
# ticks, and reversing moved both negative -- the right encoder is no longer mirrored.
# With the stale (1, -1), drive_arc's path estimate had the right wheel backwards, so its
# curvature loop never engaged on a reverse arc. Only drive_arc uses this; re-check it
# after any encoder wiring or firmware change.
ENCODER_FORWARD_SIGN = (1, 1)

# Verified 2026-09-27, robot on blocks, wheels free to spin: direction="forward" at
# +50 PWM on both sides drove all four wheels backwards, confirmed by eye. -1 makes
# direction="forward" match real forward rotation. Re-check this after any change to
# wiring, motor board firmware, or which physical wheel is "left" vs "right".
FORWARD_PWM_SIGN = -1

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


# Motion guard (mcp_server/motion_guard.py): stall / encoder-fault / slip / bump -- the
# sensors stopped agreeing with what was commanded. Same latch + choke point as the tilt guard.
#   ELLAMA_MOTION_GUARD=on|warn|off   (default on; warn logs events without ever stopping)
MOTION_MODE = os.environ.get("ELLAMA_MOTION_GUARD", "on").lower()
if MOTION_MODE not in ("on", "warn", "off"):
    MOTION_MODE = "on"
MOTION = motion_guard.MotionGuard(
    forward_pwm_sign=FORWARD_PWM_SIGN,
    track_width_in=bridge.load_config()["track_width_in"],
    mode=MOTION_MODE)


class GuardFault(Exception):
    """Raised by RobotLink.send_drive when a guard (tilt or motion) refuses a drive command.
    Drive loops let it propagate: the tool's own `finally` sends the stop, and the tool returns
    the fault (see _tilt_guarded); autonomous threads end with reason "error: ... guard ...".
    `info` carries a snapshot of both guards."""

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
        self.ser = serial.Serial(port, bridge.BAUD_RATE, timeout=0.05)
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
        self.video = None  # recording.VideoRecorder while a recording has video on
        self.last_imu = None

        # Serial writes come from tool threads AND the pump (the tilt trip sends "stop" from
        # there, ~20 ms after the offending IMU sample), so a line must never interleave.
        self._wlock = threading.RLock()
        self.tilt = TILT  # module-level: a latched fault and the level reference survive connect()
        self.motion = MOTION  # likewise: connect() must not wipe a latched motion fault
        self.on_tilt_trip = None  # callable(fault_dict); set by the server to abort autonomous runs

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
                    if sl is not None and sr is not None:
                        tripped = self.motion.on_enc(sl, sr, now)
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
                    mf = self.motion.on_imu(dz, agx, agy, now)
                    tripped = tripped or mf
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
                # Outside the state lock: stop first (the guard already latched, so send_drive
                # refuses from here on), then tell whoever is running an autonomous loop.
                self.emergency_stop()
                cb = self.on_tilt_trip
                if cb is not None:
                    try:
                        cb(tripped)
                    except Exception:
                        pass

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
           and calls clear_tilt_fault / clear_motion_fault.

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
        m = self.motion
        if m.mode != "off" and m.tripped:
            return "motion guard tripped: " + m.fault["reason"]
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
            self.motion.on_command(left, right, time.monotonic())
            self.send_line(f"{left} {right}")

    def _guard_info(self):
        """Both guards, each annotated with whether the sensor it reads is even present --
        an "off" tilt guard and one with no IMU look identical without this."""
        tilt = self.tilt.snapshot()
        motion = self.motion.snapshot()
        imu_live = self.sensors.trusted_live("imu")
        enc_live = self.sensors.trusted_live("enc")
        tilt["sensor"] = "imu"
        tilt["sensor_live"] = imu_live
        tilt["active"] = tilt["mode"] == "on" and imu_live and tilt["ready"]
        if not imu_live:
            tilt["inactive_because"] = "no live IMU: nothing measures lean"
        elif not tilt["ready"]:
            tilt["inactive_because"] = "no level reference captured yet"
        motion["sensors"] = ["enc", "imu"]
        motion["sensor_live"] = {"enc": enc_live, "imu": imu_live}
        motion["active"] = motion["mode"] == "on" and (enc_live or imu_live)
        if not (enc_live or imu_live):
            motion["inactive_because"] = "no live encoders or IMU: nothing to compare commands against"
        elif not imu_live:
            motion["inactive_because"] = "no live IMU: bump and slip detection are blind"
        elif not enc_live:
            motion["inactive_because"] = "no live encoders: stall and direction detection are blind"
        return {"tilt": tilt, "motion": motion,
                "policy": self.policy.snapshot(self.sensors)}

    def send_stop(self):
        self.motion.on_command(0, 0, time.monotonic())
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
                motion=self.motion.snapshot(),
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
        if self.video is not None:
            self.video.stop()
            self.video = None
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
        link.on_tilt_trip = _on_guard_trip
        _connect_error = None
    except Exception as e:
        _connect_error = str(e)
        raise
    return link


def _on_guard_trip(fault):
    """Called from the IMU pump thread right after the guard trips and "stop" is sent: end any
    autonomous run (perform). Blocking tools notice on their next
    send_drive, which raises TiltFault."""
    _run["abort"].set()


def _shutdown():
    _run["abort"].set()
    if link is not None:
        link.emergency_stop()


atexit.register(_shutdown)
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, lambda signum, frame: (_shutdown(), sys.exit(1)))

try:
    _connect(os.environ.get("ELLAMA_SERIAL_PORT"))
except Exception:
    pass  # tools report the connection error; the server itself still starts


# ---------------------------------------------------------------------------
# Phone camera (Android IP-camera app) -- read-only vision, never moves the robot.
# Camera images are advisory: they do NOT replace the confirmed_safe gate above.
# ---------------------------------------------------------------------------

CAM_PORTS = (8080, 4444)  # IP Webcam's default, and the one this phone is set to
# Single-JPEG endpoints of the apps we know: "Android IP Camera" (what this phone runs, over
# https on 4444; a snapshot takes ~5-8s) and Pavel Khlebovich's "IP Webcam".
CAM_SNAPSHOT_PATHS = ("/video/snapshot", "/shot.jpg")
# Live MJPEG endpoints of the same apps, tried in order: "Android IP Camera" (what this phone
# runs; ~15 fps at 360x480 as of 2026-10-02) and IP Webcam.
CAM_STREAM_PATHS = ("/video/mjpeg", "/video")
CAM_PROBE_TIMEOUT_S = 8       # finding the camera / grabbing a stream frame: usually ~1-3 s
CAM_SNAPSHOT_TIMEOUT_S = 12   # a full-res still takes ~1-8 s; past this the photo has failed
CAM_MAX_WIDTH = 1280

# auto_photo (start_recording): take a photo each time the robot comes to rest after moving.
AUTO_PHOTO_STILL_S = 0.7          # both wheels below STILL_SPEED_IN_S for this long = stopped
AUTO_PHOTO_STILL_SPEED_IN_S = 0.5
AUTO_PHOTO_MIN_TRAVEL_IN = 4.0    # summed |left|+|right| wheel travel since the last photo
AUTO_PHOTO_MIN_TURN_DEG = 10.0    # or this much IMU yaw change
AUTO_PHOTO_POLL_S = 0.1
_cam = None  # cached (base_url, snapshot_path) once found
CAM_CACHE_FILE = Path.home() / ".cache" / "ellama" / "camera_url"  # last address the phone answered on
# IP Webcam's optional TLS uses a self-signed cert, so verification is off -- acceptable
# for a read-only camera on the home LAN, and this context is used for nothing else.
_cam_ssl = ssl.create_default_context()
_cam_ssl.check_hostname = False
_cam_ssl.verify_mode = ssl.CERT_NONE


def _cam_open(url, timeout):
    return urllib.request.urlopen(url, timeout=timeout, context=_cam_ssl)

_cam_lock = threading.Lock()


def _local_ip():
    """Primary LAN IP, via the connect-a-UDP-socket trick (sends no packets)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    finally:
        s.close()


def _read_jpeg(r, limit=2_000_000):
    """The first complete JPEG in a file-like (a bare JPEG or a multipart MJPEG stream), else None."""
    buf = b""
    while len(buf) < limit:
        chunk = r.read(8192)
        if not chunk:
            break
        buf += chunk
        a = buf.find(b"\xff\xd8")
        if a >= 0:
            b = buf.find(b"\xff\xd9", a + 2)
            if b >= 0:
                return buf[a:b + 2]
    return None


def _probe(base):
    """The snapshot path of the camera app on base, else None. The app is recognised by its
    live stream returning a JPEG, which is quick even when the single-photo endpoint is slow
    or wedged (the two path tuples are in the same app order)."""
    for snap, stream in zip(CAM_SNAPSHOT_PATHS, CAM_STREAM_PATHS):
        try:
            with _cam_open(base + stream, CAM_PROBE_TIMEOUT_S) as r:
                if _read_jpeg(r) is not None:
                    return snap
        except Exception:
            pass
    for snap in CAM_SNAPSHOT_PATHS:   # an app with no stream: fall back to the photo itself
        try:
            with _cam_open(base + snap, CAM_SNAPSHOT_TIMEOUT_S) as r:
                if r.read(2) == b"\xff\xd8":
                    return snap
        except Exception:
            pass
    return None


def _discover_camera():
    """Scan the local /24 for a phone camera server: cheap TCP connect on each candidate
    port of every host first, then verify (http, then https) only the ones that answered.
    Returns (base_url, snapshot_path)."""
    prefix = _local_ip().rsplit(".", 1)[0]

    def port_open(hp):
        try:
            with socket.create_connection(hp, timeout=1.5):  # phones on Wi-Fi are slow to wake
                return hp
        except OSError:
            return None

    targets = [(f"{prefix}.{i}", p) for i in range(1, 255) for p in CAM_PORTS]
    with ThreadPoolExecutor(max_workers=128) as ex:
        open_targets = [t for t in ex.map(port_open, targets) if t]
    for host, port in open_targets:
        for scheme in ("https", "http"):
            base = f"{scheme}://{host}:{port}"
            path = _probe(base)
            if path:
                return base, path
    raise RuntimeError(
        f"no phone camera found on {prefix}.0/24 ports {CAM_PORTS} -- is the app's server "
        "started and the phone on the same Wi-Fi? Or set ELLAMA_CAM_URL."
    )


def _saved_camera():
    """(base_url, snapshot_path) from the last time the phone was found, trusted as is: a
    stale one fails its first fetch and _fetch_snapshot rediscovers. Skips the /24 scan,
    which takes ~6 s and seems to stall the phone's camera server."""
    try:
        base, path = CAM_CACHE_FILE.read_text().split()
    except (OSError, ValueError):
        return None
    return base, path


def _save_camera(cam):
    try:
        CAM_CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
        CAM_CACHE_FILE.write_text(f"{cam[0]} {cam[1]}\n")
    except OSError:
        pass


def _camera(rediscover=False):
    """(base_url, snapshot_path): the saved address if it still answers, else a subnet
    scan; cached for the life of the process."""
    global _cam
    with _cam_lock:
        if rediscover:
            _cam = None
        if _cam is None:
            explicit = os.environ.get("ELLAMA_CAM_URL", "").rstrip("/")
            if explicit:
                _cam = (explicit, _probe(explicit) or CAM_SNAPSHOT_PATHS[0])
            else:
                _cam = (None if rediscover else _saved_camera()) or _discover_camera()
                _save_camera(_cam)
        return _cam


def _reachable(base, timeout=2.0):
    """True if something accepts TCP connections at base's host:port."""
    from urllib.parse import urlparse
    u = urlparse(base)
    try:
        with socket.create_connection((u.hostname, u.port or (443 if u.scheme == "https" else 80)),
                                      timeout=timeout):
            return True
    except OSError:
        return False


def _fetch_snapshot():
    """A full-res JPEG from the phone's photo endpoint, or a RuntimeError saying why not. It
    never substitutes a frame of the live stream. If the cached address is dead (phone got a
    new DHCP lease) it rediscovers once and retries; if the address answers but the photo does
    not come back, that is reported as is, without a rescan."""
    last = None
    for attempt in (0, 1):
        try:
            base, path = _camera(rediscover=attempt == 1)
        except Exception as e:
            last = e
            continue
        try:
            with _cam_open(base + path, CAM_SNAPSHOT_TIMEOUT_S) as r:
                img = r.read()
            if img[:2] == b"\xff\xd8":
                return img
            last = RuntimeError(f"{base}{path} did not return a JPEG")
        except Exception as e:
            last = e
        if _reachable(base):
            raise RuntimeError(
                f"the camera app is up at {base} but its photo endpoint ({path}) returned no "
                f"image within {CAM_SNAPSHOT_TIMEOUT_S} s ({last}); look() does not fall back to "
                "the live stream. Try restarting the camera app on the phone.")
    raise RuntimeError(f"camera fetch failed: {last}")


def _open_stream():
    """Open the phone's MJPEG stream (for recording). Rediscovers the phone once if the
    cached address is stale. Returns a file-like to read multipart MJPEG from."""
    last = None
    for attempt in (0, 1):
        base, _ = _camera(rediscover=attempt == 1)
        for path in CAM_STREAM_PATHS:
            try:
                return _cam_open(base + path, 10)
            except Exception as e:
                last = e
    raise RuntimeError(f"no video stream: {last}")


_run = {"running": False, "abort": threading.Event(), "status": "idle", "info": {}, "result": None,
        "ended": threading.Event()}


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
    elif link.motion.tripped and link.motion.mode != "off":
        hint = ("look at the robot first -- something stopped it (see fault) -- then "
                "clear_motion_fault(confirmed_safe=true) after asking the operator. If the "
                "detector is wrong for this job (pushing something trips bump), "
                "set_guard('motion', checks={'bump': false}) turns just that one off.")
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
    if _run["running"]:
        return {"ok": False, "error": "an autonomous run (perform) is in progress; call stop() first"}
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
    """Read-only. Both guards: mode, thresholds, whether the sensor each one reads is live,
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
    name: Literal["tilt", "motion"],
    mode: Optional[Literal["on", "warn", "off"]] = None,
    limit_deg: Optional[float] = None,
    lookahead_s: Optional[float] = None,
    stall: Optional[bool] = None,
    direction: Optional[bool] = None,
    slip: Optional[bool] = None,
    bump: Optional[bool] = None,
    stall_speed_in_s: Optional[float] = None,
    slip_ratio: Optional[float] = None,
    bump_hard_g: Optional[float] = None,
    bump_soft_g: Optional[float] = None,
    encoders_required: Optional[bool] = None,
) -> dict:
    """Turn a guard (or one of the motion guard's detectors) on or off and retune it, while
    running. Does not move the robot.

    `mode` is "on" (trips and latches), "warn" (records what it would have tripped on and
    never stops the robot -- use this to tune on a real robot) or "off".

    tilt:   limit_deg (default 25), lookahead_s. Raise limit_deg for a ramp the robot is
            meant to climb; the guard measures lean from where it was level, so a ramp is a
            real lean to it.
    motion: the four detectors stall / direction / slip / bump switch on and off
            individually, plus stall_speed_in_s, slip_ratio, bump_hard_g, bump_soft_g.
            Pushing something heavy trips bump on the push itself -- turn bump off for that
            job and leave stall and direction watching. encoders_required=false stops an
            encoder fault from latching on a run that does not use the encoders.

    What this deliberately does NOT do: clear a latched fault. Switching a guard off
    suspends its enforcement, but what it saw stays on the record and comes back if you
    switch it on again. Only clear_tilt_fault / clear_motion_fault release a latch, and
    they require the operator to have actually looked at the robot.

    Changes are written into any active recording, so a run's CSV shows the guard config it
    was driven under.
    """
    if link is None:
        return _not_connected()
    if name == "tilt":
        ok, msg = link.tilt.configure(mode=mode, limit_deg=limit_deg, lookahead_s=lookahead_s)
    else:
        checks = {k: v for k, v in (("stall", stall), ("direction", direction),
                                    ("slip", slip), ("bump", bump)) if v is not None}
        ok, msg = link.motion.configure(
            mode=mode, checks=checks or None, encoders_required=encoders_required,
            stall_speed_in_s=stall_speed_in_s, slip_ratio=slip_ratio,
            bump_hard_g=bump_hard_g, bump_soft_g=bump_soft_g)
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
    _run["abort"].set()
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
def clear_motion_fault(confirmed_safe: bool) -> dict:
    """Release the motion guard after it stopped the robot because the sensors stopped matching
    the commands: a wheel commanded to turn that wasn't (stall), an encoder not producing what
    the gyro says it should (encoder_fault), wheels turning while the chassis wasn't (slip), or
    an impact (bump). Every drive tool is refused until this is called. Does not move the robot.
    Set confirmed_safe=true only after the human operator has LOOKED at the robot and what it
    was doing -- the fault says what the data looked like, not what is in the way -- and said
    it is clear. For an `encoder_fault` the cause is the sensor, so check the encoder (magnet
    gap, wiring) before driving something that depends on it again."""
    if link is None:
        return _not_connected()
    if not confirmed_safe:
        return {"ok": False, "error": "confirmed_safe must be true, set only after the operator "
                "has looked at the robot and says it is clear", "motion_guard": link.motion.snapshot()}
    ok, msg = link.motion.clear()
    return {"ok": ok, "message": msg, "motion_guard": link.motion.snapshot()}


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
    _run["abort"].set()
    if link is None:
        return _not_connected()
    link.send_listen()
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def look(width: int = 640):
    """Read-only. Take a photo with the phone camera mounted on the robot and return it, so
    you can see what the robot currently sees. Never moves the robot, no confirmation
    needed. The phone is found automatically on the local network (or via ELLAMA_CAM_URL).
    A full-res photo takes ~1-8 seconds; if the phone's photo endpoint does not return one
    within 12 s this returns an error (it never substitutes a frame of the live video
    stream). `width` downsizes the image (default 640, max 1280) to keep it cheap. This is advisory vision only -- a JPEG glance is not an obstacle sensor, so it
    does NOT replace asking the operator before any move. Use save_photo() to keep the
    picture as a file on this computer."""
    width = max(64, min(int(width), CAM_MAX_WIDTH))
    try:
        img = PILImage.open(BytesIO(_fetch_snapshot()))
        if img.width > width:
            img = img.resize((width, round(img.height * width / img.width)))
        buf = BytesIO()
        img.convert("RGB").save(buf, "JPEG", quality=80)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return Image(data=buf.getvalue(), format="jpeg")


TAG_MM = float(os.environ.get("ELLAMA_TAG_MM", "250"))
TAG_DICT = os.environ.get("ELLAMA_TAG_DICT", "DICT_4X4_50")


@mcp.tool()
def tags(tag_mm: float = 0.0, dict_name: str = "") -> dict:
    """Read-only. Take a full-res photo with the phone camera and report every ArUco tag in
    it: its id, its BEARING in degrees off the camera's centre line (negative = left of
    centre, which is what you steer by) and a rough RANGE in mm. Never moves the robot.

    Bearing is trustworthy -- it depends only on the lens field of view. Range is derived
    from the tag's apparent size with an uncalibrated focal length, so treat it as a gate
    ("closer than a metre") rather than a measurement.

    An empty `tags` list with live=true means the camera looked and saw no tag -- which is
    NOT the same as live=false, the camera being unreachable. A tag whose `weak` flag is
    true was detected near the pixel limit and will drop in and out; move closer or use
    bigger tags rather than trusting it.

    `tag_mm` is the printed size of the tags in millimetres (default ELLAMA_TAG_MM, 250).
    All tags must be printed the same size for range to mean anything."""
    try:
        import cv2
        import numpy as np
        sys.path.insert(0, str(REPO_ROOT / "tools"))
        from find_tags import find_tags as _find_tags
    except Exception as e:
        return {"ok": False, "live": False, "error": f"tag detection unavailable: {e}"}

    try:
        raw = _fetch_snapshot()
    except Exception as e:
        return {"ok": False, "live": False, "error": str(e)}

    img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {"ok": False, "live": False, "error": "camera returned something that is not an image"}

    found = _find_tags(img, dict_name or TAG_DICT, tag_mm or TAG_MM)
    for t in found:
        t["weak"] = t["px"] < 25
    return {
        "ok": True,
        "live": True,
        "measuring": bool(found),
        "tags": found,
        "frame": {"width": img.shape[1], "height": img.shape[0]},
        "tag_mm": tag_mm or TAG_MM,
        "dict": dict_name or TAG_DICT,
    }


VISION_MODEL = os.environ.get("ELLAMA_VISION_MODEL", "gemma3:4b")
VISION_WIDTH = int(os.environ.get("ELLAMA_VISION_WIDTH", "768"))
VISION_TIMEOUT_S = float(os.environ.get("ELLAMA_VISION_TIMEOUT_S", "300"))
OLLAMA_URL = os.environ.get("OLLAMA_HOST", "http://localhost:11434").rstrip("/")


@mcp.tool()
def describe_view(question: str = "", width: int = 0) -> dict:
    """Read-only. Take a photo with the phone camera and have a vision model describe it in
    words. Never moves the robot. Use this when you need to know what is actually in front
    of the robot -- a doorway, a person, clutter on the floor -- that no sensor reports.

    `question` asks something specific ("is the floor clear ahead?", "what is the dark shape
    on the left?"); empty asks for a plain description. Ask narrow questions: the answer is
    a few sentences of prose from a small model, so it is advisory, not a measurement, and
    it can be confidently wrong about things it half-sees.

    This is NOT an obstacle sensor and does NOT replace asking the operator before a move.
    For tags use tags(), which measures bearing geometrically instead of guessing.

    Takes roughly 10-60 s: the photo, then a vision model running on the CPU."""
    try:
        raw = _fetch_snapshot()
    except Exception as e:
        return {"ok": False, "error": str(e)}

    try:
        img = PILImage.open(BytesIO(raw))
        w = max(256, min(int(width or VISION_WIDTH), 1280))
        if img.width > w:
            img = img.resize((w, round(img.height * w / img.width)))
        buf = BytesIO()
        img.convert("RGB").save(buf, "JPEG", quality=80)
        b64 = base64.b64encode(buf.getvalue()).decode()
    except Exception as e:
        return {"ok": False, "error": f"could not prepare the image: {e}"}

    prompt = question.strip() or "Describe what you see. Two or three sentences."
    body = json.dumps({
        "model": VISION_MODEL, "stream": False, "keep_alive": "30m",
        "messages": [{"role": "user", "content": prompt, "images": [b64]}],
    }).encode()
    t0 = time.monotonic()
    try:
        req = urllib.request.Request(f"{OLLAMA_URL}/api/chat", body,
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=VISION_TIMEOUT_S) as r:
            answer = json.load(r)["message"]["content"].strip()
    except Exception as e:
        return {"ok": False, "error": f"vision model {VISION_MODEL} failed: {e}",
                "hint": f"is ollama running, and is {VISION_MODEL} pulled?"}

    return {"ok": True, "model": VISION_MODEL, "question": prompt, "answer": answer,
            "image_width": img.width, "seconds": round(time.monotonic() - t0, 1),
            "advisory": "prose from a small vision model; not an obstacle sensor"}


PHONE_LOW_BATTERY_PCT = 20


def _phone_get(path, timeout=6.0):
    """Body of GET <camera base><path>. A dead cached address (new DHCP lease) is rediscovered
    once, the same way _fetch_snapshot does."""
    last = None
    for attempt in (0, 1):
        try:
            base, _ = _camera(rediscover=attempt == 1)
            with _cam_open(base + path, timeout) as r:
                return r.read()
        except Exception as e:
            last = e
    raise RuntimeError(f"phone camera app did not answer {path}: {last}")


def _phone_info():
    info = json.loads(_phone_get("/info.json"))
    torch = (info.get("settings") or {}).get("torch")
    return {
        "battery_percent": info.get("batteryPercent"),
        "torch_on": None if torch is None else str(torch).lower() in ("on", "true", "1"),
    }


@mcp.tool()
def phone_status():
    """Read-only. The robot's phone (camera + brain): battery percent and whether the torch is
    on, from the camera app's /info.json. Never moves the robot, no confirmation needed. Check
    it before a long scan or before using the torch, which drains the battery quickly;
    `low_battery` is true at or below 20%."""
    try:
        s = _phone_info()
    except Exception as e:
        return {"ok": False, "error": str(e)}
    pct = s["battery_percent"]
    return {"ok": True, **s, "low_battery": pct is not None and pct <= PHONE_LOW_BATTERY_PCT}


@mcp.tool()
def torch(on: bool):
    """Turn the phone's torch (flashlight) on or off, then read the state back. Never moves the
    robot, no confirmation needed. Use it when the camera view is dark (see save_photo's
    `brightness`) so tags and doorways are readable, and turn it off again when idle, stalled
    or waiting on the operator: it drains the phone, which is also the robot's camera and
    brain. Returns the phone's battery percent too; `torch_on` is what the phone reports after
    the change (null if the app doesn't say)."""
    try:
        _phone_get(f"/?torch={'on' if on else 'off'}")
        s = _phone_info()
    except Exception as e:
        return {"ok": False, "error": str(e)}
    pct = s["battery_percent"]
    result = {"ok": True, **s, "low_battery": pct is not None and pct <= PHONE_LOW_BATTERY_PCT}
    if s["torch_on"] is not None and s["torch_on"] != bool(on):
        result["ok"] = False
        result["error"] = f"asked for torch {'on' if on else 'off'} but the phone reports it {'on' if s['torch_on'] else 'off'}"
    return result


PHOTO_DIR = Path(os.environ.get("ELLAMA_PHOTO_DIR") or REPO_ROOT / "recordings" / "photos")
PHOTO_MOVED_IN = 0.2    # wheel travel (|left|+|right|) during the fetch that counts as "moved"
PHOTO_MOVED_DEG = 1.0   # or this much IMU yaw change


def _write_photo(jpeg, dest_dir, name, meta):
    """Save jpeg as dest_dir/<YYYYmmdd_HHMMSS>[_name].jpg with a .json sidecar holding meta.
    The folder gets its own .gitignore so photos never show up in git status. Returns the
    jpg path."""
    dest_dir.mkdir(parents=True, exist_ok=True)
    gi = dest_dir / ".gitignore"
    if not gi.exists():
        gi.write_text("*\n")   # ignores everything here, itself included
    stem = time.strftime("%Y%m%d_%H%M%S")
    slug = re.sub(r"[^A-Za-z0-9_-]+", "_", name or "").strip("_")[:40]
    if slug:
        stem += "_" + slug
    path, n = dest_dir / f"{stem}.jpg", 2
    while path.exists():   # two photos in the same second
        path, n = dest_dir / f"{stem}_{n}.jpg", n + 1
    path.write_bytes(jpeg)
    path.with_suffix(".json").write_text(json.dumps({**meta, "file": path.name}, indent=2))
    return path


@mcp.tool()
def save_photo(name: str = "", preview_width: int = 0, open_in_browser: bool = False):
    """Read-only. Take a full-res photo with the phone camera and SAVE it on this computer,
    returning its path, so you can open the file or run image recognition / any script on it
    (all of that happens off the robot). Never moves the robot, no confirmation needed.

    Returns `path` (the .jpg; a .json with the same facts sits next to it), `width`/`height`,
    `brightness` (mean 0-255; low = consider the phone torch), `robot` (the odometry pose at
    the moment of the photo: dist_avg_in and heading_deg since zero_odometry(), IMU
    yaw/pitch/roll; null if the bridge isn't connected) and `moved_during_capture` (true if the
    wheels or yaw changed during the ~1-2 s fetch, so the picture may be blurred or from a
    slightly different spot).

    The look-decide-move loop: drive_distance / turn_degrees (it stops itself) -> save_photo
    -> look at the file, decide -> next move. Compare `robot` between photos to place them.
    name is an optional label for the filename. preview_width > 0 also returns the picture
    itself, downsized to that width (max 1280), so you can see it in this same call; leave 0
    to keep the call cheap and open the file later. open_in_browser shows it in a browser
    window on this computer. If the phone's photo endpoint doesn't answer within 12 s this
    fails with an error; it never substitutes a frame of the live stream."""
    def pose():
        if link is None:
            return None
        with link.lock:
            return (link.enc.dist_l, link.enc.dist_r, link.orient.yaw)

    before, t0 = pose(), time.monotonic()
    try:
        jpeg = _fetch_snapshot()
        im = PILImage.open(BytesIO(jpeg))
        im.load()
    except Exception as e:
        return {"ok": False, "error": str(e)}
    elapsed, after = time.monotonic() - t0, pose()
    moved = None
    if before is not None and after is not None:
        moved = (abs(after[0] - before[0]) + abs(after[1] - before[1]) > PHOTO_MOVED_IN
                 or abs(after[2] - before[2]) > PHOTO_MOVED_DEG)
    robot = None
    if link is not None:
        s = link.telemetry_snapshot()
        robot = {k: s.get(k) for k in ("dist_avg_in", "heading_deg", "yaw_deg", "pitch_deg",
                                       "roll_deg", "telemetry_live", "imu_live")}
    brightness = round(sum(im.convert("L").resize((64, 64)).getdata()) / 4096)
    meta = {"taken_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "name": name or None,
            "width": im.width, "height": im.height, "brightness": brightness,
            "robot": robot, "moved_during_capture": moved, "elapsed_s": round(elapsed, 2)}
    try:
        path = _write_photo(jpeg, PHOTO_DIR, name, meta)
    except OSError as e:
        return {"ok": False, "error": f"could not save the photo: {e}"}
    result = {"ok": True, "path": str(path), "width": im.width, "height": im.height,
              "bytes": len(jpeg), "brightness": brightness, "robot": robot,
              "moved_during_capture": moved, "elapsed_s": round(elapsed, 2)}
    if open_in_browser:
        page = path.with_suffix(".html")
        page.write_text(
            f'<!doctype html><meta charset="utf-8"><title>{path.name}</title>'
            '<body style="margin:0;background:#111;display:flex;justify-content:center;'
            f'align-items:center;min-height:100vh"><img src="{path.name}" '
            'style="max-width:100vw;max-height:100vh;object-fit:contain"></body>')
        try:
            result["opened_in_browser"] = bool(webbrowser.open(page.as_uri()))
        except Exception:
            result["opened_in_browser"] = False
    width = max(0, min(int(preview_width), CAM_MAX_WIDTH))
    if width <= 0:
        return result
    small = im.convert("RGB")
    if small.width > width:
        small = small.resize((width, round(small.height * width / small.width)))
    buf = BytesIO()
    small.save(buf, "JPEG", quality=80)
    return [json.dumps(result), Image(data=buf.getvalue(), format="jpeg")]


@mcp.tool()
def start_recording(name: Optional[str] = None, auto_photo: bool = False,
                    video: bool = True) -> dict:
    """Read-only. Start recording telemetry to recordings/<name>/<name>.csv (name defaults
    to run_<timestamp>): one row per encoder/IMU sample, in the same format as
    calibration/live_view.py --csv, so analyze_csv.py reads it. Works while the stick
    controller drives -- this server sends nothing to the robot unless a drive tool is
    called. Never moves the robot, no confirmation needed. Use add_marker() and
    take_photo() during the recording, stop_recording() when done.

    auto_photo=True: also take a photo by itself whenever the robot comes to rest (wheels
    still for ~0.7 s) after moving at least ~4 in of wheel travel or ~10 deg of yaw since
    the last photo, plus one at the start once still. Each PHOTO row is stamped when the
    robot stopped; if it moves again before the snapshot arrives (~5-8 s), a MARK row
    says so. Hold still until you've seen it happen, or accept the flag.

    video=True (default): also record the phone's live video stream to video.mp4 in the
    recording folder, plus video_times.csv (when each frame arrived, on the same time_s
    clock as the CSV -- use it to sync, since the phone's frame rate varies). Also saves
    commands.csv: every drive command the stick sent (time_s,left,right), if the bridge
    has the firmware that relays them. A failed video start doesn't stop telemetry
    recording; it's reported in the result."""
    if link is None:
        return _not_connected()
    try:
        path = link.recorder.start(name)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    video_status = "off"
    if video:
        link.video = recording.VideoRecorder(_open_stream, link.recorder.now, link.recorder.dir)
        link.video.start()
        video_status = "recording (check video_error in stop_recording if video.mp4 is empty)"
    if auto_photo:
        threading.Thread(target=_auto_photo_loop, args=(link, path), daemon=True).start()
    return {"ok": True, "csv": str(path), "auto_photo": auto_photo, "video": video_status,
            **link.telemetry_snapshot()}


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


_photo_lock = threading.Lock()  # path choice + PHOTO row must be atomic across threads


def _capture_photo(lnk, note=""):
    """Stamp a PHOTO row now, then fetch and save the snapshot. Returns (result dict,
    stamped pose (dist_l, dist_r, yaw)) -- the pose lets callers tell if the robot moved
    during the slow fetch."""
    rec = lnk.recorder
    with _photo_lock:
        path = rec.next_photo_path()
        label = f"{path.name} {note}".strip()
        with lnk.lock:
            t = rec.mark(lnk.enc, lnk.orient, "PHOTO", label)
            pose = (lnk.enc.dist_l, lnk.enc.dist_r, lnk.orient.yaw)
    try:
        path.write_bytes(_fetch_snapshot())
    except Exception as e:
        with lnk.lock:
            rec.mark(lnk.enc, lnk.orient, "MARK", f"{path.name} FAILED: {e}")
        return {"ok": False, "error": str(e)}, pose
    return {"ok": True, "photo": str(path), "time_s": None if t is None else round(t, 3)}, pose


def _auto_photo_loop(lnk, csv_path):
    """Watches for the robot coming to rest and photographs it. Runs until this recording
    (identified by its csv path) ends."""
    rec = lnk.recorder
    last = None          # pose at the last photo: (dist_l, dist_r, yaw); None until the first
    still_since = None
    while rec.active and rec.csv_path == csv_path:
        time.sleep(AUTO_PHOTO_POLL_S)
        now = time.monotonic()
        sl, sr = rec.speed
        if max(abs(sl), abs(sr)) >= AUTO_PHOTO_STILL_SPEED_IN_S:
            still_since = None
            continue
        if still_since is None:
            still_since = now
        if now - still_since < AUTO_PHOTO_STILL_S:
            continue
        with lnk.lock:
            cur = (lnk.enc.dist_l, lnk.enc.dist_r, lnk.orient.yaw)
        if last is not None:
            travel = abs(cur[0] - last[0]) + abs(cur[1] - last[1])
            turn = abs(cur[2] - last[2])
            if travel < AUTO_PHOTO_MIN_TRAVEL_IN and turn < AUTO_PHOTO_MIN_TURN_DEG:
                continue
        result, pose = _capture_photo(lnk, "auto")
        last = pose
        with lnk.lock:
            after = (lnk.enc.dist_l, lnk.enc.dist_r, lnk.orient.yaw)
            moved = (abs(after[0] - pose[0]) + abs(after[1] - pose[1]) >= 1.0
                     or abs(after[2] - pose[2]) >= 3.0)
            if moved and result.get("ok") and rec.csv_path == csv_path:
                rec.mark(lnk.enc, lnk.orient, "MARK",
                         f"{Path(result['photo']).name} robot moved during capture")
        still_since = None


@mcp.tool()
def take_photo(note: str = "") -> dict:
    """Read-only. While recording, take a full-size photo with the phone camera and save
    it next to the CSV (photo_001.jpg, ...), with a PHOTO row at the moment of the request
    (the snapshot itself takes ~5-8 seconds). Never moves the robot. Use look() instead to
    just view the camera without saving."""
    if link is None:
        return _not_connected()
    if not link.recorder.active:
        return {"ok": False, "error": "not recording; call start_recording() first"}
    result, _ = _capture_photo(link, note)
    return result


@mcp.tool()
def stop_recording() -> dict:
    """Read-only. Stop recording and close the CSV. Returns its path, duration and how
    many samples, markers and photos were saved. Never moves the robot."""
    if link is None:
        return _not_connected()
    video_summary = None
    if link.video is not None:
        video_summary = link.video.stop()
        link.video = None
    summary = link.recorder.stop()
    if summary is None:
        return {"ok": False, "error": "not recording"}
    out = {"ok": True, **summary}
    if video_summary is not None:
        out["video"] = video_summary
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
    haven't personally verified, DRIVING_POLICY.md's floor is 5-10% duty (13-26). Capped
    at MAX_SINGLE_MOVE_IN=60 inches per call -- chain further confirmed calls for more,
    checking read_telemetry() between them.

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
    if distance_inches > MAX_SINGLE_MOVE_IN:
        return {
            "ok": False,
            "error": f"distance_inches {distance_inches} exceeds the {MAX_SINGLE_MOVE_IN}in "
            "per-call cap -- chain multiple confirmed calls instead, checking "
            "read_telemetry() between them.",
        }

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
        if open_loop_s > MAX_JOG_DURATION_S:
            return {"ok": False, "error":
                    "no encoder feedback: this move would be open-loop for %.1fs, over the "
                    "%.0fs cap for an unmeasured move. Use a shorter distance or a higher pwm, "
                    "or bring the encoders back." % (open_loop_s, MAX_JOG_DURATION_S),
                    "telemetry": snap}

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

    Uses the IMU, not the wheel encoders, as ground truth for how far the robot has
    actually rotated. Verified 2026-09-27 (robot on the ground, four 90deg turns each
    way): the encoder-differential estimate (dr-dl over track_width_in) read ~92 deg per
    commanded 90deg turn while the robot visually rotated only ~47 deg -- wheel scrub
    during an in-place spin makes the wheels turn much more than the chassis does, so
    the encoders overestimate rotation by roughly 2x. The IMU yaw delta over those same
    turns averaged ~49 deg, matching the ~47 deg observed by eye. The response still
    includes encoder_degrees_estimate for reference, explicitly labeled unreliable.

    Capped at MAX_SINGLE_TURN_DEG=180 per call. direction "left" is counter-clockwise
    viewed from above, matching calibration/bridge.py's heading convention.

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
    if degrees > MAX_SINGLE_TURN_DEG:
        return {
            "ok": False,
            "error": f"degrees {degrees} exceeds the {MAX_SINGLE_TURN_DEG}deg per-call cap "
            "-- chain multiple confirmed calls instead, checking read_telemetry() between "
            "them.",
        }

    snap = link.telemetry_snapshot()
    start = link.last_counts()
    # The IMU is ground truth for rotation. Falling back to the encoder differential is a
    # real option, not an equal one: it over-reads an in-place turn by ~2x from wheel
    # scrub, and TURN_SCRUB_FACTOR only undoes the average of that.
    if snap["imu_live"]:
        feedback_source = "imu"
    elif snap["telemetry_live"] and start is not None:
        feedback_source = "encoders"
    else:
        return {
            "ok": False,
            "error": "nothing can measure this turn: no live IMU, and no live encoders on "
            "both wheels either. A timed turn is not this tool -- use jog(left_pwm, "
            "right_pwm, duration_s) and watch it, or bring a sensor back.",
            "telemetry": snap,
        }

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

    def encoder_turn_deg():
        """Degrees turned per the wheel differential, scrub-corrected, or None."""
        cur = link.last_counts()
        if cur is None or start is None:
            return None
        dl = (cur[0] - start[0]) * in_per_tick
        dr = (cur[1] - start[1]) * in_per_tick
        return math.degrees((dr - dl) / track_width) / TURN_SCRUB_FACTOR
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
            if feedback_source == "imu":
                with link.lock:
                    current_yaw = link.orient.yaw
                turned_so_far = abs(current_yaw - start_yaw)
            else:
                turned_so_far = encoder_turn_deg()
                if turned_so_far is None:
                    abort_reason = ("an encoder stopped reporting mid-turn -- with no IMU "
                                    "there is nothing left to measure the turn, so it was "
                                    "cut short")
                    break
                turned_so_far = abs(turned_so_far)
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
        "actual_degrees_turned": (round(end_yaw - start_yaw, 1) if feedback_source == "imu"
                                  else None if enc_deg is None
                                  else round(enc_deg / TURN_SCRUB_FACTOR, 1)),
        "measurement_note": ("IMU gyro-integrated yaw" if feedback_source == "imu" else
                             "NO IMU: turned from the encoder differential divided by the "
                             "measured %.2fx in-place scrub factor. Scrub varies with the "
                             "surface, so treat this as approximate and check by eye."
                             % TURN_SCRUB_FACTOR),
        "imu_degrees": round(end_yaw - start_yaw, 1) if snap["imu_live"] else None,
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
    rear sweeps out along the path. The camera cannot see behind the robot, so only back
    up into space the operator has said is clear. Forward then reverse arcs of the same
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
    MAX_JOG_DURATION_S=10s and MAX_PWM_CEILING=125 per side.

    By default (warn_lights=True) the WLED strip does a 1.5 s warning pattern (on, off, on,
    off, on, solid green at 50%) right before the motors start, then is restored when the
    move ends; pass warn_lights=False to skip it. A strip that can't be reached never
    blocks the move.

    require_telemetry defaults to the server's sensor policy (set_required_sensors): it
    insists on live encoders only if "enc" is in the required set, which it is by default.
    Pass True or False to override for one call -- False ONLY for a bench test the operator
    is watching (e.g. robot on blocks while the encoder link is down): it still needs
    confirmed_safe, and the distances come back as null. False also relaxes the motion
    guard's encoder-fault latch for the duration of the move, since there are no encoders
    to fault on; whatever the latch was set to before is put back afterwards, so a jog
    never re-arms a check the operator switched off with set_guard().
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if not (0 < duration_s <= MAX_JOG_DURATION_S):
        return {"ok": False, "error": f"duration_s must be > 0 and <= {MAX_JOG_DURATION_S}"}

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
    # Bench mode (require_telemetry=false) has no encoders to fault on, so relax the
    # motion guard for the duration -- but only ever relax it, and put back exactly what
    # was there. Forcing it back to True on the way out used to silently undo an
    # operator's set_guard(encoders_required=false), re-arming the encoder-fault latch
    # behind their back on the next move.
    prior_encoders_required = link.motion.encoders_required
    link.motion.encoders_required = prior_encoders_required and bool(require_telemetry)
    try:
        while time.monotonic() - t_start < duration_s:
            link.send_drive(left_pwm, right_pwm)
            time.sleep(min(DRIVE_KEEPALIVE_S, max(0.0, duration_s - (time.monotonic() - t_start))))
    finally:
        link.motion.encoders_required = prior_encoders_required
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
    mouth closes; False starts the motors straight away. Capped at MAX_JOG_DURATION_S and MAX_PWM_CEILING."""
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if direction not in ("forward", "reverse"):
        return {"ok": False, "error": "direction must be 'forward' or 'reverse'"}
    if not (0 < pwm <= MAX_PWM_CEILING):
        return {"ok": False, "error": f"pwm must be 1-{MAX_PWM_CEILING}"}
    if not (0 < duration_s <= MAX_JOG_DURATION_S):
        return {"ok": False, "error": f"duration_s must be > 0 and <= {MAX_JOG_DURATION_S}"}
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


def _run_status(wait_s: float = 0.0) -> dict:
    """State of the current autonomous run (perform): running / the reason it ended, the latest
    info, and the result. wait_s (max 60) blocks until the run ends or that long passes."""
    if _run["running"] and wait_s > 0:
        _run["ended"].wait(min(wait_s, 60.0))
    return {"ok": True, "running": _run["running"], "status": _run["status"],
            "info": _run["info"], "result": _run["result"]}


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


PERFORM_MAX_S = 420.0   # one perform() call never runs longer than this, whatever the file says
PERFORM_FPS = 30
PERFORM_BLINK_S = 1.5   # the usual pre-move warning blink, before a dancing show's countdown


def _perform_loop(show, rend, dance, brightness, lead_in_s, start_s, end_s, audio, audio_offset_s,
                  require_telemetry, tmp_wav):
    """Runs on its own thread for the whole show (shares _run with the other autonomous runs:
    stop() / listen() abort it, the drive tools refuse while it runs). One loop at PERFORM_FPS
    paints the light bar from the show clock and, when dancing, re-sends the current beat's
    wheel command every DRIVE_KEEPALIVE_S. Always finishes with a stop."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    proc, reason, prev_cmd, step_k, brake_until = None, "finished", (0, 0), -1, 0.0
    last_send = last_check = last_info = 0.0
    lead_sec = None
    t = start_s
    lnk = link
    if lnk is not None and dance:
        lnk.motion.encoders_required = bool(require_telemetry)  # open-loop: encoders optional
    try:
        _mouth.acquire(brightness)
        if dance:
            _blink(brightness=min(brightness, 127))  # same warning the drive tools give
        wall0 = time.monotonic() + lead_in_s          # when show time == start_s
        a_pos0 = start_s - audio_offset_s             # where in the audio file that moment falls
        audio_wall = wall0 + max(0.0, -a_pos0)
        rend.seek(start_s)
        k0 = max(0, bisect.bisect_right(show.beats, start_s) - 1)
        while True:
            now = time.monotonic()
            t = start_s + (now - wall0)
            if _run["abort"].is_set():
                reason = "stopped"
                break
            if t >= end_s:
                break
            if audio and proc is None and now >= audio_wall:
                cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet"]
                if a_pos0 > 0:
                    cmd += ["-ss", f"{a_pos0:.3f}"]
                proc = subprocess.Popen(cmd + [audio], stdin=subprocess.DEVNULL)
            if t < start_s and int(wall0 - now) != lead_sec:  # lead-in: a white blip each second
                lead_sec = int(wall0 - now)
                rend.pulse(0.6)
            try:
                sock.sendto(rend.frame(t), (WLED_HOST, 21324))
            except OSError:
                pass
            if dance and lnk is not None:
                if require_telemetry and now - last_check > 0.5:
                    last_check = now
                    if not lnk.telemetry_snapshot()["telemetry_live"]:
                        reason = "telemetry_stale"
                        break
                if t >= start_s:
                    k = bisect.bisect_right(show.beats, t) - 1
                    if k >= 0:
                        name, left, right = dance.move(k - k0)
                    else:
                        name, left, right = "hold", 0, 0
                    if k != step_k:
                        step_k = k
                        if perform_show.reverses(prev_cmd, (left, right)):
                            brake_until = now + STOP_SETTLE_S  # no sign reversal without a stop
                    if now - last_send >= DRIVE_KEEPALIVE_S:
                        last_send = now
                        if now < brake_until:
                            lnk.send_stop()
                            prev_cmd = (0, 0)
                        else:
                            lnk.send_drive(left, right)
                            prev_cmd = (left, right)
            if now - last_info >= 1.0:
                last_info = now
                _run["info"] = {"t_s": round(t, 1), "of_s": round(end_s, 1),
                                   "beat": max(0, bisect.bisect_right(show.beats, t) - 1), "dancing": bool(dance)}
            time.sleep(max(0.0, 1.0 / PERFORM_FPS - (time.monotonic() - now)))
    except Exception as e:
        reason = f"error: {e}"
    finally:
        if lnk is not None and dance:
            lnk.motion.encoders_required = True
            try:
                lnk.send_stop()
            except Exception:
                pass
        if proc is not None and proc.poll() is None:
            proc.terminate()
        for _ in range(3):  # leave the bar dark, not frozen on the last frame
            try:
                sock.sendto(rend.dark(), (WLED_HOST, 21324))
            except OSError:
                pass
            time.sleep(1 / PERFORM_FPS)
        _mouth.release(2.6)
        if tmp_wav:
            try:
                os.unlink(tmp_wav)
            except OSError:
                pass
        _run["status"] = reason
        _run["result"] = dict(status=reason, ended_at_s=round(t, 1))
        _run["running"] = False
        _run["ended"].set()
        _speak_lock.release()


@mcp.tool()
def perform(midi_path: Optional[str] = None, bpm: Optional[float] = None,
            audio_path: Optional[str] = None, synth: bool = False, dance: bool = False,
            confirmed_safe: bool = False, dance_pwm: int = 60, spin_pwm: int = 85,
            brightness: int = 127, lead_in_s: float = 3.0, audio_offset_s: float = 0.0,
            start_s: float = 0.0, max_seconds: Optional[float] = None,
            duration_s: float = 60.0, require_telemetry: Optional[bool] = None) -> dict:
    """Light-bar show (and optionally a dance) timed to a song. Starts it and returns at once;
    follow with perform_status(wait_s=...) and end early with stop().

    Timing source (give one): midi_path = a .mid file, whose notes drive the bar (pitch = where
    along the bar a coloured spot lands, kick = burst from the middle, snare = flash at both
    ends, hats = sparkle) and whose tempo map gives the beat; or bpm (+ duration_s) = just
    pulse and dance on a steady beat, which is the safer choice when the recording you will
    play does not line up with any MIDI you have.
    Sound (all optional): audio_path = a recording (mp3/wav/anything ffplay plays) played from
    this computer's speakers in sync; synth=True = a small chiptune render of the MIDI instead
    (the robot "sings" the melody). With neither, start your own player at the end of the
    countdown. audio_offset_s = seconds the recording lags the MIDI/beat grid (+ = audio late;
    ffplay also takes a few tenths of a second to start, so expect to tune this once).
    Rehearse a piece of it with start_s / max_seconds. lead_in_s = countdown before the music
    (a white blip per second), after the green warning blink if dancing.

    dance=True also drives the wheels, open-loop, one move per beat (8-beat phrase: two arc
    sways, rock forward/back, spin each way; it nets out near where it started). That MOVES
    the robot, so it needs confirmed_safe=true after asking the operator, same as the drive
    tools -- say it is a dance, for how long, and at what dance_pwm / spin_pwm. Forward effort
    is touchy and spins need much more PWM (scrub), so they are separate. Each beat's command
    is re-sent every 0.1 s, a stop is inserted when a wheel has to reverse, and stop() /
    listen() ends the show with a stop. Lights-only (dance=False) needs no confirmation.
    brightness is the strip's master (0-255, default 127 = 50%). Capped at 420 s."""
    if not (midi_path or bpm):
        return {"ok": False, "error": "give midi_path (a .mid file) or bpm"}
    if midi_path and not Path(midi_path).expanduser().is_file():
        return {"ok": False, "error": f"no such file: {midi_path}"}
    if audio_path and not Path(audio_path).expanduser().is_file():
        return {"ok": False, "error": f"no such file: {audio_path}"}
    if synth and not midi_path:
        return {"ok": False, "error": "synth=True needs midi_path (nothing to render from a bare bpm)"}
    if synth and audio_path:
        return {"ok": False, "error": "pick one sound source: audio_path or synth"}
    if bpm is not None and not (40 <= bpm <= 240):
        return {"ok": False, "error": "bpm must be 40-240"}
    if not (0 <= brightness <= 255):
        return {"ok": False, "error": "brightness must be 0-255"}
    if not (0 <= lead_in_s <= 30):
        return {"ok": False, "error": "lead_in_s must be 0-30"}
    if not (0 <= dance_pwm <= MAX_PWM_CEILING and 0 <= spin_pwm <= MAX_PWM_CEILING):
        return {"ok": False, "error": f"dance_pwm / spin_pwm must be 0-{MAX_PWM_CEILING}"}
    if dance:
        if link is None:
            return _not_connected()
        gate = _confirm_gate(confirmed_safe)
        if gate:
            return gate
        snap = link.telemetry_snapshot()
        if require_telemetry is None:
            require_telemetry = "enc" in link.policy.required
        if require_telemetry and not snap["telemetry_live"]:
            return {"ok": False, "error": "no live encoder telemetry -- refusing to dance without it "
                    "(pass require_telemetry=false only for a bench run you are watching)",
                    "telemetry": snap}
    elif _run["running"]:
        return {"ok": False, "error": "an autonomous run / show is in progress; call stop() first"}
    try:
        show = (perform_show.Show.from_midi(str(Path(midi_path).expanduser())) if midi_path
                else perform_show.Show.from_bpm(bpm, duration_s))
    except Exception as e:
        return {"ok": False, "error": f"could not read the timing source: {e}"}
    if not (0 <= start_s < show.length_s):
        return {"ok": False, "error": f"start_s must be 0 to {show.length_s:.1f}"}
    end_s = min(show.length_s, start_s + PERFORM_MAX_S)
    if max_seconds is not None:
        end_s = min(end_s, start_s + max(1.0, max_seconds))
    tmp_wav = None
    audio = str(Path(audio_path).expanduser()) if audio_path else None
    try:
        if synth:
            tmp_wav = tempfile.NamedTemporaryFile(suffix=".wav", delete=False).name
            perform_show.synth_wav(show, tmp_wav)
            audio = tmp_wav
        rend = perform_show.LightRender(show)
        dancer = perform_show.Dance(pwm=dance_pwm, spin_pwm=spin_pwm, sign=FORWARD_PWM_SIGN) if dance else None
    except Exception as e:
        if tmp_wav:
            os.unlink(tmp_wav)
        return {"ok": False, "error": f"could not prepare the show: {e}"}
    if not _speak_lock.acquire(blocking=False):
        if tmp_wav:
            os.unlink(tmp_wav)
        return {"ok": False, "error": "the strip is busy (speaking or another show)"}
    _run["abort"].clear()
    _run["ended"].clear()
    _run.update(running=True, status="performing", info={}, result=None)
    threading.Thread(target=_perform_loop, daemon=True,
                     args=(show, rend, dancer, brightness, lead_in_s, start_s, end_s,
                           audio, audio_offset_s, require_telemetry, tmp_wav)).start()
    return {"ok": True, "started": True, "dancing": dance, "show_seconds": round(end_s - start_s, 1),
            "bpm": round(show.bpm_hint, 1) if show.bpm_hint else None, "beats": len(show.beats),
            "notes": len(show.notes), "pitch_range": [rend.lo, rend.hi],
            "music_starts_in_s": round(lead_in_s + (PERFORM_BLINK_S if dance else 0.0), 1),
            "audio": "synth" if synth else ("file" if audio_path else "none (start your own player)")}


@mcp.tool()
def perform_status(wait_s: float = 0.0) -> dict:
    """Read-only. State of the perform() show: running / the reason it ended (finished, stopped,
    telemetry_stale, error: ...), the show clock and beat while it runs. wait_s (max 60) blocks
    until it ends or that long passes. Never moves the robot."""
    return _run_status(wait_s)


if __name__ == "__main__":
    mcp.run()
