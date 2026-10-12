#!/usr/bin/env python3
"""MCP server that lets an AI agent drive eLlama over computer_bridge.ino, closing the
loop on the wheel encoders instead of driving open-loop for a fixed duration.

Talks over the same USB-serial link as calibration/live_view.py and toandfro.py did --
reuses calibration/bridge.py for serial I/O, line parsing and the distance/heading math,
so there is exactly one place that logic lives.

Read DRIVING_POLICY.md before calling anything here. The short version, enforced by this
file: every tool that can move the robot requires confirmed_safe=true, which must only be
set after asking the human operator in the moment -- this server has no obstacle sensors
(no lidar, no camera) wired in, only wheel encoders and an IMU, so there is no way for it
to "see" that the area is clear. Per the operator's own stated policy, a sensor-verified
small bump forward could skip the ask -- but that only applies once real proximity/vision
sensing is actually feeding this server, which is not the case yet. Until then, ask first,
every time, regardless of move size. First-time-testing-a-new-motor and full-power moves
require asking even after obstacle sensing exists.

Setup:
    pip install mcp pyserial
    python3 mcp_server/robot_server.py            # run directly, or register with an
                                                    # MCP client (see mcp_server/README.md)

Env:
    ELLAMA_SERIAL_PORT   explicit serial port (e.g. /dev/ttyUSB0); default auto-detects
                         via calibration/bridge.py's port picker.
    ELLAMA_CAM_URL       explicit IP Webcam base URL (e.g. https://192.168.0.50:4444);
                         default scans the local /24 subnet for the phone.
"""

import array
import atexit
import bisect
import functools
import json
import math
import os
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
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
from pathlib import Path
from typing import Literal, Optional

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
import bridge  # noqa: E402  (calibration/bridge.py)
import recording  # noqa: E402  (mcp_server/recording.py)
import line_vision  # noqa: E402  (mcp_server/line_vision.py)
import phone_display  # noqa: E402  (mcp_server/phone_display.py)
import person_track  # noqa: E402  (mcp_server/person_track.py)
import person_vision  # noqa: E402  (mcp_server/person_vision.py)
import camera_stream  # noqa: E402  (mcp_server/camera_stream.py)
import line_follow  # noqa: E402  (mcp_server/line_follow.py)
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

# Refuse to drive if the last encoder packet is older than this -- driving without live
# feedback is exactly the open-loop, no-undo situation DRIVING_POLICY.md warns about.
TELEMETRY_STALE_S = 2.0

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
        self.lock = threading.Lock()
        self.recorder = recording.Recorder(Path(__file__).resolve().parent.parent / "recordings")
        self.video = None  # recording.VideoRecorder while a recording has video on
        self.last_enc_wall = None  # time.monotonic() of the last ENC packet seen
        self.last_imu = None
        self.last_imu_wall = None  # time.monotonic() of the last IMU packet seen

        # Serial writes come from tool threads AND the pump (the tilt trip sends "stop" from
        # there, ~20 ms after the offending IMU sample), so a line must never interleave.
        self._wlock = threading.RLock()
        self.tilt = TILT  # module-level: a latched fault and the level reference survive connect()
        self.motion = MOTION  # likewise: connect() must not wipe a latched motion fault
        self.on_tilt_trip = None  # callable(fault_dict); set by the server to abort autonomous runs

        self._stop_pump = threading.Event()
        self._pump_thread = threading.Thread(target=self._pump, daemon=True)
        self._pump_thread.start()

    def _pump(self):
        """Drains the serial reader queue into enc/orient state, forever. This is the only
        thread that mutates self.enc / self.orient -- tools only ever read them."""
        while not self._stop_pump.is_set():
            try:
                raw = self.reader.q.get(timeout=0.2)
            except Exception:
                continue
            kind, payload = bridge.parse_line(raw)
            tripped = None
            with self.lock:
                if kind == "ENC":
                    left, right, ms = payload
                    sl, sr = self.enc.update(left, right, ms)
                    self.last_enc_wall = time.monotonic()
                    self.recorder.on_enc(self.enc, self.orient, sl, sr)
                    if sl is not None and sr is not None:
                        tripped = self.motion.on_enc(sl, sr, self.last_enc_wall)
                elif kind == "CMD":
                    self.recorder.on_cmd(*payload)  # the stick's command, overheard by the bridge
                elif kind == "IMU":
                    ax, ay, az, gx, gy, gz, ms = payload
                    agx, agy, agz, dx, dy, dz = self.orient.update(ax, ay, az, gx, gy, gz, ms)
                    self.recorder.on_imu(self.enc, self.orient, (agx, agy, agz))
                    self.last_imu = dict(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, ms=ms)
                    self.last_imu_wall = time.monotonic()
                    tripped = self.tilt.update(self.orient.pitch, self.orient.roll, ms,
                                               gyro_dps=math.hypot(dx, dy))
                    mf = self.motion.on_imu(dz, agx, agy, self.last_imu_wall)
                    tripped = tripped or mf
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

    def _tilt_refusal(self):
        """Why a nonzero drive command must not go out right now, or None -- covers both
        guards (name kept; see _guard_info). Fail-closed: no verdict from the IMU is a "no",
        not a "probably fine"."""
        m = self.motion
        if m.mode != "off" and m.tripped:
            return "motion guard tripped: " + m.fault["reason"]
        t = self.tilt
        if not t.enabled:
            return None
        if t.tripped:
            return "tilt guard tripped: " + t.fault["reason"]
        if not t.ready:
            return "tilt guard has no level reference yet (needs ~0.5 s of a still, live IMU)"
        age = None if self.last_imu_wall is None else time.monotonic() - self.last_imu_wall
        if age is None or age > IMU_STALE_FAULT_S:
            return "tilt guard has no live IMU (last sample %s ago) -- refusing to drive blind" % (
                "never" if age is None else "%.1fs" % age)
        return None

    def send_drive(self, left, right):
        left = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(left)))
        right = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(right)))
        with self._wlock:  # check and write atomically vs. the trip's "stop" (also under it)
            if left or right:  # a zero command is always allowed -- it's how loops stop
                why = self._tilt_refusal()
                if why is not None:
                    self.emergency_stop()
                    raise GuardFault(why, self._guard_info())
            self.motion.on_command(left, right, time.monotonic())
            self.send_line(f"{left} {right}")

    def _guard_info(self):
        return {"tilt": self.tilt.snapshot(), "motion": self.motion.snapshot()}

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
        with self.lock:
            return self.enc.last  # (left, right) ticks, or None if nothing seen yet

    def telemetry_snapshot(self):
        with self.lock:
            age = None if self.last_enc_wall is None else time.monotonic() - self.last_enc_wall
            imu_age = None if self.last_imu_wall is None else time.monotonic() - self.last_imu_wall
            return dict(
                left_ticks=self.enc.last[0] if self.enc.last else None,
                right_ticks=self.enc.last[1] if self.enc.last else None,
                dist_l_in=round(self.enc.dist_l, 3),
                dist_r_in=round(self.enc.dist_r, 3),
                dist_avg_in=round(self.enc.dist_avg, 3),
                heading_deg=round(self.enc.heading_deg, 2),
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
    autonomous run (line-follow / turn-to-me / perform). Blocking tools notice on their next
    send_drive, which raises TiltFault."""
    _follow["abort"].set()


def _shutdown():
    _follow["abort"].set()
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
CAM_TIMEOUT_S = 20
CAM_MAX_WIDTH = 1280

# auto_photo (start_recording): take a photo each time the robot comes to rest after moving.
AUTO_PHOTO_STILL_S = 0.7          # both wheels below STILL_SPEED_IN_S for this long = stopped
AUTO_PHOTO_STILL_SPEED_IN_S = 0.5
AUTO_PHOTO_MIN_TRAVEL_IN = 4.0    # summed |left|+|right| wheel travel since the last photo
AUTO_PHOTO_MIN_TURN_DEG = 10.0    # or this much IMU yaw change
AUTO_PHOTO_POLL_S = 0.1
_cam = None  # cached (base_url, snapshot_path) once found
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


def _probe(base):
    """The snapshot path that returns a real JPEG on base, else None."""
    for path in CAM_SNAPSHOT_PATHS:
        try:
            with _cam_open(base + path, CAM_TIMEOUT_S) as r:
                if r.read(2) == b"\xff\xd8":
                    return path
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


def _camera(rediscover=False):
    """(base_url, snapshot_path), discovered once and cached."""
    global _cam
    with _cam_lock:
        if rediscover:
            _cam = None
        if _cam is None:
            explicit = os.environ.get("ELLAMA_CAM_URL", "").rstrip("/")
            if explicit:
                _cam = (explicit, _probe(explicit) or CAM_SNAPSHOT_PATHS[0])
            else:
                _cam = _discover_camera()
        return _cam


def _fetch_snapshot():
    """GET a JPEG from the phone; if the cached address has gone stale (phone got a new
    DHCP lease), rediscover once and retry."""
    last = None
    for attempt in (0, 1):
        try:
            base, path = _camera(rediscover=attempt == 1)
            with _cam_open(base + path, CAM_TIMEOUT_S) as r:
                return r.read()
        except Exception as e:
            last = e
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


_follow = {"running": False, "abort": threading.Event(), "status": "idle", "info": {}, "result": None,
           "ended": threading.Event()}
_frames = None  # camera_stream.FrameSource, created on first use

FOLLOW_MAX_CRUISE_PWM = line_follow.F_MAX  # start_line_follow's speed_pwm cap. Forward effort is touchy, so
                                           # it stays modest; turns may go to TURN_OUTER_MAX / T_SPIN_MAX


def _phone_display_url():
    """Where phone/display_server.py listens: ELLAMA_PHONE_DISPLAY, else port 8080 on the phone
    that serves the camera."""
    explicit = os.environ.get("ELLAMA_PHONE_DISPLAY", "").rstrip("/")
    if explicit:
        return explicit
    from urllib.parse import urlparse
    return f"http://{urlparse(_camera()[0]).hostname}:8080"


def _frame_source():
    global _frames
    if _frames is None:
        _frames = camera_stream.FrameSource(_open_stream)
    _frames.start()
    return _frames


mcp = FastMCP("ellama-robot")


def _not_connected():
    return {"ok": False, "error": f"not connected to the robot bridge: {_connect_error}"}


def _tilt_gate():
    """Early, readable refusal for any motion tool; send_drive enforces the same thing as the
    last line of defence."""
    why = link._tilt_refusal() if link is not None else None
    if why is None:
        return None
    if link.motion.tripped and link.motion.mode != "off":
        hint = ("look at the robot first -- something stopped it (see fault) -- then "
                "clear_motion_fault(confirmed_safe=true) after asking the operator")
    elif link.tilt.tripped or not link.tilt.ready:
        hint = ("if the robot is upright and still: clear_tilt_fault(confirmed_safe=true) after "
                "asking the operator; if there is no level reference yet, wait a second with "
                "the IMU live, or call set_level_reference")
    else:
        hint = "check the IMU board's power / link, then connect()"
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
    if _follow["running"]:
        return {"ok": False, "error": "an autonomous run (line-follow / turn-to-me) is in progress; call stop() first"}
    if confirmed_safe:
        return None
    return {
        "ok": False,
        "error": (
            "confirmed_safe must be true, and only after asking the human operator in "
            "this specific moment -- this server has no obstacle sensors, so there is no "
            "way to verify the area is clear on its own. See DRIVING_POLICY.md. State the "
            "exact move (distance/degrees, direction, PWM) and wait for a go-ahead before "
            "calling again with confirmed_safe=true."
        ),
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
    _follow["abort"].set()
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
    _follow["abort"].set()
    if link is None:
        return _not_connected()
    link.send_listen()
    return {"ok": True, **link.telemetry_snapshot()}


@mcp.tool()
def look(width: int = 640):
    """Read-only. Take a photo with the phone camera mounted on the robot and return it, so
    you can see what the robot currently sees. Never moves the robot, no confirmation
    needed. The phone is found automatically on the local network (or via ELLAMA_CAM_URL).
    Slow: expect ~5-8 seconds. `width` downsizes the image (default 640, max 1280) to keep
    it cheap. This is advisory vision only -- a JPEG glance is not an obstacle sensor, so it
    does NOT replace asking the operator before any move."""
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


@mcp.tool()
def line_view(width: int = 640):
    """Read-only. Take a phone-camera photo, find the dark tape line in it, and return the
    photo with the detection drawn on (red = dark pixels, green dots = line center per band
    with its lateral offset in inches) plus the numbers: `offset_in` (+ = line is right of
    the robot's center, nearest band), `lean_deg` (+ = line leans right going away),
    `lookahead_offset_in` (farthest band) and `found`. Offsets use the known 3/4in tape
    width as the scale, so they're approximate. Never moves the robot, no confirmation
    needed; same ~5-8s snapshot as look()."""
    width = max(64, min(int(width), CAM_MAX_WIDTH))
    try:
        import cv2
        import numpy as np
        bgr = cv2.imdecode(np.frombuffer(_fetch_snapshot(), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return {"ok": False, "error": "camera returned something that isn't an image"}
        if bgr.shape[1] > width:
            bgr = cv2.resize(bgr, (width, round(bgr.shape[0] * width / bgr.shape[1])))
        result, annotated = line_vision.analyze(bgr)
        ok, jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
    except Exception as e:
        return {"ok": False, "error": str(e)}
    import json
    return [json.dumps(result), Image(data=jpg.tobytes(), format="jpeg")]


@mcp.tool()
def person_view():
    """Read-only. Grab the newest live camera frame, find people in it, and return it with
    the detection drawn on (green = the largest person, orange = others; vertical line = their
    horizontal center) plus the numbers: `n_people`, and per person `bearing_deg` (+ = to the
    RIGHT of where the robot points) and `x_frac`. Legs-only views are fine -- every visible
    keypoint votes for the center. Never moves the robot, no confirmation needed. This is
    what turn_to_me would act on, so use it to check the detection first."""
    try:
        import cv2
        src = _frame_source()
        t0 = time.monotonic()
        while src.latest() is None and time.monotonic() - t0 < 10:
            time.sleep(0.05)
        fr = src.latest()
        if fr is None or src.age_s() > 1.0:
            return {"ok": False, "error": "no live camera frames", "camera": src.status()}
        result, annotated = person_vision.analyze(fr.img)
        ok, jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
    except Exception as e:
        return {"ok": False, "error": str(e)}
    import json
    return [json.dumps(result), Image(data=jpg.tobytes(), format="jpeg")]


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
    warn_lights: bool = True,
) -> dict:
    """Closed-loop straight-line move: drives at `pwm` (0-255, clamped to
    MAX_PWM_CEILING=125) until the wheel encoders report `distance_inches` traveled, then
    stops. Requires a live encoder feed (refuses otherwise) and confirmed_safe=true, set
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

    snap = link.telemetry_snapshot()
    if not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback",
            "telemetry": snap,
        }

    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    start = link.last_counts()
    ticks_per_inch = 1.0 / link.in_per_tick()
    target_ticks = distance_inches * ticks_per_inch
    signed_pwm = pwm * (FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN)
    timeout_s = max(4.0, distance_inches * 0.5 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    try:
        link.send_drive(signed_pwm, signed_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                link.send_drive(signed_pwm, signed_pwm)
                last_send = time.monotonic()
            cur = link.last_counts()
            traveled_ticks = (abs(cur[0] - start[0]) + abs(cur[1] - start[1])) / 2.0
            if traveled_ticks >= target_ticks:
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
    actual_inches = (abs(end[0] - start[0]) + abs(end[1] - start[1])) / 2.0 * link.in_per_tick()

    return {
        "ok": not hit_timeout,
        "requested_inches": distance_inches,
        "direction": direction,
        "pwm_used": pwm,
        "actual_inches_traveled": round(actual_inches, 2),
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
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
    if not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback",
            "telemetry": snap,
        }
    if not snap["imu_live"]:
        return {
            "ok": False,
            "error": "no live IMU telemetry -- turn_degrees needs it as the ground truth "
            "for rotation (wheel encoders overestimate in-place turns due to scrub)",
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
    # "left" (CCW, nose swings left) needs the right wheel driving real-forward and the
    # left wheel driving real-backward -- both relative to FORWARD_PWM_SIGN, not raw PWM
    # sign, the same correction drive_distance applies.
    sign = 1 if direction == "left" else -1
    left_pwm = -sign * FORWARD_PWM_SIGN * pwm
    right_pwm = sign * FORWARD_PWM_SIGN * pwm
    timeout_s = max(4.0, degrees * 0.1 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    try:
        link.send_drive(left_pwm, right_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                link.send_drive(left_pwm, right_pwm)
                last_send = time.monotonic()
            with link.lock:
                current_yaw = link.orient.yaw
            if abs(current_yaw - start_yaw) >= degrees:
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
    dl = (end[0] - start[0]) * in_per_tick
    dr = (end[1] - start[1]) * in_per_tick
    enc_deg = math.degrees((dr - dl) / track_width)

    return {
        "ok": not hit_timeout,
        "requested_degrees": degrees,
        "direction": direction,
        "pwm_used": pwm,
        "actual_degrees_turned": round(end_yaw - start_yaw, 1),
        "encoder_degrees_estimate": round(enc_deg, 1),
        "encoder_estimate_unreliable": "wheel scrub during in-place turns inflates this "
        "~2x vs. the IMU/real rotation -- see this tool's docstring",
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
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
        return {"ok": False, "error": "no live encoder telemetry -- refusing to drive "
                "without feedback", "telemetry": snap}
    if not snap["imu_live"]:
        return {"ok": False, "error": "no live IMU telemetry -- drive_arc needs it to "
                "measure how far around the arc it has gone", "telemetry": snap}

    wheel_cap = min(abs(int(max_wheel_pwm)), MAX_PWM_CEILING)
    pwm = min(abs(int(pwm)), wheel_cap)
    if pwm == 0:
        return {"ok": False, "error": "pwm and max_wheel_pwm must be > 0"}

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = link.in_per_tick()
    half_track = link.cfg["track_width_in"] / 2.0
    diff = pwm * min(1.0, half_track / radius_inches)
    target_curv = 1.0 / radius_inches  # rad per inch of path

    travel_sign = 1 if travel == "forward" else -1

    def forward_path_in(a, b):
        # Signed along the direction of travel, so a wheel running against it (the inner
        # wheel of a tight arc) subtracts from the path instead of adding.
        l_fwd = (b[0] - a[0]) * ENCODER_FORWARD_SIGN[0]
        r_fwd = (b[1] - a[1]) * ENCODER_FORWARD_SIGN[1]
        return travel_sign * (l_fwd + r_fwd) / 2.0 * in_per_tick

    max_diff = pwm + ARC_MAX_INNER_REVERSE_PWM

    def wheel_cmd(d):
        outer = min(wheel_cap, pwm + d)
        inner = max(-ARC_MAX_INNER_REVERSE_PWM, pwm - d)
        # Forward: "left" = right wheel on the outside, matching turn_degrees' mapping.
        # Reverse: heading turns the same way, so the outside wheel swaps sides.
        outer_is_right = (direction == "left") == (travel == "forward")
        l_real, r_real = (inner, outer) if outer_is_right else (outer, inner)
        return (FORWARD_PWM_SIGN * travel_sign * l_real,
                FORWARD_PWM_SIGN * travel_sign * r_real)

    start = link.last_counts()
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
                cur = link.last_counts()
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

    end = link.last_counts()
    with link.lock:
        end_yaw = link.orient.yaw
    path_in = forward_path_in(start, end)
    turned = end_yaw - start_yaw
    eff_radius = path_in / math.radians(abs(turned)) if abs(turned) > 1 else None
    final_l, final_r = wheel_cmd(diff)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_radius_in": radius_inches,
        "requested_degrees": degrees,
        "direction": direction,
        "travel": travel,
        "pwm_used": pwm,
        "actual_degrees_turned": round(turned, 1),
        "path_length_in": round(path_in, 2),
        "effective_radius_in": None if eff_radius is None else round(eff_radius, 2),
        "final_wheel_pwm": {"left": round(final_l), "right": round(final_r)},
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "telemetry": link.telemetry_snapshot(),
    }


@mcp.tool()
@_tilt_guarded
def jog(left_pwm: int, right_pwm: int, duration_s: float, confirmed_safe: bool,
        warn_lights: bool = True, require_telemetry: bool = True) -> dict:
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

    require_telemetry=True (default) refuses to run without live encoder data. Pass False
    ONLY for a bench test the operator is watching (e.g. robot on blocks while the encoder
    link is down): it still needs confirmed_safe, and the distances come back as null.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if not (0 < duration_s <= MAX_JOG_DURATION_S):
        return {"ok": False, "error": f"duration_s must be > 0 and <= {MAX_JOG_DURATION_S}"}

    snap = link.telemetry_snapshot()
    if require_telemetry and not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback",
            "telemetry": snap,
        }

    prior_lights = _warn_blink(warn_lights)
    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    start = link.last_counts()
    t_start = time.monotonic()
    link.motion.encoders_required = bool(require_telemetry)  # bench mode: no encoders to fault on
    try:
        while time.monotonic() - t_start < duration_s:
            link.send_drive(left_pwm, right_pwm)
            time.sleep(min(DRIVE_KEEPALIVE_S, max(0.0, duration_s - (time.monotonic() - t_start))))
    finally:
        link.motion.encoders_required = True
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
                     warn_lights: bool = True, require_telemetry: bool = True) -> dict:
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
    if require_telemetry and not link.telemetry_snapshot()["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry -- refusing to drive without feedback"}
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


def _follow_hud(cmd, src, t_s):
    """What the phone display's command panel shows for one line-follow command."""
    i = cmd.info
    lines = []
    if "phi_deg" in i:
        lines.append(f"aim {i['phi_deg']:+.1f} deg   offset {i.get('e_in', 0):+.1f} in")
        lines.append(f"forward {i.get('f', 0):+d}   turn {i.get('t', 0):+d}")
    lines.append(f"bands {i.get('bands', '-')}   {src.fps():.0f} fps")
    return dict(left=cmd.left_raw * FORWARD_PWM_SIGN, right=cmd.right_raw * FORWARD_PWM_SIGN,
                status=cmd.status, spin=bool(i.get("spin")), t_s=t_s, lines=lines)


def _follow_loop(lnk, src, ctl, rec_dir, t_rec, display=None):
    """Runs on its own thread for the whole run: newest frame -> detect -> controller -> wheels.
    Re-sends the last command at least every DRIVE_KEEPALIVE_S (the bridge goes listen-only
    after 500ms of silence) and ends the moment the controller says so, the operator calls
    stop(), or encoder telemetry dies. Always finishes with a stop."""
    import csv
    import cv2
    log = open(rec_dir / "follow_log.csv", "w", newline="")
    lw = csv.writer(log)
    lw.writerow(["time_s", "frame", "found", "n_bands", "e_near_in", "phi_deg", "spin", "f", "t",
                 "left_raw", "right_raw", "status"])
    times = open(rec_dir / "video_times.csv", "w", newline="")
    tw = csv.writer(times)
    tw.writerow(["frame", "time_s"])
    writer = None
    prev_x, last_seq, last_send = None, -1, 0.0
    cmd = line_follow.Command()
    send_now = False
    reason = None
    shown = None
    lnk.motion.encoders_required = False  # camera-only: a dead encoder is logged, not a stop
    try:
        ctl.start(time.monotonic())
        while True:
            now = time.monotonic()
            if _follow["abort"].is_set():
                ctl.stop()
                reason = "stopped"
                break
            # Heartbeat is the IMU stream, not the encoders: the follower is camera-only, so a
            # dead encoder must not stop a run (the board link going quiet still does).
            snap_age = lnk.telemetry_snapshot()["imu_age_s"]
            if snap_age is None or snap_age > TELEMETRY_STALE_S:
                ctl.status = "telemetry_stale"
                reason = "telemetry_stale"
                break
            fr = src.latest()
            if fr is not None and fr.seq != last_seq:
                last_seq = fr.seq
                res, ann = line_vision.analyze(fr.img, prev_x_frac=prev_x)
                prev_x = res.get("x_frac") if res["found"] else None
                cmd = ctl.update(res, fr.t)
                if display is not None:
                    shown = (ann, _follow_hud(cmd, src, t_rec()))
                    display.push(*shown)
                t = t_rec()
                if writer is None:
                    h, w = fr.img.shape[:2]
                    writer = cv2.VideoWriter(str(rec_dir / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
                writer.write(fr.img)
                tw.writerow([fr.seq, f"{t:.3f}"])
                i = cmd.info
                lw.writerow([f"{t:.3f}", fr.seq, int(res["found"]), res["n_bands"], i.get("e_in", ""),
                             i.get("phi_deg", ""), int(bool(i.get("spin", False))), i.get("f", ""),
                             i.get("t", ""), cmd.left_raw, cmd.right_raw, cmd.status])
                _follow["info"] = dict(i, status=cmd.status, fps=round(src.fps(), 1))
                log.flush()
                times.flush()   # a crash mid-run must not lose the data that explains it
                send_now = True
            else:
                cmd = ctl.check(now)
            if cmd.status != "running":
                reason = cmd.status
                break
            if send_now or now - last_send >= DRIVE_KEEPALIVE_S:
                if _follow["abort"].is_set():   # stop() may have landed since the check above
                    continue
                lnk.send_drive(cmd.left_raw, cmd.right_raw)
                last_send, send_now = now, False
            time.sleep(0.01)
    except Exception as e:
        reason = f"error: {e}"
    finally:
        lnk.motion.encoders_required = True
        try:
            lnk.send_stop()
        except Exception:
            pass
        if writer is not None:
            writer.release()
        log.close()
        times.close()
        summary = lnk.recorder.stop()
        if display is not None:
            try:
                display.close(None if shown is None else (shown[0], dict(shown[1], status=reason or "ended", spin=False)))
            except Exception:
                pass
        _follow["status"] = reason or "ended"
        _follow["result"] = dict(status=_follow["status"], frames=ctl.frames, dir=str(rec_dir),
                                 duration_s=None if summary is None else summary["duration_s"],
                                 last=_follow["info"])
        _follow["running"] = False
        _follow["ended"].set()


@mcp.tool()
def start_line_follow(confirmed_safe: bool, speed_pwm: int = 58, max_seconds: float = 90.0,
                      name: Optional[str] = None, show_on_phone: bool = False,
                      ignore_finish_s: float = 0.0) -> dict:
    """Start following the tape line with the phone camera, on its own thread, and return at
    once; poll line_follow_status(wait_s=...) for the end. ONE confirmed_safe covers the whole
    run: set it only after telling the operator the exact run (speed_pwm, max_seconds, that it
    stops if the line is lost / the finish strip is seen / camera or telemetry go stale) and
    getting a go-ahead. The line must already be in view (>=3 bands) and the finish strip not.

    It stops by itself and reports why: finish_candidate (something like the end strip is
    reached -- may be paint or glare), end_of_tape (the line vanished right after a cross strip
    was seen), lost (no line for 0.5s), stale (no camera frame for 0.5s), telemetry_stale,
    timeout, stopped (stop() or listen() was called -- call stop() at any time to abort).
    The reason is a CLAIM, not a verdict: call line_follow_report and look at the images before
    deciding what it was. No searching for a lost line.
    Records telemetry, video, and a per-frame follow_log.csv to recordings/<name>/.
    speed_pwm is the cruise forward effort, 50-64 (default 58; forward effort is touchy, so
    it is capped low); turns may go up to 100 on the outer wheel, and a spin that is not
    getting anywhere is boosted up to 85.

    show_on_phone (default False): also show the line-detection overlay with a motor-command
    panel (wheel bars, aim angle, status) live on the phone's screen. Needs
    phone/display_server.py running in Termux and show.html open in the phone's browser (see
    phone/README.md); if the phone can't be reached the run still starts and the result says
    so. Purely cosmetic: it can't affect the run.

    ignore_finish_s (default 0, max 30): for resuming mid-course after you've judged a stop
    was a FALSE finish (chipped paint, glare mistaken for the finish strip). For this many
    seconds after the start, a finish strip in view doesn't block the start and doesn't end
    the run. Lost-line, stale and timeout stops still apply. Leave at 0 for a normal start."""
    if link is None:
        return _not_connected()
    if not (0 <= ignore_finish_s <= 30):
        return {"ok": False, "error": "ignore_finish_s must be 0-30"}
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if link.recorder.active:
        return {"ok": False, "error": "a recording is already active; stop_recording() first"}
    if not (line_follow.F_MOVE_MIN <= speed_pwm <= FOLLOW_MAX_CRUISE_PWM):
        return {"ok": False, "error": f"speed_pwm must be {line_follow.F_MOVE_MIN}-{FOLLOW_MAX_CRUISE_PWM}"}
    if not (0 < max_seconds <= 300):
        return {"ok": False, "error": "max_seconds must be in (0, 300]"}
    snap = link.telemetry_snapshot()
    if not snap["imu_live"]:   # camera-only follower: encoders not required
        return {"ok": False, "error": "no live IMU telemetry (board link down?)", "telemetry": snap}
    src = _frame_source()
    t0 = time.monotonic()
    while src.latest() is None and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    fr = src.latest()
    if fr is None or src.age_s() > 1.0:
        return {"ok": False, "error": "no live camera frames", "camera": src.status()}
    # A stream that just (re)opened delivers its first frames late and uneven, which trips
    # the follower's stale-frame stop within half a second. Let it settle first.
    while not src.is_warm() and time.monotonic() - t0 < 12:
        time.sleep(0.1)
    if not src.is_warm():
        return {"ok": False, "error": "camera stream not steady yet (needs ~20 frames at >=10 fps "
                "with no gaps over 0.3s); try again in a few seconds", "camera": src.status()}
    fr = src.latest()
    res, _ = line_vision.analyze(fr.img)
    ctl = line_follow.LineFollower(max_run_s=max_seconds, forward_pwm_sign=FORWARD_PWM_SIGN,
                                   f_cruise=speed_pwm, ignore_finish_s=ignore_finish_s)
    ok, why = ctl.can_start(res)
    if not ok:
        return {"ok": False, "error": why}
    try:
        path = link.recorder.start(name or time.strftime("follow_%Y%m%d_%H%M%S"))
    except Exception as e:
        return {"ok": False, "error": str(e)}
    display, display_info = None, None
    if show_on_phone:
        try:
            display = phone_display.PhoneDisplay(_phone_display_url())
            ok, err = display.check()
            display_info = {"ok": ok, "url": display.url, **({"error": err} if err else {})}
            if ok:
                display.start()
            else:
                display = None
        except Exception as e:
            display, display_info = None, {"ok": False, "error": str(e)}
    _follow["abort"].clear()
    _follow["ended"].clear()
    _follow.update(running=True, status="running", info={}, result=None)
    threading.Thread(target=_follow_loop, daemon=True,
                     args=(link, src, ctl, link.recorder.dir, link.recorder.now, display)).start()
    out = {"ok": True, "started": True, "dir": str(link.recorder.dir), "speed_pwm": speed_pwm,
           "max_seconds": max_seconds, "bands_at_start": res["n_bands"], "camera": src.status()}
    if display_info is not None:
        out["phone_display"] = display_info
    return out


@mcp.tool()
def line_follow_status(wait_s: float = 0.0) -> dict:
    """Read-only. State of the line-follow run: running / the reason it ended (finish_candidate,
    end_of_tape, lost, stale, telemetry_stale, timeout, stopped, error: ...; these are claims --
    verify with line_follow_report), the latest offsets and wheel efforts (info.boost > 0 means
    the stall check is adding spin effort), and the recording folder. wait_s (max 60) blocks until the run ends or that long
    passes -- use it instead of polling in a tight loop. Never moves the robot."""
    if _follow["running"] and wait_s > 0:
        _follow["ended"].wait(min(wait_s, 60.0))
    return {"ok": True, "running": _follow["running"], "status": _follow["status"],
            "info": _follow["info"], "result": _follow["result"],
            "camera": None if _frames is None else _frames.status()}


@mcp.tool()
def line_follow_report(run: Optional[str] = None, n_rows: int = 10, n_frames: int = 6,
                       raw: bool = True):
    """Read-only. Everything needed to judge why a line-follow run stopped, in one call:
    the end reason, the last `n_rows` rows of the run's follow_log.csv (offset, aim angle,
    bands, spin, wheel efforts), a contact sheet of the last `n_frames` video frames with the
    detection drawn on (oldest at left; red = dark pixels, green dots = line), and a fresh
    full-resolution still of what the robot sees right now with the same overlay. With
    `raw` (default true) it also returns the same frames and the same still WITHOUT any
    overlay, because the red tint can hide small things (a paint chip, a glare edge, a thin
    crossbar): the images come back in the order annotated sheet, raw sheet, annotated still,
    raw still. `run` is a recordings/ folder name (default: the latest line-follow run). Use it
    after any stop to tell a real finish from a false one (paint, glare), and to see which side
    the line left on. Never moves the robot, no confirmation needed; the still takes ~5-8s."""
    import csv
    import json
    try:
        import cv2
        import numpy as np
        rec_root = Path(__file__).resolve().parent.parent / "recordings"
        if run:
            d = rec_root / run
        else:
            logs = sorted(rec_root.glob("*/follow_log.csv"), key=lambda q: q.stat().st_mtime)
            if not logs:
                return {"ok": False, "error": "no line-follow runs recorded yet"}
            d = logs[-1].parent
        if not (d / "follow_log.csv").exists():
            return {"ok": False, "error": f"no follow_log.csv in {d}"}
        rows = list(csv.DictReader(open(d / "follow_log.csv")))
        n_rows, n_frames = max(1, min(n_rows, 40)), max(1, min(n_frames, 12))
        last_seen = next((r for r in reversed(rows) if r["found"] == "1"), None)
        summary = {"ok": True, "dir": str(d), "running": _follow["running"],
                   "status": _follow["status"] if _follow["running"] else (
                       (_follow["result"] or {}).get("status") if _follow["result"] else None),
                   "rows_total": len(rows), "last_rows": rows[-n_rows:],
                   "last_line_seen": None if last_seen is None else dict(
                       time_s=last_seen["time_s"], e_near_in=last_seen["e_near_in"],
                       phi_deg=last_seen["phi_deg"],
                       side="right" if float(last_seen["phi_deg"] or 0) > 0 else "left")}
        out = [json.dumps(summary)]
        cap = cv2.VideoCapture(str(d / "video.mp4"))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        tiles, raw_tiles = [], []
        for i in range(max(0, n - n_frames), n):
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, f = cap.read()
            if not ok:
                continue
            _, ann = line_vision.analyze(f)
            for dst, img in ((tiles, ann), (raw_tiles, f)):
                img = cv2.resize(img, (240, round(img.shape[0] * 240 / img.shape[1])))
                cv2.putText(img, str(i), (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                dst.append(img)
        for sheet in ((tiles, raw_tiles) if raw else (tiles,)):
            if sheet:
                hmax = max(t.shape[0] for t in sheet)
                sheet = [cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in sheet]
                ok, jpg = cv2.imencode(".jpg", np.hstack(sheet), [cv2.IMWRITE_JPEG_QUALITY, 80])
                out.append(Image(data=jpg.tobytes(), format="jpeg"))
        try:
            bgr = cv2.imdecode(np.frombuffer(_fetch_snapshot(), np.uint8), cv2.IMREAD_COLOR)
            bgr = cv2.resize(bgr, (640, round(bgr.shape[0] * 640 / bgr.shape[1])))
            res, ann = line_vision.analyze(bgr)
            ok, jpg = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, 80])
            out[0] = json.dumps({**summary, "still_detection": res})
            out.append(Image(data=jpg.tobytes(), format="jpeg"))
            if raw:
                ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                out.append(Image(data=jpg.tobytes(), format="jpeg"))
        except Exception as e:
            out[0] = json.dumps({**summary, "still_error": str(e)})
        return out
    except Exception as e:
        return {"ok": False, "error": str(e)}


TURN_MAX_SECONDS = 3600.0
TURN_SPIN_DPS_RANGE = (5.0, 60.0)


def _check_turn_params(max_seconds, lost_timeout_s, spin_speed_dps):
    """Error string if any turn_to_me setting is out of range (None values are skipped)."""
    if max_seconds is not None and not (0 <= max_seconds <= TURN_MAX_SECONDS):
        return f"max_seconds must be 0 (no limit) to {TURN_MAX_SECONDS:.0f}"
    if lost_timeout_s is not None and not (0 <= lost_timeout_s <= 600):
        return "lost_timeout_s must be 0 (never end when the person is gone) to 600"
    if spin_speed_dps is not None and not (TURN_SPIN_DPS_RANGE[0] <= spin_speed_dps <= TURN_SPIN_DPS_RANGE[1]):
        return f"spin_speed_dps must be {TURN_SPIN_DPS_RANGE[0]:.0f}-{TURN_SPIN_DPS_RANGE[1]:.0f}"
    return None


def _person_loop(lnk, src, trk, rec_dir, t_rec):
    """Runs on its own thread for the whole turn-to-me run: newest frame -> person detector ->
    tracker (sets a world-frame yaw target) and, every ~10ms tick, IMU yaw -> spin command.
    Shares the _follow state with the line follower (one autonomous run at a time, stop() /
    listen() abort it). Re-sends the last command at least every DRIVE_KEEPALIVE_S and always
    finishes with a stop."""
    import csv
    import cv2
    log = open(rec_dir / "turn_log.csv", "w", newline="")
    lw = csv.writer(log)
    lw.writerow(["time_s", "frame", "n_people", "bearing_deg", "yaw", "target_yaw", "err_deg", "spin",
                 "t", "left_raw", "right_raw", "status"])
    times = open(rec_dir / "video_times.csv", "w", newline="")
    tw = csv.writer(times)
    tw.writerow(["frame", "time_s"])
    writer = None
    last_seq, last_send, last_row = -1, 0.0, 0.0
    cmd = line_follow.Command()
    n_people, bearing, reason = 0, None, None
    try:
        with lnk.lock:
            yaw = lnk.orient.yaw
        trk.start(time.monotonic(), yaw)
        while True:
            now = time.monotonic()
            if _follow["abort"].is_set():
                trk.stop()
                reason = "stopped"
                break
            snap = lnk.telemetry_snapshot()
            if snap["telemetry_age_s"] is None or snap["telemetry_age_s"] > TELEMETRY_STALE_S:
                reason = "telemetry_stale"
                break
            if not snap["imu_live"]:
                reason = "imu_stale"
                break
            with lnk.lock:
                yaw = lnk.orient.yaw
            fr = src.latest()
            new_frame = fr is not None and fr.seq != last_seq
            if new_frame:
                last_seq = fr.seq
                res, _ = person_vision.analyze(fr.img, draw=False)
                n_people = res["n_people"]
                bearing = res["bearing_deg"]
                trk.on_frame(res, fr.t)
                if writer is None:
                    h, w = fr.img.shape[:2]
                    writer = cv2.VideoWriter(str(rec_dir / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
                writer.write(fr.img)
                tw.writerow([fr.seq, f"{t_rec():.3f}"])
                times.flush()
            cmd = trk.step(now, yaw)
            i = cmd.info
            if new_frame or now - last_row >= 0.1:
                last_row = now
                lw.writerow([f"{t_rec():.3f}", last_seq, n_people, "" if bearing is None else bearing,
                             f"{yaw:.2f}", i.get("target_yaw", ""), i.get("err_deg", ""),
                             int(bool(i.get("spin", False))), i.get("t", ""), cmd.left_raw, cmd.right_raw,
                             cmd.status])
                log.flush()
                _follow["info"] = dict(i, status=cmd.status, n_people=n_people, bearing_deg=bearing,
                                       yaw=round(yaw, 1), fps=round(src.fps(), 1))
            if cmd.status != "running":
                reason = cmd.status
                break
            if now - last_send >= DRIVE_KEEPALIVE_S or (new_frame and (cmd.left_raw or cmd.right_raw)):
                if _follow["abort"].is_set():
                    continue
                lnk.send_drive(cmd.left_raw, cmd.right_raw)
                last_send = now
            time.sleep(0.01)
    except Exception as e:
        reason = f"error: {e}"
    finally:
        try:
            lnk.send_stop()
        except Exception:
            pass
        if writer is not None:
            writer.release()
        log.close()
        times.close()
        summary = lnk.recorder.stop()
        _follow["status"] = reason or "ended"
        _follow["result"] = dict(status=_follow["status"], frames=trk.frames, dir=str(rec_dir),
                                 duration_s=None if summary is None else summary["duration_s"],
                                 last=_follow["info"])
        _follow["running"] = False
        _follow["ended"].set()


@mcp.tool()
def turn_to_me(confirmed_safe: bool, max_seconds: float = 30.0, name: Optional[str] = None,
               spin_speed_dps: float = 18.0, lost_timeout_s: float = 1.0) -> dict:
    """Spin in place to face the person the phone camera sees, then HOLD: keep watching and
    spin again whenever they move (it never drives forward/back). Runs on its own thread and
    returns at once; poll turn_to_me_status(wait_s=...). Call stop() to end it.

    BEFORE calling: tell the operator the run (max_seconds, spins in place only, stops if it
    loses the person for lost_timeout_s) and get a go-ahead; ONE confirmed_safe covers the run. If the
    tool says several people are in view, ASK the operator who to face / to clear the frame --
    do not guess. Needs exactly one person in view at the start (legs only is fine; check with
    person_view() first). It does not search for a lost person.

    Tunable, and changeable mid-run with turn_to_me_set(): max_seconds (default 30; 0 = no
    time limit), lost_timeout_s (default 1.0; 0 = never end because the person is gone, it
    just holds its last heading), spin_speed_dps (fastest spin it will ask for, 5-60, default
    18; faster follows a walking person better but overshoots small corrections more).

    Ends by itself with a reason: lost (no person for lost_timeout_s), stale (no camera frame
    for 0.5s), telemetry_stale / imu_stale, timeout (max_seconds), stopped (stop() or
    listen()). While running it follows the person nearest its current target if several
    appear. Records telemetry, video and turn_log.csv to recordings/<name>/."""
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if link.recorder.active:
        return {"ok": False, "error": "a recording is already active; stop_recording() first"}
    bad = _check_turn_params(max_seconds, lost_timeout_s, spin_speed_dps)
    if bad:
        return {"ok": False, "error": bad}
    snap = link.telemetry_snapshot()
    if not snap["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry", "telemetry": snap}
    if not snap["imu_live"]:
        return {"ok": False, "error": "no live IMU telemetry (the spin is closed on IMU yaw)",
                "telemetry": snap}
    src = _frame_source()
    t0 = time.monotonic()
    while src.latest() is None and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    if src.latest() is None or src.age_s() > 1.0:
        return {"ok": False, "error": "no live camera frames", "camera": src.status()}
    # look at a few fresh frames: need exactly one person throughout, not a lucky single frame
    counts, seen = [], -1
    t0 = time.monotonic()
    while len(counts) < 4 and time.monotonic() - t0 < 3:
        fr = src.latest()
        if fr is not None and fr.seq != seen:
            seen = fr.seq
            counts.append(person_vision.analyze(fr.img, draw=False)[0]["n_people"])
        time.sleep(0.02)
    if len(counts) < 2:
        return {"ok": False, "error": "couldn't get fresh camera frames", "camera": src.status()}
    if max(counts) > 1:
        return {"ok": False, "error": "more than one person is in view -- ask the operator who to "
                "face (or to clear the frame) before starting", "people_per_frame": counts}
    if min(counts) == 0:
        return {"ok": False, "error": "no person clearly in view (not seen in every frame) -- check "
                "with person_view()", "people_per_frame": counts}
    trk = person_track.PersonTracker(max_run_s=max_seconds, forward_pwm_sign=FORWARD_PWM_SIGN,
                                     spin_u_max=spin_speed_dps, lost_timeout_s=lost_timeout_s)
    try:
        path = link.recorder.start(name or time.strftime("turn_%Y%m%d_%H%M%S"))
    except Exception as e:
        return {"ok": False, "error": str(e)}
    _follow["abort"].clear()
    _follow["ended"].clear()
    _follow.update(running=True, status="running", info={}, result=None, tracker=trk)
    threading.Thread(target=_person_loop, daemon=True,
                     args=(link, src, trk, link.recorder.dir, link.recorder.now)).start()
    return {"ok": True, "started": True, "dir": str(link.recorder.dir), "max_seconds": max_seconds,
            "lost_timeout_s": lost_timeout_s, "spin_speed_dps": spin_speed_dps,
            "people_per_frame": counts, "camera": src.status()}


@mcp.tool()
def turn_to_me_set(max_seconds: Optional[float] = None, lost_timeout_s: Optional[float] = None,
                   spin_speed_dps: Optional[float] = None) -> dict:
    """Change a RUNNING turn_to_me on the fly (give only what you want to change; no
    confirmation needed since the run is already approved and this only adjusts it).
    max_seconds: total run time limit measured from the start of the run (0 = no limit).
    lost_timeout_s: how long with nobody in view before it gives up (0 = never; it holds its
    last heading). spin_speed_dps: fastest spin it will ask for, 5-60 (higher = follows a
    walking person better, overshoots small corrections more). Returns the settings now in
    effect. Fails if no turn_to_me run is in progress."""
    trk = _follow.get("tracker")
    if not _follow["running"] or trk is None or trk.status != "running":
        return {"ok": False, "error": "no turn_to_me run is in progress"}
    bad = _check_turn_params(max_seconds, lost_timeout_s, spin_speed_dps)
    if bad:
        return {"ok": False, "error": bad}
    if max_seconds is not None:
        trk.max_run_s = max_seconds
    if lost_timeout_s is not None:
        trk.lost_timeout_s = lost_timeout_s
    if spin_speed_dps is not None:
        trk.spin_u_max = spin_speed_dps
    return {"ok": True, "max_seconds": trk.max_run_s, "lost_timeout_s": trk.lost_timeout_s,
            "spin_speed_dps": trk.spin_u_max}


@mcp.tool()
def turn_to_me_status(wait_s: float = 0.0) -> dict:
    """Read-only. State of the turn_to_me run: running / the reason it ended (lost, stale,
    telemetry_stale, imu_stale, timeout, stopped, error: ...), the latest bearing to the
    person, heading error in degrees (err_deg; + = needs a left turn), whether it is facing
    them, and the recording folder. wait_s (max 60) blocks until the run ends or that long
    passes. Never moves the robot."""
    return line_follow_status(wait_s)


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
    """Runs on its own thread for the whole show (shares _follow with the other autonomous runs:
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
            if _follow["abort"].is_set():
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
                _follow["info"] = {"t_s": round(t, 1), "of_s": round(end_s, 1),
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
        _follow["status"] = reason
        _follow["result"] = dict(status=reason, ended_at_s=round(t, 1))
        _follow["running"] = False
        _follow["ended"].set()
        _speak_lock.release()


@mcp.tool()
def perform(midi_path: Optional[str] = None, bpm: Optional[float] = None,
            audio_path: Optional[str] = None, synth: bool = False, dance: bool = False,
            confirmed_safe: bool = False, dance_pwm: int = 60, spin_pwm: int = 85,
            brightness: int = 127, lead_in_s: float = 3.0, audio_offset_s: float = 0.0,
            start_s: float = 0.0, max_seconds: Optional[float] = None,
            duration_s: float = 60.0, require_telemetry: bool = True) -> dict:
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
        if require_telemetry and not snap["telemetry_live"]:
            return {"ok": False, "error": "no live encoder telemetry -- refusing to dance without it "
                    "(pass require_telemetry=false only for a bench run you are watching)",
                    "telemetry": snap}
    elif _follow["running"]:
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
    _follow["abort"].clear()
    _follow["ended"].clear()
    _follow.update(running=True, status="performing", info={}, result=None)
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
    return line_follow_status(wait_s)


if __name__ == "__main__":
    mcp.run()
