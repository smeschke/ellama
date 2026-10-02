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

import atexit
import math
import os
import signal
import socket
import ssl
import sys
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
from pathlib import Path
from typing import Literal, Optional

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
import bridge  # noqa: E402  (calibration/bridge.py)
import recording  # noqa: E402  (mcp_server/recording.py)
import line_vision  # noqa: E402  (mcp_server/line_vision.py)
import person_track  # noqa: E402  (mcp_server/person_track.py)
import person_vision  # noqa: E402  (mcp_server/person_vision.py)
import camera_stream  # noqa: E402  (mcp_server/camera_stream.py)
import line_follow  # noqa: E402  (mcp_server/line_follow.py)

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
            with self.lock:
                if kind == "ENC":
                    left, right, ms = payload
                    sl, sr = self.enc.update(left, right, ms)
                    self.last_enc_wall = time.monotonic()
                    self.recorder.on_enc(self.enc, self.orient, sl, sr)
                elif kind == "CMD":
                    self.recorder.on_cmd(*payload)  # the stick's command, overheard by the bridge
                elif kind == "IMU":
                    ax, ay, az, gx, gy, gz, ms = payload
                    acc = self.orient.update(ax, ay, az, gx, gy, gz, ms)[:3]
                    self.recorder.on_imu(self.enc, self.orient, acc)
                    self.last_imu = dict(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, ms=ms)
                    self.last_imu_wall = time.monotonic()

    def send_line(self, text):
        self.ser.write((text + "\n").encode())
        self.ser.flush()

    def send_drive(self, left, right):
        left = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(left)))
        right = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(right)))
        self.send_line(f"{left} {right}")

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
        _connect_error = None
    except Exception as e:
        _connect_error = str(e)
        raise
    return link


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

FOLLOW_MAX_CRUISE_PWM = 70  # start_line_follow's speed_pwm is capped here (turns may go to OUTER_MAX)


def _frame_source():
    global _frames
    if _frames is None:
        _frames = camera_stream.FrameSource(_open_stream)
    _frames.start()
    return _frames


mcp = FastMCP("ellama-robot")


def _not_connected():
    return {"ok": False, "error": f"not connected to the robot bridge: {_connect_error}"}


def _confirm_gate(confirmed_safe: bool):
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
def drive_distance(
    distance_inches: float,
    direction: Literal["forward", "reverse"],
    pwm: int,
    confirmed_safe: bool,
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
def turn_degrees(
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
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
def drive_arc(
    radius_inches: float,
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
    max_wheel_pwm: int = MAX_PWM_CEILING,
    travel: Literal["forward", "reverse"] = "forward",
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
def jog(left_pwm: int, right_pwm: int, duration_s: float, confirmed_safe: bool) -> dict:
    """Raw open-loop command: drive at exactly (left_pwm, right_pwm) for duration_s
    seconds, then stop. No encoder target and no direction-aware sign correction (unlike
    drive_distance/turn_degrees) -- positive does not necessarily mean forward, see
    FORWARD_PWM_SIGN. This is for bench/commissioning tests where a fixed, observable
    duration matters more than hitting a distance or angle (checking direction, deadband,
    turn convention by eye). Same confirmed_safe gate as the other drive tools. Capped at
    MAX_JOG_DURATION_S=10s and MAX_PWM_CEILING=125 per side.
    """
    if link is None:
        return _not_connected()
    gate = _confirm_gate(confirmed_safe)
    if gate:
        return gate
    if not (0 < duration_s <= MAX_JOG_DURATION_S):
        return {"ok": False, "error": f"duration_s must be > 0 and <= {MAX_JOG_DURATION_S}"}

    snap = link.telemetry_snapshot()
    if not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback",
            "telemetry": snap,
        }

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
        time.sleep(STOP_SETTLE_S)

    end = link.last_counts()
    in_per_tick = link.in_per_tick()
    return {
        "ok": True,
        "left_pwm": left_pwm,
        "right_pwm": right_pwm,
        "duration_s": duration_s,
        "dist_l_in": round((end[0] - start[0]) * in_per_tick, 2),
        "dist_r_in": round((end[1] - start[1]) * in_per_tick, 2),
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "telemetry": link.telemetry_snapshot(),
    }


def _follow_loop(lnk, src, ctl, rec_dir, t_rec):
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
    try:
        ctl.start(time.monotonic())
        while True:
            now = time.monotonic()
            if _follow["abort"].is_set():
                ctl.stop()
                reason = "stopped"
                break
            snap_age = lnk.telemetry_snapshot()["telemetry_age_s"]
            if snap_age is None or snap_age > TELEMETRY_STALE_S:
                ctl.status = "telemetry_stale"
                reason = "telemetry_stale"
                break
            fr = src.latest()
            if fr is not None and fr.seq != last_seq:
                last_seq = fr.seq
                res, _ = line_vision.analyze(fr.img, prev_x_frac=prev_x)
                prev_x = res.get("x_frac") if res["found"] else None
                cmd = ctl.update(res, fr.t)
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
        _follow["result"] = dict(status=_follow["status"], frames=ctl.frames, dir=str(rec_dir),
                                 duration_s=None if summary is None else summary["duration_s"],
                                 last=_follow["info"])
        _follow["running"] = False
        _follow["ended"].set()


@mcp.tool()
def start_line_follow(confirmed_safe: bool, speed_pwm: int = 58, max_seconds: float = 90.0,
                      name: Optional[str] = None) -> dict:
    """Start following the tape line with the phone camera, on its own thread, and return at
    once; poll line_follow_status(wait_s=...) for the end. ONE confirmed_safe covers the whole
    run: set it only after telling the operator the exact run (speed_pwm, max_seconds, that it
    stops if the line is lost / the finish strip is seen / camera or telemetry go stale) and
    getting a go-ahead. The line must already be in view (>=3 bands) and the finish strip not.

    It stops by itself and reports why: finished (cross strip reached), lost (no line for
    0.5s), stale (no camera frame for 0.5s), telemetry_stale, timeout, stopped (stop() or
    listen() was called -- call stop() at any time to abort). No searching for a lost line.
    Records telemetry, video, and a per-frame follow_log.csv to recordings/<name>/.
    speed_pwm is the cruise forward effort, 50-70 (default 58); turns may go up to 90 on the
    outer wheel."""
    if link is None:
        return _not_connected()
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
    if not snap["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry", "telemetry": snap}
    src = _frame_source()
    t0 = time.monotonic()
    while src.latest() is None and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    fr = src.latest()
    if fr is None or src.age_s() > 1.0:
        return {"ok": False, "error": "no live camera frames", "camera": src.status()}
    res, _ = line_vision.analyze(fr.img)
    ctl = line_follow.LineFollower(max_run_s=max_seconds, forward_pwm_sign=FORWARD_PWM_SIGN,
                                   f_cruise=speed_pwm)
    ok, why = ctl.can_start(res)
    if not ok:
        return {"ok": False, "error": why}
    try:
        path = link.recorder.start(name or time.strftime("follow_%Y%m%d_%H%M%S"))
    except Exception as e:
        return {"ok": False, "error": str(e)}
    _follow["abort"].clear()
    _follow["ended"].clear()
    _follow.update(running=True, status="running", info={}, result=None)
    threading.Thread(target=_follow_loop, daemon=True,
                     args=(link, src, ctl, link.recorder.dir, link.recorder.now)).start()
    return {"ok": True, "started": True, "dir": str(link.recorder.dir), "speed_pwm": speed_pwm,
            "max_seconds": max_seconds, "bands_at_start": res["n_bands"], "camera": src.status()}


@mcp.tool()
def line_follow_status(wait_s: float = 0.0) -> dict:
    """Read-only. State of the line-follow run: running / the reason it ended (finished, lost,
    stale, telemetry_stale, timeout, stopped, error: ...), the latest offsets and wheel
    efforts, and the recording folder. wait_s (max 60) blocks until the run ends or that long
    passes -- use it instead of polling in a tight loop. Never moves the robot."""
    if _follow["running"] and wait_s > 0:
        _follow["ended"].wait(min(wait_s, 60.0))
    return {"ok": True, "running": _follow["running"], "status": _follow["status"],
            "info": _follow["info"], "result": _follow["result"],
            "camera": None if _frames is None else _frames.status()}


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


if __name__ == "__main__":
    mcp.run()
