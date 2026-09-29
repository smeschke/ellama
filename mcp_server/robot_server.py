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
"""

import atexit
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Literal, Optional

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
import bridge  # noqa: E402  (calibration/bridge.py)

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

# Encoder count direction when that wheel rolls forward. Verified 2026-09-28: a 6in
# forward drive_distance read left +6.77in, right -6.66in (right encoder is mirrored).
ENCODER_FORWARD_SIGN = (1, -1)

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
                    self.enc.update(left, right, ms)
                    self.last_enc_wall = time.monotonic()
                elif kind == "IMU":
                    ax, ay, az, gx, gy, gz, ms = payload
                    self.orient.update(ax, ay, az, gx, gy, gz, ms)
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


def _confirm_gate(confirmed_safe: bool):
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
    if link is None:
        return _not_connected()
    link.send_listen()
    return {"ok": True, **link.telemetry_snapshot()}


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
) -> dict:
    """Closed-loop curved drive: both wheels roll forward, the outer one faster, so the
    robot drives an arc of `radius_inches` (measured to the robot's center) until the
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

    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    link.send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = link.in_per_tick()
    half_track = link.cfg["track_width_in"] / 2.0
    diff = pwm * min(1.0, half_track / radius_inches)
    target_curv = 1.0 / radius_inches  # rad per inch of path

    def forward_path_in(a, b):
        # Signed, so a reversing inner wheel subtracts from the path instead of adding.
        l_fwd = (b[0] - a[0]) * ENCODER_FORWARD_SIGN[0]
        r_fwd = (b[1] - a[1]) * ENCODER_FORWARD_SIGN[1]
        return (l_fwd + r_fwd) / 2.0 * in_per_tick

    max_diff = pwm + ARC_MAX_INNER_REVERSE_PWM

    def wheel_cmd(d):
        outer = min(MAX_PWM_CEILING, pwm + d)
        inner = max(-ARC_MAX_INNER_REVERSE_PWM, pwm - d)
        # "left" = right wheel on the outside, matching turn_degrees' mapping.
        l_real, r_real = (inner, outer) if direction == "left" else (outer, inner)
        return FORWARD_PWM_SIGN * l_real, FORWARD_PWM_SIGN * r_real

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


if __name__ == "__main__":
    mcp.run()
