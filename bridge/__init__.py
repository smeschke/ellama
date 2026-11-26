"""
Shared code for reading computer_bridge.ino's serial output and turning it
into distances and angles.

This is the PC-side client library for the bridge firmware, and it is shared: the MCP
server (mcp_server/robot_server.py) and the live dashboard (calibration/live_view.py)
both import it so they compute distance, heading and range from the same code. It lived
in calibration/ until 2026-10-24, which read as calibration-only.

computer_bridge.ino is listen-only until it is sent a drive line, and the
scripts here never send one, so they can run while you drive with the stick.

Serial lines this understands (v2, from computer_bridge.ino v2):
  @ <seq> ENC <left> <right> <ms> *CC
  @ <seq> IMU <ax> <ay> <az> <gx> <gy> <gz> <ms> *CC
  @ <seq> CMD <left> <right> *CC
  @ <seq> <NAME> <value>... <ms> *CC   e.g. "@ 123 TOF_FRONT 812 -1 48213 *5A"
  @ <seq> DROPPED <n> *CC / @ <seq> FOREIGN <n> *CC
  @ status ... *00                        bridge chatter (never telemetry)
Legacy v1 lines without the "@ ... *CC" wrapper are still accepted during the
changeover but count as deprecated: they have no checksum, so a torn line can
still invent a sensor name.

The last form is the current scheme (firmware/libraries/LlamaSensors/LlamaSensors.h): the
packet carries its own name, so adding a sensor needs no table here and no change to the
bridge firmware. Such a line parses generically into ("SENSOR", (name, values, ms)) and
callers switch on the name -- there is deliberately no registry of known sensors to keep
in step with the firmware.

Values stay raw ints, because what counts as an invalid reading is sensor-specific: -1
means "could not measure" for a ToF range sensor, but is a perfectly good gyro LSB. Use
TofState for the range sensors, which applies that interpretation.

Measured constants (wheel size, gear ratio, track width) live in config.json
next to this file, so live_view.py can write calibration results there instead
of you editing source.
"""

import json
import math
import os
import queue
import re
import sys
import threading
import time
from pathlib import Path

import serial
import serial.tools.list_ports as list_ports

BAUD_RATE = 921600  # v2 bridge firmware; reflash computer_bridge.ino together with this
RESET_SETTLE_S = 2.0  # opening the port resets the ESP32; wait for boot + ESP-NOW join

# Mirrors SENSOR_VALUE_INVALID in LlamaSensors.h: a sensor that reported but could not
# measure. Only meaningful for sensors whose values are physical magnitudes, like ranges.
SENSOR_VALUE_INVALID = -1

# Mirrors SENSOR_VALUE_UNAVAILABLE in LlamaSensors.h: used where -1 is a physically
# possible reading, as an encoder's cumulative count is. The encoders sit by the wheels
# and get damaged, so a packet with one channel unavailable is routine -- it must never be
# mistaken for a count, and -1 * encoder_sign would not even round-trip to itself.
SENSOR_VALUE_UNAVAILABLE = -2147483648

# ToF range sensors send millimetres, one value per channel in the sending sketch's
# sensors[] order. These labels are display only -- the firmware knows nothing about
# them, and an unlisted sensor just falls back to "<name>[<index>]".
TOF_CHANNEL_LABELS = {
    "TOF_FRONT": ("front_left", "front_right"),
    "TOF_REAR": ("rear_left", "rear_right"),
}

MM_PER_IN = 25.4

# IMU scale -- must match the robot_imu_* sketch config (+-2g, +-250 dps)
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
COMPLEMENTARY_ALPHA = 0.98

# Ground-verified by the operator: rotating the nose left (counter-clockwise
# seen from above) must increase yaw. The IMU board's mounting reports gz with
# the opposite sign on this unit, so the filter negates it -- every consumer
# (live_view, the daemon's /state, turn_degrees, realtime_map) reads the
# corrected yaw from OrientationState. Flip back to +1 after any IMU remount,
# verified by hand: motors off, rotate nose left, yaw must climb.
IMU_YAW_SIGN = -1

# config.json lives in calibration/, not beside this file: it holds measured constants
# that the calibration scripts produce and save_config() writes back. Resolved explicitly
# because load_config() swallows FileNotFoundError -- a wrong path here would not raise,
# it would silently fall back to DEFAULT_CONFIG and quietly change every distance the
# robot reports.
CONFIG_PATH = Path(__file__).resolve().parent.parent / "calibration" / "config.json"
DEFAULT_CONFIG = {
    "wheel_diameter_in": 10.0,
    "ticks_per_rev": 4096,       # AS5600, per revolution of the MAGNET shaft (not the wheel)
    "gear_ratio": 4.5,           # encoder-shaft turns per wheel turn; ~90t/20t teeth, confirmed by the tape-measured 30in run
    "track_width_in": 20.0,      # PLACEHOLDER until calibrated -- wheel-to-wheel distance
    "encoder_sign": [1, 1],      # [left, right]; -1 flips a wheel whose magnet/encoder counts backward
}


# One dashboard serves several robots: ELLAMA_ROBOT picks
# calibration/robots/<name>.json. Unset means the historical
# calibration/config.json. A named profile that does not exist is a loud error,
# never a silent fallback -- a typo must not mis-calibrate distances.
ROBOT_NAME = os.environ.get("ELLAMA_ROBOT") or "default"
PROFILES_DIR = CONFIG_PATH.parent / "robots"


def resolve_config_path():
    if ROBOT_NAME == "default":
        return CONFIG_PATH
    p = PROFILES_DIR / (ROBOT_NAME + ".json")
    if not p.exists():
        raise RuntimeError("ELLAMA_ROBOT=%r: no such profile %s "
                           "(see calibration/robots/README.md)" % (ROBOT_NAME, p))
    return p


# Lowercase so a profile name is also a safe file name on every OS.
PROFILE_NAME_RE = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")


def list_profiles():
    """Names of the robot profiles in calibration/robots/, sorted."""
    return sorted(p.stem for p in PROFILES_DIR.glob("*.json"))


def set_robot(name):
    """Switch the active profile at runtime ("default" = calibration/config.json).

    Updates ROBOT_NAME and ENC_SIGN together; callers holding a cfg dict must
    reload it. An unknown name raises and leaves the active profile unchanged."""
    global ROBOT_NAME, ENC_SIGN
    if name != "default" and not (PROFILES_DIR / (str(name) + ".json")).exists():
        raise RuntimeError("no such robot profile %r (have: %s)"
                           % (name, ", ".join(list_profiles()) or "none"))
    old = ROBOT_NAME
    ROBOT_NAME = name
    try:
        sign = tuple(load_config()["encoder_sign"])
    except Exception:
        ROBOT_NAME = old
        raise
    ENC_SIGN = sign


def save_profile(name, cfg, overwrite=False):
    """Write calibration/robots/<name>.json. Refuses a bad name, "default", and
    (unless overwrite) an existing profile. Returns the path."""
    if not isinstance(name, str) or not PROFILE_NAME_RE.match(name) or name == "default":
        raise ValueError("robot name must be lowercase letters, digits, - or _ "
                         "(starting with a letter), and not 'default'")
    p = PROFILES_DIR / (name + ".json")
    if p.exists() and not overwrite:
        raise FileExistsError("profile %r already exists" % name)
    PROFILES_DIR.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(cfg, indent=2) + "\n")
    return p


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        cfg.update(json.loads(resolve_config_path().read_text()))
    except FileNotFoundError:
        pass
    return cfg


ENC_SIGN = tuple(load_config()["encoder_sign"])  # applied to ENC counts in parse_line


def save_config(cfg):
    resolve_config_path().write_text(json.dumps(cfg, indent=2) + "\n")


def in_per_tick(cfg):
    return math.pi * cfg["wheel_diameter_in"] / (cfg["ticks_per_rev"] * cfg["gear_ratio"])


# ---------------- serial ----------------

_IGNORE_DEVICE_RE = re.compile(r"^/dev/ttyS\d+$")
_IGNORE_DESC_RE = re.compile(r"bluetooth", re.I)


def list_candidate_ports():
    names = []
    for p in list_ports.comports():
        if _IGNORE_DEVICE_RE.match(p.device):
            continue
        if _IGNORE_DESC_RE.search(p.description or ""):
            continue
        names.append(p.device)
    return sorted(names)


def pick_port(explicit):
    if explicit:
        return explicit
    candidates = list_candidate_ports()
    if not candidates:
        sys.exit("No serial ports found. Plug in the bridge ESP32 and try again, or pass --port explicitly.")
    if len(candidates) > 1:
        print("Multiple candidate ports found, using the first. Pass --port to pick a different one:")
        for c in candidates:
            print(" ", c)
    return candidates[0]


class SerialReader:
    """Background thread that only drains the serial port into a queue, so
    CSV writes/plotting never delay reads."""

    def __init__(self, ser):
        self.ser = ser
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _run(self):
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                return
            if raw:
                self.q.put(raw)


# A SensorPacket name as the bridge prints it: the firmware sanitises anything outside
# [A-Za-z0-9_], and every sender here uses upper case.
_SENSOR_NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")

# Words computer_bridge.ino prints that look like a sensor name but are not one.
_BRIDGE_WORDS = frozenset({"DROPPED", "UNK"})


def _v2_checksum(payload_text):
    """XOR of every byte in payload_text (the part between "@ " and " *")."""
    cs = 0
    for b in payload_text.encode("ascii", errors="ignore"):
        cs ^= b
    return cs


def _classify_tokens(parts):
    """Shared dispatch for v1 and v2 token lists (no "@", no checksum)."""
    if not parts:
        return None, ""
    if parts[0] == "ENC" and len(parts) == 4:
        left, right, ms = (int(x) for x in parts[1:4])
        # Check for the sentinel BEFORE applying the sign: negating it would turn it
        # into an ordinary-looking large count. An unavailable wheel becomes None.
        left = None if left == SENSOR_VALUE_UNAVAILABLE else left * ENC_SIGN[0]
        right = None if right == SENSOR_VALUE_UNAVAILABLE else right * ENC_SIGN[1]
        return "ENC", (left, right, ms)
    if parts[0] == "CMD" and len(parts) == 3:
        return "CMD", tuple(int(x) for x in parts[1:3])
    if parts[0] == "IMU" and len(parts) == 8:
        return "IMU", tuple(int(x) for x in parts[1:8])

    # Generic SensorPacket line: a name, at least one value, then the sender's ms.
    if len(parts) >= 3 and parts[0] not in _BRIDGE_WORDS and _SENSOR_NAME_RE.match(parts[0]):
        nums = tuple(int(x) for x in parts[1:])
        return "SENSOR", (parts[0], nums[:-1], nums[-1])
    return None, " ".join(parts)


def parse_line(raw):
    """Classify one line of computer_bridge.ino output (v2 preferred, v1 fallback).

    v2: "@ <seq> <NAME> <fields...> *<CC>" with XOR checksum. A bad checksum or a
    torn frame returns (None, text) so the caller drops it instead of inventing
    a sensor. v1 lines without the wrapper are still accepted as deprecated.

    Returns one of:
      ("ENC", (left, right, ms))                    encoder packet, signs applied
      ("IMU", (ax, ay, az, gx, gy, gz, ms))         IMU packet
      ("CMD", (left, right))                        an overheard drive command
      ("SENSOR", (name, values, ms))                any SensorPacket sender; values is a
                                                    tuple of raw ints, uninterpreted
      (None, decoded_text)                          bridge chatter or garbage
    """
    text = raw.decode("ascii", errors="ignore").strip()
    if not text:
        return None, text
    if text.startswith("@"):
        # v2 frame. "@ status ..." is bridge chatter, never telemetry.
        if text.startswith("@ status"):
            return None, text
        try:
            head, _, cc_hex = text.rpartition(" *")
            if not head.startswith("@ ") or not cc_hex:
                return None, text
            payload = head[2:]
            want = int(cc_hex.strip()[:2], 16)
            if _v2_checksum(payload) != want:
                return None, text
            tokens = payload.split()
            if len(tokens) < 2:
                return None, text
            int(tokens[0])  # seq: validated, otherwise ignored
            if tokens[1] in ("DROPPED", "FOREIGN"):
                # Bridge's own counters: keep the v1 "DROPPED <n>" shape the
                # pump already parses, without the framing around it.
                return None, " ".join(tokens[1:])
            kind, payload_out = _classify_tokens(tokens[1:])
            if kind is None:
                return None, text
            return kind, payload_out
        except ValueError:
            return None, text
    try:
        kind, payload_out = _classify_tokens(text.split())
        if kind is None:
            return None, text
        return kind, payload_out
    except ValueError:
        pass
    return None, text


# ---------------- state ----------------

# A wheel that arrived as None (board silent or slot unwired) keeps its last
# distance; it reads unavailable only after this long with no real count. Two
# single-encoder boards interleave left-only and right-only packets, so judging
# availability per packet would flicker -- the latch holds while either board
# streams, without masking a board that actually died.
ENC_CHANNEL_STALE_S = 1.0

class EncoderState:
    """Per-wheel distance/speed since the last zero(), plus a differential-drive
    heading estimate from the wheel difference.

    Either wheel's count may arrive as None, meaning that encoder is not responding --
    a routine state, since the encoders sit by the wheels and get damaged. That wheel's
    distance and speed then hold their last known values and `available` reports which
    side is live; the other wheel keeps updating normally. heading_deg needs both wheels,
    so it returns None while either is missing rather than silently drifting off a
    frozen value.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.first = [None, None]   # per-wheel counts at the zero point
        self.last = [None, None]    # per-wheel counts at the previous sample
        self.last_ms = [None, None]  # per-wheel sender clocks; two boards don't share one
        self.last_seen = [None, None]  # wall time of the last real count per wheel
        self.dist_l = self.dist_r = 0.0
        self.available = [False, False]   # [left, right] latched per wheel (see update)

    @property
    def dist_avg(self):
        return (self.dist_l + self.dist_r) / 2.0

    @property
    def heading_deg(self):
        """Heading from the wheel difference, or None if either encoder is missing.

        Returning None rather than a number computed from one live wheel and one frozen
        one is deliberate: that figure looks plausible and drifts without bound.
        """
        if not all(self.available):
            return None
        # Positive when the right wheel has travelled farther (turning left/CCW).
        return math.degrees((self.dist_r - self.dist_l) / self.cfg["track_width_in"])

    def zero(self):
        """Make the current position the origin for distance and heading."""
        self.first = list(self.last)
        self.dist_l = self.dist_r = 0.0

    def update(self, left, right, ms, now=None):
        """Fold in one ENC packet. Either count may be None (encoder not responding).

        Two single-encoder boards interleave left-only and right-only packets on
        unrelated sender clocks: each wheel is differenced against its own
        clock, and availability latches per wheel with a timeout instead of
        flickering with every packet. A board that actually dies still reads
        unavailable once its channel goes quiet past ENC_CHANNEL_STALE_S.
        `now` is wall time for the staleness latch (tests pass it explicitly).
        """
        if now is None:
            now = time.monotonic()
        k = in_per_tick(self.cfg)
        counts = (left, right)
        dists = [self.dist_l, self.dist_r]
        speeds = [None, None]

        for i, c in enumerate(counts):
            if c is None:
                # Hold the last distance and keep `last`: the next real reading is
                # differenced on this wheel's own clock, so a silent gap reads as
                # the average speed across it, not a spike and not a zero.
                continue
            dt = None
            if self.last_ms[i] is not None and ms > self.last_ms[i]:
                dt = (ms - self.last_ms[i]) / 1000.0
            if self.first[i] is None:
                self.first[i] = c
            dists[i] = (c - self.first[i]) * k
            if dt is not None and self.last[i] is not None:
                speeds[i] = (c - self.last[i]) * k / dt
            self.last[i] = c
            self.last_ms[i] = ms
            self.last_seen[i] = now

        self.dist_l, self.dist_r = dists
        self.available = [t is not None and (now - t) < ENC_CHANNEL_STALE_S
                          for t in self.last_seen]
        return tuple(speeds)


# Zero-point (bias) update for the yaw gyro while the robot is stopped.
#
# The IMU firmware measures the gyro bias once, in the ~1 s still window at boot. An
# MPU-6050 (GY-521) warms up for minutes after power-on and its bias moves with it, so
# a cold boot bias leaves the yaw creeping by tenths of a degree per second at rest
# (0.3-0.4 deg/s measured 2026-11-10). The fix: whenever the wheel encoders say the
# robot is standing still, (1) re-learn the z bias from what the gyro reads, and (2) do
# not integrate yaw at all, since a robot whose wheels are still is not turning.
#
# "Still" needs all of: both encoders reporting and fresh, neither wheel moved more
# than STILL_DIST_IN for STILL_AFTER_S, and the bias-corrected gyro under
# STILL_GYRO_MAX_DPS (so a robot lifted and rotated by hand, or skidding with locked
# wheels, still integrates). Without encoders this does nothing, which is the old
# behaviour.
ZUPT_ENABLED = True
STILL_AFTER_S = 0.75        # wheels quiet this long before the robot counts as stopped
STILL_DIST_IN = 0.05        # per-wheel movement below this is encoder jitter, not motion
STILL_GYRO_MAX_DPS = 2.0    # a real rotation faster than this is never "still"
ENC_FRESH_S = 0.5           # encoder packets older than this say nothing about motion
BIAS_TAU_S = 2.0            # bias learning time constant while still
BIAS_MAX_DPS = 10.0         # clamp: a bias this large means something is wrong


class OrientationState:
    """Complementary filter on the PC side; dt from the packet's own ms. Yaw also gets
    a zero-point update while the wheels are still (see ZUPT_ENABLED above): call
    note_wheels() on every encoder packet to feed it."""

    def __init__(self):
        self.pitch = self.roll = self.yaw = 0.0
        self.last_ms = None
        self.gz_bias_dps = 0.0      # learned on top of the firmware's boot bias
        self.still = False
        self.still_s = 0.0          # total time yaw was held by the zero-point update
        self._wheel_ref = None      # (dist_l, dist_r) when the wheels last moved
        self._wheels_quiet_since = None
        self._last_enc_t = None

    def zero_yaw(self):
        self.yaw = 0.0

    def note_wheels(self, dist_l, dist_r, now=None):
        """Feed one encoder reading (inches; None for a wheel that is not reporting)."""
        now = time.monotonic() if now is None else now
        self._last_enc_t = now
        if dist_l is None or dist_r is None:
            self._wheel_ref = self._wheels_quiet_since = None
            return
        ref = self._wheel_ref
        if (ref is None or abs(dist_l - ref[0]) > STILL_DIST_IN
                or abs(dist_r - ref[1]) > STILL_DIST_IN):
            self._wheel_ref = (dist_l, dist_r)
            self._wheels_quiet_since = now

    def wheels_still(self, now=None):
        now = time.monotonic() if now is None else now
        return (ZUPT_ENABLED and self._wheels_quiet_since is not None
                and self._last_enc_t is not None
                and now - self._last_enc_t < ENC_FRESH_S
                and now - self._wheels_quiet_since >= STILL_AFTER_S)

    def update(self, ax, ay, az, gx, gy, gz, ms, now=None):
        agx, agy, agz = ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G
        dx, dy, dz_raw = gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS
        dz = dz_raw - self.gz_bias_dps
        still = self.wheels_still(now) and abs(dz) < STILL_GYRO_MAX_DPS
        self.still = still
        if self.last_ms is not None and ms > self.last_ms:
            dt = (ms - self.last_ms) / 1000.0
            acc_pitch = math.degrees(math.atan2(-agx, math.hypot(agy, agz)))
            acc_roll = math.degrees(math.atan2(agy, agz))
            self.pitch = COMPLEMENTARY_ALPHA * (self.pitch + dy * dt) + (1 - COMPLEMENTARY_ALPHA) * acc_pitch
            self.roll = COMPLEMENTARY_ALPHA * (self.roll + dx * dt) + (1 - COMPLEMENTARY_ALPHA) * acc_roll
            if still and dt < 1.0:
                a = min(1.0, dt / BIAS_TAU_S)
                b = self.gz_bias_dps + a * (dz_raw - self.gz_bias_dps)
                self.gz_bias_dps = max(-BIAS_MAX_DPS, min(BIAS_MAX_DPS, b))
                self.still_s += dt
            else:
                self.yaw += IMU_YAW_SIGN * dz * dt
        self.last_ms = ms
        return agx, agy, agz, dx, dy, dz

    def zupt_snapshot(self):
        return {"enabled": ZUPT_ENABLED, "still": self.still,
                "gz_bias_dps": round(self.gz_bias_dps, 4),
                "held_s": round(self.still_s, 1)}



class TofState:
    """Latest reading per ToF channel, keyed by (sensor name, channel index).

    A range sensor sends SENSOR_VALUE_INVALID when it could not measure -- it is
    reporting, just with nothing in range. That is different from a sensor that has gone
    silent, so an invalid reading is stored as None and still refreshes the timestamp;
    callers distinguish "nothing in range" from "board is dead" by the age of the entry.
    """

    #: samples kept per channel for median_mm(); small, since this runs per packet
    HISTORY = 5

    def __init__(self):
        self.channels = {}   # (name, index) -> dict(mm, inches, ms, invalid)
        self.recent = {}     # (name, index) -> list of recent valid mm, newest last
        self.last_wall = {}  # (name, index) -> time.monotonic() of last packet touching it

    def update(self, name, values, ms):
        """Store one sensor's packet. Returns the list of keys it touched."""
        now = time.monotonic()
        keys = []
        for i, raw in enumerate(values):
            invalid = raw == SENSOR_VALUE_INVALID
            key = (name, i)
            self.last_wall[key] = now
            self.channels[key] = dict(
                mm=None if invalid else raw,
                inches=None if invalid else raw / MM_PER_IN,
                ms=ms,
                invalid=invalid,
            )
            if not invalid:
                hist = self.recent.setdefault(key, [])
                hist.append(raw)
                if len(hist) > self.HISTORY:
                    del hist[0]
            keys.append(key)
        return keys

    @staticmethod
    def label(name, index):
        """Human-readable channel name for display, e.g. "front_left"."""
        labels = TOF_CHANNEL_LABELS.get(name)
        if labels and index < len(labels):
            return labels[index]
        return f"{name.lower()}[{index}]"

    def mm(self, name, index):
        """Distance in mm, or None if the sensor could not measure (or never reported)."""
        entry = self.channels.get((name, index))
        return entry["mm"] if entry else None

    def median_mm(self, name, index, now=None, stale_after_s=2.0):
        """Median of the recent valid readings for one channel, or None if there are none.

        The firmware deliberately sends any distance inside the sensor's physical range
        without consulting ST's RangeStatus, because that status rejects plenty of good
        measurements and a signal-rate threshold would only be calibrated to whatever room
        it was measured in. The cost is that occasional noisy or short samples arrive, and
        a median over a few samples is what removes them -- here, where it can be changed
        without reflashing a board bolted to the robot.

        Use this rather than mm() for anything that steers; use mm() when you want the
        latest raw sample, such as a live plot.

        now/stale_after_s expire channels a torn line invented at a reconnect:
        opening the port resets the ESP32, and a boot-time splice can parse as
        one packet with extra values. Those keys never refresh, so without this
        they would steer closest_mm forever. Pass now=None for the old
        never-expire behavior.
        """
        if now is not None:
            last = self.last_wall.get((name, index), 0.0)
            if now - last > stale_after_s:
                return None
        hist = self.recent.get((name, index))
        if not hist:
            return None
        return sorted(hist)[len(hist) // 2]

    def closest_mm(self, now=None, stale_after_s=2.0):
        """Smallest valid distance across every channel, or None if nothing is in range.

        This is the figure obstacle avoidance actually wants, and keeping it here means
        callers never have to remember to filter the invalid readings out first.
        """
        medians = [self.median_mm(*key, now=now, stale_after_s=stale_after_s)
                   for key in self.channels]
        valid = [m for m in medians if m is not None]
        return min(valid) if valid else None
