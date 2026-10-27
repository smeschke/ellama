"""
Shared code for reading computer_bridge.ino's serial output and turning it
into distances and angles.

This is the PC-side client library for the bridge firmware, and it is shared: the MCP
server (mcp_server/robot_server.py) and the live dashboard (calibration/live_view.py)
both import it so they compute distance, heading and range from the same code. It lived
in calibration/ until 2026-10-24, which read as calibration-only and is why
teleop/command_server.py grew its own copy of the parsing instead of importing this.

computer_bridge.ino is listen-only until it is sent a drive line, and the
scripts here never send one, so they can run while you drive with the stick.

Serial lines this understands:
  ENC <left> <right> <ms>                  cumulative AS5600 counts per wheel (legacy packet)
  IMU <ax> <ay> <az> <gx> <gy> <gz> <ms>   raw accel LSB (+-2g), bias-corrected gyro LSB (+-250 dps)
                                           (legacy packet)
  CMD <left> <right>                       a drive command the bridge overheard (e.g. from the stick)
  <NAME> <value>... <ms>                   any SensorPacket sender, e.g. "TOF_FRONT 812 -1 48213"

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
import queue
import re
import sys
import threading
from pathlib import Path

import serial
import serial.tools.list_ports as list_ports

BAUD_RATE = 115200
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

# config.json lives in calibration/, not beside this file: it holds measured constants
# that the calibration scripts produce and save_config() writes back. Resolved explicitly
# because load_config() swallows FileNotFoundError -- a wrong path here would not raise,
# it would silently fall back to DEFAULT_CONFIG and quietly change every distance the
# robot reports.
CONFIG_PATH = Path(__file__).resolve().parent.parent / "calibration" / "config.json"
DEFAULT_CONFIG = {
    "wheel_diameter_in": 10.0,
    "ticks_per_rev": 4096,       # AS5600, per revolution of the MAGNET shaft (not the wheel)
    "gear_ratio": 6.33,          # encoder-shaft turns per wheel turn; empirical, same as the bench sketch
    "track_width_in": 20.0,      # PLACEHOLDER until calibrated -- wheel-to-wheel distance
    "encoder_sign": [1, 1],      # [left, right]; -1 flips a wheel whose magnet/encoder counts backward
}


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        cfg.update(json.loads(CONFIG_PATH.read_text()))
    except FileNotFoundError:
        pass
    return cfg


ENC_SIGN = tuple(load_config()["encoder_sign"])  # applied to ENC counts in parse_line


def save_config(cfg):
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2) + "\n")


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


def parse_line(raw):
    """Classify one line of computer_bridge.ino output.

    Returns one of:
      ("ENC", (left, right, ms))                    legacy encoder packet, signs applied
      ("IMU", (ax, ay, az, gx, gy, gz, ms))         legacy IMU packet
      ("CMD", (left, right))                        an overheard drive command
      ("SENSOR", (name, values, ms))                any SensorPacket sender; values is a
                                                    tuple of raw ints, uninterpreted
      (None, decoded_text)                          bridge chatter or garbage

    The SENSOR branch is generic on purpose: a new sender needs no change here, which is
    the point of the name travelling inside the packet.
    """
    text = raw.decode("ascii", errors="ignore").strip()
    parts = text.split()
    if not parts:
        return None, text
    try:
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
        # Requiring every remaining field to be an integer is what keeps bridge chatter
        # out -- "UNK len=5" fails on "len=5", and "DROPPED 3" has too few fields to be
        # a reading, so neither needs special-casing beyond the name check.
        if len(parts) >= 3 and parts[0] not in _BRIDGE_WORDS and _SENSOR_NAME_RE.match(parts[0]):
            nums = tuple(int(x) for x in parts[1:])
            return "SENSOR", (parts[0], nums[:-1], nums[-1])
    except ValueError:
        pass
    return None, text


# ---------------- state ----------------

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
        self.last_ms = None
        self.dist_l = self.dist_r = 0.0
        self.available = [False, False]   # [left, right] reporting as of the last packet

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

    def update(self, left, right, ms):
        """Fold in one ENC packet. Either count may be None (encoder not responding)."""
        k = in_per_tick(self.cfg)
        counts = (left, right)
        self.available = [c is not None for c in counts]
        dists = [self.dist_l, self.dist_r]
        speeds = [None, None]

        dt = None
        if self.last_ms is not None and ms > self.last_ms:
            dt = (ms - self.last_ms) / 1000.0

        for i, c in enumerate(counts):
            if c is None:
                # Hold the last distance; a missing encoder is not a wheel at zero. Clear
                # `last` so the next real reading is not differenced against a stale one
                # across the gap, which would show up as a single huge speed spike.
                self.last[i] = None
                continue
            if self.first[i] is None:
                self.first[i] = c
            dists[i] = (c - self.first[i]) * k
            if dt is not None and self.last[i] is not None:
                speeds[i] = (c - self.last[i]) * k / dt
            self.last[i] = c

        self.dist_l, self.dist_r = dists
        self.last_ms = ms
        return tuple(speeds)


class OrientationState:
    """Complementary filter on the PC side; dt from the packet's own ms."""

    def __init__(self):
        self.pitch = self.roll = self.yaw = 0.0
        self.last_ms = None

    def zero_yaw(self):
        self.yaw = 0.0

    def update(self, ax, ay, az, gx, gy, gz, ms):
        agx, agy, agz = ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G
        dx, dy, dz = gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS
        if self.last_ms is not None and ms > self.last_ms:
            dt = (ms - self.last_ms) / 1000.0
            acc_pitch = math.degrees(math.atan2(-agx, math.hypot(agy, agz)))
            acc_roll = math.degrees(math.atan2(agy, agz))
            self.pitch = COMPLEMENTARY_ALPHA * (self.pitch + dy * dt) + (1 - COMPLEMENTARY_ALPHA) * acc_pitch
            self.roll = COMPLEMENTARY_ALPHA * (self.roll + dx * dt) + (1 - COMPLEMENTARY_ALPHA) * acc_roll
            self.yaw += dz * dt
        self.last_ms = ms
        return agx, agy, agz, dx, dy, dz



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

    def update(self, name, values, ms):
        """Store one sensor's packet. Returns the list of keys it touched."""
        keys = []
        for i, raw in enumerate(values):
            invalid = raw == SENSOR_VALUE_INVALID
            key = (name, i)
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

    def median_mm(self, name, index):
        """Median of the recent valid readings for one channel, or None if there are none.

        The firmware deliberately sends any distance inside the sensor's physical range
        without consulting ST's RangeStatus, because that status rejects plenty of good
        measurements and a signal-rate threshold would only be calibrated to whatever room
        it was measured in. The cost is that occasional noisy or short samples arrive, and
        a median over a few samples is what removes them -- here, where it can be changed
        without reflashing a board bolted to the robot.

        Use this rather than mm() for anything that steers; use mm() when you want the
        latest raw sample, such as a live plot.
        """
        hist = self.recent.get((name, index))
        if not hist:
            return None
        return sorted(hist)[len(hist) // 2]

    def closest_mm(self):
        """Smallest valid distance across every channel, or None if nothing is in range.

        This is the figure obstacle avoidance actually wants, and keeping it here means
        callers never have to remember to filter the invalid readings out first.
        """
        medians = [self.median_mm(*key) for key in self.channels]
        valid = [m for m in medians if m is not None]
        return min(valid) if valid else None
