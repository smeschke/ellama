"""
Shared code for reading computer_bridge.ino's serial output and turning it
into distances and angles. Used by live_view.py.

computer_bridge.ino is listen-only until it is sent a drive line, and the
scripts here never send one, so they can run while you drive with the stick.

Serial lines this understands:
  ENC <left> <right> <ms>                  cumulative AS5600 counts per wheel
  IMU <ax> <ay> <az> <gx> <gy> <gz> <ms>   raw accel LSB (+-2g), bias-corrected gyro LSB (+-250 dps)

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

# IMU scale -- must match the robot_imu_* sketch config (+-2g, +-250 dps)
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
COMPLEMENTARY_ALPHA = 0.98

CONFIG_PATH = Path(__file__).with_name("config.json")
DEFAULT_CONFIG = {
    "wheel_diameter_in": 10.0,
    "ticks_per_rev": 4096,       # AS5600, per revolution of the MAGNET shaft (not the wheel)
    "gear_ratio": 6.33,          # encoder-shaft turns per wheel turn; empirical, same as the bench sketch
    "track_width_in": 20.0,      # PLACEHOLDER until calibrated -- wheel-to-wheel distance
}


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        cfg.update(json.loads(CONFIG_PATH.read_text()))
    except FileNotFoundError:
        pass
    return cfg


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


def parse_line(raw):
    """Return ("ENC", (left, right, ms)), ("IMU", (ax, ay, az, gx, gy, gz, ms)),
    or (None, decoded_text) for anything else (bridge chatter, garbage)."""
    text = raw.decode("ascii", errors="ignore").strip()
    parts = text.split()
    try:
        if parts and parts[0] == "ENC" and len(parts) == 4:
            return "ENC", tuple(int(x) for x in parts[1:4])
        if parts and parts[0] == "IMU" and len(parts) == 8:
            return "IMU", tuple(int(x) for x in parts[1:8])
    except ValueError:
        pass
    return None, text


# ---------------- state ----------------

class EncoderState:
    """Per-wheel distance/speed since the last zero(), plus a differential-drive
    heading estimate from the wheel difference."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.first = None      # (left, right) counts at the zero point
        self.last = None       # (left, right) counts at the previous sample
        self.last_ms = None
        self.dist_l = self.dist_r = 0.0

    @property
    def dist_avg(self):
        return (self.dist_l + self.dist_r) / 2.0

    @property
    def heading_deg(self):
        # Positive when the right wheel has travelled farther (turning left/CCW).
        return math.degrees((self.dist_r - self.dist_l) / self.cfg["track_width_in"])

    def zero(self):
        """Make the current position the origin for distance and heading."""
        self.first = self.last
        self.dist_l = self.dist_r = 0.0

    def update(self, left, right, ms):
        k = in_per_tick(self.cfg)
        if self.first is None:
            self.first = (left, right)
        self.dist_l = (left - self.first[0]) * k
        self.dist_r = (right - self.first[1]) * k
        speed_l = speed_r = None
        if self.last_ms is not None and ms > self.last_ms:
            dt = (ms - self.last_ms) / 1000.0
            speed_l = (left - self.last[0]) * k / dt
            speed_r = (right - self.last[1]) * k / dt
        self.last, self.last_ms = (left, right), ms
        return speed_l, speed_r


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
