"""Telemetry recording for robot_server.py: one CSV per recording.

The CSV has the same columns as calibration/live_view.py --csv (so analyze_csv.py reads
it) plus a trailing "note" column. A row is written per ENC or IMU sample, holding the
latest value of the other stream, with source "ENC" / "IMU". Any other SensorPacket sender
writes a row with its own name as the source (e.g. "TOF_FRONT") and its values in "note",
so a sensor added later lands in the CSV with no change here. add_marker() writes an
extra row with source "MARK", and a guard or sensor-policy change writes one with source
"CONFIG": same numeric columns (the state at that moment) and the text or setting in
"note".

Columns that can be unknown rather than wrong: a numeric cell holds "nan" when the value
is not known, which for enc_heading_deg means one of the encoders was not responding. It
is never a zero -- a missing encoder must not read as a stationary wheel. "nan" rather
than an empty cell because ellama's analyze_csv.py runs float() over every numeric column,
and float("nan") is a number while float("") raises. Aggregates over such a column come
out NaN, which is the honest answer: part of the run was not measured.

Alongside the CSV, a recording directory can also hold:
  commands.csv     time_s,left,right -- every drive command the bridge overheard (the stick's),
                   stamped on arrival, same time_s clock as the main CSV. Empty if the bridge
                   firmware is too old to relay them.

Lock order: RobotLink.lock first, then Recorder's own lock -- never the reverse.
"""

import csv
import re
import threading
import time
from pathlib import Path

HEADER = ["time_s", "source", "dist_l_in", "dist_r_in", "dist_avg_in",
          "speed_l_in_s", "speed_r_in_s", "enc_heading_deg",
          "pitch_deg", "roll_deg", "yaw_deg", "acc_x_g", "acc_y_g", "acc_z_g",
          "range_closest_mm", "note"]


def _num(value, fmt="%.4f"):
    """Format a number, or "nan" when it is not known. Never substitutes a zero."""
    return "nan" if value is None else fmt % value


class Recorder:
    def __init__(self, base_dir):
        self.base_dir = Path(base_dir)
        self._lock = threading.Lock()
        self._file = self._writer = None
        self._cmd_file = self._cmd_writer = None
        self._cmds = 0
        self.dir = self.csv_path = None
        self._t0 = 0.0
        self._rows = self._markers = 0
        self._speed = (0.0, 0.0)
        self._acc = (0.0, 0.0, 0.0)
        self._range_mm = None

    @property
    def active(self):
        return self._file is not None

    @property
    def speed(self):
        """Latest (left, right) wheel speed in in/s from the most recent ENC sample."""
        return self._speed

    def start(self, name=None):
        """Open recordings/<name>/<name>.csv. Raises if already recording or name exists."""
        with self._lock:
            if self._file is not None:
                raise RuntimeError(f"already recording to {self.csv_path}")
            name = re.sub(r"[^A-Za-z0-9_-]", "_", name) if name else time.strftime("run_%Y%m%d_%H%M%S")
            d = self.base_dir / name
            if d.exists():
                raise RuntimeError(f"{d} already exists; pick another name")
            d.mkdir(parents=True)
            self.dir, self.csv_path = d, d / f"{name}.csv"
            self._file = open(self.csv_path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow(HEADER)
            self._cmd_file = open(d / "commands.csv", "w", newline="")
            self._cmd_writer = csv.writer(self._cmd_file)
            self._cmd_writer.writerow(["time_s", "left", "right"])
            self._cmds = 0
            self._t0 = time.monotonic()
            self._rows = self._markers = 0
            self._speed = (0.0, 0.0)
            self._acc = (0.0, 0.0, 0.0)
            self._range_mm = None
            return self.csv_path

    def stop(self):
        """Close the file. Returns a summary dict, or None if not recording."""
        with self._lock:
            if self._file is None:
                return None
            self._file.close()
            self._file = self._writer = None
            self._cmd_file.close()
            self._cmd_file = self._cmd_writer = None
            return dict(csv=str(self.csv_path), dir=str(self.dir),
                        duration_s=round(time.monotonic() - self._t0, 1),
                        samples=self._rows, markers=self._markers,
                        commands=self._cmds)

    def on_enc(self, enc, orient, speed_l, speed_r):
        self._speed = (speed_l if speed_l is not None else 0.0,
                       speed_r if speed_r is not None else 0.0)
        self._write("ENC", enc, orient, "")

    def on_imu(self, enc, orient, acc):
        self._acc = tuple(acc)
        self._write("IMU", enc, orient, "")

    def on_sensor(self, name, values, enc, orient):
        """One packet from any other SensorPacket sender. The source column is the sensor's
        own name, so nothing here enumerates the sensors that exist."""
        self._write(str(name).upper(), enc, orient, " ".join(str(v) for v in values))

    def on_range(self, closest_mm):
        """Latest closest-obstacle distance, carried on every subsequent row."""
        self._range_mm = closest_mm

    def on_config(self, what):
        """A guard or sensor-policy change, so a run's CSV shows what it was driven under."""
        with self._lock:
            if self._writer is None:
                return None
        return self._write_raw("CONFIG", what)

    def on_cmd(self, left, right):
        """Log an overheard drive command (the human's stick), stamped on arrival."""
        with self._lock:
            if self._cmd_writer is None:
                return
            self._cmd_writer.writerow([f"{time.monotonic() - self._t0:.3f}", left, right])
            self._cmd_file.flush()
            self._cmds += 1

    def now(self):
        """Seconds since this recording started, on the same clock as every row/file."""
        return time.monotonic() - self._t0

    def mark(self, enc, orient, source, note):
        """Write a MARK row carrying the current state. Returns its time_s."""
        t = self._write(source, enc, orient, note)
        if t is not None:
            with self._lock:
                self._markers += 1
        return t

    def _write(self, source, enc, orient, note):
        with self._lock:
            if self._writer is None:
                return None
            t = time.monotonic() - self._t0
            self._writer.writerow([
                f"{t:.3f}", source,
                f"{enc.dist_l:.4f}", f"{enc.dist_r:.4f}", f"{enc.dist_avg:.4f}",
                f"{self._speed[0]:.4f}", f"{self._speed[1]:.4f}",
                # None while either encoder is missing: heading from one live wheel and one
                # frozen one would be a plausible-looking number that drifts without bound.
                _num(enc.heading_deg, "%.3f"),
                f"{orient.pitch:.3f}", f"{orient.roll:.3f}", f"{orient.yaw:.3f}",
                *(f"{a:.4f}" for a in self._acc),
                _num(self._range_mm, "%.0f"), note])
            self._file.flush()
            if source in ("ENC", "IMU"):
                self._rows += 1
            return t

    def _write_raw(self, source, note):
        """A row that carries only the note -- used when there is no enc/orient to hand."""
        with self._lock:
            if self._writer is None:
                return None
            t = time.monotonic() - self._t0
            row = [f"{t:.3f}", source] + ["nan"] * (len(HEADER) - 3) + [note]
            self._writer.writerow(row)
            self._file.flush()
            return t
