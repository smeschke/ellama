"""Telemetry recording for robot_server.py: one CSV per recording plus saved photos.

The CSV has the same columns as calibration/live_view.py --csv (so analyze_csv.py reads
it) plus a trailing "note" column. A row is written per ENC or IMU sample, holding the
latest value of the other stream, with source "ENC" / "IMU". add_marker() and photos write
extra rows with source "MARK" / "PHOTO": same numeric columns (the state at that moment)
and the text or photo filename in "note".

Alongside the CSV, a recording directory can also hold:
  commands.csv     time_s,left,right -- every drive command the bridge overheard (the stick's),
                   stamped on arrival, same time_s clock as the main CSV. Empty if the bridge
                   firmware is too old to relay them.
  video.mp4        the phone camera's stream (VideoRecorder below)
  video_times.csv  frame,time_s -- when each video frame arrived, same clock. Use this, not
                   the nominal mp4 frame rate, to line video up with the telemetry.

Lock order: RobotLink.lock first, then Recorder's own lock -- never the reverse.
"""

import csv
import re
import threading
import time
from pathlib import Path

HEADER = ["time_s", "source", "dist_l_in", "dist_r_in", "dist_avg_in",
          "speed_l_in_s", "speed_r_in_s", "enc_heading_deg",
          "pitch_deg", "roll_deg", "yaw_deg", "acc_x_g", "acc_y_g", "acc_z_g", "note"]


class Recorder:
    def __init__(self, base_dir):
        self.base_dir = Path(base_dir)
        self._lock = threading.Lock()
        self._file = self._writer = None
        self._cmd_file = self._cmd_writer = None
        self._cmds = 0
        self.dir = self.csv_path = None
        self._t0 = 0.0
        self._rows = self._markers = self._photos = 0
        self._speed = (0.0, 0.0)
        self._acc = (0.0, 0.0, 0.0)

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
            self._rows = self._markers = self._photos = 0
            self._speed = (0.0, 0.0)
            self._acc = (0.0, 0.0, 0.0)
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
                        samples=self._rows, markers=self._markers, photos=self._photos,
                        commands=self._cmds)

    def on_enc(self, enc, orient, speed_l, speed_r):
        self._speed = (speed_l if speed_l is not None else 0.0,
                       speed_r if speed_r is not None else 0.0)
        self._write("ENC", enc, orient, "")

    def on_imu(self, enc, orient, acc):
        self._acc = tuple(acc)
        self._write("IMU", enc, orient, "")

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
        """Write a MARK / PHOTO row carrying the current state. Returns its time_s."""
        t = self._write(source, enc, orient, note)
        if t is not None:
            with self._lock:
                if source == "PHOTO":
                    self._photos += 1
                else:
                    self._markers += 1
        return t

    def next_photo_path(self):
        with self._lock:
            return self.dir / f"photo_{self._photos + 1:03d}.jpg"

    def _write(self, source, enc, orient, note):
        with self._lock:
            if self._writer is None:
                return None
            t = time.monotonic() - self._t0
            self._writer.writerow([
                f"{t:.3f}", source,
                f"{enc.dist_l:.4f}", f"{enc.dist_r:.4f}", f"{enc.dist_avg:.4f}",
                f"{self._speed[0]:.4f}", f"{self._speed[1]:.4f}", f"{enc.heading_deg:.3f}",
                f"{orient.pitch:.3f}", f"{orient.roll:.3f}", f"{orient.yaw:.3f}",
                *(f"{a:.4f}" for a in self._acc), note])
            self._file.flush()
            if source in ("ENC", "IMU"):
                self._rows += 1
            return t


class VideoRecorder:
    """Records an MJPEG stream to <dir>/video.mp4 plus <dir>/video_times.csv.

    A thread reads `open_stream()` (a callable returning a file-like of multipart MJPEG),
    cuts it into JPEG frames, and writes each to the mp4 and its arrival time (via `clock`,
    seconds since the recording began) to the times file. The mp4 is written at a nominal
    `fps`; the phone's real rate wanders, so the times file is the truth for syncing. If the
    stream drops it reconnects until stop(). Frames are written as received -- orientation
    and size are the phone's.
    """

    def __init__(self, open_stream, clock, out_dir, fps=15.0):
        self._open, self._clock, self.dir, self.fps = open_stream, clock, Path(out_dir), fps
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0
        self.reconnects = 0
        self.error = None
        self.size = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        return dict(video=str(self.dir / "video.mp4"), frames=self.frames, size=self.size,
                    reconnects=self.reconnects, error=self.error)

    def _run(self):
        import cv2
        import numpy as np

        writer = None
        times = open(self.dir / "video_times.csv", "w", newline="")
        tw = csv.writer(times)
        tw.writerow(["frame", "time_s"])
        try:
            while not self._stop.is_set():
                try:
                    stream = self._open()
                except Exception as e:
                    self.error = f"stream open failed: {e}"
                    self._stop.wait(1.0)
                    continue
                self.error = None
                buf = b""
                try:
                    while not self._stop.is_set():
                        chunk = stream.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while True:
                            a = buf.find(b"\xff\xd8")
                            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                            if a < 0 or b < 0:
                                break
                            jpg, buf = buf[a:b + 2], buf[b + 2:]
                            t = self._clock()
                            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                            if img is None:
                                continue
                            if writer is None:
                                h, w = img.shape[:2]
                                self.size = (w, h)
                                writer = cv2.VideoWriter(str(self.dir / "video.mp4"),
                                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                                         self.fps, (w, h))
                            elif img.shape[1] != self.size[0] or img.shape[0] != self.size[1]:
                                img = cv2.resize(img, self.size)
                            writer.write(img)
                            tw.writerow([self.frames, f"{t:.3f}"])
                            self.frames += 1
                            if self.frames % 15 == 0:
                                times.flush()
                except Exception as e:
                    self.error = f"stream read failed: {e}"
                finally:
                    try:
                        stream.close()
                    except Exception:
                        pass
                if not self._stop.is_set():
                    self.reconnects += 1
                    self._stop.wait(0.5)
        finally:
            if writer is not None:
                writer.release()
            times.close()
