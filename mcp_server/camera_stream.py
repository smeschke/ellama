"""One shared reader for the phone's live MJPEG stream.

The phone app serves a limited number of simultaneous streams, and each extra reader drops
every reader's frame rate (two readers each got ~13 fps instead of ~16.6 on 2026-10-02). So
anything that wants frames -- the line follower, a recorder, a live preview -- subscribes to
one FrameSource instead of opening its own connection.

    src = FrameSource(open_stream)       # open_stream() -> file-like of multipart MJPEG
    src.start()
    f = src.latest()                     # Frame(img, t, seq) or None; never blocks
    src.subscribe(callback)              # callback(frame) on the reader thread, every frame
    src.stop()

`t` is time.monotonic() when the frame finished arriving on the computer (not when the phone
exposed it -- the phone's capture/encode delay is on top of that; see line_follow.py's
CAMERA_DELAY_S). Reconnects by itself if the stream drops.
"""

import threading
import time
from collections import namedtuple

import cv2
import numpy as np

Frame = namedtuple("Frame", "img t seq")


class FrameSource:
    def __init__(self, open_stream):
        self._open = open_stream
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._frame = None
        self._subs = []
        self.seq = 0
        self.reconnects = 0
        self.error = None
        self._times = []  # recent arrival times, for fps

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._thread = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def latest(self):
        with self._lock:
            return self._frame

    def age_s(self):
        """Seconds since the newest frame arrived, or None if there hasn't been one."""
        f = self.latest()
        return None if f is None else time.monotonic() - f.t

    def fps(self):
        with self._lock:
            ts = list(self._times)
        return 0.0 if len(ts) < 2 or ts[-1] == ts[0] else (len(ts) - 1) / (ts[-1] - ts[0])

    def subscribe(self, callback):
        with self._lock:
            self._subs.append(callback)

    def unsubscribe(self, callback):
        with self._lock:
            if callback in self._subs:
                self._subs.remove(callback)

    def status(self):
        f = self.latest()
        return dict(running=self.running, fps=round(self.fps(), 1), frames=self.seq,
                    age_s=None if f is None else round(time.monotonic() - f.t, 2),
                    reconnects=self.reconnects, error=self.error)

    def _run(self):
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
                        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is None:
                            continue
                        self._publish(img)
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

    def _publish(self, img):
        now = time.monotonic()
        with self._lock:
            self.seq += 1
            self._frame = Frame(img, now, self.seq)
            self._times.append(now)
            if len(self._times) > 30:
                self._times.pop(0)
            subs = list(self._subs)
        for cb in subs:
            try:
                cb(self._frame)
            except Exception:
                pass  # a bad subscriber must not kill the stream
