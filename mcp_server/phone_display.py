"""Optional live display of what a run sees and does, on the robot phone's screen.

PhoneDisplay takes the annotated camera frame plus the current motor command, draws a command
panel on it, and POSTs it as a JPEG to phone/display_server.py running in Termux on the phone
(its browser page show.html shows the newest frame). All drawing, encoding and network work
happens on this class's own thread; push() only stores the latest frame, so the control loop
can't be slowed or broken by a slow or dead phone. Frames are dropped, never queued.
"""

import threading
import time
import urllib.request

import cv2
import numpy as np

SHOW_WIDTH = 540          # the stream is ~270 px wide; upscale so the panel text is readable
MAX_FPS = 8.0
POST_TIMEOUT_S = 1.5
WHEEL_FULL_SCALE = 125    # raw pwm the bars are scaled to (the policy ceiling)


def draw_hud(img, hud):
    """Overlay the motor command panel on a BGR frame. `hud` keys (all optional): left, right
    (wheel command, + = forward), status, spin (bool), t_s (elapsed), lines (list of str)."""
    h0, w0 = img.shape[:2]
    out = cv2.resize(img, (SHOW_WIDTH, round(h0 * SHOW_WIDTH / w0)), interpolation=cv2.INTER_LINEAR)
    h, w = out.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    # bottom panel with a bar per wheel: grows up for forward (green), down for reverse (red)
    ph = 150
    panel = out[h - ph:h].copy()
    cv2.rectangle(out, (0, h - ph), (w, h), (0, 0, 0), -1)
    out[h - ph:h] = cv2.addWeighted(out[h - ph:h], 0.35, panel, 0.65, 0)
    mid = h - ph // 2
    half = ph // 2 - 14
    for k, (name, v) in enumerate((("L", hud.get("left")), ("R", hud.get("right")))):
        x = 24 + k * 70
        cv2.rectangle(out, (x, mid - half), (x + 40, mid + half), (90, 90, 90), 1)
        cv2.line(out, (x - 4, mid), (x + 44, mid), (160, 160, 160), 1)
        if v is not None:
            n = int(max(-1.0, min(1.0, v / WHEEL_FULL_SCALE)) * half)
            col = (80, 220, 80) if n >= 0 else (60, 60, 230)
            cv2.rectangle(out, (x + 2, mid - n if n > 0 else mid), (x + 38, mid if n > 0 else mid - n), col, -1)
            cv2.putText(out, f"{v:+d}", (x - 2, h - 4), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(out, name, (x + 12, h - ph + 14), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    lines = list(hud.get("lines") or [])
    for i, s in enumerate(lines[:5]):
        cv2.putText(out, s, (170, h - ph + 24 + i * 24), font, 0.58, (255, 255, 255), 1, cv2.LINE_AA)

    # status banner on top
    status = hud.get("status", "")
    col = (0, 200, 0) if status == "running" else (0, 165, 255) if status in ("", "idle") else (0, 0, 255)
    cv2.rectangle(out, (0, 0), (w, 34), (0, 0, 0), -1)
    cv2.putText(out, status.upper() or "-", (10, 24), font, 0.7, col, 2, cv2.LINE_AA)
    if hud.get("spin"):
        cv2.putText(out, "SPIN", (w // 2 - 30, 24), font, 0.7, (0, 220, 255), 2, cv2.LINE_AA)
    if hud.get("t_s") is not None:
        cv2.putText(out, f"{hud['t_s']:.1f}s", (w - 90, 24), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return out


class PhoneDisplay:
    def __init__(self, url, max_fps=MAX_FPS):
        self.url = url.rstrip("/")
        self.min_dt = 1.0 / max_fps
        self.sent = 0
        self.errors = 0
        self.last_error = None
        self._lock = threading.Lock()
        self._item = None
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def check(self):
        """(ok, error). Is the phone's display server reachable? Never raises."""
        try:
            with urllib.request.urlopen(self.url + "/show.html", timeout=3) as r:
                return r.status == 200, None
        except Exception as e:
            return False, str(e)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=3)

    def close(self, final=None):
        """Stop the thread, first sending `final` = (img, hud) (e.g. the end-of-run frame) if given."""
        if final is not None:
            self.push(*final)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                with self._lock:
                    if self._item is None:
                        break
                time.sleep(0.02)
            time.sleep(0.15)       # let the POST that just took it finish
        self.stop()

    def push(self, img, hud):
        """Hand over the newest frame (must not be modified afterwards). Never blocks."""
        with self._lock:
            self._item = (img, hud)
        self._wake.set()

    def status(self):
        return dict(url=self.url, sent=self.sent, errors=self.errors, last_error=self.last_error)

    def _run(self):
        last = 0.0
        while not self._stop.is_set():
            self._wake.wait(0.5)
            self._wake.clear()
            wait = self.min_dt - (time.monotonic() - last)
            if wait > 0:
                self._stop.wait(wait)
            with self._lock:
                item, self._item = self._item, None
            if item is None or self._stop.is_set():
                continue
            last = time.monotonic()
            try:
                shown = draw_hud(*item)
                ok, jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 75])
                req = urllib.request.Request(self.url + "/frame", data=jpg.tobytes(), method="POST",
                                             headers={"Content-Type": "image/jpeg"})
                urllib.request.urlopen(req, timeout=POST_TIMEOUT_S).close()
                self.sent += 1
            except Exception as e:     # a dead phone must never matter to the run
                self.errors += 1
                self.last_error = str(e)
