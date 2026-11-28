#!/usr/bin/env python3
"""show-apriltag module: watch a camera and show the AprilTags it sees. Never drives.

Registered by register(mcp, ctx) from robot_server, like tag_turn and
plan_module. Its page lives at GET /apriltag on the daemon, linked from the operator
page. Detection is tools/find_tags.py (the official AprilRobotics library).

  Page      enter the camera address, see the latest frame with each tag
            outlined and labelled, and a list of tag ids and pixel centres.
  MCP tool  apriltags_in_view() returns the same list, so an agent can look
            without opening the page.

One persistent MJPEG connection to the camera (StreamCache, below). A worker
thread detects on every new frame and the page shows the result as a live MJPEG
stream (/apriltag/stream.mjpg), so picture and outlines always match. Nobody
watching for IDLE_S: the worker stops and the camera connection is closed.
"""

import re
import sys
import threading
import time
import urllib.request
from pathlib import Path

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse, Response, StreamingResponse

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import find_tags  # noqa: E402

DEFAULT_FAMILY = "tag36h11"
FRAME_MAX_AGE_S = 4.0
IDLE_S = 10.0          # no viewer for this long: stop detecting, close the camera
DETECT_THREADS = 4     # 640x480: ~24 ms with 1 thread, ~7 ms with 4


FRAME_STALE_S = 4.0  # cached frame older than this = stream trouble


def stream_url(url):
    """Bare phone address -> its MJPEG stream. Video only, never snapshots."""
    if not url:
        return ""
    base = url.rstrip("/")
    if base.endswith((".jpg", ".jpeg", ".png", ".mjpg")) or "/video" in base:
        return url
    return base + "/video/mjpeg"


class StreamCache:
    """One persistent MJPEG connection; consumers read the latest frame.

    The reader thread opens the stream once and keeps it open, replacing
    the cached frame as complete JPEGs arrive. A second consumer never
    opens a second connection, so the phone neither clicks nor throttles.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.url = ""
        self.jpg = None
        self.t = 0.0
        self.gen = 0  # bumped to retire the reader thread
        self.thread = None

    def ensure(self, url):
        url = stream_url(url)
        with self.lock:
            if url and url == self.url and self.thread and self.thread.is_alive():
                return url
            if not url:
                return ""
            self.gen += 1
            self.url = url
            self.jpg = None
            self.t = 0.0
            gen = self.gen
            self.thread = threading.Thread(target=self._read, args=(gen,),
                                           daemon=True)
            self.thread.start()
            return url

    def _read(self, gen):
        import time as _t
        with self.lock:
            url = self.url
        while True:
            try:
                req = urllib.request.Request(
                    url, headers={"User-Agent": "ellama-tag/1.0"})
                with urllib.request.urlopen(req, timeout=10) as r:
                    buf = bytearray()
                    while True:
                        with self.lock:
                            if gen != self.gen:
                                return  # superseded or stopped
                        chunk = r.read1(65536)  # whatever has arrived; read() waits to fill
                        if not chunk:
                            break  # server closed; reconnect below
                        buf += chunk
                        if len(buf) > 2_000_000:
                            buf = buf[-500_000:]
                        # Newest complete JPEG only: taking the oldest one per read
                        # let a backlog build up (seconds of lag) whenever several
                        # frames arrived in one read. Older frames are dropped.
                        eoi = buf.rfind(b"\xff\xd9")
                        soi = buf.rfind(b"\xff\xd8\xff", 0, eoi) if eoi > 0 else -1
                        if soi >= 0:
                            with self.lock:
                                self.jpg = bytes(buf[soi:eoi + 2])
                                self.t = _t.monotonic()
                            buf = buf[eoi + 2:]
            except Exception:
                with self.lock:
                    if gen != self.gen:
                        return
                _t.sleep(2.0)  # back off, then reconnect

    def stop(self):
        """Retire the reader thread and drop the connection."""
        with self.lock:
            self.gen += 1
            self.url = ""
            self.jpg = None

    def get(self, max_age_s=FRAME_STALE_S):
        with self.lock:
            if self.jpg is None:
                return None
            if time.monotonic() - self.t > max_age_s:
                return None
            return self.jpg

    def age(self):
        with self.lock:
            return None if self.jpg is None else time.monotonic() - self.t

    def latest(self, max_age_s=FRAME_STALE_S):
        """(jpg, arrival time on time.monotonic()) of the newest frame, or (None, None)."""
        with self.lock:
            if self.jpg is None or time.monotonic() - self.t > max_age_s:
                return None, None
            return self.jpg, self.t


def normalize_address(raw):
    """'192 168 0 52 4444', '192.168.0.52:4444' or a full URL -> an http(s) URL."""
    s = (raw or "").strip()
    m = re.fullmatch(r"(\d{1,3})[\s.,]+(\d{1,3})[\s.,]+(\d{1,3})[\s.,]+(\d{1,3})"
                     r"(?:[\s:,]+(\d{1,5}))?(/\S*)?", s)
    if m:
        s = "%s.%s.%s.%s:%s%s" % (m[1], m[2], m[3], m[4], m[5] or "8080", m[6] or "")
    if s and not re.match(r"https?://", s, re.I):
        s = "http://" + s
    return s


def annotate(gray, tags):
    """JPEG of the frame with each tag outlined and its id written on it."""
    import cv2
    import numpy as np
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for tag_id, (x, y), corners in tags:
        cv2.polylines(img, [np.asarray(corners, dtype=np.int32)], True, (0, 220, 0), 3)
        cv2.putText(img, str(tag_id), (int(x) - 12, int(y) + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpg.tobytes() if ok else None


class TagView:
    """Detects continuously while someone is watching, stops when nobody is.

    A worker thread runs detection on every new camera frame and keeps the newest
    result (tags + outlined JPEG). Anything that wants tags calls touch(); after
    IDLE_S without a touch the worker exits and the camera connection is closed."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.stream = StreamCache()
        self.address = ""
        self.family = DEFAULT_FAMILY
        self.result = None      # {"tags", "jpg", "t", "seq"} for the newest frame
        self.seq = 0
        self.error = ""
        self.last_touch = 0.0
        self.worker = None
        self._det = (None, None)  # (family, detector), reused across frames

    def use(self, address=None, family=None):
        with self.lock:
            if family:
                self.family = family
            if address is not None and address != self.address:
                self.address, self.result = address, None
                self.stream.stop()
        self.touch()

    def touch(self):
        """Someone wants tags: keep (or start) the camera and the detector."""
        with self.lock:
            self.last_touch = time.monotonic()
            if not self.address:
                return
            self.stream.ensure(normalize_address(self.address))
            if self.worker is None or not self.worker.is_alive():
                self.worker = threading.Thread(target=self._run, name="apriltag-detect",
                                               daemon=True)
                self.worker.start()

    def _detector(self, family):
        from apriltag import apriltag
        if self._det[0] != family:
            self._det = (family, apriltag(family, threads=DETECT_THREADS))
        return self._det[1]

    def _run(self):
        import cv2
        import numpy as np
        seen = None
        while True:
            with self.lock:
                if time.monotonic() - self.last_touch > IDLE_S:
                    self.stream.stop()      # nobody watching: hang up on the camera
                    self.worker = None
                    return
                family = self.family
            jpg, frame_t = self.stream.latest(FRAME_MAX_AGE_S)
            if jpg is None or jpg is seen:
                time.sleep(0.005)
                continue
            seen = jpg
            try:
                gray = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
                tags = find_tags.detect(gray, detector=self._detector(family))
                out, err = annotate(gray, tags), ""
            except Exception as e:  # bad frame or unknown family: say so, keep going
                tags, err = None, "detect failed: %s" % e
            with self.cond:
                self.error = err
                if tags is not None:
                    self.seq += 1
                    # frame_t: when the frame arrived (time.monotonic), so a driver can
                    # refuse frames from before its last move.
                    self.result = {"tags": tags, "jpg": out, "t": time.time(), "seq": self.seq,
                                   "frame_t": frame_t, "size": (gray.shape[1], gray.shape[0])}
                    self.cond.notify_all()

    def wait_new(self, after_seq, timeout_s):
        """The newest result once its seq passes after_seq, or None on timeout."""
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after_seq, timeout_s)
            return self.result if self.seq > after_seq else None

    def status(self):
        with self.lock:
            res, age = self.result, self.stream.age()
            out = {"address": self.address, "stream_url": self.stream.url,
                   "family": self.family, "error": self.error,
                   "detecting": self.worker is not None and self.worker.is_alive(),
                   "frame_age_s": None if age is None else round(age, 1)}
        if not self.address:
            out["note"] = "enter the camera address"
        elif age is None or age > FRAME_MAX_AGE_S:
            out["note"] = "no picture from %s (connecting, or wrong address?)" % (
                out["stream_url"] or normalize_address(self.address))
        out["tags"] = [] if res is None else [
            {"id": int(i), "center_px": [round(float(x)), round(float(y))]}
            for i, (x, y), _ in res["tags"]]
        out["detected_at"] = None if res is None else round(res["t"], 2)
        return out


VIEW = TagView()


def apriltags_in_view(camera_address: str = "", family: str = "") -> dict:
    """Read-only: the AprilTags the camera sees right now (ids and pixel centres). Never
    drives. camera_address like '192.168.0.52:4444'; blank reuses the one already set
    (on the /apriltag page or a previous call). family defaults to tag36h11."""
    with VIEW.lock:
        after = VIEW.seq
    VIEW.use(camera_address or None, family or None)
    VIEW.wait_new(after, 8.0)   # a fresh frame, not one from before the call
    return VIEW.status()


# Set when the daemon is told to stop. uvicorn waits for every open response to finish
# before exiting, and a live stream never finishes by itself, so without this a page
# left open would make Ctrl-C hang until the browser tab closed.
SHUTTING_DOWN = threading.Event()


def _end_streams_on_shutdown():
    try:
        import uvicorn
    except ImportError:
        return
    orig = uvicorn.Server.handle_exit
    if getattr(orig, "_ends_apriltag_streams", False):
        return

    def handle_exit(self, sig, frame):
        SHUTTING_DOWN.set()
        orig(self, sig, frame)

    handle_exit._ends_apriltag_streams = True
    uvicorn.Server.handle_exit = handle_exit


def register(mcp, ctx):
    mcp.tool()(apriltags_in_view)
    _end_streams_on_shutdown()

    @mcp.custom_route("/apriltag", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/apriltag/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        q = request.query_params
        VIEW.use(q.get("cam"), q.get("family") or None)
        return JSONResponse(VIEW.status())

    @mcp.custom_route("/apriltag/stream.mjpg", methods=["GET"], include_in_schema=False)
    async def stream(request: Request):
        """The outlined frames as a live MJPEG stream, one part per detected frame."""
        import anyio
        VIEW.touch()

        async def parts():
            seq = 0
            while not SHUTTING_DOWN.is_set() and not await request.is_disconnected():
                VIEW.touch()
                res = await anyio.to_thread.run_sync(VIEW.wait_new, seq, 1.0)
                if res is None or res["jpg"] is None:
                    continue
                seq = res["seq"]
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                       % len(res["jpg"]) + res["jpg"] + b"\r\n")

        return StreamingResponse(parts(), media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"cache-control": "no-store"})

    @mcp.custom_route("/apriltag/frame.jpg", methods=["GET"], include_in_schema=False)
    async def frame(request: Request):
        res = VIEW.result
        if res is None or res["jpg"] is None:
            return JSONResponse({"error": "no frame yet"}, status_code=503)
        return Response(res["jpg"], media_type="image/jpeg",
                        headers={"cache-control": "no-store"})


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama AprilTags</title>
<style>body{font-family:system-ui,sans-serif;margin:12px;background:#161616;color:#e8e8e8}
h1{font-size:1.2em}img{max-width:min(92vw,640px);border:1px solid #3a3a3a;border-radius:8px;background:#1e1e1e}
#s{color:#9a9a9a}input{font:inherit;padding:6px 10px;border-radius:8px;border:1px solid #3a3a3a;background:#1e1e1e;color:#e8e8e8}
table{border-collapse:collapse;margin-top:8px}td,th{padding:4px 14px;border-bottom:1px solid #3a3a3a;text-align:left}</style></head><body>
<h1>Show AprilTags (watches only, never drives)</h1>
<p>Camera: <input id=cam size=26 placeholder="192.168.0.52:4444">
Family: <input id=fam size=12 value="tag36h11"></p>
<img id=f alt="tags appear here once the camera connects">
<p id=s>enter the camera address&hellip;</p>
<table><thead><tr><th>Tag id</th><th>Centre (px)</th></tr></thead><tbody id=tags></tbody></table>
<script>
const $ = id => document.getElementById(id);
try { const c = localStorage.getItem('ellama.apriltag.cam'); if (c) $('cam').value = c; } catch (e) {}
$('cam').addEventListener('change', () => {
  try { localStorage.setItem('ellama.apriltag.cam', $('cam').value); } catch (e) {} });
let streaming = '';
async function tick(){
  const cam = $('cam').value.trim();
  if (!cam) return;
  try {
    const r = await fetch('/apriltag/state?cam=' + encodeURIComponent(cam)
                          + '&family=' + encodeURIComponent($('fam').value.trim()), {cache:'no-store'});
    const s = await r.json();
    const n = s.tags.length;
    $('s').textContent = s.error || s.note ||
      `${n} tag${n === 1 ? '' : 's'} in view (camera frame ${s.frame_age_s}s old, ${s.stream_url})`;
    $('tags').innerHTML = s.tags.map(t =>
      `<tr><td>${t.id}</td><td>${t.center_px[0]}, ${t.center_px[1]}</td></tr>`).join('');
    if (streaming !== cam) { streaming = cam; $('f').src = '/apriltag/stream.mjpg?t=' + Date.now(); }
  } catch (e) { $('s').textContent = 'daemon not answering'; }
}
// a dropped stream (daemon restart, camera change) comes back on its own
$('f').addEventListener('error', () => { streaming = ''; });
setInterval(tick, 500); tick();
</script></body></html>"""
