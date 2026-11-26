#!/usr/bin/env python3
"""tag-turn module: watch for a tag-like shape, center the robot on it.

Registered by register(mcp, ctx) from robot_server, like watch_and_repeat.
Its page lives at GET /tag on the daemon, separate from the operator page
(hub) and the route page. One window per job.

One persistent MJPEG stream per camera, held open server-side and shared
by the watch poll, the frame view, and the center loop. Opening a fresh
connection per poll made the phone click (stream start/stop) and doubled
the latency; the cache means every consumer reads the latest frame
instantly and the phone sees exactly one viewer.

Workflow (buttons on the page, or the tag_center MCP tool):
  Watch      the page polls /tag/state, which detects on the cached frame.
             No camera, or a dark stream, is reported, not hidden.
  Center     turn toward the tag in announced steps (<=5deg) until within
             3deg, up to 10 tries. Pressing Center is the human confirmation
             for the run. The hub STOP latch overrules everything.

Moves go through the guarded drive tools on ctx (operator stop, tilt guard,
sensor policy, lease all apply). The module holds its own lease as
"tag-module" and releases it when the run ends, aborts, or the thread dies.
"""

import os
import threading
import time
import urllib.request

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse, Response

CTX = None
HOLDER = "tag-module"

TOL_DEG = 3.0
STEP_DEG = 5.0
MAX_TRIES = 10
SCAN_PWM = 60
FRAME_STALE_S = 4.0  # cached frame older than this = stream trouble

DICT_NAMES = ["DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50",
              "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11",
              "DICT_APRILTAG_25h9"]


def _trace_event(kind, **fields):
    try:
        import trace as _tr
        _tr.event(kind, **fields)
    except Exception:
        pass


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
                        chunk = r.read(32768)
                        if not chunk:
                            break  # server closed; reconnect below
                        buf += chunk
                        if len(buf) > 2_000_000:
                            buf = buf[-500_000:]
                        soi = buf.find(b"\xff\xd8\xff")
                        if soi < 0:
                            continue
                        eoi = buf.find(b"\xff\xd9", soi + 3)
                        if eoi > soi:
                            with self.lock:
                                self.jpg = bytes(buf[soi:eoi + 2])
                                self.t = _t.monotonic()
                            buf = buf[eoi + 2:]
            except Exception:
                with self.lock:
                    if gen != self.gen:
                        return
                _t.sleep(2.0)  # back off, then reconnect

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


def detect_any(frame_bytes):
    """Every tag-like shape: [(dict_name, tag_id, cx_px, size_px, w_px)]."""
    import cv2
    import numpy as np
    img = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8),
                       cv2.IMREAD_GRAYSCALE)
    if img is None:
        return []
    h, w = img.shape
    aruco = cv2.aruco
    out = []
    for name in DICT_NAMES:
        if not hasattr(aruco, name):
            continue
        det = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(getattr(aruco, name)),
            aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(img)
        if ids is None:
            continue
        for i, tag_id in enumerate(ids.flatten().tolist()):
            pts = corners[i].reshape(4, 2)
            out.append((name, tag_id, float(pts[:, 0].mean()),
                        float(max(pts[:, 0].max() - pts[:, 0].min(),
                                  pts[:, 1].max() - pts[:, 1].min())), w))
    return out


def sighting_on_frame(frame, tag_id=None, fov_deg=60.0):
    """(found, bearing_deg, label) from one cached JPEG."""
    if frame is None:
        return False, 0.0, "no frame yet (stream connecting?)"
    dets = detect_any(frame)
    if tag_id is not None:
        dets = [d for d in dets if d[1] == tag_id]
    if not dets:
        return False, 0.0, "nothing in view"
    dets.sort(key=lambda d: d[3], reverse=True)
    _, tid, cx, size, w = dets[0]
    bearing = (cx - w / 2.0) / (w / 2.0) * (fov_deg / 2.0)
    return True, bearing, f"tag {tid}"


class TagSession:
    def __init__(self, ctx):
        self.ctx = ctx
        self.lock = threading.Lock()
        self.phase = "idle"
        self.note = "enter the phone address, then watch"
        self.tries = 0
        self.bearing = 0.0
        self.camera_url = os.environ.get("ELLAMA_CAMERA_URL_ARROW",
                                         os.environ.get("ELLAMA_CAMERA_URL", ""))
        self.tag_id = None
        self.stop_evt = threading.Event()
        self.thread = None
        self.lease = None
        self.cmdlog = []
        self.stream = StreamCache()
        if self.camera_url:
            self.stream.ensure(self.camera_url)

    def log_cmd(self, text):
        with self.lock:
            self.cmdlog.append(f"{time.strftime('%H:%M:%S')} {text}")
            self.cmdlog = self.cmdlog[-30:]

    def status(self):
        with self.lock:
            age = self.stream.age()
            return {"phase": self.phase, "note": self.note,
                    "tries": self.tries, "bearing_deg": round(self.bearing, 1),
                    "camera": bool(self.camera_url),
                    "frame_age_s": None if age is None else round(age, 1),
                    "tag_id": self.tag_id if self.tag_id is not None else "any",
                    "lease_held": self.lease is not None,
                    "log": list(self.cmdlog)}

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def use_camera(self, camera_url):
        if camera_url:
            self._set(camera_url=camera_url)
            self.stream.ensure(camera_url)
        return self.camera_url

    def look(self, tag_id=None):
        """Detect on the cached frame. Never drives, never opens a stream."""
        if not self.camera_url:
            self._set(note="no camera: enter the phone address above")
            return False, 0.0, "no camera"
        jpg = self.stream.get()
        if jpg is None:
            self._set(note="stream connecting or dark (one viewer per phone; "
                           "close other tabs on this stream?)")
            return False, 0.0, "no frame"
        found, bearing, label = sighting_on_frame(jpg, tag_id)
        with self.lock:
            if found:
                self.bearing = bearing
                self.note = f"{label} at {bearing:+.1f}deg"
            else:
                self.note = "watching: nothing in view"
        return found, bearing, label

    def center(self, tag_id, camera_url, who):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return 409, {"ok": False, "error": "a centering run is already going"}
            if camera_url:
                self.camera_url = camera_url
                self.stream.ensure(camera_url)
            if not self.camera_url:
                return 409, {"ok": False, "error": "no camera: enter the phone address first"}
            self.tag_id = tag_id
            self.tries = 0
            self.phase = "centering"
            self.note = "starting"
            self.stop_evt.clear()
            self.thread = threading.Thread(target=self._run, args=(who,),
                                           daemon=True)
            self.thread.start()
            return 200, {"ok": True}

    def abort(self, who):
        self.stop_evt.set()
        self.log_cmd(f"ABORT by {who}")
        self._set(note=f"abort by {who}")
        return 200, {"ok": True}

    def _fresh_frame(self, timeout_s=12.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.stop_evt.is_set():
                return None
            jpg = self.stream.get()
            if jpg is not None:
                return jpg
            time.sleep(0.2)
        return None

    def _run(self, who):
        ctx = self.ctx
        self.log_cmd(f"lease request by {who}")
        try:
            lr = ctx.take_drive_lease(HOLDER, who, "tag center")
            if not lr.get("ok"):
                self.log_cmd(f"LEASE REFUSED: {lr.get('error', '?')}")
                self._set(phase="failed",
                          note=f"no lease: {lr.get('error', '?')}")
                return
            self.lease = lr["lease"]
            self.log_cmd(f"lease held: {self.lease[:8]}...")
            _trace_event("tag_center_start", who=who)
            for n in range(1, MAX_TRIES + 1):
                if self.stop_evt.is_set():
                    self.log_cmd("abort flag -> stopping")
                    self._set(phase="aborted")
                    return
                self._set(tries=n)
                jpg = self._fresh_frame()
                if jpg is None:
                    self.log_cmd("stream dark mid-run -> stopping")
                    self._set(phase="failed",
                              note="stream went dark mid-run -- stopping")
                    return
                found, bearing, label = sighting_on_frame(jpg, self.tag_id)
                self._set(bearing=bearing if found else 0.0)
                if not found:
                    self.log_cmd(f"try {n}: tag lost -> stopping")
                    self._set(phase="failed",
                              note=f"tag lost on try {n} -- stopping")
                    return
                if abs(bearing) <= TOL_DEG:
                    self.log_cmd(f"try {n}: {label} {bearing:+.1f}deg "
                                 "within tolerance -> DONE")
                    self._set(phase="done",
                              note=f"centered {bearing:+.1f}deg after {n}")
                    _trace_event("tag_center_done", tries=n)
                    return
                step = min(abs(bearing), STEP_DEG)
                direction = "right" if bearing > 0 else "left"
                cmd = (f"turn_degrees({step:.1f}, {direction}, "
                       f"pwm={SCAN_PWM})  [{label} {bearing:+.1f}deg]")
                self.log_cmd(f"try {n}: DRIVE {cmd}")
                self._set(note=f"try {n}: {label} {bearing:+.1f}deg "
                               f"-> turn {step:.1f} {direction}")
                r = ctx.turn_degrees(step, direction, SCAN_PWM, True,
                                     lease=self.lease)
                if not r.get("ok"):
                    self.log_cmd(f"TURN REFUSED: {r.get('error', '?')}")
                    self._set(phase="failed",
                              note=f"turn refused: {r.get('error', '?')}")
                    return
                actual = r.get("actual_degrees_turned", "?")
                self.log_cmd(f"  -> ok, turned {actual}deg")
            self.log_cmd(f"not centered after {MAX_TRIES} tries")
            self._set(phase="failed",
                      note=f"not centered after {MAX_TRIES} tries")
        finally:
            if self.lease:
                try:
                    ctx.release_drive_lease(self.lease)
                    self.log_cmd("lease released")
                except Exception as e:
                    self.log_cmd(f"lease release failed: {e}")
                self.lease = None
            if self.phase == "centering":
                self._set(phase="idle")


def register(mcp, ctx):
    global CTX, SESSION
    CTX = ctx
    SESSION = TagSession(ctx)

    def center_tool(tag_id=None, camera_url=""):
        """Watch for a tag-like shape and center the robot on it. tag_id
        None means any tag in any known dictionary. Pressing Center on the
        /tag page is the human confirmation; the hub STOP latch overrules."""
        code, out = SESSION.center(tag_id, camera_url, "mcp")
        return out

    mcp.tool()(center_tool)

    @mcp.custom_route("/tag", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/tag/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        import anyio
        q = request.query_params
        if q.get("cam"):
            SESSION.use_camera(q["cam"])
        tag = q.get("tag_id")
        try:
            tag = int(tag) if tag not in (None, "") else None
        except ValueError:
            tag = None
        found, bearing, label = await anyio.to_thread.run_sync(
            SESSION.look, tag)
        st = SESSION.status()
        st["seen"] = found
        return JSONResponse(st)

    @mcp.custom_route("/tag/frame", methods=["GET"], include_in_schema=False)
    async def frame(request: Request):
        cam = request.query_params.get("cam", "")
        if cam:
            SESSION.use_camera(cam)
        jpg = SESSION.stream.get(max_age_s=30.0)
        if jpg is None:
            return JSONResponse({"error": "no frame yet (stream connecting?)"},
                                status_code=503)
        return Response(jpg, media_type="image/jpeg",
                        headers={"cache-control": "no-store"})

    @mcp.custom_route("/tag/center", methods=["POST"], include_in_schema=False)
    async def center(request: Request):
        import json as _json
        try:
            b = _json.loads((await request.body()).decode() or "{}")
        except Exception:
            b = {}
        who = str(b.get("who") or
                  (request.client.host if request.client else "tag page"))[:80]
        code, out = SESSION.center(b.get("tag_id"), b.get("camera_url", ""),
                                   who)
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/tag/abort", methods=["POST"], include_in_schema=False)
    async def abort(request: Request):
        import json as _json
        try:
            b = _json.loads((await request.body()).decode() or "{}")
        except Exception:
            b = {}
        who = str(b.get("who") or "tag page")[:80]
        code, out = SESSION.abort(who)
        return JSONResponse(out, status_code=code)


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama tag turn</title>
<style>body{font-family:system-ui,sans-serif;margin:12px;background:#161616;color:#e8e8e8}
h1{font-size:1.2em}img{max-width:min(92vw,640px);border:1px solid #3a3a3a;border-radius:8px;background:#1e1e1e}
#s{white-space:pre-wrap;color:#9a9a9a}button{font:inherit;padding:8px 16px;margin:4px;border-radius:8px;border:0}
#go{background:#2e6b3a;color:#fff}#stop{background:#c62828;color:#fff}
input{font:inherit;padding:6px 10px;border-radius:8px;border:1px solid #3a3a3a;background:#1e1e1e;color:#e8e8e8}</style></head><body>
<h1>Tag turn (own window; hub STOP overrules)</h1>
<p>Camera: <input id=cam size=28 placeholder="http://192.168.0.52:4444">
Tag id (blank = any tag shape): <input id=tid size=6></p>
<p><button id=go>Center</button> <button id=stop>Abort</button></p>
<img id=f src="" alt="camera (set the address above)">
<p id=s>enter the phone address, then watching starts&hellip;</p>
<h2>Drive commands this window sent</h2>
<pre id=log>(none yet — press Center to run)</pre>
<script>
const $ = id => document.getElementById(id);
const q = () => {
  const t = $('tid').value;
  return '?cam=' + encodeURIComponent($('cam').value)
       + (t ? '&tag_id=' + encodeURIComponent(t) : '');
};
function streamSrc(){
  const c = $('cam').value.trim().replace(/\\/+$/, '');
  if (!c) return '';
  if (/(\\.jpg|\\.jpeg|\\.png|\\.mjpg)$/i.test(c) || c.includes('/video')) return c;
  return c + '/video/mjpeg';
}
function syncVideo(){
  const src = streamSrc();
  if (src && $('f').src !== src) $('f').src = src;
}
async function tick(){
  if (!$('cam').value) return;
  try {
    const r = await fetch('/tag/state' + q(), {cache:'no-store'});
    const s = await r.json();
    const age = s.frame_age_s == null ? 'detecting…' : s.frame_age_s + 's old';
    $('s').textContent = `${s.phase}: ${s.note} (tries ${s.tries}, detect frame ${age})`;
    if (s.log && s.log.length) $('log').textContent = s.log.join('\n');
  } catch(e) {}
}
async function post(p, b){
  const r = await fetch(p, {method:'POST', headers:{'content-type':'application/json'}, body: JSON.stringify(b)});
  return await r.text();
}
$('go').onclick = async () => { const t = $('tid').value;
  const r = await post('/tag/center', {tag_id: t === '' ? null : parseInt(t, 10), camera_url: $('cam').value});
  try {
    const b = JSON.parse(r);
    if (b && b.ok === false) $('s').textContent = 'center refused: ' + (b.error || '?');
  } catch(e) {}
};
$('stop').onclick = () => post('/tag/abort', {});
$('cam').addEventListener('change', syncVideo);
setInterval(tick, 1000); syncVideo(); tick();
</script></body></html>"""
