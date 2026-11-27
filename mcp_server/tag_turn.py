#!/usr/bin/env python3
"""tag-turn module: Show AprilTags, plus a Center button that turns the robot onto a tag.

Registered by register(mcp, ctx) from robot_server. Page: GET /tag on the daemon.
Vision is apriltag_module.VIEW (the same camera connection and detection as the
Show AprilTags page); the moves are vision_drive.center, shared with the plan page.

Drive lease: held for as long as the page is open (vision_drive.PageLease, holder
"tag-turn", shown on the operator page). Close the page and within 3 s the robot is
braked and the lease released.

Center spins toward the tag at a PWM stepped by how far off it is (50, 63, 75, 88,
100); inside 3 deg it brakes, waits for the camera to catch up, and looks again.
It brakes at once on: tag out of the frame (gives up after 0.5 s), camera frozen,
Stop, the hub STOP latch, a guard refusing, losing the lease, or 20 s without
centering. Pressing Center is the go-ahead for one run.
"""

import sys
import threading
import time
from pathlib import Path

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vision_drive as vd  # noqa: E402

HOLDER = "tag-turn"
DEADBAND_DEG = vd.DEADBAND_DEG
PWM_STEPS = vd.SPIN_STEPS
pwm_for = vd.spin_pwm


class TagTurn:
    def __init__(self, ctx, vision=None):
        self.ctx = ctx
        self._vision = vision          # None = the shared apriltag_module.VIEW
        self.lock = threading.Lock()
        self.thread = None
        self.stop_evt = threading.Event()
        self.phase = "idle"
        self.note = "press Center to turn onto the tag"
        self.pwm = 0
        self.log = []
        self.page = vd.PageLease(ctx, HOLDER, self._log, on_quiet=self._quiet)

    @property
    def view(self):
        return self._vision if self._vision is not None else vd._vision()

    @property
    def lease(self):
        return self.page.token

    def _log(self, text):
        with self.lock:
            self.log.append(time.strftime("%H:%M:%S ") + text)
            self.log = self.log[-60:]

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def ping(self, who, cam=None):
        """The page is open: keep the camera and the lease."""
        self.view.use(cam or None)
        self.page.ping(who or "tag turn page")

    def watchdog(self):
        return self.page.watchdog()

    def _quiet(self):
        self.stop_evt.set()
        if self.thread is not None:
            self.thread.join(2.0)
        try:
            if self.ctx.link is not None:
                self.ctx.link.send_stop()
        except Exception:
            pass

    def center(self, tag_id, who):
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                return 409, {"ok": False, "error": "already turning"}
        if self.lease is None:
            return 409, {"ok": False, "error": "this page does not hold the drive lease: "
                         + (self.page.error or "open the page first")}
        if self.ctx.link is None:
            return 409, {"ok": False, "error": "no robot connected"}
        if self.ctx.OPSTOP.latched:
            return 409, {"ok": False, "error": "operator stop is latched: Resume on the operator page"}
        self.stop_evt.clear()
        self._set(phase="turning", note="starting", pwm=0)
        t = threading.Thread(target=self._run, args=(tag_id, who), name="tag-turn", daemon=True)
        with self.lock:
            self.thread = t
        t.start()
        return 200, {"ok": True}

    def stop(self, who):
        self.stop_evt.set()
        try:
            if self.ctx.link is not None:
                self.ctx.link.send_stop()
        except Exception:
            pass
        self._log("Stop by %s" % who)
        return 200, {"ok": True}

    def _run(self, tag_id, who):
        drv = vd.Driver(self.ctx, self.lease, self.stop_evt, self._log, view=self.view,
                        status=lambda note, pwm=0: self._set(note=note, pwm=pwm))
        want = "tag %d" % tag_id if tag_id is not None else "the biggest tag"
        self._log("Center on %s by %s (hfov %g, %s)" % (want, who, drv.hfov, drv.hfov_source))
        ok, note = vd.center(drv, tag_id)
        phase = "done" if ok else ("stopped" if note == "stopped" else "failed")
        self._set(phase=phase, note=note, pwm=0)

    def status(self):
        hfov, src = vd.hfov_deg()
        with self.lock:
            out = {"phase": self.phase, "note": self.note, "pwm": self.pwm,
                   "log": list(self.log), "deadband_deg": DEADBAND_DEG,
                   "hfov_deg": hfov, "hfov_source": src}
        out["holding_lease"] = self.lease is not None
        out["lease_error"] = self.page.error
        out["drive_lease"] = self.ctx.LEASE.snapshot()
        out["vision"] = self.view.status()
        return out


SESSION = None


def _watchdog_loop():
    while True:
        try:
            if SESSION is not None:
                SESSION.watchdog()
        except Exception:
            pass
        time.sleep(0.2)


def tag_turn_status() -> dict:
    """Read-only: the tag turn page's state, its drive lease and its log. Never drives;
    centering is started from the /tag page, which holds the lease while it is open."""
    return SESSION.status()


def register(mcp, ctx):
    global SESSION
    SESSION = TagTurn(ctx)
    mcp.tool()(tag_turn_status)
    threading.Thread(target=_watchdog_loop, name="tag-turn-watchdog", daemon=True).start()

    def who_of(request, body=None):
        w = (body or {}).get("who") or (request.client.host if request.client else "")
        return "tag turn page (%s)" % str(w)[:60]

    async def body_of(request):
        import json
        try:
            b = json.loads((await request.body()).decode() or "{}")
            return b if isinstance(b, dict) else {}
        except Exception:
            return {}

    @mcp.custom_route("/tag", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/tag/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        q = request.query_params
        SESSION.ping(who_of(request), q.get("cam"))
        st = SESSION.status()
        tag = q.get("tag_id")
        try:
            tag = int(tag) if tag not in (None, "") else None
        except ValueError:
            tag = None
        res = SESSION.view.result
        b, _, tid = vd.sight(res, tag, st["hfov_deg"]) if res else (None, None, None)
        st["live_bearing_deg"] = None if b is None else round(b, 1)
        st["live_tag_id"] = tid
        return JSONResponse(st)

    @mcp.custom_route("/tag/center", methods=["POST"], include_in_schema=False)
    async def center(request: Request):
        b = await body_of(request)
        tag = b.get("tag_id")
        try:
            tag = int(tag) if tag not in (None, "") else None
        except (TypeError, ValueError):
            return JSONResponse({"ok": False, "error": "tag id must be a number"}, status_code=400)
        code, out = SESSION.center(tag, who_of(request, b))
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/tag/stop", methods=["POST"], include_in_schema=False)
    async def stop(request: Request):
        b = await body_of(request)
        code, out = SESSION.stop(who_of(request, b))
        return JSONResponse(out, status_code=code)


# Raw string: a "\n" here must reach the browser as backslash-n, not a line break
# inside a JavaScript string (that syntax error is what blanked the old page).
PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama tag turn</title>
<style>body{font-family:system-ui,sans-serif;margin:12px;background:#161616;color:#e8e8e8}
h1{font-size:1.2em}h2{font-size:1em;margin-top:18px}
.view{position:relative;display:inline-block;width:min(92vw,640px)}
.view img{display:block;width:100%;min-height:120px;border:1px solid #3a3a3a;border-radius:8px;background:#1e1e1e}
.mid{position:absolute;top:0;bottom:0;left:50%;border-left:2px dashed #5aa0ff;opacity:.7}
#s,#cfg,#lease{color:#9a9a9a}#b{font-size:1.4em;margin:8px 0}
input{font:inherit;padding:6px 10px;border-radius:8px;border:1px solid #3a3a3a;background:#1e1e1e;color:#e8e8e8}
button{font:inherit;padding:10px 22px;margin:4px;border-radius:8px;border:0;color:#fff}
#go{background:#2e6b3a}#go:disabled{background:#3a3a3a;color:#888}#stop{background:#c62828}
table{border-collapse:collapse;margin-top:8px}td,th{padding:4px 14px;border-bottom:1px solid #3a3a3a;text-align:left}
pre{white-space:pre-wrap;color:#c8c8c8}</style></head><body>
<h1>Tag turn: turn the robot onto an AprilTag</h1>
<p>Camera: <input id=cam size=22 placeholder="192.168.0.52:4444">
Family: <input id=fam size=10 value="tag36h11" disabled>
Tag id (blank = biggest): <input id=tid size=5></p>
<p><button id=go disabled>Center</button> <button id=stop>Stop</button></p>
<p id=lease>checking the drive lease&hellip;</p>
<div class=view><img id=f alt="tags appear here once the camera connects"><div class=mid></div></div>
<p id=b>&nbsp;</p>
<p id=s>enter the camera address&hellip;</p>
<p id=cfg></p>
<table><thead><tr><th>Tag id</th><th>Centre (px)</th></tr></thead><tbody id=tags></tbody></table>
<h2>What this page did</h2>
<pre id=log>(nothing yet)</pre>
<script>
const $ = id => document.getElementById(id);
try { const c = localStorage.getItem('ellama.apriltag.cam'); if (c) $('cam').value = c; } catch (e) {}
$('cam').addEventListener('change', () => {
  try { localStorage.setItem('ellama.apriltag.cam', $('cam').value); } catch (e) {} });
let streaming = '';
async function tick(){
  const cam = $('cam').value.trim();
  try {
    const r = await fetch('/tag/state?cam=' + encodeURIComponent(cam)
                          + '&tag_id=' + encodeURIComponent($('tid').value.trim()), {cache:'no-store'});
    const s = await r.json();
    const l = s.drive_lease || {};
    $('lease').textContent = s.holding_lease
      ? 'This page holds the drive lease (released when you close it).'
      : 'No drive lease: ' + (s.lease_error || 'not taken yet');
    $('go').disabled = !s.holding_lease || s.phase === 'turning' || !cam;
    const lb = s.live_bearing_deg;
    $('b').textContent = lb == null ? 'tag not in view'
      : 'tag ' + s.live_tag_id + ': ' + Math.abs(lb).toFixed(1) + '° ' + (lb > 0 ? 'right' : 'left')
        + (Math.abs(lb) <= s.deadband_deg ? '  (centred)' : '');
    const v = s.vision || {};
    $('s').textContent = s.phase + ': ' + s.note + (s.pwm ? ' (pwm ' + s.pwm + ')' : '')
      + (v.note ? '  |  ' + v.note : '');
    $('cfg').textContent = 'field of view ' + s.hfov_deg + '° (' + s.hfov_source + ')';
    $('tags').innerHTML = (v.tags || []).map(t =>
      '<tr><td>' + t.id + '</td><td>' + t.center_px[0] + ', ' + t.center_px[1] + '</td></tr>').join('');
    if (s.log && s.log.length) $('log').textContent = s.log.join('\n');
    if (cam && streaming !== cam) { streaming = cam; $('f').src = '/apriltag/stream.mjpg?t=' + Date.now(); }
  } catch (e) { $('s').textContent = 'daemon not answering'; }
}
$('f').addEventListener('error', () => { streaming = ''; });
async function post(p, b){
  const r = await fetch(p, {method:'POST', headers:{'content-type':'application/json'}, body: JSON.stringify(b)});
  try { return await r.json(); } catch (e) { return {}; }
}
$('go').onclick = async () => {
  const t = $('tid').value.trim();
  const b = await post('/tag/center', {tag_id: t === '' ? null : t});
  if (b && b.ok === false) $('s').textContent = 'Center refused: ' + (b.error || '?');
};
$('stop').onclick = () => post('/tag/stop', {});
setInterval(tick, 500); tick();
</script></body></html>"""
