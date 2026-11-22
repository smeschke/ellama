#!/usr/bin/env python3
"""The operator's side of the daemon: plain HTTP routes next to the MCP endpoint.

Registered by register(mcp, ctx) from robot_server, like admin_tools and drive_tools.
These routes only exist on the wire in daemon mode (robot_server.py --http); under stdio
there is no HTTP server and they are never reachable.

Why they are not MCP tools: they are for the person standing at the robot, on whatever
they have in their hand. A stop button must work from a phone's browser with no client
library, while a tool call is blocked mid-move and while the model that made it is still
thinking. Tool functions run in worker threads (the MCP library hands sync tools to
anyio.to_thread), so the event loop that serves these routes stays free during a move.

Hold-to-drive (/drive/hold + /drive/stop) is the one exception to "nothing here
moves": press-and-hold buttons for the operator driving directly, like the stick.
The press itself is the human confirmation, so no confirmed_safe token is needed;
everything else still applies -- the operator stop latch, the sensor policy and
tilt guard (via send_drive, the same choke point), one-driver-at-a-time (the page
takes the drive lease as "dashboard"), and a watchdog that brakes if holds stop
arriving. Anyone who can open the page can drive, so keep it on the home network.
"""

import json
import sys
import threading
import time
from pathlib import Path

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse

CTX = None


def frame_conventions(link=None):
    """The robot-frame settings this daemon drives under, for the dashboard's
    frame row: which side is left, which way yaw grows, per-wheel encoder
    signs, and the gear both sides use. Static intent plus live config --
    nothing here moves the robot."""
    root = str(Path(__file__).resolve().parent.parent)
    if root not in sys.path:
        sys.path.insert(0, root)  # bridge/ and drive_math live beside mcp_server/
    import bridge
    from drive_math import DRIVE_SWAP_SIDES
    cfg = getattr(link, "cfg", None) or {}
    return dict(
        drive_swap_sides=bool(DRIVE_SWAP_SIDES),
        imu_yaw_sign=int(getattr(bridge, "IMU_YAW_SIGN", 1)),
        encoder_sign=list(cfg.get("encoder_sign", list(bridge.DEFAULT_CONFIG["encoder_sign"]))),
        gear_ratio_used=cfg.get("gear_ratio", bridge.DEFAULT_CONFIG["gear_ratio"]),
    )

# ---- hold-to-drive -----------------------------------------------------------
# The page sends /drive/hold every 250 ms while a button is held; the watchdog
# brakes if holds stop (page closed, network dropped, finger slipped). Release
# sends /drive/stop. Drives are momentary send_drive calls, never the blocking
# closed-loop tools, so a stuck page cannot own the wheels for longer than the
# watchdog interval plus the motor ramp.
DASH_HOLDER = "dashboard"
DASH_HOLD_TIMEOUT_S = 0.8
DASH_MIN_PWM = 20
DASH_MAX_PWM = 125
_dash_token = None
_dash_driving = False
_dash_last_hold = 0.0
_dash_lock = threading.Lock()
_watchdog_started = False


def _trace_event(kind, **fields):
    """Best-effort trace into recordings/trace.jsonl. Never raises."""
    try:
        import trace as _tr
        if hasattr(_tr, "event"):
            _tr.event(kind, **fields)
    except Exception:
        pass


def _dash_reset_for_tests():
    """Forget dashboard drive state. Tests only; the daemon never calls this."""
    global _dash_token, _dash_driving, _dash_last_hold
    with _dash_lock:
        _dash_token = None
        _dash_driving = False
        _dash_last_hold = 0.0


def dash_vector(cmd, pwm):
    """(left, right) PWM for a dashboard button at slider PWM. Raises ValueError
    on an unknown button. Matches turn_degrees' wheel mapping: spin left is
    left-backward, right-forward."""
    cmds = {
        "forward": (pwm, pwm),
        "back": (-pwm, -pwm),
        "spin_left": (-pwm, pwm),
        "spin_right": (pwm, -pwm),
    }
    try:
        return cmds[cmd]
    except (KeyError, TypeError):
        raise ValueError("cmd must be one of %s" % ", ".join(sorted(cmds)))


def _dash_ensure_lease(ctx, who):
    """A lease token for the page, taking one if free. Returns (token, None)
    or (None, refusal-dict)."""
    global _dash_token
    with _dash_lock:
        tok = _dash_token
    if tok is not None and ctx.LEASE.check(tok) is None:
        return tok, None
    ok, info = ctx.LEASE.take(DASH_HOLDER, who or "dashboard page",
                              purpose="dashboard hold-to-drive", ttl_s=30)
    if not ok:
        with _dash_lock:
            _dash_token = None
        return None, info
    with _dash_lock:
        _dash_token = info["lease"]
    return info["lease"], None


def dash_hold(ctx, who, cmd, pwm, now=None):
    """One hold tick: drive (left, right) for cmd at pwm right now. Returns
    (status_code, dict). Momentary: the watchdog owns stopping, not this."""
    now = time.monotonic() if now is None else now
    try:
        pwm = int(pwm)
    except (TypeError, ValueError):
        return 400, {"ok": False, "error": "pwm must be a number"}
    pwm = max(DASH_MIN_PWM, min(DASH_MAX_PWM, pwm))
    try:
        left, right = dash_vector(cmd, pwm)
    except ValueError as e:
        return 400, {"ok": False, "error": str(e)}
    if ctx.link is None:
        return 409, {"ok": False, "error": "no robot connected"}
    if ctx.OPSTOP.latched:
        return 409, {"ok": False, "error": "operator stop is latched -- press Resume first"}
    tok, err = _dash_ensure_lease(ctx, who)
    if err is not None:
        return 409, {"ok": False, **{k: v for k, v in err.items() if k != "drive_lease"}}
    if ctx.LEASE.check(tok) is not None:
        return 409, {"ok": False, "error": "lost the drive lease mid-hold"}
    ctx.LEASE.begin()
    try:
        ctx.link.send_drive(left, right)
    except Exception as e:
        try:
            ctx.link.send_stop()
        except Exception:
            pass
        return 409, {"ok": False, "error": "guard: %s" % e}
    finally:
        ctx.LEASE.end()
    global _dash_driving, _dash_last_hold
    with _dash_lock:
        started = not _dash_driving
        _dash_driving = True
        _dash_last_hold = now
    if started:
        _trace_event("dash_drive", action="start", cmd=cmd, pwm=pwm, who=who)
    return 200, {"ok": True, "driving": cmd, "pwm": pwm}


def dash_stop(ctx, who, now=None, reason="released"):
    """Brake now and give up the page's turn. Always 200: stopping never fails."""
    now = time.monotonic() if now is None else now
    global _dash_driving, _dash_token
    with _dash_lock:
        was = _dash_driving
        _dash_driving = False
        tok = _dash_token
        _dash_token = None
    if ctx.link is not None:
        try:
            ctx.link.send_stop()
        except Exception:
            pass
    if tok is not None:
        try:
            ctx.LEASE.release(tok)
        except Exception:
            pass
    if was:
        _trace_event("dash_drive", action="stop", reason=reason, who=who)
    return 200, {"ok": True, "was_driving": was}


def dash_watchdog(ctx, now=None):
    """Brake a drive whose holds stopped arriving. Returns True when it fired."""
    now = time.monotonic() if now is None else now
    with _dash_lock:
        stale = _dash_driving and now - _dash_last_hold > DASH_HOLD_TIMEOUT_S
    if not stale:
        return False
    dash_stop(ctx, "watchdog", now=now, reason="holds stopped arriving")
    _trace_event("dash_drive", action="watchdog_stop")
    return True


def _dash_watchdog_loop():
    while True:
        try:
            if CTX is not None:
                dash_watchdog(CTX)
        except Exception:
            pass
        time.sleep(0.2)


def _state():
    link = CTX.link
    out = {
        "connected": link is not None,
        "connect_error": CTX._connect_error if link is None else None,
        "operator_stop": CTX.OPSTOP.snapshot(),
        "drive_lease": CTX.LEASE.snapshot(),
        "frame": frame_conventions(link),
    }
    if link is not None:
        try:
            out["tilt_guard"] = link.tilt.snapshot()
        except Exception as e:
            out["tilt_guard_error"] = repr(e)
        try:
            out["telemetry"] = link.telemetry_snapshot()
        except Exception as e:  # a status page must not 500 because a sensor went away
            out["telemetry_error"] = repr(e)
    return out


async def _who_and_reason(request: Request):
    """Optional JSON body {"who": ..., "reason": ...}; a bare POST is a valid stop."""
    who = request.client.host if request.client else "operator"
    reason = ""
    try:
        body = json.loads(await request.body() or b"{}")
        if isinstance(body, dict):
            who = str(body.get("who") or who)[:80]
            reason = str(body.get("reason") or "")[:200]
    except ValueError:
        pass
    return who, reason


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama operator</title>
<style>
  :root { --bg:#f6f5f2; --fg:#1b1b1a; --muted:#6b6a66; --card:#fff; --line:#dcdad3;
          --stop:#c62828; --go:#2e6b3a; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#161615; --fg:#ecebe7; --muted:#9a9993; --card:#212120; --line:#363531;
            --stop:#e2504b; --go:#58a568; }
  }
  * { box-sizing: border-box; }
  body { margin:0; padding:12px; background:var(--bg); color:var(--fg);
         font: 20px/1.35 system-ui, sans-serif; }
  /* Wide landscape screens: three equal columns, one screen, no page scroll.
       left: STOP, status, driver, activity | center: live, drive | right: camera 1, camera 2
     Each camera is at most 640x480 and shrinks to fit its third of the screen (and, with
     two stacked, the height). Columns scroll inside if they overflow. */
  .layout { display:grid; gap:12px;
            grid-template-columns: repeat(3, minmax(0, 1fr));
            grid-template-areas: "side center cams"; }
  .side { grid-area:side; }
  .center { grid-area:center; display:flex; flex-direction:column; gap:12px; }
  .cams { grid-area:cams; display:flex; flex-direction:column; gap:12px; }
  .side, .center, .cams { min-height:0; min-width:0; }
  .center > .card, .cams > .card { margin-top:0; }
  @media (min-width: 1100px) and (orientation: landscape) {
    body { height:100vh; overflow:hidden; }
    main, .layout { height:100%; }
    .side { display:flex; flex-direction:column; overflow:auto; }
    .center { overflow:auto; }
    #tracecard { flex:1; min-height:160px; display:flex; flex-direction:column; }
    #livecard { flex:1; min-height:0; display:flex; flex-direction:column; }
    #livecard .viewbox { flex:1; min-height:0; }
    .cams > .card { flex:1; min-height:0; display:flex; flex-direction:column; }
    .cams .viewbox { flex:1; min-height:0; }
  }
  @media (max-width: 1099px), (orientation: portrait) {
    .layout { grid-template-columns: minmax(0, 1fr);
              grid-template-areas: "side" "center" "cams"; }
    #livecard .viewbox { aspect-ratio: 600 / 700; }
    .cams .viewbox { aspect-ratio: 4 / 3; }
    .tracebox { max-height:320px; overflow:auto; }
  }
  .viewbox { position:relative; margin-top:8px; }
  #live { position:absolute; inset:0; width:100%; height:100%; object-fit:contain;
          object-position:top; }
  /* camera: a 640x480 frame, never larger, centred; the stream keeps its own aspect */
  .cams .viewbox { container-type: size; }
  .camframe { position:absolute; inset:0; margin:auto; background:#000; border-radius:8px;
              overflow:hidden;
              width:  min(640px, 100cqw, calc(100cqh * 4 / 3));
              height: min(480px, 100cqh, calc(100cqw * 3 / 4)); }
  .camframe img { position:absolute; inset:0; width:100%; height:100%; object-fit:contain; }
  .camframe img[hidden], .camempty[hidden] { display:none; }
  .camempty { position:absolute; inset:0; display:flex; align-items:center;
              justify-content:center; color:#b4b3ad; padding:20px; text-align:center;
              font-size:18px; }
  .camempty a { color:#8fbaf0; }
  .camhead { display:flex; align-items:center; gap:10px; }
  .camhead input { flex:1; min-width:0; font:inherit; font-size:18px; padding:7px 10px;
                   border-radius:8px; border:1px solid var(--line); background:var(--bg);
                   color:var(--fg); }
  .camhead button { width:auto; padding:8px 18px; border-radius:8px; background:#2f6fd0;
                    font-size:18px; }
  .camhead button.on { background:var(--muted); }
  .camstatus { display:block; min-height:1.35em; font-size:16px; }
  button { width:100%; border:0; border-radius:12px; color:#fff; font:inherit;
           font-weight:700; cursor:pointer; }
  h1 { font-size:20px; margin:0 0 8px; }
  #stop { background:var(--stop); font-size:52px; padding:24px 0; letter-spacing:3px; }
  #drivebtns { display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px; margin-top:10px; }
  #drivebtns button { font-size:22px; padding:16px 0; color:#fff; }
  #drivebtns button[data-cmd="forward"] { background:var(--go); grid-column:2; }
  #drivebtns button[data-cmd="back"] { background:var(--stop); grid-column:2; grid-row:2; }
  #drivebtns button[data-cmd="spin_left"] { background:#2f6fd0; grid-column:1; grid-row:2; }
  #drivebtns button[data-cmd="spin_right"] { background:#2f6fd0; grid-column:3; grid-row:2; }
  #drivebtns button:disabled { opacity:.35; }
  .pwmrow { display:flex; align-items:center; gap:12px; margin-top:12px; font-weight:700; }
  .pwmrow .pwmname { width:9.5em; }
  .pwmrow span:not(.pwmname) { width:2.2em; text-align:right; font-variant-numeric:tabular-nums; }
  .pwmrow input { flex:1; height:28px; }
  #drivemsg { display:block; min-height:1.2em; }
  #resume { background:var(--go); font-size:22px; padding:14px 0; margin-top:10px; }
  #resume[hidden] { display:none; }
  #cleartilt { background:#c98a12; font-size:22px; padding:14px 0; margin-top:10px; }
  #cleartilt[hidden] { display:none; }
  #tiltmsg { display:block; min-height:0; }
  .card { background:var(--card); border:1px solid var(--line); border-radius:12px;
          padding:10px 14px; margin-top:12px; }
  .banner { font-weight:700; font-size:22px; }
  .latched { color:var(--stop); }
  .tracebox { flex:1; min-height:0; overflow:auto; margin-top:4px; }
  .tracebox dl { margin:0; font-size:17px; }
  dl { display:grid; grid-template-columns:auto 1fr; gap:4px 16px; margin:8px 0 0; }
  dt { color:var(--muted); } dd { margin:0; font-variant-numeric: tabular-nums;
                                   overflow-wrap:anywhere; }
  small { color:var(--muted); font-size:16px; }
</style></head><body><main>
<div class="layout">
<div class="side">
<h1>eLlama operator</h1>
<button id="stop">STOP</button>
<button id="resume" hidden>Resume: let it drive again</button>
<button id="cleartilt" hidden>Clear tilt fault (robot is level)</button>
<small id="tiltmsg"></small>
<div class="card">
  <div class="banner" id="banner">connecting…</div>
  <dl id="facts"></dl>
</div>
<div class="card">
  <div class="banner" id="driver">…</div>
  <dl id="leasefacts"></dl>
</div>
<div class="card" id="tracecard">
  <div class="banner">Recent activity</div>
  <div class="tracebox"><dl id="trace"></dl></div>
</div>
<p><small>STOP brakes now and refuses every move until Resume is pressed here; the AI
cannot release it. Full activity log: recordings/trace.jsonl.</small></p>
</div>
<div class="center">
<div class="card" id="livecard">
  <div class="banner">Live</div>
  <div class="viewbox"><canvas id="live" width="600" height="700"></canvas></div>
</div>
<div class="card" id="drivecard">
  <div class="banner">Drive</div>
  <div id="drivebtns">
    <button data-cmd="forward">Forward</button>
    <button data-cmd="spin_left">Spin left</button>
    <button data-cmd="back">Back</button>
    <button data-cmd="spin_right">Spin right</button>
  </div>
  <label class="pwmrow"><span class="pwmname">Fwd / back PWM</span> <span id="pwmval_fb">70</span>
    <input id="pwm_fb" type="range" min="20" max="125" value="70">
  </label>
  <label class="pwmrow"><span class="pwmname">Spin PWM</span> <span id="pwmval_lr">70</span>
    <input id="pwm_lr" type="range" min="20" max="125" value="70">
  </label>
  <small id="drivemsg"></small>
  <small>Press and hold. Release (or a dropped connection) brakes within a second. Same guards as every drive.</small>
</div>
</div>
<div class="cams">
<div class="card camcard" data-cam="1">
  <div class="camhead">
    <span class="banner">Arrow-end camera</span>
    <input class="camaddr" placeholder="192.168.0.33:8080" autocomplete="off"
           spellcheck="false" aria-label="Arrow-end camera MJPEG stream URL">
    <button class="camconnect">Connect</button>
  </div>
  <small class="camstatus"></small>
  <div class="viewbox"><div class="camframe">
    <img class="camimg" alt="arrow-end camera" hidden>
    <div class="camempty">Mount a phone at the arrow end, enter its address (like 192.168.0.33:8080) and press Connect.</div>
  </div></div>
</div>
<div class="card camcard" data-cam="2">
  <div class="camhead">
    <span class="banner">Tail-end camera</span>
    <input class="camaddr" placeholder="192.168.0.34:8080" autocomplete="off"
           spellcheck="false" aria-label="Tail-end camera MJPEG stream URL">
    <button class="camconnect">Connect</button>
  </div>
  <small class="camstatus"></small>
  <div class="viewbox"><div class="camframe">
    <img class="camimg" alt="tail-end camera" hidden>
    <div class="camempty">Mount a phone at the tail end, enter its address (like 192.168.0.34:8080) and press Connect.</div>
  </div></div>
</div>
</div>
</div>
<script>
const $ = id => document.getElementById(id);
async function post(path) {
  try { await fetch(path, {method:'POST', headers:{'content-type':'application/json'},
                           body: JSON.stringify({who:'operator page'})}); }
  catch (e) {}
  refresh();
}
$('stop').onclick = () => post('/stop');
$('resume').onclick = () => post('/resume');
$('cleartilt').onclick = async () => {
  try {
    const r = await (await fetch('/tilt/clear', {method:'POST',
      headers:{'content-type':'application/json'},
      body: JSON.stringify({who:'operator page'})})).json();
    $('tiltmsg').textContent = r.ok ? '' : (r.message || 'refused');
  } catch (e) { $('tiltmsg').textContent = 'daemon not reachable'; }
  refresh();
};
function row(k, v) { return `<dt>${k}</dt><dd>${v}</dd>`; }
async function refresh() {
  let s;
  try { s = await (await fetch('/state', {cache:'no-store'})).json(); }
  catch (e) { $('banner').textContent = 'daemon not reachable'; $('facts').innerHTML = ''; return; }
  const st = s.operator_stop, t = s.telemetry || {};
  $('resume').hidden = !st.latched;
  const tg = s.tilt_guard;
  const tiltTripped = !!(tg && tg.tripped);
  $('cleartilt').hidden = !tiltTripped;
  if (!tiltTripped) $('tiltmsg').textContent = '';
  else if (!$('tiltmsg').textContent && tg.fault && tg.fault.reason)
    $('tiltmsg').textContent = `Tripped: ${tg.fault.reason}`;
  $('banner').className = 'banner' + (st.latched ? ' latched' : '');
  $('banner').textContent = st.latched
    ? `STOPPED by ${st.who}, ${Math.round(st.latched_for_s)}s ago`
    : (s.connected ? 'Free to drive' : 'No robot connected');
  let h = '';
  if (!s.connected) h += row('robot', s.connect_error || 'not connected');
  h += row('encoders', !s.connected ? '—' : (t.telemetry_live ? 'live' : 'not reporting'));
  h += row('IMU', !s.connected ? '—' : (t.imu_live ? 'live' : 'not reporting'));
  h += row('tilt', t.tilt
    ? `${t.tilt.tilt_deg}° of ${t.tilt.limit_deg}°` + (t.tilt.tripped ? ' (tripped)' : '') : '—');
  h += row('range ahead', t.range && t.range.closest_in != null ? `${t.range.closest_in} in`
    : (!s.connected ? '—' : (t.range && t.range.live ? 'nothing in range' : 'not reporting')));
  h += row('odometry', t.dist_avg_in !== undefined ? `${Math.abs(t.dist_avg_in).toFixed(1)} in` : '—');
  h += row('stick', t.stick ? (t.stick.active
    ? `driving (${t.stick.left}, ${t.stick.right})` : `idle ${Math.round(t.stick.age_s)}s ago`) : '—');
  const f = s.frame || {};
  h += row('frame', `drive ${f.drive_swap_sides ? 'L/R swapped' : 'L/R straight'}`
    + ` · yaw ${f.imu_yaw_sign === -1 ? 'flipped' : 'straight'}`
    + ` · enc [${(f.encoder_sign || []).join(',')}] · gear ${f.gear_ratio_used ?? '—'}`);
  $('facts').innerHTML = h;
  const L = s.drive_lease;
  $('driver').textContent = L.held ? `Driving: ${L.holder}`
    : (L.required ? 'Nobody is driving' : 'Nobody holds the lease (not required)');
  let g = '';
  g += row('operator', L.held ? L.operator : '—');
  g += row('purpose', L.held ? (L.purpose || '—') : '—');
  g += row('status', !L.held ? '—'
    : (L.move_in_progress ? 'move in progress'
       : `quiet ${Math.round(L.quiet_for_s)}s of ${Math.round(L.ttl_s)}s`));
  $('leasefacts').innerHTML = g;
  const canDrive = s.connected && !st.latched
    && !(L.held && L.holder !== 'dashboard');
  document.querySelectorAll('#drivebtns button').forEach(b => b.disabled = !canDrive);
}
async function refreshTrace() {
  let d;
  try { d = await (await fetch('/api/trace?n=12', {cache:'no-store'})).json(); }
  catch (e) { return; }
  const evs = (d.events || []).slice().reverse();
  $('trace').innerHTML = evs.map(e => {
    const when = e.t ? new Date(e.t * 1000).toLocaleTimeString() : '';
    let what = e.kind || '?';
    if (e.kind === 'tool') what = (e.ok === false ? '✕ ' : '') + (e.tool || 'tool')
      + (e.elapsed_s !== undefined ? ` (${e.elapsed_s}s)` : '');
    else if (e.kind === 'model') what = `model ${e.model || ''} ${e.purpose || ''}`.trim()
      + (e.error ? ' (failed)' : '');
    else if (e.kind === 'operator') what = `operator: ${e.action || ''} by ${e.who || ''}`.trim();
    else if (e.kind === 'approval') what = `${e.approved ? 'approved' : 'refused'} ${e.tool || ''}`.trim();
    else if (e.kind === 'stick') what = `stick driving (${e.left}, ${e.right})`;
    const extra = e.error && e.kind !== 'tool' ? ` — ${String(e.error).slice(0, 80)}` : '';
    return row(when, `${what}${extra}`);
  }).join('');
}
/* Live canvas: the live_view.py panels in the browser. Client-side history
   from /state samples, so the daemon needs no new endpoint. */
const LH = {dl: [], dr: [], yaw: [], stickL: [], stickR: [], close: []};
const LHR = {};   // per range channel: {name: [mm|null, ...]}, same length as LH.close
const LCAP = 240;
function lpush(k, v) {
  const a = LH[k];
  a.push(v === undefined ? null : v);
  if (a.length > LCAP) a.splice(0, a.length - LCAP);
}
function css(v, fb) {
  const s = getComputedStyle(document.documentElement).getPropertyValue(v).trim();
  return s || fb;
}
function ltext(g, s, x, y, size, color) {
  g.fillStyle = color;
  g.font = `${size}px system-ui, sans-serif`;
  g.fillText(s, x, y);
}
function lbar(g, x, y, w, h, frac, color, track) {
  g.fillStyle = track;
  g.fillRect(x, y, w, h);
  const f = Math.max(0, Math.min(1, frac || 0));
  g.fillStyle = color;
  g.fillRect(x, y, w * f, h);
}
function lcbar(g, x, y, w, h, value, full, color, track, dim) {
  g.fillStyle = track;
  g.fillRect(x, y, w, h);
  const mid = x + w / 2;
  g.fillStyle = dim;
  g.fillRect(mid - 0.5, y, 1, h);
  const f = full ? Math.max(-1, Math.min(1, value / full)) : 0;
  g.fillStyle = color;
  const px = Math.round(w / 2 * f);
  if (px >= 0) g.fillRect(mid, y, px, h);
  else g.fillRect(mid + px, y, -px, h);
}
function lspark(g, x, y, w, h, vals, lo, hi, color) {
  const pts = vals.filter(v => v != null);
  if (pts.length < 2) return;
  lo = lo !== undefined ? lo : Math.min(...pts);
  hi = hi !== undefined ? hi : Math.max(...pts);
  const span = Math.max(hi - lo, 1e-6), step = w / Math.max(vals.length - 1, 1);
  g.strokeStyle = color;
  g.lineWidth = 2;
  g.beginPath();
  let pen = false;
  vals.forEach((v, i) => {
    if (v == null) { pen = false; return; }
    const px = x + i * step, py = y + h - h * (v - lo) / span;
    if (!pen) { g.moveTo(px, py); pen = true; }
    else g.lineTo(px, py);
  });
  g.stroke();
}
function drawLive(t) {
  // One column, laid out for a 600x700 canvas shown at about 1:1 in a third of the
  // screen, so the text is real size: DRIVE, ENCODERS, IMU, RANGE top to bottom.
  const cv = $('live');
  if (!cv) return;
  const g = cv.getContext('2d');
  const FG = css('--fg', '#1b1b1a'), MUT = css('--muted', '#6b6a66');
  const GOOD = css('--go', '#2e6b3a'), BAD = css('--stop', '#c62828');
  const BG = css('--card', '#fff'), LINE = css('--line', '#dcdad3');
  const BLUE = '#3a86d4', ORANGE = '#d4902a';
  const W = cv.width, H1 = 19, V = 20, SM = 17;
  g.fillStyle = BG;
  g.fillRect(0, 0, W, cv.height);
  let y = 2;
  const sens = t.sensors || {};
  // status strip: DRIVE, ENC, IMU, RANGE liveness
  const chips = [];
  const st = t.stick;
  chips.push(['DRIVE', st && st.active ? ['live', GOOD] : ['idle', MUT]]);
  const enc = sens['enc'];
  chips.push(['ENC', enc ? (enc.live ? [`${enc.rate_hz ?? ''} Hz`, GOOD] : ['SILENT', BAD]) : ['never', MUT]]);
  const imu = sens['imu'];
  chips.push(['IMU', imu ? (imu.live ? [`${imu.rate_hz ?? ''} Hz`, GOOD] : ['SILENT', BAD]) : ['never', MUT]]);
  const rngLive = t.range && t.range.live;
  chips.push(['RANGE', t.range ? (rngLive ? ['live', GOOD] : ['silent', BAD]) : ['never', MUT]]);
  const cw = W / 4;
  chips.forEach(([name, [state, col]], i) => {
    const x = i * cw + 2;
    g.fillStyle = LINE;
    g.fillRect(x, y, cw - 4, 52);
    g.fillStyle = col;
    g.fillRect(x, y + 48, cw - 4, 4);
    ltext(g, name, x + 8, y + 21, H1, FG);
    ltext(g, String(state), x + 8, y + 42, SM, col);
  });
  y += 62;
  // DRIVE: stick bars + history
  const idle = !(st && st.active);
  ltext(g, 'DRIVE', 4, y + 18, H1, MUT);
  ltext(g, idle && st ? `stick idle ${Math.round(st.age_s)}s` : 'stick + history', 90, y + 18, SM, MUT);
  y += 28;
  const lv = st ? st.left : 0, rv = st ? st.right : 0;
  ltext(g, 'L', 4, y + 19, V, BLUE);
  lcbar(g, 28, y, W - 110, 24, idle ? 0 : lv, 255, idle ? MUT : BLUE, LINE, MUT);
  ltext(g, `${lv >= 0 ? '+' : ''}${lv}`, W - 70, y + 19, V, FG);
  ltext(g, 'R', 4, y + 51, V, ORANGE);
  lcbar(g, 28, y + 32, W - 110, 24, idle ? 0 : rv, 255, idle ? MUT : ORANGE, LINE, MUT);
  ltext(g, `${rv >= 0 ? '+' : ''}${rv}`, W - 70, y + 51, V, FG);
  y += 62;
  lspark(g, 28, y, W - 110, 56, LH.stickL, -255, 255, BLUE);
  lspark(g, 28, y, W - 110, 56, LH.stickR, -255, 255, ORANGE);
  y += 66;
  // ENCODERS: distances, heading, history
  ltext(g, 'ENCODERS', 4, y + 18, H1, MUT);
  ltext(g, t.heading_deg == null ? 'heading: needs both' : `heading ${t.heading_deg.toFixed(1)}°`, 140, y + 18, SM, MUT);
  y += 28;
  const av = t.encoders_available || [false, false];
  ltext(g, `L ${av[0] && t.dist_l_in !== undefined ? t.dist_l_in.toFixed(2) + ' in' : 'NOT RESPONDING'}`, 4, y + 19, V, av[0] ? FG : BAD);
  ltext(g, `R ${av[1] && t.dist_r_in !== undefined ? t.dist_r_in.toFixed(2) + ' in' : 'NOT RESPONDING'}`, W / 2, y + 19, V, av[1] ? FG : BAD);
  y += 28;
  {
    const pts = LH.dl.concat(LH.dr).filter(v => v != null);
    let lo = pts.length ? Math.min(...pts) : 0, hi = pts.length ? Math.max(...pts) : 0;
    if (hi - lo < 2) { const mid = (hi + lo) / 2; lo = mid - 1; hi = mid + 1; }
    lspark(g, 4, y, W - 8, 44, LH.dl, lo, hi, BLUE);
    lspark(g, 4, y, W - 8, 44, LH.dr, lo, hi, ORANGE);
  }
  y += 54;
  // IMU: horizon, compass, angles, accel
  ltext(g, 'IMU', 4, y + 18, H1, MUT);
  y += 28;
  const pitch = t.pitch_deg || 0, roll = t.roll_deg || 0, yaw = t.yaw_deg || 0;
  const r = 38, cx = 44, cx2 = 132, cy = y + 42;
  g.fillStyle = LINE;
  g.beginPath(); g.arc(cx, cy, r, 0, 7); g.fill();
  const off = Math.max(-r, Math.min(r, pitch * r / 60));
  const a = -roll * Math.PI / 180, dx = Math.cos(a), dy = Math.sin(a);
  g.strokeStyle = FG; g.lineWidth = 3;
  g.beginPath();
  g.moveTo(cx - dx * r + dy * off, cy - dy * r + dx * off);
  g.lineTo(cx + dx * r + dy * off, cy + dy * r + dx * off);
  g.stroke();
  g.strokeStyle = MUT; g.lineWidth = 1;
  g.beginPath(); g.arc(cx, cy, r, 0, 7); g.stroke();
  g.fillStyle = LINE;
  g.beginPath(); g.arc(cx2, cy, r, 0, 7); g.fill();
  const ya = (yaw - 90) * Math.PI / 180;
  g.strokeStyle = GOOD; g.lineWidth = 4;
  g.beginPath(); g.moveTo(cx2, cy);
  g.lineTo(cx2 + Math.cos(ya) * (r - 8), cy + Math.sin(ya) * (r - 8)); g.stroke();
  ltext(g, `pitch ${pitch.toFixed(1)}°`, 186, y + 20, V, FG);
  ltext(g, `roll  ${roll.toFixed(1)}°`, 186, y + 48, V, FG);
  ltext(g, `yaw   ${yaw.toFixed(1)}°`, 186, y + 76, V, FG);
  const il = t.imu_last || {};
  const gx = (il.ax || 0) / 16384, gy = (il.ay || 0) / 16384, gz = (il.az || 0) / 16384;
  [['x', gx], ['y', gy], ['z', gz]].forEach(([ax, v], i) => {
    ltext(g, 'a' + ax, 336, y + 18 + i * 28, SM, MUT);
    lcbar(g, 364, y + 6 + i * 28, W - 440, 14, v, 2, '#8a6fd0', LINE, MUT);
    ltext(g, `${v >= 0 ? '+' : ''}${v.toFixed(2)}`, W - 66, y + 18 + i * 28, SM, FG);
  });
  y += 92;
  // RANGE: per channel + closest
  const cc = t.range ? t.range.closest_mm : null;
  ltext(g, 'RANGE', 4, y + 18, H1, MUT);
  ltext(g, 'closest: ' + (cc == null ? 'nothing in range' : `${cc} mm`), 100, y + 18, V, cc == null ? MUT : FG);
  y += 28;
  const ch = (t.range && t.range.channels) || {};
  const names = Object.keys(ch).slice(0, 4);
  if (!names.length) { ltext(g, 'no range packets — is the ToF board powered?', 4, y + 18, SM, MUT); y += 30; }
  names.forEach((k, i) => {
    const c = ch[k];
    ltext(g, k.slice(0, 14), 4, y + 18 + i * 28, SM, FG);
    if (c.mm == null) ltext(g, 'no target', 150, y + 18 + i * 28, SM, MUT);
    else {
      lbar(g, 150, y + 4 + i * 28, W - 250, 18, c.mm / 2000, i % 2 ? ORANGE : BLUE, LINE);
      ltext(g, `${c.mm} mm`, W - 90, y + 18 + i * 28, SM, FG);
    }
  });
  y += names.length * 28 + 6;
  // history: one line per channel, same colours as the bars, fixed 0-2 m scale
  const secs = Math.round(LCAP * 0.35);
  ltext(g, `range history, last ${secs} s`, 4, y + 16, SM, MUT);
  y += 24;
  const gx0 = 44, gw = W - gx0 - 4, gh = Math.max(40, Math.min(150, cv.height - y - 6));
  g.strokeStyle = LINE; g.lineWidth = 1;
  g.strokeRect(gx0 + 0.5, y + 0.5, gw - 1, gh - 1);
  g.beginPath(); g.moveTo(gx0, y + gh / 2 + 0.5); g.lineTo(gx0 + gw, y + gh / 2 + 0.5); g.stroke();
  ltext(g, '2 m', 4, y + 14, SM, MUT);
  ltext(g, '1 m', 4, y + gh / 2 + 6, SM, MUT);
  ltext(g, '0', 4, y + gh - 2, SM, MUT);
  names.forEach((k, i) => lspark(g, gx0, y, gw, gh,
    (LHR[k] || []).map(v => v == null ? null : Math.min(v, 2000)), 0, 2000, i % 2 ? ORANGE : BLUE));
}
async function liveTick() {
  let s;
  try { s = await (await fetch('/state', {cache:'no-store'})).json(); }
  catch (e) { return; }
  const t = s.telemetry || {};
  lpush('dl', t.dist_l_in);
  lpush('dr', t.dist_r_in);
  lpush('yaw', t.yaw_deg);
  lpush('stickL', t.stick ? t.stick.left : null);
  lpush('stickR', t.stick ? t.stick.right : null);
  lpush('close', t.range ? t.range.closest_mm : null);
  const chs = (t.range && t.range.channels) || {};
  for (const k of Object.keys(chs)) if (!LHR[k]) LHR[k] = new Array(LH.close.length - 1).fill(null);
  for (const k of Object.keys(LHR)) {
    LHR[k].push(chs[k] ? chs[k].mm : null);
    if (LHR[k].length > LCAP) LHR[k].splice(0, LHR[k].length - LCAP);
  }
  drawLive(t);
}
// Separate speeds: forward/back and spin need different PWM (a spin scrubs the wheels
// sideways). Each slider remembers its value in this browser.
const dashPwm = {fb: 70, lr: 70};
let dashTimer = null;
const dashAxis = cmd => (cmd === 'forward' || cmd === 'back') ? 'fb' : 'lr';
function dashMsg(s) { const m = $('drivemsg'); if (m) m.textContent = s || ''; }
async function dashHold(cmd) {
  try {
    const r = await fetch('/drive/hold', {method:'POST',
      headers:{'content-type':'application/json'},
      body: JSON.stringify({cmd, pwm: dashPwm[dashAxis(cmd)]})});
    const d = await r.json();
    if (!d.ok) { dashMsg(d.error || 'refused'); dashRelease(); }
    else dashMsg('');
  } catch (e) { dashMsg('not reachable'); dashRelease(); }
}
async function dashRelease() {
  if (dashTimer) { clearInterval(dashTimer); dashTimer = null; }
  try { await fetch('/drive/stop', {method:'POST'}); } catch (e) {}
  dashMsg('');
}
document.querySelectorAll('#drivebtns button').forEach(b => {
  const cmd = b.dataset.cmd;
  b.addEventListener('pointerdown', e => {
    e.preventDefault();
    try { b.setPointerCapture(e.pointerId); } catch (err) {}
    dashHold(cmd);
    if (dashTimer) clearInterval(dashTimer);
    dashTimer = setInterval(() => dashHold(cmd), 250);
  });
  b.addEventListener('pointerup', dashRelease);
  b.addEventListener('pointercancel', dashRelease);
  b.addEventListener('lostpointercapture', dashRelease);
  b.addEventListener('contextmenu', e => e.preventDefault());
});
['fb', 'lr'].forEach(ax => {
  const sl = $('pwm_' + ax), key = 'ellama.pwm.' + ax;
  const set = v => { dashPwm[ax] = +v; sl.value = v; $('pwmval_' + ax).textContent = v; };
  try { const v = localStorage.getItem(key); if (v && +v >= 20 && +v <= 125) set(v); } catch (e) {}
  sl.addEventListener('input', e => {
    set(e.target.value);
    try { localStorage.setItem(key, e.target.value); } catch (err) {}
  });
});
/* Cameras: each slot shows one MJPEG stream, straight from the camera to this browser
   (the daemon never sees the video). Accepts a full URL, "192.168.0.33:8080" or
   "192 168 0 33 8080". A URL with a path is used exactly as given. Without a path the
   slot tries the usual stream paths (IP Webcam's /video, Android IP Camera's
   /video/mjpeg, ...), and without a scheme both http and https, keeping the first that
   gives a picture; the status line shows the URL it settled on. If the stream drops it
   starts over every 3 s. */
const CAM_PATHS = ['/video', '/video/mjpeg', '/mjpeg', '/mjpegfeed', '/stream', '/video.mjpg'];
function camCandidates(raw) {
  let s = (raw || '').trim();
  if (!s) return null;
  const m = s.match(/^(\\d{1,3})[\\s.,]+(\\d{1,3})[\\s.,]+(\\d{1,3})[\\s.,]+(\\d{1,3})(?:[\\s:,]+(\\d{1,5}))?(\\/\\S*)?$/);
  if (m) s = `${m[1]}.${m[2]}.${m[3]}.${m[4]}:${m[5] || 8080}${m[6] || ''}`;
  const schemes = /^https?:\\/\\//i.test(s) ? [''] : ['http://', 'https://'];
  const out = [];
  for (const sc of schemes) {
    let u;
    try { u = new URL(sc + s); } catch (e) { return null; }
    if (u.pathname !== '/' || u.search) out.push(u.href);
    else for (const p of CAM_PATHS) out.push(u.origin + p);
  }
  return out;
}
function makeCamera(card) {
  const n = card.dataset.cam, key = 'ellama.camera.' + n;
  const q = c => card.querySelector(c);
  const input = q('.camaddr'), btn = q('.camconnect'), img = q('.camimg'),
        empty = q('.camempty'), status = q('.camstatus');
  let gen = 0, on = false, url = '';
  const save = () => { try { localStorage.setItem(key, JSON.stringify({url: input.value, on})); }
                       catch (e) {} };
  const show = (pic, msg) => { img.hidden = !pic; empty.hidden = pic;
                               if (msg !== undefined) empty.textContent = msg; };
  function stop() {
    gen++; on = false;
    img.onload = img.onerror = null;
    img.removeAttribute('src');                 // closes the MJPEG connection
    btn.textContent = 'Connect'; btn.classList.remove('on');
    show(false, "Enter the camera's address (like 192.168.0.33:8080) and press Connect.");
    status.textContent = '';
  }
  let cands = [];
  function open(g, i) {
    if (g !== gen) return;
    url = cands[i];
    img.onload = () => { if (g === gen) show(true); };
    img.onerror = () => {
      if (g !== gen) return;
      img.removeAttribute('src');
      if (i + 1 < cands.length) { open(g, i + 1); return; }   // next path / scheme
      const https = cands.find(c => c.startsWith('https:'));
      const tried = cands.length > 1 ? `${cands.length} stream paths at ${new URL(cands[0]).host}`
                                     : cands[0];
      if (https) {
        // A self-signed camera (e.g. Android IP Camera) fails silently until this
        // browser has accepted its certificate once; say how, with a link to do it.
        const msg = document.createElement('span');
        msg.append(`No stream (tried ${tried}). If this camera uses https with a self-signed certificate, `,
          Object.assign(document.createElement('a'), {href: new URL(https).origin + '/',
            target: '_blank', rel: 'noopener', textContent: 'open it in a new tab'}),
          ' and accept the warning once, then come back. Retrying every 3 s…');
        show(false); empty.replaceChildren(msg);
      } else {
        show(false, `No stream (tried ${tried}). Retrying every 3 s…`);
      }
      status.textContent = 'no stream; retrying every 3 s';
      setTimeout(() => open(g, 0), 3000);
    };
    status.textContent = `trying ${url}…`;
    img.src = url;
  }
  function start() {
    const c = camCandidates(input.value);
    if (!c) { status.textContent = 'Enter an address like 192.168.0.33:8080'; return; }
    stop();
    on = true; cands = c;
    btn.textContent = 'Disconnect'; btn.classList.add('on');
    show(false, `Connecting to ${new URL(c[0]).host}…`);
    save();
    open(gen, 0);
  }
  btn.onclick = () => { if (on) { stop(); save(); } else start(); };
  input.addEventListener('keydown', e => { if (e.key === 'Enter') start(); });
  setInterval(() => {
    if (on && img.naturalWidth && !img.hidden)
      status.textContent = `${url} · ${img.naturalWidth}×${img.naturalHeight}`;
  }, 1000);
  try {
    const s = JSON.parse(localStorage.getItem(key) || 'null');
    if (s && s.url) { input.value = s.url; if (s.on) start(); }
  } catch (e) {}
}
document.querySelectorAll('.camcard').forEach(makeCamera);
refresh(); setInterval(refresh, 1000);
liveTick(); setInterval(liveTick, 350);
refreshTrace(); setInterval(refreshTrace, 2000);
</script></main></body></html>
"""


def register(mcp, ctx):
    global CTX, _watchdog_started
    CTX = ctx
    if not _watchdog_started:
        _watchdog_started = True
        threading.Thread(target=_dash_watchdog_loop, name="dash-watchdog",
                         daemon=True).start()

    @mcp.custom_route("/drive/hold", methods=["POST"], include_in_schema=False)
    async def drive_hold(request: Request):
        try:
            body = json.loads(await request.body() or b"{}")
        except ValueError:
            body = {}
        if not isinstance(body, dict):
            body = {}
        who = request.client.host if request.client else "dashboard"
        try:
            who = str(body.get("who") or who)[:80]
        except Exception:
            pass
        code, out = dash_hold(CTX, who, body.get("cmd"), body.get("pwm", 70))
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/drive/stop", methods=["POST"], include_in_schema=False)
    async def drive_stop(request: Request):
        who = request.client.host if request.client else "dashboard"
        code, out = dash_stop(CTX, who)
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/api/trace", methods=["GET"], include_in_schema=False)
    async def trace_tail(request: Request):
        try:
            n = int(request.query_params.get("n", "60"))
        except ValueError:
            n = 60
        n = max(1, min(n, 400))
        try:
            import trace as _tr
            events = _tr.tail(n)
        except Exception as e:  # a status page must not 500
            return JSONResponse({"events": [], "error": repr(e)})
        return JSONResponse({"events": events})

    @mcp.custom_route("/", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        return JSONResponse(_state())

    @mcp.custom_route("/stop", methods=["POST"], include_in_schema=False)
    async def stop(request: Request):
        who, reason = await _who_and_reason(request)
        return JSONResponse({"ok": True, "operator_stop": CTX.operator_stop(who, reason)})

    @mcp.custom_route("/tilt/clear", methods=["POST"], include_in_schema=False)
    async def clear_tilt(request: Request):
        who, _ = await _who_and_reason(request)
        ok, msg, snap = CTX.operator_clear_tilt(who)
        return JSONResponse({"ok": ok, "message": msg, "tilt_guard": snap},
                            status_code=200 if ok else 409)

    @mcp.custom_route("/lease/revoke", methods=["POST"], include_in_schema=False)
    async def revoke(request: Request):
        who, _ = await _who_and_reason(request)
        was, snap = CTX.operator_revoke_lease(who)
        return JSONResponse({"ok": True, "was_held": was, "drive_lease": snap})

    @mcp.custom_route("/resume", methods=["POST"], include_in_schema=False)
    async def resume(request: Request):
        who, _ = await _who_and_reason(request)
        was, snap = CTX.operator_resume(who)
        return JSONResponse({"ok": True, "was_latched": was, "operator_stop": snap})
