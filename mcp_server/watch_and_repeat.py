#!/usr/bin/env python3
"""watch_and_repeat: drive a closed loop once, then let the robot drive it again.

Registered by register(mcp, ctx) from robot_server, like daemon_http. Its page lives at
GET /route on the daemon (http://127.0.0.1:8765/route), separate from the operator page.

Workflow (buttons on the page, or the route_* MCP tools):
  Start recording  sample the full telemetry snapshot at RECORD_HZ. Nothing here
                   drives: the operator drives with the stick or the operator page.
  Keyframe         pin the current pose as a waypoint the route must pass through
                   (and grab a camera frame if a camera URL was given).
  End recording    stop sampling and save recordings/routes/<name>/.
  Process route    route_math.process: dead-reckon, close the loop, simplify, legs.
  Follow route     run a follower (FOLLOWERS) under the drive lease.
  Abort            latch the operator stop (brakes now) and stop the follower.

Followers are pluggable so a route can later be followed by camera or ToF instead of
dead reckoning alone: each recording keeps every telemetry field, and each waypoint
carries its sensor snapshot. A follower is any object with
    name, description, run(route, ctx, lease, stop_evt, progress, opts) -> dict
that moves the robot only through the guarded drive tools on ctx (so the operator
stop, tilt guard, sensor policy and lease all apply), checks stop_evt between moves,
and reports progress via progress(**fields). Register it in FOLLOWERS.

Pressing Follow on the page is the human confirmation for the whole route, the same
reasoning as the operator page's hold-to-drive: the person pressing it is watching the
robot and has the Abort button on the same page.
"""

import json
import math
import re
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse, Response

sys.path.insert(0, str(Path(__file__).resolve().parent))
import route_math  # noqa: E402

CTX = None
HOLDER = "watch_and_repeat"
RECORD_HZ = 10.0
DEFAULT_DRIVE_PWM = 40
DEFAULT_TURN_PWM = 60
TEST_TURN_DEG = 10.0
ROUTES_DIR = Path(__file__).resolve().parent.parent / "recordings" / "routes"


def _trace_event(kind, **fields):
    try:
        import trace as _tr
        _tr.event(kind, **fields)
    except Exception:
        pass


def _safe_name(name):
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", (name or "").strip()).strip("._")
    return name[:60] or time.strftime("route_%Y%m%d_%H%M%S")


# ---- followers ---------------------------------------------------------------

class LegsFollower:
    """Turn in place by the IMU (turn_degrees), then drive straight by the encoders
    (drive_distance), once per leg. Pure dead reckoning: no camera, no ToF."""

    name = "legs"
    description = "IMU turns + encoder straights (dead reckoning)"

    def run(self, route, ctx, lease, stop_evt, progress, opts):
        drive_pwm = int(opts.get("drive_pwm", DEFAULT_DRIVE_PWM))
        turn_pwm = int(opts.get("turn_pwm", DEFAULT_TURN_PWM))
        legs = route.get("legs") or []
        done = []
        for i, leg in enumerate(legs):
            if stop_evt.is_set():
                return {"ok": False, "aborted": True, "legs_done": done,
                        "error": "aborted before leg %d" % (i + 1)}
            progress(leg=i + 1, legs=len(legs), step="turn", target=leg)
            rec = {"leg": i + 1, "turn_deg": leg["turn_deg"], "drive_in": leg["drive_in"]}
            if leg["turn_deg"]:
                r = ctx.turn_degrees(degrees=abs(leg["turn_deg"]),
                                     direction="left" if leg["turn_deg"] > 0 else "right",
                                     pwm=turn_pwm, confirmed_safe=True, lease=lease)
                rec["turn"] = {k: r.get(k) for k in ("ok", "actual_degrees_turned",
                                                     "hit_timeout", "error")}
                if not r.get("ok"):
                    done.append(rec)
                    return {"ok": False, "legs_done": done, "failed_leg": i + 1,
                            "error": "turn failed: %s" % (r.get("error") or r.get("abort_reason")
                                                         or "timeout")}
            if stop_evt.is_set():
                done.append(rec)
                return {"ok": False, "aborted": True, "legs_done": done,
                        "error": "aborted during leg %d" % (i + 1)}
            if leg["drive_in"] > 0:
                progress(leg=i + 1, legs=len(legs), step="drive", target=leg)
                r = ctx.drive_distance(distance_inches=leg["drive_in"], direction="forward",
                                       pwm=drive_pwm, confirmed_safe=True, lease=lease)
                rec["drive"] = {k: r.get(k) for k in ("ok", "actual_inches_traveled",
                                                      "hit_timeout", "error")}
                if not r.get("ok"):
                    done.append(rec)
                    return {"ok": False, "legs_done": done, "failed_leg": i + 1,
                            "error": "drive failed: %s" % (r.get("error") or r.get("abort_reason")
                                                          or "timeout")}
            done.append(rec)
        return {"ok": True, "legs_done": done}


FOLLOWERS = {f.name: f for f in (LegsFollower(),)}


# ---- the session -------------------------------------------------------------

class RouteSession:
    """One recording/route at a time. Every public method returns (code, dict)."""

    def __init__(self, ctx, routes_dir=ROUTES_DIR, record_hz=RECORD_HZ):
        self.ctx = ctx
        self.routes_dir = Path(routes_dir)
        self.record_hz = record_hz
        self.lock = threading.RLock()
        self.mode = "idle"          # idle | recording | recorded | processed | following
        self.name = None
        self.samples = []
        self.keyframes = []         # [{"idx", "t", "frame"}]
        self.route = None
        self.camera_url = ""
        self.t0 = None
        self.follow = None          # progress / result of the last follow
        self.stop_evt = threading.Event()
        self._rec_thread = None
        self._rec_stop = threading.Event()
        self._jsonl = None
        self._png_cache = (None, None)

    # -- helpers
    def _dir(self):
        return self.routes_dir / self.name

    def _busy(self):
        if self.mode == "recording":
            return "recording in progress; press End recording first"
        if self.mode == "following":
            return "the robot is following a route; Abort or wait"
        return None

    def status(self):
        with self.lock:
            out = {"mode": self.mode, "name": self.name, "samples": len(self.samples),
                   "keyframes": len(self.keyframes),
                   "elapsed_s": (round(time.monotonic() - self.t0, 1)
                                 if self.mode == "recording" and self.t0 else None),
                   "followers": {k: f.description for k, f in FOLLOWERS.items()},
                   "follow": self.follow,
                   "saved": self.list_routes()}
            if self.route:
                r = self.route
                out["route"] = {k: r.get(k) for k in (
                    "closure", "path_length_in", "route_length_in", "total_turn_deg",
                    "tol_in", "keyframes")}
                out["route"]["waypoints"] = len(r["waypoints"])
                out["route"]["legs"] = r["legs"]
            return out

    def list_routes(self):
        try:
            return sorted(p.name for p in self.routes_dir.iterdir()
                          if (p / "samples.jsonl").exists())
        except OSError:
            return []

    # -- recording
    def start_recording(self, name=None, camera_url=""):
        with self.lock:
            busy = self._busy()
            if busy:
                return 409, {"ok": False, "error": busy}
            if self.ctx.link is None:
                return 409, {"ok": False, "error": "no robot connected"}
            self.name = _safe_name(name)
            self._dir().mkdir(parents=True, exist_ok=True)
            self.samples, self.keyframes, self.route, self.follow = [], [], None, None
            self.camera_url = (camera_url or "").strip()
            self._jsonl = open(self._dir() / "samples.jsonl", "w")
            self.t0 = time.monotonic()
            self.mode = "recording"
            self._rec_stop.clear()
            self._rec_thread = threading.Thread(target=self._record_loop,
                                                name="route-recorder", daemon=True)
            self._rec_thread.start()
        _trace_event("route", action="record_start", name=self.name)
        return 200, {"ok": True, "name": self.name}

    def sample_once(self):
        """Take one sample now. The recorder thread calls this; tests call it directly."""
        link = self.ctx.link
        if link is None:
            return None
        try:
            tel = link.telemetry_snapshot()
        except Exception:
            return None
        with self.lock:
            if self.mode != "recording":
                return None
            s = {"t": round(time.monotonic() - self.t0, 3), "tel": tel}
            self.samples.append(s)
            if self._jsonl:
                self._jsonl.write(json.dumps(s, default=str) + "\n")
            return s

    def _record_loop(self):
        period = 1.0 / self.record_hz
        while not self._rec_stop.is_set():
            self.sample_once()
            self._rec_stop.wait(period)

    def keyframe(self, note=""):
        with self.lock:
            if self.mode != "recording":
                return 409, {"ok": False, "error": "not recording"}
        s = self.sample_once()  # a fresh sample, so the keyframe is exactly "now"
        with self.lock:
            if s is None and not self.samples:
                return 409, {"ok": False, "error": "no telemetry yet"}
            idx = len(self.samples) - 1
            kf = {"idx": idx, "t": self.samples[idx]["t"], "note": str(note)[:200],
                  "frame": None}
            cam, d = self.camera_url, self._dir()
        if cam:
            kf["frame"] = self._grab_frame(cam, d, len(self.keyframes) + 1)
        with self.lock:
            self.keyframes.append(kf)
            n = len(self.keyframes)
        _trace_event("route", action="keyframe", n=n, idx=idx)
        return 200, {"ok": True, "keyframe": n, **kf}

    @staticmethod
    def _grab_frame(url, d, n):
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
            from realtime_map import fetch_camera_frame
            data = fetch_camera_frame(url, timeout=3.0)
        except Exception:
            data = None
        if not data:
            return None
        p = d / ("keyframe_%02d.jpg" % n)
        p.write_bytes(data)
        return p.name

    def stop_recording(self):
        with self.lock:
            if self.mode != "recording":
                return 409, {"ok": False, "error": "not recording"}
            self._rec_stop.set()
        if self._rec_thread is not None:
            self._rec_thread.join(timeout=2)
        with self.lock:
            if self._jsonl:
                self._jsonl.close()
                self._jsonl = None
            (self._dir() / "keyframes.json").write_text(json.dumps(self.keyframes, indent=1))
            self.mode = "recorded"
            out = {"ok": True, "name": self.name, "samples": len(self.samples),
                   "keyframes": len(self.keyframes),
                   "duration_s": self.samples[-1]["t"] if self.samples else 0.0}
        _trace_event("route", action="record_stop", name=self.name,
                     samples=out["samples"], keyframes=out["keyframes"])
        return 200, out

    def load(self, name):
        with self.lock:
            busy = self._busy()
            if busy:
                return 409, {"ok": False, "error": busy}
            d = self.routes_dir / _safe_name(name)
            if not (d / "samples.jsonl").exists():
                return 404, {"ok": False, "error": "no recording named %r" % name}
            samples = [json.loads(line) for line in
                       (d / "samples.jsonl").read_text().splitlines() if line.strip()]
            kfp = d / "keyframes.json"
            keyframes = json.loads(kfp.read_text()) if kfp.exists() else []
            self.name, self.samples, self.keyframes = d.name, samples, keyframes
            self.follow = None
            rp = d / "route.json"
            self.route = json.loads(rp.read_text()) if rp.exists() else None
            self.mode = "processed" if self.route else "recorded"
            return 200, {"ok": True, "name": self.name, "samples": len(samples),
                         "keyframes": len(keyframes), "mode": self.mode}

    # -- processing
    def process(self, tol_in=route_math.DEFAULT_TOL_IN):
        with self.lock:
            busy = self._busy()
            if busy:
                return 409, {"ok": False, "error": busy}
            if not self.samples:
                return 409, {"ok": False, "error": "nothing recorded yet"}
            try:
                tol_in = max(0.5, min(24.0, float(tol_in)))
                route = route_math.process(self.samples,
                                           [k["idx"] for k in self.keyframes], tol_in)
            except (TypeError, ValueError) as e:
                return 400, {"ok": False, "error": str(e)}
            route["name"] = self.name
            self.route = route
            self.mode = "processed"
            (self._dir()).mkdir(parents=True, exist_ok=True)
            (self._dir() / "route.json").write_text(json.dumps(route, indent=1))
        try:
            (self._dir() / "route.png").write_bytes(self.map_png())
        except Exception:
            pass
        _trace_event("route", action="process", name=self.name,
                     legs=len(route["legs"]), closure_in=(route["closure"] or {}).get("dist_in"))
        return 200, {"ok": True, **self.status()["route"]}

    # -- following
    def _take_lease(self, who):
        ok, info = self.ctx.LEASE.take(HOLDER, who or "route page",
                                       purpose="follow a recorded route", ttl_s=60)
        if not ok:
            return None, info
        return info["lease"], None

    def _progress(self, **fields):
        with self.lock:
            if self.follow is not None:
                self.follow.update(fields)

    def follow_route(self, follower="legs", opts=None, who="route page", lease=None,
                     background=True):
        """Start following. With lease=None the session takes (and later releases) its
        own lease; an MCP caller passes the lease it already holds."""
        opts = dict(opts or {})
        with self.lock:
            busy = self._busy()
            if busy:
                return 409, {"ok": False, "error": busy}
            if self.route is None:
                return 409, {"ok": False, "error": "process the route first"}
            f = FOLLOWERS.get(follower)
            if f is None:
                return 400, {"ok": False, "error": "unknown follower %r; have %s"
                             % (follower, ", ".join(FOLLOWERS))}
            if self.ctx.link is None:
                return 409, {"ok": False, "error": "no robot connected"}
            if self.ctx.OPSTOP.latched:
                return 409, {"ok": False,
                             "error": "operator stop is latched -- Resume on the operator page"}
            own = lease is None
            if own:
                lease, err = self._take_lease(who)
                if err is not None:
                    return 409, {"ok": False, **{k: v for k, v in err.items()
                                                 if k != "drive_lease"}}
            self.stop_evt.clear()
            self.mode = "following"
            self.follow = {"follower": f.name, "running": True, "leg": 0,
                           "legs": len(self.route["legs"]), "opts": opts,
                           "started": time.strftime("%H:%M:%S")}
            route = self.route
        _trace_event("route", action="follow_start", name=self.name, follower=f.name)

        def _run():
            try:
                res = f.run(route, self.ctx, lease, self.stop_evt, self._progress, opts)
            except Exception as e:  # a follower bug must still brake and free the lease
                res = {"ok": False, "error": "follower crashed: %r" % e}
                try:
                    self.ctx.link.send_stop()
                except Exception:
                    pass
            if own:
                try:
                    self.ctx.LEASE.release(lease)
                except Exception:
                    pass
            with self.lock:
                self.follow.update(running=False, result=res)
                self.mode = "processed"
            _trace_event("route", action="follow_end", ok=res.get("ok"),
                         error=res.get("error"))
            return res

        if background:
            threading.Thread(target=_run, name="route-follower", daemon=True).start()
            return 200, {"ok": True, "following": True, "legs": len(route["legs"])}
        res = _run()
        return (200 if res.get("ok") else 409), res

    def test_turn(self, who="route page", turn_pwm=DEFAULT_TURN_PWM):
        """One small left turn so the operator can check left really is left."""
        with self.lock:
            busy = self._busy()
            if busy:
                return 409, {"ok": False, "error": busy}
            if self.ctx.link is None:
                return 409, {"ok": False, "error": "no robot connected"}
            lease, err = self._take_lease(who)
            if err is not None:
                return 409, {"ok": False, **{k: v for k, v in err.items()
                                             if k != "drive_lease"}}
            prev = self.mode
            self.mode = "following"
        try:
            r = self.ctx.turn_degrees(degrees=TEST_TURN_DEG, direction="left",
                                      pwm=int(turn_pwm), confirmed_safe=True, lease=lease)
        finally:
            try:
                self.ctx.LEASE.release(lease)
            except Exception:
                pass
            with self.lock:
                self.mode = prev
        out = {k: r.get(k) for k in ("ok", "actual_degrees_turned", "error")}
        out["check"] = ("Did the nose swing LEFT (counter-clockwise from above)? If it "
                        "went right, the turn direction is inverted -- do not Follow.")
        return (200 if r.get("ok") else 409), out

    def abort(self, who="route page"):
        self.stop_evt.set()
        snap = self.ctx.operator_stop(who, "route abort")
        with self.lock:
            was = self.mode == "following"
        return 200, {"ok": True, "was_following": was, "operator_stop": snap,
                     "note": "operator stop is latched; Resume on the operator page"}

    # -- the map
    def map_png(self):
        with self.lock:
            key = (self.name, len(self.samples), id(self.route),
                   (self.follow or {}).get("leg"))
            if self._png_cache[0] == key:
                return self._png_cache[1]
            samples = list(self.samples)
            route = self.route
            kf = [k["idx"] for k in self.keyframes]
        png = render_route_png(samples, kf, route,
                               title=("%s (%s)" % (self.name or "no recording", self.mode)))
        with self.lock:
            self._png_cache = (key, png)
        return png


def render_route_png(samples, keyframe_idx, route, title=""):
    """Recorded trail (thin), processed route (thick, numbered waypoints), keyframes."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    import io
    import realtime_map as rm
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(7, 7), facecolor=rm.DARK_BG)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.set_facecolor(rm.DARK_PANEL)
    raw = route_math.integrate(samples) if samples else []
    if raw:
        ax.plot([p[0] for p in raw], [p[1] for p in raw], color=rm.DARK_DIM,
                linewidth=1, label="recorded (raw)")
        ax.plot(raw[0][0], raw[0][1], "o", color=rm.DARK_START, markersize=9)
        for k in keyframe_idx:
            if 0 <= k < len(raw):
                ax.plot(raw[k][0], raw[k][1], "D", color="#f2c94c", markersize=8)
        if route is None:
            lx, ly, lth = raw[-1]
            ax.add_patch(rm._robot_patch(lx, ly, lth))
    if route is not None:
        tr = route.get("trail") or []
        if tr:
            ax.plot([p[0] for p in tr], [p[1] for p in tr], color=rm.DARK_TRAIL,
                    linewidth=1, alpha=0.6, label="loop-closed")
        wps = route.get("waypoints") or []
        ax.plot([w["x"] for w in wps], [w["y"] for w in wps], "-o", color=rm.DARK_END,
                linewidth=2.5, markersize=5, label="route")
        for n, w in enumerate(wps):
            ax.annotate(str(n), (w["x"], w["y"]), color=rm.DARK_INK, fontsize=8,
                        xytext=(4, 4), textcoords="offset points")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (in, forward at start)", color=rm.DARK_DIM)
    ax.set_ylabel("y (in, left)", color=rm.DARK_DIM)
    ax.set_title(title, color=rm.DARK_INK)
    ax.tick_params(colors=rm.DARK_DIM)
    for spine in ax.spines.values():
        spine.set_color("#3a3a3a")
    ax.grid(True, alpha=0.2, color=rm.DARK_DIM)
    if raw or route:
        leg = ax.legend(loc="upper right", fontsize=8, facecolor=rm.DARK_PANEL,
                        edgecolor="#3a3a3a")
        for t in leg.get_texts():
            t.set_color(rm.DARK_INK)
    else:
        ax.set_xlim(-60, 60)
        ax.set_ylim(-60, 60)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=90)
    return buf.getvalue()


SESSION: Optional[RouteSession] = None


# ---- MCP tools ---------------------------------------------------------------

def route_status() -> dict:
    """Read-only. The watch_and_repeat state: mode (idle/recording/recorded/processed/
    following), the current recording, the processed route's legs and loop-closure
    error, and the last follow's progress. The operator records and processes on the
    daemon page at /route; this never moves the robot."""
    return {"ok": True, **SESSION.status()}


def route_process(tol_in: float = route_math.DEFAULT_TOL_IN, name: Optional[str] = None) -> dict:
    """Read-only. Turn the current (or named, saved) recording into a route: dead-reckon,
    close the loop, simplify to waypoints within tol_in inches (keyframes always kept),
    and compile turn+drive legs. Never moves the robot."""
    if name:
        code, out = SESSION.load(name)
        if code != 200:
            return out
    return SESSION.process(tol_in)[1]


def route_follow(confirmed_safe: bool, lease: Optional[str] = None, follower: str = "legs",
                 drive_pwm: int = DEFAULT_DRIVE_PWM,
                 turn_pwm: int = DEFAULT_TURN_PWM) -> dict:
    """MOVES THE ROBOT around the whole processed route, leg by leg, and blocks until it
    finishes, fails, or the operator aborts. confirmed_safe must be true, set only after
    the human operator agreed to THIS route run in this moment: state the leg count and
    total distance from route_status() first. Every leg goes through the same guards as
    turn_degrees/drive_distance; pass your drive lease if the daemon requires one."""
    if not confirmed_safe:
        st = SESSION.status().get("route")
        return {"ok": False, "error": "confirmed_safe must be true, set only after the "
                "operator agreed to this route run. Tell them the legs and distance first.",
                "route": st}
    code, out = SESSION.follow_route(follower, {"drive_pwm": drive_pwm, "turn_pwm": turn_pwm},
                                     who="MCP client", lease=lease, background=False)
    return out


ROUTE_TOOLS = (route_status, route_process, route_follow)


# ---- HTTP --------------------------------------------------------------------

async def _body(request: Request):
    try:
        b = json.loads(await request.body() or b"{}")
        return b if isinstance(b, dict) else {}
    except ValueError:
        return {}


def _who(request, body):
    who = request.client.host if request.client else "route page"
    return str(body.get("who") or who)[:80]


def register(mcp, ctx):
    global CTX, SESSION
    CTX = ctx
    SESSION = RouteSession(ctx)
    for fn in ROUTE_TOOLS:
        mcp.tool()(fn)

    def j(res):
        code, out = res
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/route", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/route/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        st = SESSION.status()
        st["operator_stop"] = CTX.OPSTOP.snapshot()
        st["connected"] = CTX.link is not None
        return JSONResponse(st)

    @mcp.custom_route("/route/map.png", methods=["GET"], include_in_schema=False)
    async def map_png(request: Request):
        import anyio
        try:
            png = await anyio.to_thread.run_sync(SESSION.map_png)
        except Exception as e:
            return JSONResponse({"error": repr(e)}, status_code=503)
        return Response(png, media_type="image/png", headers={"cache-control": "no-store"})

    @mcp.custom_route("/route/record/start", methods=["POST"], include_in_schema=False)
    async def rec_start(request: Request):
        b = await _body(request)
        return j(SESSION.start_recording(b.get("name"), b.get("camera_url", "")))

    @mcp.custom_route("/route/record/stop", methods=["POST"], include_in_schema=False)
    async def rec_stop(request: Request):
        return j(SESSION.stop_recording())

    @mcp.custom_route("/route/keyframe", methods=["POST"], include_in_schema=False)
    async def keyframe(request: Request):
        import anyio
        b = await _body(request)
        return j(await anyio.to_thread.run_sync(SESSION.keyframe, b.get("note", "")))

    @mcp.custom_route("/route/load", methods=["POST"], include_in_schema=False)
    async def load(request: Request):
        b = await _body(request)
        return j(SESSION.load(b.get("name", "")))

    @mcp.custom_route("/route/process", methods=["POST"], include_in_schema=False)
    async def process(request: Request):
        import anyio
        b = await _body(request)
        return j(await anyio.to_thread.run_sync(
            SESSION.process, b.get("tol_in", route_math.DEFAULT_TOL_IN)))

    @mcp.custom_route("/route/follow", methods=["POST"], include_in_schema=False)
    async def follow(request: Request):
        b = await _body(request)
        opts = {"drive_pwm": b.get("drive_pwm", DEFAULT_DRIVE_PWM),
                "turn_pwm": b.get("turn_pwm", DEFAULT_TURN_PWM)}
        return j(SESSION.follow_route(b.get("follower", "legs"), opts, who=_who(request, b)))

    @mcp.custom_route("/route/test_turn", methods=["POST"], include_in_schema=False)
    async def test_turn(request: Request):
        import anyio
        b = await _body(request)
        return j(await anyio.to_thread.run_sync(
            SESSION.test_turn, _who(request, b), b.get("turn_pwm", DEFAULT_TURN_PWM)))

    @mcp.custom_route("/route/abort", methods=["POST"], include_in_schema=False)
    async def abort(request: Request):
        b = await _body(request)
        return j(SESSION.abort(_who(request, b)))


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama watch and repeat</title>
<style>
  :root { --bg:#f6f5f2; --fg:#1b1b1a; --muted:#6b6a66; --card:#fff; --line:#dcdad3;
          --stop:#c62828; --go:#2e6b3a; --key:#b8860b; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#161615; --fg:#ecebe7; --muted:#9a9993; --card:#212120; --line:#363531;
            --stop:#e2504b; --go:#58a568; --key:#f2c94c; }
  }
  * { box-sizing:border-box; }
  body { margin:0; padding:12px; background:var(--bg); color:var(--fg);
         font:18px/1.35 system-ui, sans-serif; }
  .layout { display:grid; gap:12px; grid-template-columns:minmax(0,1fr) minmax(0,1.3fr); }
  @media (max-width: 900px) { .layout { grid-template-columns:minmax(0,1fr); } }
  h1 { font-size:24px; margin:0 0 8px; } h1 a { font-size:16px; color:var(--muted); }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px;
          padding:12px; margin-top:12px; }
  .banner { font-weight:600; }
  button { font:inherit; padding:12px 14px; border-radius:8px; border:1px solid var(--line);
           background:var(--card); color:var(--fg); cursor:pointer; min-height:48px; }
  button:disabled { opacity:.4; cursor:default; }
  #abort { width:100%; background:var(--stop); color:#fff; border:none; font-size:26px;
           font-weight:700; min-height:72px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; margin-top:8px; }
  .row > button { flex:1 1 140px; }
  #rec.on { background:var(--stop); color:#fff; border-color:var(--stop); }
  #key { border-color:var(--key); }
  #follow { background:var(--go); color:#fff; border-color:var(--go); }
  label { display:flex; gap:8px; align-items:center; margin-top:8px; flex-wrap:wrap; }
  input, select { font:inherit; padding:6px 8px; border-radius:6px; border:1px solid var(--line);
                  background:var(--bg); color:var(--fg); min-width:0; }
  input[type=range] { flex:1; padding:0; }
  #map { width:100%; border-radius:8px; border:1px solid var(--line); background:#1e1e1e; }
  #status { white-space:pre-wrap; font-variant-numeric:tabular-nums; }
  small { color:var(--muted); }
  #msg { color:var(--muted); min-height:1.4em; }
  table { border-collapse:collapse; width:100%; font-size:15px; font-variant-numeric:tabular-nums; }
  td, th { padding:2px 6px; text-align:right; border-bottom:1px solid var(--line); }
  tr.cur { background:color-mix(in srgb, var(--go) 25%, transparent); }
</style></head><body>
<h1>eLlama watch and repeat <a href="/">operator page &rarr;</a></h1>
<div class="layout">
<div>
  <button id="abort">ABORT</button>
  <small>Brakes now and latches the operator stop. Resume it on the operator page.</small>

  <div class="card">
    <div class="banner">1. Record</div>
    <label>Name <input id="name" placeholder="route_YYYYMMDD_HHMMSS"></label>
    <label>Camera (optional) <input id="cam" placeholder="192.168.0.33:8080"></label>
    <div class="row">
      <button id="rec">Start recording</button>
      <button id="key" disabled>Keyframe</button>
    </div>
    <small>Drive the loop with the stick or the operator page, and finish where you
    started. A keyframe pins the current pose as a corner the route must pass through.</small>
  </div>

  <div class="card">
    <div class="banner">2. Process</div>
    <label>Simplify tolerance <input id="tol" type="number" value="3" min="0.5" max="24"
      step="0.5" style="width:5em"> in</label>
    <div class="row"><button id="process" disabled>Process route</button></div>
    <label>Or load a saved run <select id="saved"></select>
      <button id="load">Load</button></label>
  </div>

  <div class="card">
    <div class="banner">3. Follow</div>
    <label>Follower <select id="follower"></select></label>
    <label>Drive PWM <span id="dv">40</span>
      <input id="dpwm" type="range" min="20" max="125" value="40"></label>
    <label>Turn PWM <span id="tv">60</span>
      <input id="tpwm" type="range" min="20" max="125" value="60"></label>
    <div class="row">
      <button id="testturn">Test turn (10&deg; left)</button>
      <button id="follow" disabled>Follow route</button>
    </div>
    <small>Pressing Follow is your go-ahead for the whole route: stay with the robot and
    keep ABORT in reach. Run Test turn once first: the nose must swing left.</small>
  </div>
  <p id="msg"></p>
</div>
<div>
  <div class="card">
    <div class="banner" id="banner">connecting&hellip;</div>
    <div id="status"></div>
  </div>
  <div class="card"><img id="map" src="/route/map.png" alt="recorded trail and route"></div>
  <div class="card"><div class="banner">Legs</div><table id="legs"></table></div>
</div>
</div>
<script>
const $ = id => document.getElementById(id);
let st = {};
async function post(path, body) {
  try {
    const r = await fetch(path, {method:'POST', headers:{'content-type':'application/json'},
                                 body: JSON.stringify(body || {})});
    const j = await r.json();
    $('msg').textContent = j.ok ? (j.check || 'ok') : ('refused: ' + (j.error || r.status));
    tick();
    return j;
  } catch (e) { $('msg').textContent = 'network error: ' + e; }
}
$('abort').onclick = () => post('/route/abort');
$('rec').onclick = () => st.mode === 'recording'
  ? post('/route/record/stop')
  : post('/route/record/start', {name: $('name').value, camera_url: $('cam').value});
$('key').onclick = () => post('/route/keyframe');
$('process').onclick = () => post('/route/process', {tol_in: parseFloat($('tol').value)});
$('load').onclick = () => post('/route/load', {name: $('saved').value});
$('testturn').onclick = () => post('/route/test_turn', {turn_pwm: +$('tpwm').value});
$('follow').onclick = () => post('/route/follow', {follower: $('follower').value,
  drive_pwm: +$('dpwm').value, turn_pwm: +$('tpwm').value});
$('dpwm').oninput = () => $('dv').textContent = $('dpwm').value;
$('tpwm').oninput = () => $('tv').textContent = $('tpwm').value;

function fillSelect(sel, items, labels) {
  const want = JSON.stringify(items);
  if (sel.dataset.items === want) return;
  const keep = sel.value;
  sel.innerHTML = '';
  for (const k of items) {
    const o = document.createElement('option');
    o.value = k; o.textContent = labels ? k + ' - ' + labels[k] : k;
    sel.appendChild(o);
  }
  if (items.includes(keep)) sel.value = keep;
  sel.dataset.items = want;
}
let lastMapKey = '';
async function tick() {
  try {
    st = await (await fetch('/route/state', {cache:'no-store'})).json();
  } catch (e) { $('banner').textContent = 'daemon not reachable'; return; }
  const m = st.mode, rec = m === 'recording', fol = m === 'following';
  const stopped = st.operator_stop && st.operator_stop.latched;
  $('banner').textContent = (stopped ? 'OPERATOR STOP LATCHED - ' : '') +
    (st.connected ? '' : 'no robot - ') + 'mode: ' + m + (st.name ? '  (' + st.name + ')' : '');
  $('rec').textContent = rec ? 'End recording' : 'Start recording';
  $('rec').classList.toggle('on', rec);
  $('rec').disabled = fol;
  $('key').disabled = !rec;
  $('process').disabled = rec || fol || !st.samples;
  $('follow').disabled = !st.route || rec || fol || stopped;
  $('testturn').disabled = rec || fol || stopped;
  $('load').disabled = rec || fol;
  fillSelect($('follower'), Object.keys(st.followers || {}), st.followers);
  fillSelect($('saved'), st.saved || []);
  const lines = [];
  lines.push('samples ' + st.samples + '   keyframes ' + st.keyframes +
             (st.elapsed_s != null ? '   ' + st.elapsed_s + ' s' : ''));
  if (st.route) {
    const r = st.route, c = r.closure || {};
    lines.push('route ' + r.waypoints + ' waypoints, ' + r.legs.length + ' legs, ' +
               r.route_length_in + ' in, turns sum ' + r.total_turn_deg + '\\u00b0');
    lines.push('loop closure error ' + c.dist_in + ' in, ' + c.dtheta_deg + '\\u00b0' +
               (c.dist_in > 12 ? '   (large: dead reckoning drifted, or you did not end at the start)' : ''));
  }
  const f = st.follow;
  if (f) {
    lines.push('follow [' + f.follower + '] ' + (f.running ? 'running: leg ' + f.leg + '/' +
               f.legs + ' ' + (f.step || '') : 'finished'));
    if (f.result) lines.push('  result: ' + (f.result.ok ? 'ok' : (f.result.error || 'failed')));
  }
  $('status').textContent = lines.join('\\n');
  const t = $('legs');
  if (st.route) {
    let h = '<tr><th>#</th><th>turn&deg;</th><th>drive in</th><th>actual</th></tr>';
    const done = (f && f.result && f.result.legs_done) || [];
    st.route.legs.forEach((L, i) => {
      const d = done[i] || {};
      const act = (d.turn && d.turn.actual_degrees_turned != null ? d.turn.actual_degrees_turned + '\\u00b0 ' : '') +
                  (d.drive && d.drive.actual_inches_traveled != null ? d.drive.actual_inches_traveled + ' in' : '');
      const cur = f && f.running && f.leg === i + 1 ? ' class="cur"' : '';
      h += '<tr' + cur + '><td>' + (i + 1) + '</td><td>' + L.turn_deg + '</td><td>' +
           L.drive_in + '</td><td>' + act + '</td></tr>';
    });
    t.innerHTML = h;
  } else t.innerHTML = '';
  const key = [st.name, st.samples, st.mode, f && f.leg].join('|');
  if (key !== lastMapKey) { lastMapKey = key; $('map').src = '/route/map.png?ts=' + Date.now(); }
}
setInterval(tick, 1000); tick();
</script></body></html>
"""
