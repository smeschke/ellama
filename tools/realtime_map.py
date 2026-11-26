#!/usr/bin/env python3
"""Realtime top-down path mapper for eLlama. Polls the daemon, never drives.

Reads GET /state from mcp_server/robot_server.py --http (the single owner of
the bridge), dead-reckons pose from encoder distance + IMU yaw, optionally
casts live ToF rays into a log-odds occupancy grid, and saves one camera
keyframe per stop.

Usage:
  tools/realtime_map.py [--daemon-url http://127.0.0.1:8765] [--save map.png]
  tools/realtime_map.py --replay recordings/stick_test/stick_test.csv --save map.png
  tools/realtime_map.py --once   # single poll, print pose, exit (smoke test)

Single driver: stick (CMD overhear -> telemetry.stick) and daemon FBLR
(dash_hold -> drive_lease holder=dashboard) both just move the robot; pose
integrates the resulting motion either way. Source is display context only.
"""

import argparse
import csv
import json
import math
import os
import re
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for bridge/
import bridge  # noqa: E402  (channel labels; serial already a project dep)

try:
    import numpy as np
except ImportError:  # pragma: no cover
    print("needs numpy (mcp_server/.venv has it via --system-site-packages)",
          file=sys.stderr)
    raise

STALE_S = 1.5
MOVING_SPEED_IN_S = 0.5
STOP_HOLD_S = 2.0
RANGE_MAX_MM = 2000
MM_PER_IN = 25.4

GRID_RES_IN = 2.0
GRID_W_CELLS = 600  # 100 ft square, robot starts centered
GRID_H_CELLS = 600
L_OCC = 0.85
L_FREE = -0.12
L_MIN, L_MAX = -4.0, 4.0

# Ray origins relative to robot center, inches (x forward, y left). Front bumper
# is roughly 16 in ahead of center on this suitcase base; sensors sit near the
# edges. Rear boards face backward, so their rays run theta + pi.
RAY_ORIGINS = {
    "front_left": (10.0, 6.0),
    "front_right": (10.0, -6.0),
    "rear_left": (-10.0, 6.0),
    "rear_right": (-10.0, -6.0),
}
DEFAULT_ORIGIN = (10.0, 0.0)


def ray_geometry(label, sensor=""):
    """(origin, reverse) for a ToF channel label: rear boards look backward."""
    origin = RAY_ORIGINS.get(label, DEFAULT_ORIGIN)
    reverse = label.startswith("rear_") or "REAR" in str(sensor).upper()
    if reverse and label not in RAY_ORIGINS:
        origin = (-10.0, 0.0)
    return origin, reverse


# Beam splay in degrees, + toward robot-left: the two front boards are toed
# slightly off center to widen coverage instead of staring straight ahead.
# Measure yours and adjust; the VL53L0X cone itself is ~25 deg wide.
RAY_SPLAY_DEG = {
    "front_left": 12.0,
    "front_right": -12.0,
    "rear_left": 12.0,
    "rear_right": -12.0,
}
DEFAULT_SPLAY_DEG = 0.0


def _robot_pts(x, y, th):
    """Heading triangle corners at a pose: nose 8 in ahead, 6 in behind."""
    import math
    c, s = math.cos(th), math.sin(th)
    pts = [(8, 0), (-6, 5), (-6, -5)]
    return [(x + fx * c - fy * s, y + fx * s + fy * c) for fx, fy in pts]


def _robot_patch(x, y, th):
    import matplotlib.patches as patches
    return patches.Polygon(_robot_pts(x, y, th), closed=True,
                           facecolor=DARK_TRAIL, edgecolor=DARK_INK,
                           linewidth=1.2, zorder=5)


def wrap_deg(d):
    d = (d + 180.0) % 360.0 - 180.0
    return d


def fetch_state(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def source_label(state):
    """stick | dashboard | other | quiet. Display context only."""
    tel = (state.get("telemetry") or {})
    lease = (state.get("drive_lease") or {})
    stick = tel.get("stick") or {}
    if stick.get("active"):
        return "stick"
    if lease.get("held") and lease.get("move_in_progress"):
        holder = lease.get("holder") or ""
        return "dashboard" if holder == "dashboard" else "other"
    return "quiet"


class PoseTracker:
    """Dead-reckon x (forward at start), y (left), theta CCW from enc+IMU."""

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.prev_dist = None
        self.prev_yaw = None
        self.last_t = None
        self.speed = 0.0

    def update(self, dist_avg_in, yaw_deg, t, enc_live, imu_live):
        moved = False
        if self.prev_dist is None:
            self.prev_dist, self.prev_yaw, self.last_t = dist_avg_in, yaw_deg, t
            return 0.0, 0.0
        dt = max(t - self.last_t, 1e-6)
        dd = (dist_avg_in - self.prev_dist) if enc_live else 0.0
        dh = wrap_deg(yaw_deg - self.prev_yaw) if (imu_live and yaw_deg is not None
                                                  and self.prev_yaw is not None) else 0.0
        # A physical left turn arrives as yaw DECREASING (verified IRL: a left
        # spin reads left encoder down / right encoder up, and the dashboard
        # agrees), so theta integrates -dh to render left turns CCW like the
        # dashboard. If the IMU sign is ever corrected upstream, drop this
        # negation instead of "fixing" the dashboard to match.
        dh = -dh
        if enc_live:
            self.prev_dist = dist_avg_in
        if imu_live and yaw_deg is not None:
            self.prev_yaw = yaw_deg
        self.last_t = t
        self.speed = dd / dt
        if abs(dd) > 1e-9 or abs(dh) > 1e-9:
            moved = True
        th_mid = self.theta + math.radians(dh) / 2.0
        self.x += dd * math.cos(th_mid)
        self.y += dd * math.sin(th_mid)
        self.theta += math.radians(dh)
        return dd, dh


class OccupancyGrid:
    def __init__(self, res_in=GRID_RES_IN, w=GRID_W_CELLS, h=GRID_H_CELLS):
        self.res = res_in
        self.w, self.h = w, h
        self.ox, self.oy = w // 2, h // 2  # robot start cell
        self.logodds = np.zeros((h, w), dtype=np.float32)
        self.seen = np.zeros((h, w), dtype=np.int32)
        self.hits = 0
        self.rays = 0

    def _cell(self, x_in, y_in):
        return (int(round(self.ox + x_in / self.res)),
                int(round(self.oy + y_in / self.res)))

    def _add(self, cx, cy, v):
        if 0 <= cx < self.w and 0 <= cy < self.h:
            self.logodds[cy, cx] = min(L_MAX, max(L_MIN, self.logodds[cy, cx] + v))
            self.seen[cy, cx] += 1

    def cast(self, px, py, theta, dist_mm, origin, reverse=False, splay_deg=0.0):
        """One ToF ray. dist_mm None = clear to max range, no hit. Rear boards
        look backward, so their rays run along theta + pi; splay_deg toes the
        beam off center (+ toward robot-left)."""
        self.rays += 1
        fx, fy = origin
        sx = px + fx * math.cos(theta) - fy * math.sin(theta)
        sy = py + fx * math.sin(theta) + fy * math.cos(theta)
        ang = theta + (math.pi if reverse else 0.0) + math.radians(splay_deg)
        max_in = RANGE_MAX_MM / MM_PER_IN
        hit_in = max_in if dist_mm is None else min(dist_mm / MM_PER_IN, max_in)
        steps = max(int(hit_in / (self.res / 2.0)), 1)
        for i in range(1, steps):
            d = hit_in * i / steps
            self._add(*self._cell(sx + d * math.cos(ang),
                                  sy + d * math.sin(ang)), L_FREE)
        if dist_mm is not None and dist_mm <= RANGE_MAX_MM:
            self._add(*self._cell(sx + hit_in * math.cos(ang),
                                  sy + hit_in * math.sin(ang)), L_OCC)
            self.hits += 1

    def prob(self):
        # Unobserved cells stay masked so they render as panel-dark, never as
        # mid-grey: only measured free/occupied space shows. A zero-observation
        # cell is unknown, not 50/50.
        p = 1.0 / (1.0 + np.exp(-self.logodds))
        return np.ma.masked_where(self.seen == 0, p)


def fetch_camera_frame(url, timeout=5.0):
    """Grab one JPEG from an MJPEG/snapshot URL. Returns bytes or None."""
    if not url:
        return None
    cands = [url]
    base = url.rstrip("/")
    if not os.path.splitext(base)[1]:
        cands += [base + "/shot.jpg", base + "/photo.jpg", base + "/video/mjpeg"]
    last = None
    for u in cands:
        try:
            req = urllib.request.Request(u, headers={"User-Agent": "ellama-map/1.0"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                data = r.read(4_000_000)
            soi = data.find(b"\xff\xd8\xff")
            eoi = data.find(b"\xff\xd9", soi + 3) if soi >= 0 else -1
            if soi >= 0 and eoi > soi:
                return data[soi:eoi + 2]
            if data[:2] == b"\xff\xd8":
                return data
            last = "not a jpeg"
        except Exception as e:  # noqa: BLE001 - tried next candidate
            last = str(e)[:100]
    print(f"camera: no frame ({last})", file=sys.stderr)
    return None


def keyframe_name(t, pose, facing):
    """Frame filename: pose plus which end the camera is mounted on, so a
    swapped phone cannot silently mislabel a heading (arrow-end looks along
    theta, tail-end looks along theta + pi)."""
    facing = re.sub(r"[^A-Za-z0-9]+", "", facing or "arrow") or "arrow"
    base = (f"stop_t{t:07.1f}_x{pose[0]:+.1f}_y{pose[1]:+.1f}"
            .replace("+", "p").replace("-", "m"))
    return f"{base}_cam-{facing}.jpg"


class StopKeyframes:
    def __init__(self, frames_dir, camera_url, facing="arrow"):
        self.dir = Path(frames_dir) if frames_dir else None
        self.camera_url = camera_url
        self.facing = facing
        self.still_since = None
        self.was_moving = False
        self.saved = 0

    def update(self, moving, t, pose, log_row=None):
        if moving:
            self.still_since = None
            self.was_moving = True
            return None
        if self.still_since is None:
            self.still_since = t
        if self.was_moving and (t - self.still_since) >= STOP_HOLD_S:
            self.was_moving = False
            return self.capture(t, pose)
        return None

    def capture(self, t, pose):
        if not self.dir:
            return None
        frame = fetch_camera_frame(self.camera_url)
        if frame is None:
            return None
        self.dir.mkdir(parents=True, exist_ok=True)
        p = self.dir / keyframe_name(t, pose, self.facing)
        p.write_bytes(frame)
        self.saved += 1
        return str(p)


# Dark ops-dashboard palette, matching the live_view dashboard panels.
DARK_BG = "#161616"
DARK_PANEL = "#1e1e1e"
DARK_INK = "#e8e8e8"
DARK_DIM = "#9a9a9a"
DARK_TRAIL = "#35d0e0"
DARK_START = "#7fd97b"
DARK_END = "#ff9d5c"


def _grid_cmap():
    from matplotlib import colormaps
    cmap = colormaps["Greys"].copy()
    cmap.set_bad(DARK_PANEL)  # unobserved cells melt into the panel
    return cmap


def _fit_slice(grid, trail):
    """Cell slice + inch extent framing observed cells and trail (saved maps
    only; the live window keeps a fixed frame so it never jumps)."""
    import numpy as np
    res = grid.res
    seen_ys, seen_xs = np.where(grid.seen > 0)
    pts = [((cx - grid.ox) * res, (cy - grid.oy) * res)
           for cx, cy in zip(seen_xs.tolist(), seen_ys.tolist())]
    pts += list(trail)
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    hx = max((max(xs) - min(xs)) / 2 + 20.0, 60.0)
    hy = max((max(ys) - min(ys)) / 2 + 20.0, 60.0)
    x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
    c0 = max(int(x0 / res + grid.ox), 0)
    c1 = min(int(x1 / res + grid.ox) + 1, grid.w)
    r0 = max(int(y0 / res + grid.oy), 0)
    r1 = min(int(y1 / res + grid.oy) + 1, grid.h)
    return (r0, r1, c0, c1, x0, x1, y0, y1)


def _render_fig(trail, grid, title):
    # Explicit Agg figure: never touches the global backend, so a live
    # interactive window (--show) and PNG saves coexist in one process.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(8, 8), facecolor=DARK_BG)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.set_facecolor(DARK_PANEL)
    sl = _fit_slice(grid, trail)
    if sl is None:
        arr = grid.prob()
        extent = [-(grid.w // 2) * grid.res, (grid.w // 2) * grid.res,
                  -(grid.h // 2) * grid.res, (grid.h // 2) * grid.res]
    else:
        r0, r1, c0, c1, x0, x1, y0, y1 = sl
        arr = grid.prob()[r0:r1, c0:c1]
        extent = [x0, x1, y0, y1]
    ax.imshow(arr, origin="lower", cmap=_grid_cmap(), vmin=0.0, vmax=1.0,
              extent=extent, interpolation="nearest")
    xs = [p[0] for p in trail]
    ys = [p[1] for p in trail]
    if xs:
        ax.plot(xs, ys, linewidth=2, color=DARK_TRAIL)
        ax.plot(xs[0], ys[0], "o", markersize=8, color=DARK_START)
        lx, ly = trail[-1][0], trail[-1][1]
        lth = trail[-1][2] if len(trail[-1]) > 2 else 0.0
        ax.plot(lx, ly, "s", markersize=8, color=DARK_END)
        ax.add_patch(_robot_patch(lx, ly, lth))
    ax.set_aspect("equal")
    ax.set_xlabel("x (in, forward at start)", color=DARK_DIM)
    ax.set_ylabel("y (in, left)", color=DARK_DIM)
    ax.set_title(title or "eLlama path (trail) + ToF walls", color=DARK_INK)
    ax.tick_params(colors=DARK_DIM)
    for spine in ax.spines.values():
        spine.set_color("#3a3a3a")
    ax.grid(True, alpha=0.2, color=DARK_DIM)
    return fig


def save_map_png(path, trail, grid, title=""):
    data = render_png_bytes(trail, grid, title)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(data)
    return path


def render_png_bytes(trail, grid, title=""):
    import io
    fig = _render_fig(trail, grid, title)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    return buf.getvalue()


VIEW_PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>eLlama map</title>
<style>body{font-family:system-ui,sans-serif;margin:12px;background:#161616;color:#e8e8e8}
h2{color:#e8e8e8;font-weight:600}
img{max-width:min(92vw,720px);border:1px solid #3a3a3a;border-radius:8px;background:#1e1e1e}
#s{white-space:pre-wrap;color:#9a9a9a}</style></head><body>
<h2>eLlama map (live)</h2>
<img id=m src="/map.png" alt="top-down trail and walls">
<p id=s>connecting&hellip;</p>
<script>
async function tick(){
  try {
    const r = await fetch('/status',{cache:'no-store'});
    const s = await r.json();
    document.getElementById('s').textContent =
      `t=${s.t}s src=${s.src} x=${s.x} y=${s.y} th=${s.theta} rays=${s.rays} hits=${s.hits} shots=${s.shots}`;
  } catch(e) {}
  document.getElementById('m').src = '/map.png?ts=' + Date.now();
}
setInterval(tick, 2000); tick();
</script></body></html>"""


def serve_forever(port, get_state):
    """Blocking: serve the auto-refreshing map page (Chrome target)."""
    import http.server

    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                body, ctype = VIEW_PAGE, "text/html"
            elif path == "/map.png":
                try:
                    body, ctype = get_state("png"), "image/png"
                except Exception:
                    self.send_error(503)
                    return
            elif path == "/status":
                body, ctype = get_state("status"), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("content-type", ctype)
            self.send_header("cache-control", "no-store")
            self.send_header("content-length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), H)
    print(f"map page at http://127.0.0.1:{srv.server_port}/ -- open it in Chrome")
    srv.serve_forever()


def live_window():
    """Interactive window for --show. Returns a state list, or None headless."""
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        plt.ion()
        plt.show(block=False)
    except Exception as e:  # noqa: BLE001 - no DISPLAY, etc.
        print(f"live window unavailable ({e}); continuing headless", file=sys.stderr)
        return None
    ext = None
    return [fig, ax, None, None, ext, None]


def live_draw(v, trail, grid, title=""):
    import matplotlib.pyplot as plt
    fig, ax, im, ln, ext, bot = v
    if ext is None:
        ext = [-(grid.w // 2) * grid.res, (grid.w // 2) * grid.res,
               -(grid.h // 2) * grid.res, (grid.h // 2) * grid.res]
        fig.patch.set_facecolor(DARK_BG)
        ax.set_facecolor(DARK_PANEL)
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        ax.set_aspect("equal")
        ax.set_xlabel("x (in, forward at start)", color=DARK_DIM)
        ax.set_ylabel("y (in, left)", color=DARK_DIM)
        ax.tick_params(colors=DARK_DIM)
        for spine in ax.spines.values():
            spine.set_color("#3a3a3a")
        ax.grid(True, alpha=0.2, color=DARK_DIM)
        v[4] = ext
    if im is None:
        im = ax.imshow(grid.prob(), origin="lower", cmap=_grid_cmap(),
                       vmin=0.0, vmax=1.0, extent=ext)
        v[2] = im
    else:
        im.set_data(grid.prob())
    xs = [p[0] for p in trail]
    ys = [p[1] for p in trail]
    if ln is None:
        (ln,) = ax.plot(xs, ys, linewidth=2, color=DARK_TRAIL)
        v[3] = ln
    else:
        ln.set_data(xs, ys)
    if trail:
        lx, ly = trail[-1][0], trail[-1][1]
        lth = trail[-1][2] if len(trail[-1]) > 2 else 0.0
        if bot is None:
            bot = _robot_patch(lx, ly, lth)
            ax.add_patch(bot)
            v[5] = bot
        else:
            bot.set_xy(_robot_pts(lx, ly, lth))
    ax.set_title(title or "eLlama live path", color=DARK_INK)
    fig.canvas.draw_idle()
    plt.pause(0.001)


def poll_loop(args):
    url = args.daemon_url.rstrip("/") + "/state"
    pose = PoseTracker()
    grid = OccupancyGrid()
    trail = []
    keys = StopKeyframes(args.frames_dir, args.camera_url,
                         getattr(args, "camera_facing", "arrow"))
    log_f = open(args.log, "w", newline="") if args.log else None
    log_w = None
    if log_f:
        log_w = csv.writer(log_f)
        log_w.writerow(["time_s", "source", "x_in", "y_in", "theta_deg",
                        "dist_avg_in", "yaw_deg", "note"])
    if args.once:
        try:
            state = fetch_state(url)
        except Exception as e:  # noqa: BLE001 - single attempt
            print(f"daemon not reachable: {e}", file=sys.stderr)
            if log_f:
                log_f.close()
            return 1
        tel = state.get("telemetry") or {}
        print(f"connected={state.get('connected')} "
              f"telemetry_live={tel.get('telemetry_live')} "
              f"imu_live={tel.get('imu_live')} "
              f"dist_avg_in={tel.get('dist_avg_in')} yaw_deg={tel.get('yaw_deg')} "
              f"source={source_label(state)}")
        if log_f:
            log_f.close()
        return 0
    t0 = time.monotonic()
    n = 0
    print(f"polling {url} at ~{args.poll_hz} Hz (read-only, no lease)")
    view = live_window() if args.show else None
    status = {"t": 0, "src": "-", "x": 0, "y": 0, "theta": 0,
              "rays": 0, "hits": 0, "shots": 0}
    if args.serve:
        import threading

        def _view(kind):
            if kind == "png":
                return render_png_bytes(trail, grid)
            return json.dumps(status).encode()

        threading.Thread(target=serve_forever, args=(args.serve, _view),
                         daemon=True).start()
    try:
        while True:
            try:
                state = fetch_state(url)
            except Exception as e:  # noqa: BLE001 - link down, retry
                print(f"daemon not reachable: {e}", file=sys.stderr)
                time.sleep(1.0 / args.poll_hz)
                continue
            tel = state.get("telemetry") or {}
            if state.get("operator_stop", {}).get("latched"):
                print("operator stop latched -- holding pose", file=sys.stderr)
                time.sleep(1.0 / args.poll_hz)
                continue
            t = time.monotonic() - t0
            enc_live = bool(tel.get("telemetry_live"))
            imu_live = bool(tel.get("imu_live"))
            dist = tel.get("dist_avg_in")
            yaw = tel.get("yaw_deg")
            if dist is None or yaw is None:
                time.sleep(1.0 / args.poll_hz)
                continue
            pose.update(dist, yaw, t, enc_live, imu_live)
            src = source_label(state)
            trail.append((pose.x, pose.y, pose.theta))
            moving = abs(pose.speed) > MOVING_SPEED_IN_S
            if not args.no_tof:
                rng = tel.get("range") or {}
                for label, ch in ((rng.get("channels") or {}).items()):
                    if not ch.get("live"):
                        continue
                    origin, reverse = ray_geometry(label, ch.get("sensor", ""))
                    grid.cast(pose.x, pose.y, pose.theta, ch.get("mm"), origin,
                              reverse, RAY_SPLAY_DEG.get(label, DEFAULT_SPLAY_DEG))
            shot = keys.update(moving, t, (pose.x, pose.y, pose.theta))
            if log_w:
                log_w.writerow([f"{t:.2f}", src, f"{pose.x:.2f}", f"{pose.y:.2f}",
                                f"{math.degrees(pose.theta):.1f}",
                                f"{dist:.3f}", f"{yaw:.1f}", shot or ""])
                if shot:
                    log_f.flush()
            n += 1
            if n == 1 or n % max(int(args.poll_hz * 2), 1) == 0:
                print(f"t={t:6.1f}s src={src:9s} x={pose.x:+7.1f} y={pose.y:+7.1f} "
                      f"th={math.degrees(pose.theta):+6.1f} "
                      f"rays={grid.rays} hits={grid.hits} shots={keys.saved}")
                status.update(t=round(t, 1), src=src, x=round(pose.x, 1),
                              y=round(pose.y, 1),
                              theta=round(math.degrees(pose.theta), 1),
                              rays=grid.rays, hits=grid.hits, shots=keys.saved)
            if args.save and (n == 1 or n % max(int(args.poll_hz * 10), 10) == 0):
                save_map_png(args.save, trail, grid)
            if view is not None and (n == 1 or n % max(int(args.poll_hz * 2), 1) == 0):
                live_draw(view, trail, grid)
            time.sleep(1.0 / args.poll_hz)
    except KeyboardInterrupt:
        pass
    finally:
        if args.save and trail:
            save_map_png(args.save, trail, grid)
            print(f"saved {args.save}")
        if log_f:
            log_f.close()
    return 0


def replay_csv(args):
    pose = PoseTracker()
    grid = OccupancyGrid()
    trail = []
    photos = []
    with open(args.replay, newline="") as f:
        rows = list(csv.DictReader(f))
    enc_rows = [r for r in rows if r.get("source") == "ENC"]
    if not enc_rows:
        raise SystemExit(f"{args.replay}: no ENC rows")
    for r in rows:
        if r.get("source") in ("MARK", "PHOTO"):
            photos.append((r.get("time_s"), r.get("note")))
    track_t, track_x, track_y, track_th = [], [], [], []
    for r in enc_rows:
        try:
            t = float(r["time_s"])
            dist = float(r["dist_avg_in"])
            yaw = float(r["yaw_deg"])
        except ValueError:
            continue
        enc_live = r.get("dist_avg_in", "") not in ("", "nan")
        imu_live = r.get("yaw_deg", "") not in ("", "nan")
        pose.update(dist, yaw, t, enc_live, imu_live)
        trail.append((pose.x, pose.y, pose.theta))
        track_t.append(t)
        track_x.append(pose.x)
        track_y.append(pose.y)
        track_th.append(pose.theta)
    # Walls pass: TOF_* rows carry per-channel mm in "note" ("97 112" = ch0 ch1,
    # -1 = that channel couldn't measure). Nearest ENC pose by timestamp.
    tof_rows = [r for r in rows
                if (r.get("source") or "").startswith("TOF_")]
    if tof_rows and not getattr(args, "no_tof", False) and track_t:
        import bisect
        for r in tof_rows:
            try:
                t = float(r["time_s"])
            except ValueError:
                continue
            vals = (r.get("note") or "").split()
            i = min(bisect.bisect_left(track_t, t), len(track_t) - 1)
            px, py, th = track_x[i], track_y[i], track_th[i]
            for ch, v in enumerate(vals):
                try:
                    mm = int(v)
                except ValueError:
                    continue
                label = bridge.TofState.label(r["source"], ch)
                origin, reverse = ray_geometry(label, r["source"])
                grid.cast(px, py, th, None if mm < 0 else mm, origin, reverse,
                          RAY_SPLAY_DEG.get(label, DEFAULT_SPLAY_DEG))
    xs = [p[0] for p in trail]
    ys = [p[1] for p in trail]
    print(f"{len(enc_rows)} ENC rows -> trail {math.hypot(pose.x, pose.y):.1f} in "
          f"from start, bbox {np.ptp(xs):.0f} x {np.ptp(ys):.0f} in, "
          f"{len(tof_rows)} ToF rows -> {grid.rays} rays, {grid.hits} hits")
    if getattr(args, "log", ""):
        Path(args.log).parent.mkdir(parents=True, exist_ok=True)
        with open(args.log, "w", newline="") as lf:
            w = csv.writer(lf)
            w.writerow(["time_s", "source", "x_in", "y_in", "theta_deg",
                        "dist_avg_in", "yaw_deg", "note"])
            # Re-walk ENC rows to emit per-sample pose (cheap, same integration).
            p2 = PoseTracker()
            for r in enc_rows:
                try:
                    t = float(r["time_s"])
                    dist = float(r["dist_avg_in"])
                    yaw = float(r["yaw_deg"])
                except ValueError:
                    continue
                p2.update(dist, yaw, t,
                          r.get("dist_avg_in", "") not in ("", "nan"),
                          r.get("yaw_deg", "") not in ("", "nan"))
                w.writerow([f"{t:.3f}", "replay", f"{p2.x:.2f}", f"{p2.y:.2f}",
                            f"{math.degrees(p2.theta):.1f}",
                            f"{dist:.3f}", f"{yaw:.1f}", ""])
        print(f"wrote {args.log}")
    if photos:
        print(f"{len(photos)} MARK/PHOTO rows, e.g. {photos[0]}")
    if args.save:
        save_map_png(args.save, trail, grid, title="eLlama replay path + ToF walls")
        print(f"saved {args.save}")
    if getattr(args, "show", False):
        v = live_window()
        if v is not None:
            live_draw(v, trail, grid, title="eLlama replay path + ToF walls")
            import matplotlib.pyplot as plt
            plt.ioff()
            plt.show(block=True)
    if getattr(args, "serve", 0):
        xs = [p[0] for p in trail]
        ys = [p[1] for p in trail]
        status = {"t": round(track_t[-1], 1) if track_t else 0, "src": "replay",
                  "x": round(xs[-1], 1) if xs else 0,
                  "y": round(ys[-1], 1) if ys else 0,
                  "theta": round(math.degrees(track_th[-1]), 1) if track_th else 0,
                  "rays": grid.rays, "hits": grid.hits, "shots": 0}

        def _view(kind):
            if kind == "png":
                return render_png_bytes(
                    trail, grid, title="eLlama replay path + ToF walls")
            return json.dumps(status).encode()

        serve_forever(args.serve, _view)  # blocks until Ctrl-C
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--daemon-url", default=os.environ.get("ELLAMA_DAEMON_URL",
                                                            "http://127.0.0.1:8765"))
    ap.add_argument("--poll-hz", type=float, default=5.0)
    ap.add_argument("--save", default="map.png")
    ap.add_argument("--log", default="map_log.csv")
    ap.add_argument("--frames-dir", default="frames")
    ap.add_argument("--camera-url", default=os.environ.get("ELLAMA_CAMERA_URL", ""))
    ap.add_argument("--camera-facing", default=os.environ.get("ELLAMA_CAMERA_FACING", "arrow"),
                    help="which end the keyframe camera is mounted on: arrow or tail")
    ap.add_argument("--no-tof", action="store_true")
    ap.add_argument("--replay", default="")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--show", action="store_true",
                    help="live map window while polling (needs a display)")
    ap.add_argument("--serve", type=int, default=0, metavar="PORT",
                    help="serve an auto-refreshing map page for Chrome, e.g. --serve 8000")
    args = ap.parse_args(argv)
    if args.replay:
        return replay_csv(args)
    return poll_loop(args)


if __name__ == "__main__":
    raise SystemExit(main())
