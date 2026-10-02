#!/usr/bin/env python3
"""Closed-loop simulation of line_follow.LineFollower on a toy robot, to tune the controller
before it drives anything.

    python3 mcp_server/line_follow_sim.py            # a table of courses x plant variations
    python3 mcp_server/line_follow_sim.py --plot     # also save sim_<course>.png trajectories

The robot model is built from the manual runs (tape_manual_run1/2): forward speed ~0.095 in/s
per unit of effort above a dead zone, yaw rate with a dead zone then ~0.5 deg/s per unit,
first-order lags on both, and a camera that is `delay` seconds behind. It is a sanity check
of stability and gains, NOT a prediction: skid-steer scrub, battery, floor and the real camera
geometry are all unmodeled. The first bench test on blocks is the real check.
"""

import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import line_follow as lf  # noqa: E402
import line_vision  # noqa: E402

FRAME_W_IN, FRAME_H_IN = 12.0, 14.0   # ground area the camera sees
PX_PER_IN = 30.0
CAM_FWD_IN = 6.0                      # camera ahead of the axle (robot pose is the axle)
BAND_DIST_IN = [1.5 + 1.5 * k for k in range(8)]
FPS = 16.0
DT = 0.01


# ---- paths --------------------------------------------------------------------------------
def build_path(segments, step=0.25):
    """segments: ("s", length_in) | ("l"/"r", radius_in, degrees). Returns [(x, y)] starting at
    the origin heading +x, plus the final heading."""
    x = y = th = 0.0
    pts = [(x, y)]
    for seg in segments:
        if seg[0] == "s":
            n = max(1, int(seg[1] / step))
            for _ in range(n):
                x += step * math.cos(th)
                y += step * math.sin(th)
                pts.append((x, y))
        else:
            _, r, deg = seg
            sgn = 1 if seg[0] == "l" else -1
            n = max(1, int(r * math.radians(deg) / step))
            dth = sgn * math.radians(deg) / n
            for _ in range(n):
                th += dth / 2
                x += step * math.cos(th)
                y += step * math.sin(th)
                th += dth / 2
                pts.append((x, y))
    return pts


COURSES = {
    "straight": [("s", 80)],
    "like_the_tape": [("s", 20), ("l", 16, 40), ("s", 14), ("r", 16, 55), ("s", 12), ("l", 24, 30), ("s", 22)],
    "sharp_90": [("s", 18), ("r", 6, 90), ("s", 18)],
    "s_curve": [("s", 12), ("l", 10, 70), ("r", 10, 70), ("s", 20)],
}


# ---- the robot ----------------------------------------------------------------------------
class Plant:
    """Yaw has two regimes (see line_follow.py): linear and weak while driving, deadband then
    steeper while spinning on the spot. Blended on the (lagged) forward effort."""

    def __init__(self, g=0.5, t_db=28.0, g_drive=0.2, left_scale=0.85, tau_f=0.2, tau_t=0.15,
                 kv=0.095, f_db=45.0):
        self.g, self.t_db, self.g_drive, self.left_scale = g, t_db, g_drive, left_scale
        self.tau_f, self.tau_t, self.kv, self.f_db = tau_f, tau_t, kv, f_db
        self.f = self.t = 0.0

    def step(self, f_cmd, t_cmd):
        self.f += (f_cmd - self.f) * DT / self.tau_f
        self.t += (t_cmd - self.t) * DT / self.tau_t
        driving = self.f > self.f_db
        v = self.kv * self.f if driving else 0.0
        mag = abs(self.t) * self.g_drive if driving else self.g * max(0.0, abs(self.t) - self.t_db)
        w = mag * (self.left_scale if self.t < 0 else 1.0)   # deg/s, + right
        return v, math.copysign(w, self.t)


def measure_path(pose, path, hint):
    """What the camera would see from `pose` = (x, y, theta): band dicts like
    line_vision.analyze() makes, and whether the end-of-course strip is in view."""
    x, y, th = pose
    fx, fy = math.cos(th), math.sin(th)
    rx, ry = math.sin(th), -math.cos(th)
    cx, cy = x + CAM_FWD_IN * fx, y + CAM_FWD_IN * fy
    # start the search at the path point nearest the robot, then walk forward
    i0 = min(range(max(0, hint - 40), min(len(path), hint + 80)),
             key=lambda i: (path[i][0] - cx) ** 2 + (path[i][1] - cy) ** 2)
    bands = []
    j = i0
    for d in BAND_DIST_IN:
        while j < len(path) - 1 and (path[j][0] - cx) * fx + (path[j][1] - cy) * fy < d:
            j += 1
        px, py = path[j]
        fwd = (px - cx) * fx + (py - cy) * fy
        if fwd < d - 0.5:           # ran off the end of the tape
            break
        lat = (px - cx) * rx + (py - cy) * ry
        if abs(lat) > FRAME_W_IN / 2 - 0.4:
            continue
        bands.append(dict(band=len(bands), y=int((FRAME_H_IN - d) * PX_PER_IN), x_px=0.0,
                          width_px=int(line_vision.TAPE_WIDTH_IN * PX_PER_IN), lateral_in=lat))
    # the strip lies across the path's end; it is in view once the end is within the frame
    ex, ey = path[-1]
    d_end = (ex - cx) * fx + (ey - cy) * fy
    lat_end = (ex - cx) * rx + (ey - cy) * ry
    cross = 0.0 < d_end <= FRAME_H_IN and abs(lat_end) < FRAME_W_IN / 2
    y_frac = None if not cross else 1.0 - d_end / FRAME_H_IN
    res = dict(found=len(bands) >= 2, n_bands=len(bands), bands=bands, cross_strip=cross, cross_y_frac=y_frac)
    return res, i0, math.hypot(ex - x, ey - y)


def run(course, plant, delay=0.25, noise=0.1, start_off=(0.0, 0.0), seed=0, f_cruise=None,
        max_s=120.0, record=False):
    rnd = random.Random(seed)
    path = build_path(COURSES[course])
    # start on the path origin, offset sideways by start_off[0] in, rotated start_off[1] deg
    th0 = math.radians(start_off[1])
    pose = [0.0, start_off[0], th0]
    ctl = lf.LineFollower(max_run_s=max_s, f_cruise=f_cruise)
    hist, hint = [], 0
    res0, hint, _ = measure_path(pose, path, hint)
    ok, why = ctl.can_start(res0)
    if not ok:
        return dict(status="refused: " + why, max_err=None, t=0.0)
    t = 0.0
    ctl.start(t)
    cmd_f = cmd_t = 0.0
    next_frame = 0.0
    max_err = 0.0
    trace = []
    while ctl.status == "running" and t < max_s + 1:
        hist.append((t, tuple(pose)))
        if t >= next_frame:
            next_frame += 1.0 / FPS
            tc = t - delay
            past = next((p for (tt, p) in reversed(hist) if tt <= tc), hist[0][1])
            res, hint, _ = measure_path(past, path, hint)
            for b in res["bands"]:
                b["lateral_in"] += rnd.gauss(0, noise)
            c = ctl.update(res, t)
            # raw -> real: raw = sign * real
            lr, rr = c.left_raw * ctl.sign, c.right_raw * ctl.sign
            cmd_f, cmd_t = (lr + rr) / 2.0, (lr - rr) / 2.0
        else:
            c = ctl.check(t)
            if c.status != "running":
                cmd_f = cmd_t = 0.0
        if ctl.status != "running":
            cmd_f = cmd_t = 0.0
        v, w = plant.step(cmd_f, cmd_t)
        pose[2] -= math.radians(w) * DT
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
        err = min(math.hypot(px - pose[0], py - pose[1]) for px, py in path[max(0, hint - 40):hint + 80])
        max_err = max(max_err, err)
        if record:
            trace.append((pose[0], pose[1]))
        t += DT
    # let it coast to a stop after a terminal state, for the distance-to-strip figure
    for _ in range(int(1.0 / DT)):
        v, w = plant.step(0.0, 0.0)
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
    ex, ey = path[-1]
    out = dict(status=ctl.status, max_err=max_err, t=t, end_dist=math.hypot(ex - pose[0], ey - pose[1]))
    if record:
        out["trace"], out["path"] = trace, path
    return out


VARIANTS = {
    "nominal": dict(plant=dict(), delay=0.25),
    "weak turn": dict(plant=dict(g=0.3, t_db=35.0, g_drive=0.13), delay=0.25),
    "strong turn": dict(plant=dict(g=0.7, t_db=22.0, g_drive=0.3), delay=0.25),
    "slow camera": dict(plant=dict(), delay=0.45),
    "laggy motors": dict(plant=dict(tau_f=0.35, tau_t=0.3), delay=0.25),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--cruise", type=int, default=None, help="override F_CRUISE")
    args = ap.parse_args()
    starts = [(0.0, 0.0), (2.5, 12.0), (-2.5, -12.0)]
    print(f"{'course':14s} {'variant':13s} " + "  ".join(f"start{i}" for i in range(len(starts))))
    bad = 0
    for course in COURSES:
        for vname, v in VARIANTS.items():
            cells = []
            for si, so in enumerate(starts):
                r = run(course, Plant(**v["plant"]), delay=v["delay"], start_off=so, seed=si,
                        f_cruise=args.cruise)
                good = r["status"] == "finished"
                bad += not good
                cells.append(f"{r['status'][:8]:8s} err{(r['max_err'] if r['max_err'] is not None else 0):4.1f}in {r['t']:4.0f}s")
            print(f"{course:14s} {vname:13s} " + " | ".join(cells))
    print(f"\nruns that did not finish: {bad} of {len(COURSES) * len(VARIANTS) * len(starts)}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for course in COURSES:
            r = run(course, Plant(), record=True, start_off=(2.5, 12.0), seed=1)
            px, py = zip(*r["path"])
            tx, ty = zip(*r["trace"])
            plt.figure(figsize=(7, 5))
            plt.plot(px, py, "k", lw=6, alpha=.25, label="tape")
            plt.plot(tx, ty, "r", lw=1.2, label="robot axle")
            plt.axis("equal")
            plt.title(f"{course}: {r['status']}, max err {r['max_err']:.1f}in")
            plt.legend()
            plt.savefig(f"sim_{course}.png", dpi=80)
            plt.close()


if __name__ == "__main__":
    main()
