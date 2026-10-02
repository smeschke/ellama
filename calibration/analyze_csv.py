#!/usr/bin/env python3
"""
Analyze a CSV recorded by live_view.py --csv. Prints a summary of the run and
plots distance, speed, orientation and accel over the whole recording.
No robot or serial port needed.

Zeroing (z) in live_view shows up as distance/heading jumping back to 0; the
summary treats each zero as the start of a new segment.

The robot's path is dead-reckoned: distance from the encoders (average of both
wheels), heading from the IMU yaw (gyro) by default, or from the encoders with
--heading enc. Encoder heading depends on track_width_in and wheel slip, so the
summary also reports the track width that would make the two agree.

Usage:
    python3 calibration/analyze_csv.py calibration/ushape.csv
    python3 calibration/analyze_csv.py run.csv --save run.png       # PNGs, no window
    python3 calibration/analyze_csv.py run.csv --replay             # 8x animation window
    python3 calibration/analyze_csv.py run.csv --replay-save run.mp4   # or .gif

Requires: matplotlib, numpy (ffmpeg for .mp4, otherwise use .gif)
"""

import argparse
import csv
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

INK_PRIMARY = "#0b0b0b"
GRIDLINE = "#e1e0d9"
SURFACE = "#fcfcfb"
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]

MOVING_SPEED_IN_S = 0.5   # above this average wheel speed counts as moving


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    cols = {k: [float(r[k]) if k not in ("source", "note") else r[k] for r in rows] for k in rows[0]}
    return cols


def segments(cols, enc_idx):
    """Split encoder sample indices into runs between zeros (distance and
    heading both snap back to ~0 after being non-zero)."""
    dist = cols["dist_avg_in"]
    segs, start = [], enc_idx[0]
    for a, b in zip(enc_idx, enc_idx[1:]):
        if abs(dist[a]) > 1.0 and abs(dist[b]) < 0.05:
            segs.append((start, a))
            start = b
    segs.append((start, enc_idx[-1]))
    return [(enc_idx.index(s), enc_idx.index(e)) for s, e in segs]


def summarize(cols, enc_idx):
    t = cols["time_s"]
    src = cols["source"]
    print(f"{len(t)} rows over {t[-1] - t[0]:.1f} s "
          f"({len(enc_idx)} encoder, {src.count('IMU')} imu samples)")
    # MARK / PHOTO rows come from the MCP server's recorder (see mcp_server/recording.py).
    notes = [i for i, s_ in enumerate(src) if s_ in ("MARK", "PHOTO")]
    if notes:
        print("\nmarkers:")
        for i in notes:
            print(f"  t = {t[i]:7.1f} s  {src[i]:5}  dist {cols['dist_avg_in'][i]:+8.2f} in  "
                  f"{cols['note'][i]}")
    for n, (si, ei) in enumerate(segments(cols, enc_idx), 1):
        idx = enc_idx[si:ei + 1]
        moving = [i for i in idx
                  if (abs(cols["speed_l_in_s"][i]) + abs(cols["speed_r_in_s"][i])) / 2 > MOVING_SPEED_IN_S]
        i0, i1 = idx[0], idx[-1]
        print(f"\nsegment {n}: t = {t[i0]:.1f} to {t[i1]:.1f} s")
        print(f"  final distance  L {cols['dist_l_in'][i1]:+.2f}  R {cols['dist_r_in'][i1]:+.2f}  "
              f"avg {cols['dist_avg_in'][i1]:+.2f} in ({cols['dist_avg_in'][i1] / 12:+.3f} ft)")
        print(f"  final heading   enc {cols['enc_heading_deg'][i1]:+.1f} deg   imu yaw {cols['yaw_deg'][i1]:+.1f} deg")
        if moving:
            tm = t[moving[-1]] - t[moving[0]]
            speeds = [(abs(cols["speed_l_in_s"][i]) + abs(cols["speed_r_in_s"][i])) / 2 for i in moving]
            print(f"  moving          {t[moving[0]]:.1f} to {t[moving[-1]]:.1f} s ({tm:.1f} s), "
                  f"mean speed {sum(speeds) / len(speeds):.2f} in/s, peak {max(speeds):.2f} in/s")
        else:
            print("  no movement detected")
    pit, rol = cols["pitch_deg"], cols["roll_deg"]
    print(f"\npitch {min(pit):+.1f}..{max(pit):+.1f} deg   roll {min(rol):+.1f}..{max(rol):+.1f} deg")


def plot(cols, enc_idx, imu_idx):
    def pick(idx, *names):
        return [[cols[n][i] for i in idx] for n in names]

    te, = pick(enc_idx, "time_s")
    ti, = pick(imu_idx, "time_s")
    dl, dr, sl, sr, hdg = pick(enc_idx, "dist_l_in", "dist_r_in", "speed_l_in_s",
                               "speed_r_in_s", "enc_heading_deg")
    pit, rol, yaw, ax, ay, az = pick(imu_idx, "pitch_deg", "roll_deg", "yaw_deg",
                                     "acc_x_g", "acc_y_g", "acc_z_g")

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True, facecolor=SURFACE)
    ax_d, ax_s, ax_o, ax_a = axes
    ax_d.plot(te, dl, color=CAT[0], linewidth=2, label="left")
    ax_d.plot(te, dr, color=CAT[1], linewidth=2, label="right")
    ax_s.plot(te, sl, color=CAT[0], linewidth=1.5, label="left")
    ax_s.plot(te, sr, color=CAT[1], linewidth=1.5, label="right")
    ax_o.plot(ti, pit, color=CAT[0], linewidth=2, label="pitch")
    ax_o.plot(ti, rol, color=CAT[1], linewidth=2, label="roll")
    ax_o.plot(ti, yaw, color=CAT[2], linewidth=2, label="yaw (imu, drifts)")
    ax_o.plot(te, hdg, color=CAT[3], linewidth=2, linestyle="--", label="heading (encoders)")
    ax_a.plot(ti, ax, color=CAT[0], linewidth=1.5, label="x")
    ax_a.plot(ti, ay, color=CAT[1], linewidth=1.5, label="y")
    ax_a.plot(ti, az, color=CAT[2], linewidth=1.5, label="z")

    ax_d.set_ylabel("distance (in)")
    ax_s.set_ylabel("speed (in/s)")
    ax_o.set_ylabel("degrees")
    ax_a.set_ylabel("accel (g)")
    ax_a.set_xlabel("time (s)")
    for a in axes:
        a.legend(frameon=False, loc="upper left")
        a.set_facecolor(SURFACE)
        a.grid(color=GRIDLINE, linewidth=1)
        a.tick_params(colors=INK_PRIMARY)
    fig.tight_layout()
    return fig


def dead_reckon(cols, enc_idx, imu_idx, heading="imu"):
    """Integrate the path (x forward at start, y left, inches; heading CCW).

    Returns dict of arrays over the encoder samples: t, x, y, theta (rad),
    speed (in/s), plus the raw encoder/imu headings (deg) for comparison.
    Zeroing in live_view resets distance and heading; each reset starts a new
    segment that continues from where the last one ended, in its current heading.
    """
    t = np.array([cols["time_s"][i] for i in enc_idx])
    dist = np.array([cols["dist_avg_in"][i] for i in enc_idx])
    enc_h = np.array([cols["enc_heading_deg"][i] for i in enc_idx])
    ti = np.array([cols["time_s"][i] for i in imu_idx])
    yi = np.array([cols["yaw_deg"][i] for i in imu_idx])
    # rows in the same batch share a timestamp; interp needs increasing x
    if len(ti):
        ti, u = np.unique(ti, return_index=True)
        imu_h = np.interp(t, ti, yi[u])
    else:
        imu_h = enc_h.copy()
    if heading == "imu" and not len(imu_idx):
        print("no IMU rows; using encoder heading")
        heading = "enc"
    raw_h = imu_h if heading == "imu" else enc_h

    # continuous distance/heading across zeroings
    dd = np.diff(dist, prepend=dist[0])
    dh = np.diff(raw_h, prepend=raw_h[0])
    reset = np.zeros(len(t), bool)
    reset[1:] = (np.abs(dist[:-1]) > 1.0) & (np.abs(dist[1:]) < 0.05)
    dd[reset] = 0.0
    dh[reset] = 0.0
    ds = np.cumsum(dd)
    theta = np.deg2rad(np.cumsum(dh))
    # midpoint heading per step
    th_mid = theta - np.diff(theta, prepend=theta[0]) / 2
    step = np.diff(ds, prepend=ds[0])
    x = np.cumsum(step * np.cos(th_mid))
    y = np.cumsum(step * np.sin(th_mid))
    return dict(t=t, x=x, y=y, theta=theta, ds=ds, enc_h=enc_h, imu_h=imu_h,
                heading=heading)


def path_summary(cols, enc_idx, path):
    x, y, ds = path["x"], path["y"], path["ds"]
    print(f"\npath ({path['heading']} heading)")
    print(f"  length driven   {abs(np.diff(ds)).sum():.1f} in ({abs(np.diff(ds)).sum() / 12:.1f} ft)")
    print(f"  start -> end    {math.hypot(x[-1], y[-1]):.1f} in apart   "
          f"bounding box {np.ptp(x):.0f} x {np.ptp(y):.0f} in")
    net_turn = np.rad2deg(path["theta"][-1] - path["theta"][0])
    print(f"  net turn        {net_turn:+.1f} deg (left positive)")
    # track width that would make encoder heading match the IMU
    dl, dr = np.array(cols["dist_l_in"])[enc_idx], np.array(cols["dist_r_in"])[enc_idx]
    imu_h = np.deg2rad(path["imu_h"])
    if len(imu_h) > 1 and np.ptp(imu_h) > math.radians(30):
        # heading = (dr - dl) / track  ->  track = total wheel difference / total yaw
        dw = np.diff(dr - dl)
        dy = np.diff(imu_h)
        ok = np.abs(np.diff((dl + dr) / 2)) < 5   # skip zero resets
        ok &= np.abs(dw) < 20
        track = dw[ok].sum() / dy[ok].sum()
        print(f"  track width     {track:.1f} in would make encoder heading match the IMU "
              f"(config uses the value in config.json)")
    drift = path["enc_h"][-1] - path["imu_h"][-1]
    print(f"  heading gap     enc - imu = {drift:+.1f} deg at the end "
          f"({100 * drift / path['imu_h'][-1]:+.0f}% of the IMU turn)"
          if abs(path["imu_h"][-1]) > 1 else "")


def plot_path(cols, path):
    x, y, t = path["x"], path["y"], path["t"]
    speed = np.hypot(np.gradient(x, t + np.arange(len(t)) * 1e-9),
                     np.gradient(y, t + np.arange(len(t)) * 1e-9))
    fig, (ax, ax_h) = plt.subplots(1, 2, figsize=(13, 6), facecolor=SURFACE,
                                   gridspec_kw={"width_ratios": [1.6, 1]})
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap="viridis",
                        linewidth=3)
    lc.set_array(t[1:])
    ax.add_collection(lc)
    ax.autoscale()
    ax.plot(x[0], y[0], "o", color=CAT[2], markersize=10, label="start")
    ax.plot(x[-1], y[-1], "s", color=CAT[1], markersize=10, label="end")
    # heading ticks every ~10 s
    for k in range(0, len(t), max(1, int(len(t) / 10))):
        ax.arrow(x[k], y[k], 6 * math.cos(path["theta"][k]), 6 * math.sin(path["theta"][k]),
                 head_width=2, length_includes_head=True, color=INK_PRIMARY, alpha=0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("x (in)  [forward at start]")
    ax.set_ylabel("y (in)  [left at start]")
    ax.set_title(f"top-down path ({path['heading']} heading)")
    fig.colorbar(lc, ax=ax, label="time (s)", shrink=0.8)
    ax.legend(frameon=False)
    ax.grid(color=GRIDLINE)
    ax.set_facecolor(SURFACE)

    ax_h.plot(t, path["enc_h"], color=CAT[3], linewidth=2, linestyle="--", label="encoders")
    ax_h.plot(t, path["imu_h"], color=CAT[2], linewidth=2, label="IMU yaw")
    ax_h.set_xlabel("time (s)")
    ax_h.set_ylabel("heading (deg)")
    ax_h.set_title("heading: encoders vs IMU")
    ax_h.legend(frameon=False)
    ax_h.grid(color=GRIDLINE)
    ax_h.set_facecolor(SURFACE)
    fig.tight_layout()
    return fig


def replay(cols, path, speedup=8.0, fps=30, trail_s=None):
    """Top-down animation of the run at `speedup`x real time."""
    x, y, t, th = path["x"], path["y"], path["t"], path["theta"]
    tu, u = np.unique(t, return_index=True)          # batched timestamps
    xu, yu, thu = x[u], y[u], np.unwrap(th[u])
    n_frames = int((tu[-1] - tu[0]) * fps / speedup) + 1
    ft = tu[0] + np.arange(n_frames) * speedup / fps
    fx, fy = np.interp(ft, tu, xu), np.interp(ft, tu, yu)
    fth = np.interp(ft, tu, thu)
    v = np.hypot(np.gradient(fx), np.gradient(fy)) * fps / speedup

    fig, ax = plt.subplots(figsize=(8, 7), facecolor=SURFACE)
    ax.plot(x, y, color=GRIDLINE, linewidth=6, zorder=1)           # full route, faint
    ax.set_aspect("equal")
    pad = 20
    ax.set_xlim(x.min() - pad, x.max() + pad)
    ax.set_ylim(y.min() - pad, y.max() + pad)
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRIDLINE)
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")
    ax.plot(x[0], y[0], "o", color=CAT[2], markersize=9, zorder=2)
    trail, = ax.plot([], [], color=CAT[0], linewidth=3, zorder=3)
    dot, = ax.plot([], [], "o", color=CAT[1], markersize=13, zorder=5)
    nose = ax.annotate("", xy=(0, 0), xytext=(0, 0), zorder=6,
                       arrowprops=dict(arrowstyle="-|>", color=INK_PRIMARY, lw=2))
    label = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", color=INK_PRIMARY,
                    family="monospace")
    ax.set_title(f"replay at {speedup:g}x")

    def draw(k):
        lo = 0 if trail_s is None else max(0, k - int(trail_s * fps / speedup))
        trail.set_data(fx[lo:k + 1], fy[lo:k + 1])
        dot.set_data([fx[k]], [fy[k]])
        nose.xy = (fx[k] + 10 * math.cos(fth[k]), fy[k] + 10 * math.sin(fth[k]))
        nose.set_position((fx[k], fy[k]))
        label.set_text(f"t = {ft[k] - tu[0]:5.1f} s\nspeed {v[k]:5.1f} in/s\n"
                       f"heading {math.degrees(fth[k]):+6.1f} deg")
        return trail, dot, nose, label

    anim = FuncAnimation(fig, draw, frames=n_frames, interval=1000 / fps, blit=False)
    return fig, anim, fps


def main():
    parser = argparse.ArgumentParser(description="Summarize and plot a live_view.py CSV recording.")
    parser.add_argument("csv", help="CSV written by live_view.py --csv")
    parser.add_argument("--save", metavar="PNG", help="Save the plots (PNG, plus <name>_path.png) instead of opening a window")
    parser.add_argument("--heading", choices=["imu", "enc"], default="imu",
                        help="heading source for the path (default imu; encoder heading depends on track width)")
    parser.add_argument("--replay", action="store_true", help="show the path replay animation")
    parser.add_argument("--replay-save", metavar="FILE", help="write the replay to .mp4 (needs ffmpeg) or .gif")
    parser.add_argument("--speed", type=float, default=8.0, help="replay speed-up (default 8)")
    args = parser.parse_args()

    cols = load(args.csv)
    enc_idx = [i for i, s in enumerate(cols["source"]) if s == "ENC"]
    imu_idx = [i for i, s in enumerate(cols["source"]) if s == "IMU"]
    if not enc_idx:
        raise SystemExit("no ENC rows in this file")

    summarize(cols, enc_idx)
    path = dead_reckon(cols, enc_idx, imu_idx, args.heading)
    path_summary(cols, enc_idx, path)
    fig = plot(cols, enc_idx, imu_idx)
    fig_path = plot_path(cols, path)
    if args.save:
        base = args.save.rsplit(".", 1)[0]
        fig.savefig(args.save, dpi=120, facecolor=fig.get_facecolor())
        fig_path.savefig(base + "_path.png", dpi=120, facecolor=fig_path.get_facecolor())
        print(f"\nsaved {args.save} and {base}_path.png")
    if args.replay or args.replay_save:
        fig_r, anim, fps = replay(cols, path, args.speed)
        if args.replay_save:
            anim.save(args.replay_save, fps=fps, dpi=100)
            print(f"saved {args.replay_save}")
    if not args.save and not args.replay_save:
        plt.show()


if __name__ == "__main__":
    main()
