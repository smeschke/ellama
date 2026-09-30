#!/usr/bin/env python3
"""
Like analyze_csv.py, but corrects old recordings for the gear ratio and can
export the dead-reckoned trajectory for simulation/replay3d.html.

live_view.py bakes distances into the CSV using the gear_ratio in config.json
at record time (6.33 until the 4.5 fix). Distance is proportional to
1/gear_ratio, so this script rescales the encoder columns from the ratio the
file was recorded with to the true one. config.json is not read or changed.

Usage:
    python3 calibration/analyze_run.py calibration/30_inches.csv
    python3 calibration/analyze_run.py calibration/ushape.csv --export-json run.json
    python3 calibration/analyze_run.py run.csv --replay-save run.mp4

Files recorded after config.json is changed to 4.5: pass --recorded-gear-ratio 4.5.

Requires: matplotlib, numpy
"""

import argparse
import json

import matplotlib.pyplot as plt
import numpy as np

import analyze_csv as ac

TRUE_GEAR_RATIO = 4.5       # ~90t wheel gear / 20t encoder gear, confirmed by the 30 in run
OLD_GEAR_RATIO = 6.33       # what config.json / bridge.py used when the first CSVs were recorded


def rescale(cols, recorded, true):
    k = recorded / true
    for name in SCALED:
        cols[name] = [v * k for v in cols[name]]
    print(f"gear ratio {recorded} -> {true}: encoder distances x{k:.3f}")


SCALED = ("dist_l_in", "dist_r_in", "dist_avg_in", "speed_l_in_s", "speed_r_in_s", "enc_heading_deg")


def export_json(cols, imu_idx, path, out, hz=30):
    """Uniform-rate trajectory in feet for replay3d.html (x forward at start, y left)."""
    t, x, y, th = path["t"], path["x"], path["y"], np.unwrap(path["theta"])
    tu, u = np.unique(t, return_index=True)          # batched timestamps
    tt = np.arange(tu[0], tu[-1], 1.0 / hz)
    ti = np.array([cols["time_s"][i] for i in imu_idx])
    ti, ui = np.unique(ti, return_index=True)
    pitch = np.interp(tt, ti, np.array([cols["pitch_deg"][i] for i in imu_idx])[ui])
    roll = np.interp(tt, ti, np.array([cols["roll_deg"][i] for i in imu_idx])[ui])
    xs, ys = np.interp(tt, tu, x[u]) / 12, np.interp(tt, tu, y[u]) / 12
    speed = np.hypot(np.gradient(xs), np.gradient(ys)) * hz          # ft/s
    k = max(1, int(hz * 0.5))                                        # ~0.5 s moving average
    speed = np.convolve(np.pad(speed, k // 2, mode="edge"), np.ones(k) / k, mode="valid")[:len(speed)]
    data = dict(
        hz=hz,
        heading_source=path["heading"],
        t=np.round(tt - tt[0], 3).tolist(),
        x_ft=np.round(xs, 4).tolist(),
        y_ft=np.round(ys, 4).tolist(),
        theta=np.round(np.interp(tt, tu, th[u]), 5).tolist(),
        pitch_deg=np.round(pitch, 2).tolist(),
        roll_deg=np.round(roll, 2).tolist(),
        speed_ft_s=np.round(speed, 3).tolist(),
    )
    with open(out, "w") as f:
        json.dump(data, f, separators=(",", ":"))
    print(f"saved {out} ({len(tt)} frames at {hz} Hz, {tt[-1] - tt[0]:.1f} s)")


def main():
    p = argparse.ArgumentParser(description="Gear-ratio-corrected summary, plots, replay and 3D export.")
    p.add_argument("csv")
    p.add_argument("--recorded-gear-ratio", type=float, default=OLD_GEAR_RATIO,
                   help=f"gear_ratio in effect when recorded (default {OLD_GEAR_RATIO})")
    p.add_argument("--gear-ratio", type=float, default=TRUE_GEAR_RATIO,
                   help=f"true gear ratio (default {TRUE_GEAR_RATIO})")
    p.add_argument("--heading", choices=["imu", "enc"], default="imu")
    p.add_argument("--save", metavar="PNG", help="save time-series PNG and <name>_path.png, no window")
    p.add_argument("--export-json", metavar="FILE", help="write trajectory for simulation/replay3d.html")
    p.add_argument("--replay", action="store_true")
    p.add_argument("--replay-save", metavar="FILE")
    p.add_argument("--speed", type=float, default=8.0)
    args = p.parse_args()

    cols = ac.load(args.csv)
    rescale(cols, args.recorded_gear_ratio, args.gear_ratio)
    enc_idx = [i for i, s in enumerate(cols["source"]) if s == "ENC"]
    imu_idx = [i for i, s in enumerate(cols["source"]) if s == "IMU"]
    if not enc_idx:
        raise SystemExit("no ENC rows in this file")

    ac.summarize(cols, enc_idx)
    path = ac.dead_reckon(cols, enc_idx, imu_idx, args.heading)
    ac.path_summary(cols, enc_idx, path)
    fig = ac.plot(cols, enc_idx, imu_idx)
    fig_path = ac.plot_path(cols, path)

    if args.export_json:
        export_json(cols, imu_idx, path, args.export_json)
    if args.save:
        base = args.save.rsplit(".", 1)[0]
        fig.savefig(args.save, dpi=120, facecolor=fig.get_facecolor())
        fig_path.savefig(base + "_path.png", dpi=120, facecolor=fig_path.get_facecolor())
        print(f"saved {args.save} and {base}_path.png")
    if args.replay or args.replay_save:
        _, anim, fps = ac.replay(cols, path, args.speed)
        if args.replay_save:
            anim.save(args.replay_save, fps=fps, dpi=100)
            print(f"saved {args.replay_save}")
    if not (args.save or args.replay_save or args.export_json):
        plt.show()


if __name__ == "__main__":
    main()
