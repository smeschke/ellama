"""Build the course map from the two post runs (stitched across the AI's nudges).

    python3 analysis/linefollow_post/course_map.py

Writes post_assets/course.json (poses + tape points) and post_assets/course_map.png.
"""

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import pipeline as pl  # noqa: E402

RUNS = ["follow_post_run1", "follow_post_run2"]
OUT = os.path.join(pl.REPO, "post_assets")


def build(track_in=pl.TRACK_IN):
    runs, start = [], (0.0, 0.0)
    for name in RUNS:
        run = pl.load_run(name)
        t, x, y, yaw = pl.dead_reckon(run, start, track_in)
        start = (x[-1], y[-1])  # the AI's nudges are in-place turns: position carries over
        det = pl.analyze_run(run)
        tape = []
        spin = np.nan_to_num(run["log"].spin.values.astype(float))
        for d in det:
            if spin[min(d["i"], len(spin) - 1)] > 0:
                continue  # spinning: the pose changes faster than the camera lag can be corrected
            tl = d["t"] - pl.CAM_LAG_S
            pose = (np.interp(tl, t, x), np.interp(tl, t, y), np.interp(tl, t, yaw))
            for px, py, band in pl.ground_points(d["res"].get("bands") or [], pose):
                tape.append((d["t"], px, py, band))
        runs.append(dict(name=name, t=t, x=x, y=y, yaw=yaw, det=det, tape=tape))
    return runs


def draw(runs, path, tape_bands=(0, 1, 2, 3)):
    fig, ax = plt.subplots(figsize=(8, 9), dpi=130)
    for k, r in enumerate(runs):
        tp = np.array([(p[1], p[2]) for p in r["tape"] if p[3] in tape_bands])
        if len(tp):
            ax.scatter(tp[:, 0], tp[:, 1], s=2, c="#222222", alpha=0.35, linewidths=0)
        ax.plot(r["x"], r["y"], lw=1.5, c=["#d33", "#27a"][k], label=f"{r['name']} path")
    ax.plot([0], [0], "g^", ms=10, label="start")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("inches")
    ax.set_ylabel("inches")
    ax.legend(loc="best", fontsize=8)
    ax.set_title("Dead-reckoned course map (left encoder + IMU yaw)")
    fig.savefig(path)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    runs = build()
    draw(runs, os.path.join(OUT, "course_map.png"))
    for r in runs:
        print(r["name"], "end pose", round(r["x"][-1], 1), round(r["y"][-1], 1),
              "yaw deg", round(float(np.degrees(r["yaw"][-1])), 1),
              "tape pts", len(r["tape"]))
