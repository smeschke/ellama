"""Shared pieces for the line-follow post: dead-reckoned pose, tape projection, per-frame
detection. Pure numpy/OpenCV/pandas, works offline on recordings/<run>/ folders.

Pose: heading is the IMU yaw (CCW positive, 0 = facing +y on the map). Position is dead
reckoned from the LEFT encoder only -- the right encoder is dead and these hall encoders
cannot tell direction, so distance is only integrated while the follower is driving
(follow_log f > 0), never while it spins. In a curve the left wheel is the inner/outer wheel,
so the centre moves ds = d_left + (track/2) * d_yaw.
"""

import os
import sys

import cv2
import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "mcp_server"))
import line_vision  # noqa: E402

REC = os.path.join(REPO, "recordings")
TRACK_IN = 20.0               # wheel track used for the curve correction (uncalibrated)
AXLE_TO_NEAR_BAND_IN = 7.5    # same guess line_follow.py uses
CAM_LAG_S = 0.25              # camera frame is this much older than the IMU sample at the same time


def load_run(name):
    d = os.path.join(REC, name)
    t = pd.read_csv(os.path.join(d, name + ".csv"))
    return dict(
        name=name, dir=d,
        imu=t[t.source == "IMU"].reset_index(drop=True),
        enc=t[t.source == "ENC"].reset_index(drop=True),
        log=pd.read_csv(os.path.join(d, "follow_log.csv")),
        vt=pd.read_csv(os.path.join(d, "video_times.csv")),
    )


def dead_reckon(run, start_xy=(0.0, 0.0), track_in=TRACK_IN):
    """Returns arrays t, x, y, yaw_rad (yaw unwrapped, continuous with the IMU)."""
    imu, enc, log = run["imu"], run["enc"], run["log"]
    t = enc.time_s.values
    yaw = np.interp(t, imu.time_s.values, np.unwrap(np.radians(imu.yaw_deg.values)))
    dl = enc.dist_l_in.values
    # driving flag at each encoder sample: the follower's last logged f at or before t
    idx = np.clip(np.searchsorted(log.time_s.values, t, side="right") - 1, 0, len(log) - 1)
    f = np.nan_to_num(log.f.values.astype(float))[idx]
    driving = f > 0
    ds = np.zeros_like(t)
    ds[1:] = np.diff(dl) + 0.5 * track_in * np.diff(yaw)
    ds = np.where(driving, np.clip(ds, 0, None), 0.0)
    x = np.empty_like(t)
    y = np.empty_like(t)
    x[0], y[0] = start_xy
    for i in range(1, len(t)):
        th = yaw[i]
        x[i] = x[i - 1] + ds[i] * -np.sin(th)
        y[i] = y[i - 1] + ds[i] * np.cos(th)
    return t, x, y, yaw


def ground_points(bands, lag_pose):
    """Tape points on the map for one frame's bands. `lag_pose` = (x, y, yaw_rad)."""
    if not bands:
        return []
    x, y, th = lag_pose
    h = np.array([-np.sin(th), np.cos(th)])
    r = np.array([np.cos(th), np.sin(th)])
    pts, s = [], 0.0
    prev = None
    for b in bands:
        ipp = line_vision.TAPE_WIDTH_IN / b["width_px"]
        if prev is not None:
            s += abs(prev["y"] - b["y"]) * 0.5 * (ipp + line_vision.TAPE_WIDTH_IN / prev["width_px"])
        prev = b
        p = np.array([x, y]) + (AXLE_TO_NEAR_BAND_IN + s) * h + b["lateral_in"] * r
        pts.append((float(p[0]), float(p[1]), b["band"]))
    return pts


def read_frames(run):
    """Yield (frame_index, time_s, bgr) for the run's video, with timestamps from video_times."""
    cap = cv2.VideoCapture(os.path.join(run["dir"], "video.mp4"))
    times = run["vt"].time_s.values
    i = 0
    while True:
        ok, bgr = cap.read()
        if not ok or i >= len(times):
            break
        yield i, float(times[i]), bgr
        i += 1
    cap.release()


def analyze_run(run):
    """Per-frame detection with the same chaining the follower uses (prev line x). Returns a
    list of dicts: i, t, res (analyze result dict)."""
    out, prev = [], None
    for i, t, bgr in read_frames(run):
        res, _ = line_vision.analyze(bgr, prev)
        prev = res.get("x_frac")
        out.append(dict(i=i, t=t, res=res))
    return out
