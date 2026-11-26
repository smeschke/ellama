#!/usr/bin/env python3
"""Route math for watch_and_repeat: recorded samples -> a route the robot can follow.

Pure functions, no I/O and no robot, so every step is unit-testable.

Pipeline (process() runs all of it):
  integrate   samples (full telemetry snapshots) -> dead-reckoned trail (x, y, theta)
  close_loop  the course was a closed loop, so the end must equal the start: spread the
              end-minus-start error over the trail by path length, and report its size
  simplify    Ramer-Douglas-Peucker down to waypoints; the start, the end and every
              keyframe are pinned, so a keyframe is always a corner of the route
  to_legs     waypoints -> [{turn_deg, drive_in}] for a turn-then-drive follower

Frame: the same as tools/realtime_map.py, because the trail is integrated by its
PoseTracker (x forward at start, y left, theta CCW = a left turn). The route and the
live map therefore always agree; if one is wrong, both are.

Each waypoint also carries the sensor snapshot recorded there (ToF channels, yaw,
distance, and anything else telemetry carried) so later followers can steer by the
camera or the range sensors instead of dead reckoning alone.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from realtime_map import PoseTracker  # noqa: E402

DEFAULT_TOL_IN = 3.0
DEFAULT_MIN_TURN_DEG = 8.0   # smaller turns are below what turn_degrees reliably makes
DEFAULT_MIN_DRIVE_IN = 2.0


def wrap_rad(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def integrate(samples):
    """samples: [{"t": s, "tel": telemetry_snapshot}] -> [(x, y, theta)], one per sample.
    A sample without distance or yaw holds the previous pose, so indices still line up
    with the samples (keyframes refer to sample indices)."""
    pose = PoseTracker()
    trail = []
    for s in samples:
        tel = s.get("tel") or {}
        dist = tel.get("dist_avg_in")
        yaw = tel.get("yaw_deg")
        if dist is not None and yaw is not None:
            pose.update(float(dist), float(yaw), float(s.get("t", 0.0)),
                        bool(tel.get("telemetry_live")), bool(tel.get("imu_live")))
        trail.append((pose.x, pose.y, pose.theta))
    return trail


def path_lengths(trail):
    """Cumulative xy path length at each trail point."""
    out = [0.0]
    for (x0, y0, _), (x1, y1, _) in zip(trail, trail[1:]):
        out.append(out[-1] + math.hypot(x1 - x0, y1 - y0))
    return out


def close_loop(trail):
    """(corrected_trail, closure) with the end pulled onto the start.

    The error is spread linearly by path length: drift accumulates with distance
    driven, so a point halfway round takes half the correction. closure reports the
    raw error before correction -- a large one means dead reckoning drifted (wheel slip,
    IMU drift) or the course did not actually end where it began."""
    if len(trail) < 2:
        return list(trail), {"dx_in": 0.0, "dy_in": 0.0, "dist_in": 0.0, "dtheta_deg": 0.0}
    x0, y0, th0 = trail[0]
    xe, ye, the = trail[-1]
    ex, ey = xe - x0, ye - y0
    eth = wrap_rad(the - th0)   # a full lap turns 2*pi, which is no error
    s = path_lengths(trail)
    total = s[-1] or 1.0
    out = []
    for (x, y, th), si in zip(trail, s):
        f = si / total
        out.append((x - ex * f, y - ey * f, th - eth * f))
    closure = {"dx_in": round(ex, 2), "dy_in": round(ey, 2),
               "dist_in": round(math.hypot(ex, ey), 2),
               "dtheta_deg": round(math.degrees(eth), 1)}
    return out, closure


def _perp_dist(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / n


def _rdp(pts, lo, hi, tol, keep):
    """Mark the points of pts[lo..hi] that RDP keeps."""
    if hi <= lo + 1:
        return
    a, b = pts[lo], pts[hi]
    best, idx = -1.0, None
    for i in range(lo + 1, hi):
        d = _perp_dist(pts[i], a, b)
        if d > best:
            best, idx = d, i
    if best > tol:
        keep.add(idx)
        _rdp(pts, lo, idx, tol, keep)
        _rdp(pts, idx, hi, tol, keep)


def simplify(trail, keyframe_idx=(), tol_in=DEFAULT_TOL_IN):
    """Sorted trail indices to keep as waypoints. Start, end and keyframes are pinned;
    RDP runs separately on each span between pinned points."""
    n = len(trail)
    if n == 0:
        return []
    pts = [(x, y) for x, y, _ in trail]
    pinned = sorted({0, n - 1} | {int(k) for k in keyframe_idx if 0 <= int(k) < n})
    keep = set(pinned)
    for lo, hi in zip(pinned, pinned[1:]):
        _rdp(pts, lo, hi, tol_in, keep)
    return sorted(keep)


def to_legs(waypoints, start_theta, min_turn_deg=DEFAULT_MIN_TURN_DEG,
            min_drive_in=DEFAULT_MIN_DRIVE_IN, end_theta=None):
    """waypoints: [{"idx", "x", "y", ...}] -> legs [{turn_deg, drive_in, to_waypoint}].

    turn_deg > 0 is a left (CCW) turn. Each leg turns to face the next waypoint, then
    drives forward to it. Turns under min_turn_deg are skipped (the robot cannot make
    them reliably) and drives under min_drive_in are skipped, with the next leg aimed
    from where the robot actually is. A last turn-only leg restores end_theta
    (default: the start heading, which is right for a closed loop)."""
    legs = []
    if not waypoints:
        return legs
    cx, cy = waypoints[0]["x"], waypoints[0]["y"]
    h = start_theta
    for wi, wp in enumerate(waypoints[1:], start=1):
        dx, dy = wp["x"] - cx, wp["y"] - cy
        d = math.hypot(dx, dy)
        if d < min_drive_in:
            continue
        turn = math.degrees(wrap_rad(math.atan2(dy, dx) - h))
        if abs(turn) < min_turn_deg:
            turn = 0.0
        h = wrap_rad(h + math.radians(turn))
        # Drive the full distance even if a skipped small turn left us slightly off
        # bearing: the next leg re-aims from the waypoint, so the error does not grow.
        legs.append({"turn_deg": round(turn, 1), "drive_in": round(d, 2),
                     "to_waypoint": wi})
        cx, cy = wp["x"], wp["y"]
    final = math.degrees(wrap_rad((start_theta if end_theta is None else end_theta) - h))
    if abs(final) >= min_turn_deg:
        legs.append({"turn_deg": round(final, 1), "drive_in": 0.0,
                     "to_waypoint": len(waypoints) - 1})
    return legs


def sensor_snapshot(tel):
    """The parts of a telemetry snapshot a follower might steer by. Kept small: the full
    snapshot is in samples.jsonl."""
    tel = tel or {}
    rng = tel.get("range") or {}
    chans = {}
    for label, ch in (rng.get("channels") or {}).items():
        if isinstance(ch, dict):
            chans[label] = {"mm": ch.get("mm"), "live": ch.get("live")}
    return {"yaw_deg": tel.get("yaw_deg"), "dist_avg_in": tel.get("dist_avg_in"),
            "heading_deg": tel.get("heading_deg"), "tof": chans,
            "camera": tel.get("camera")}


def process(samples, keyframes=(), tol_in=DEFAULT_TOL_IN, closed=True,
            min_turn_deg=DEFAULT_MIN_TURN_DEG, min_drive_in=DEFAULT_MIN_DRIVE_IN):
    """samples + keyframes (sample indices) -> route dict. Raises ValueError when there
    is nothing to follow."""
    if len(samples) < 2:
        raise ValueError("need at least 2 samples; record a run first")
    raw = integrate(samples)
    if closed:
        trail, closure = close_loop(raw)
    else:
        trail, closure = list(raw), None
    kf = sorted({int(k) for k in keyframes if 0 <= int(k) < len(trail)})
    keep = simplify(trail, kf, tol_in)
    waypoints = []
    for i in keep:
        x, y, th = trail[i]
        waypoints.append({"idx": i, "x": round(x, 2), "y": round(y, 2),
                          "theta_deg": round(math.degrees(th), 1),
                          "t": samples[i].get("t"), "keyframe": i in kf,
                          "sensors": sensor_snapshot(samples[i].get("tel"))})
    legs = to_legs(waypoints, trail[0][2], min_turn_deg, min_drive_in)
    length = path_lengths(trail)[-1]
    if not any(leg["drive_in"] > 0 for leg in legs):
        raise ValueError("the recorded run never moved far enough to make a route "
                         "(path %.1f in)" % length)
    return {
        "version": 1,
        "closed": closed,
        "tol_in": tol_in,
        "min_turn_deg": min_turn_deg,
        "min_drive_in": min_drive_in,
        "samples": len(samples),
        "keyframes": kf,
        "closure": closure,
        "path_length_in": round(length, 1),
        "route_length_in": round(sum(leg["drive_in"] for leg in legs), 1),
        "total_turn_deg": round(sum(leg["turn_deg"] for leg in legs), 1),
        "waypoints": waypoints,
        "legs": legs,
        "trail_raw": [(round(x, 2), round(y, 2)) for x, y, _ in raw],
        "trail": [(round(x, 2), round(y, 2), round(th, 4)) for x, y, th in trail],
    }
