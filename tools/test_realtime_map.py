#!/usr/bin/env python3
"""Hardware-free tests for tools/realtime_map.py. Run: mcp_server/.venv/bin/python tools/test_realtime_map.py"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import realtime_map as rm  # noqa: E402


def test_pose_straight_then_turn():
    # Physical left turns arrive as yaw DECREASING (verified IRL), so the
    # turn here feeds yaw 0 -> -90 and must come out toward +y (map-left).
    p = rm.PoseTracker()
    p.update(0.0, 0.0, 0.0, True, True)
    p.update(10.0, 0.0, 1.0, True, True)
    assert abs(p.x - 10.0) < 1e-6 and abs(p.y) < 1e-6, (p.x, p.y)
    p.update(10.0, -90.0, 2.0, True, True)
    p.update(20.0, -90.0, 3.0, True, True)
    assert abs(p.x - 10.0) < 1e-6 and abs(p.y - 10.0) < 1e-6, (p.x, p.y)


def test_pose_spin_left_renders_ccw():
    # IRL ground truth for a spin in place to the left: left encoder down,
    # right encoder up, no distance travelled, yaw decreasing. The map icon
    # must rotate CCW (theta positive), matching the dashboard.
    p = rm.PoseTracker()
    p.update(0.0, 0.0, 0.0, True, True)
    p.update(0.0, -90.0, 1.0, True, True)
    assert abs(p.x) < 1e-6 and abs(p.y) < 1e-6, (p.x, p.y)
    assert p.theta > 0, math.degrees(p.theta)


def test_pose_stale_holds():
    p = rm.PoseTracker()
    p.update(0.0, 0.0, 0.0, True, True)
    p.update(5.0, 10.0, 1.0, True, True)
    x, y, th = p.x, p.y, p.theta
    p.update(99.0, 80.0, 2.0, False, False)  # stale: must not jump
    assert (p.x, p.y, p.theta) == (x, y, th)


def test_source_labels():
    stick = {"telemetry": {"stick": {"active": True}},
             "drive_lease": {"held": False, "move_in_progress": False}}
    assert rm.source_label(stick) == "stick"
    dash = {"telemetry": {"stick": {"active": False}},
            "drive_lease": {"held": True, "holder": "dashboard",
                            "move_in_progress": True}}
    assert rm.source_label(dash) == "dashboard"
    quiet = {"telemetry": {"stick": None},
             "drive_lease": {"held": False, "move_in_progress": False}}
    assert rm.source_label(quiet) == "quiet"


def test_grid_hit_and_clear():
    g = rm.OccupancyGrid()
    g.cast(0, 0, 0.0, 508, (10.0, 0.0))  # 20 in wall ahead
    assert g.hits == 1 and g.rays == 1
    before = g.logodds.copy()
    g.cast(0, 0, 0.0, None, (10.0, 0.0))  # clear: no new hit, frees cells
    assert g.hits == 1 and g.rays == 2
    assert (g.logodds != before).any()


def test_stop_keyframe_no_camera_no_crash():
    k = rm.StopKeyframes(None, "")
    assert k.update(True, 0.0, (0, 0, 0)) is None
    assert k.update(False, 1.0, (0, 0, 0)) is None
    assert k.update(False, 1.0 + rm.STOP_HOLD_S + 0.1, (0, 0, 0)) is None
    assert k.saved == 0


def test_replay_stick_test():
    csv_path = Path(__file__).resolve().parent.parent / "recordings/stick_test/stick_test.csv"
    if not csv_path.exists():
        print("skip replay: no stick_test.csv")
        return
    rc = rm.replay_csv(type("A", (), {"replay": str(csv_path), "save": ""})())
    assert rc == 0


def test_tof_labels_and_rear_geometry():
    import bridge
    assert bridge.TofState.label("TOF_FRONT", 0) == "front_left"
    assert bridge.TofState.label("TOF_FRONT", 1) == "front_right"
    assert bridge.TofState.label("TOF_REAR", 0) == "rear_left"
    assert rm.ray_geometry("front_left", "TOF_FRONT") == ((10.0, 6.0), False)
    assert rm.ray_geometry("rear_left", "TOF_REAR") == ((-10.0, 6.0), True)


def test_reverse_cast_hits_behind():
    g = rm.OccupancyGrid()
    g.cast(0, 0, 0.0, 508, (-10.0, 0.0), reverse=True)  # 20 in wall behind
    assert g.hits == 1
    import numpy as np
    prob = g.prob()
    cy, cx = np.unravel_index(int(np.argmax(prob)), prob.shape)
    assert cx < g.ox, (cx, g.ox)  # hit cell is behind the start, not ahead


def test_splayed_cast_hits_off_axis():
    g = rm.OccupancyGrid()
    g.cast(0, 0, 0.0, 508, (10.0, 0.0), splay_deg=90.0)  # beam points left
    assert g.hits == 1
    import numpy as np
    prob = g.prob()
    cy, cx = np.unravel_index(int(np.argmax(prob)), prob.shape)
    assert cy > g.oy, (cy, g.oy)  # hit cell is left of the robot, not ahead
    assert abs(cx - (g.ox + 5)) <= 1, (cx, g.ox)


def test_robot_marker_points_along_heading():
    nose = rm._robot_pts(0, 0, 0.0)[0]
    assert nose[0] > 0 and abs(nose[1]) < 1e-6, nose  # theta=0 -> nose +x
    nose = rm._robot_pts(0, 0, math.pi / 2)[0]
    assert nose[1] > 0 and abs(nose[0]) < 1e-6, nose  # theta=90deg -> nose +y


def test_keyframe_name_records_mount():
    n = rm.keyframe_name(12.3, (1.0, -2.0, 0.5), "arrow")
    assert "cam-arrow" in n and n.endswith(".jpg"), n
    n = rm.keyframe_name(12.3, (1.0, -2.0, 0.5), "tail end!")
    assert "cam-tailend" in n, n  # sanitized, no spaces


if __name__ == "__main__":
    test_pose_straight_then_turn()
    test_pose_stale_holds()
    test_source_labels()
    test_grid_hit_and_clear()
    test_stop_keyframe_no_camera_no_crash()
    test_tof_labels_and_rear_geometry()
    test_reverse_cast_hits_behind()
    test_splayed_cast_hits_off_axis()
    test_robot_marker_points_along_heading()
    test_keyframe_name_records_mount()
    test_replay_stick_test()
    print("PASS tools/test_realtime_map.py")
