#!/usr/bin/env python3
"""Hardware-free tests for route_math.py and watch_and_repeat.py.

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_watch_and_repeat.py
"""

import math
import sys
import tempfile
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import route_math as rmath  # noqa: E402
import watch_and_repeat as war  # noqa: E402
from drive_lease import DriveLease  # noqa: E402
from operator_stop import OperatorStop  # noqa: E402


def tel(dist, yaw):
    return {"dist_avg_in": dist, "yaw_deg": yaw, "telemetry_live": True, "imu_live": True,
            "range": {"channels": {"front_left": {"mm": 500, "live": True}}}}


def square_samples(side=48.0, step=2.0, turn_step=10.0, yaw_drift_per_side=0.0,
                   dist_scale=1.0):
    """Drive a CCW (left-turning) square. PoseTracker reads a left turn as yaw
    DECREASING, so yaw goes down by 90 at each corner. Returns (samples, corner_idx)."""
    samples, corners = [], []
    dist, yaw, t = 0.0, 0.0, 0.0

    def add():
        nonlocal t
        samples.append({"t": round(t, 2), "tel": tel(dist, yaw)})
        t += 0.1

    add()
    for _ in range(4):
        for _ in range(int(side / step)):
            dist += step * dist_scale
            yaw += yaw_drift_per_side * step / side
            add()
        corners.append(len(samples) - 1)
        for _ in range(int(90 / turn_step)):
            yaw -= turn_step
            add()
    return samples, corners


# ---- route_math ----------------------------------------------------------------

def test_integrate_square_closes_by_itself():
    s, _ = square_samples()
    trail = rmath.integrate(s)
    assert len(trail) == len(s)
    x, y, _ = trail[-1]
    assert abs(x) < 0.5 and abs(y) < 0.5, trail[-1]
    # first side goes along +x, second side turns LEFT (+y)
    assert trail[24][0] > 47 and abs(trail[24][1]) < 0.5
    assert trail[24 + 9 + 24][1] > 47, trail[24 + 9 + 24]


def test_close_loop_removes_drift_and_reports_it():
    s, _ = square_samples(yaw_drift_per_side=3.0, dist_scale=1.05)
    raw = rmath.integrate(s)
    fixed, closure = rmath.close_loop(raw)
    assert closure["dist_in"] > 3.0, closure
    assert math.hypot(fixed[-1][0] - fixed[0][0], fixed[-1][1] - fixed[0][1]) < 1e-6
    assert fixed[0] == raw[0]


def test_simplify_keeps_corners_and_a_midside_keyframe():
    s, corners = square_samples()
    trail = rmath.integrate(s)
    mid = 12  # halfway along the first side: RDP alone would drop it
    assert mid not in rmath.simplify(trail, (), 3.0)
    keep = rmath.simplify(trail, (mid,), 3.0)
    assert mid in keep and 0 in keep and len(s) - 1 in keep
    for c in corners[:3]:
        assert any(abs(k - c) <= 9 for k in keep), (c, keep)


def test_process_square_legs():
    s, _ = square_samples()
    route = rmath.process(s, keyframes=[12])
    legs = route["legs"]
    assert abs(sum(L["drive_in"] for L in legs) - 192) < 4, legs
    # three left turns between sides; first leg heads straight, the last turn restores
    # the start heading, which is 90 deg more: 360 total
    assert abs(route["total_turn_deg"] - 360) < 10, legs
    assert all(L["turn_deg"] >= 0 for L in legs), legs
    assert route["waypoints"][0]["sensors"]["tof"]["front_left"]["mm"] == 500
    assert any(w["keyframe"] for w in route["waypoints"])


def test_process_refuses_a_run_that_never_moved():
    s = [{"t": i * 0.1, "tel": tel(0.0, 0.0)} for i in range(20)]
    try:
        rmath.process(s)
    except ValueError:
        return
    raise AssertionError("expected ValueError")


def test_small_turns_and_drives_are_skipped():
    wps = [{"x": 0, "y": 0}, {"x": 30, "y": 1}, {"x": 31, "y": 1}, {"x": 60, "y": 1}]
    legs = rmath.to_legs(wps, 0.0, min_turn_deg=8, min_drive_in=2)
    assert [L["turn_deg"] for L in legs] == [0.0, 0.0], legs
    assert len(legs) == 2


# ---- follower + session ---------------------------------------------------------

class FakeLink:
    def __init__(self):
        self.dist, self.yaw = 0.0, 0.0
        self.stops = 0

    def telemetry_snapshot(self):
        return tel(self.dist, self.yaw)

    def send_stop(self):
        self.stops += 1


def make_ctx(fail_on=None):
    calls = []
    lease = DriveLease(required=True)
    opstop = OperatorStop()
    link = FakeLink()

    def check(kw):
        if opstop.latched:
            return {"ok": False, "error": "operator stop latched"}
        err = lease.check(kw.get("lease"))
        if err:
            return {"ok": False, "error": err}
        return None

    def turn_degrees(**kw):
        calls.append(("turn", kw["direction"], kw["degrees"]))
        r = check(kw)
        if r:
            return r
        if fail_on == len(calls):
            return {"ok": False, "error": "guard: tilt"}
        return {"ok": True, "actual_degrees_turned": kw["degrees"]}

    def drive_distance(**kw):
        calls.append(("drive", kw["direction"], kw["distance_inches"]))
        r = check(kw)
        if r:
            return r
        if fail_on == len(calls):
            return {"ok": False, "error": "guard: tilt"}
        return {"ok": True, "actual_inches_traveled": kw["distance_inches"]}

    def operator_stop(who, reason=""):
        return opstop.latch(who, reason)

    ctx = types.SimpleNamespace(link=link, LEASE=lease, OPSTOP=opstop,
                                turn_degrees=turn_degrees, drive_distance=drive_distance,
                                operator_stop=operator_stop)
    return ctx, calls


def recorded_session(ctx, tmp):
    sess = war.RouteSession(ctx, routes_dir=tmp)
    code, out = sess.start_recording("sq")
    assert code == 200, out
    sess._rec_stop.set()            # drive the samples by hand, not by the thread
    sess._rec_thread.join(timeout=2)
    s, _ = square_samples()
    for i, smp in enumerate(s):
        ctx.link.dist = smp["tel"]["dist_avg_in"]
        ctx.link.yaw = smp["tel"]["yaw_deg"]
        if i == 12:
            assert sess.keyframe("mid")[0] == 200
        else:
            sess.sample_once()
    code, out = sess.stop_recording()
    assert code == 200 and out["keyframes"] == 1, out
    return sess


def test_session_state_machine_and_follow():
    ctx, calls = make_ctx()
    with tempfile.TemporaryDirectory() as tmp:
        sess = recorded_session(ctx, tmp)
        assert sess.follow_route()[0] == 409          # not processed yet
        code, out = sess.process(3.0)
        assert code == 200 and out["legs"], out
        assert (Path(tmp) / "sq" / "route.json").exists()
        assert (Path(tmp) / "sq" / "route.png").exists()
        code, out = sess.follow_route("legs", {"drive_pwm": 30}, background=False)
        assert code == 200 and out["ok"], out
        drives = [c for c in calls if c[0] == "drive"]
        turns = [c for c in calls if c[0] == "turn"]
        assert abs(sum(c[2] for c in drives) - 192) < 4, calls
        assert all(c[1] == "left" for c in turns), calls
        assert sess.mode == "processed"
        assert ctx.LEASE.snapshot()["held"] is False   # released afterwards

        # a saved run loads back in processed state
        sess2 = war.RouteSession(ctx, routes_dir=tmp)
        code, out = sess2.load("sq")
        assert code == 200 and out["mode"] == "processed", out


def test_follow_stops_on_failed_leg():
    ctx, calls = make_ctx(fail_on=2)
    with tempfile.TemporaryDirectory() as tmp:
        sess = recorded_session(ctx, tmp)
        sess.process()
        code, out = sess.follow_route(background=False)
        assert code == 409 and out["failed_leg"] is not None, out
        assert len(calls) == 2, calls


def test_follow_refused_while_stopped_or_lease_held():
    ctx, calls = make_ctx()
    with tempfile.TemporaryDirectory() as tmp:
        sess = recorded_session(ctx, tmp)
        sess.process()
        ok, info = ctx.LEASE.take("someone else", "op")
        assert ok
        code, out = sess.follow_route(background=False)
        assert code == 409 and "held by" in out["error"], out
        ctx.LEASE.release(info["lease"])
        ctx.OPSTOP.latch("op", "test")
        code, out = sess.follow_route(background=False)
        assert code == 409 and "operator stop" in out["error"], out
        assert calls == []


def test_abort_stops_between_legs():
    ctx, calls = make_ctx()
    with tempfile.TemporaryDirectory() as tmp:
        sess = recorded_session(ctx, tmp)
        sess.process()
        orig = ctx.drive_distance

        def drive_then_abort(**kw):
            r = orig(**kw)
            sess.abort("test")
            return r
        ctx.drive_distance = drive_then_abort
        code, out = sess.follow_route(background=False)
        assert out.get("aborted") or out.get("failed_leg"), out
        assert len([c for c in calls if c[0] == "drive"]) == 1, calls
        assert ctx.OPSTOP.latched


def test_busy_modes_refuse():
    ctx, _ = make_ctx()
    with tempfile.TemporaryDirectory() as tmp:
        sess = war.RouteSession(ctx, routes_dir=tmp)
        assert sess.keyframe()[0] == 409
        assert sess.stop_recording()[0] == 409
        assert sess.process()[0] == 409
        sess.start_recording("x")
        assert sess.process()[0] == 409
        assert sess.start_recording("y")[0] == 409
        sess.stop_recording()
        ctx.link = None
        assert sess.start_recording("z")[0] == 409


if __name__ == "__main__":
    n = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            n += 1
    print("PASS mcp_server/test_watch_and_repeat.py (%d tests)" % n)
