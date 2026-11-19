#!/usr/bin/env python3
"""Hardware-free tests for the operator's stop latch and the daemon's HTTP routes.

Run: mcp_server/.venv/bin/python mcp_server/test_operator_stop.py   (or `make test`)
"""

import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("ELLAMA_SERIAL_PORT", "/dev/none")
sys.path.insert(0, str(Path(__file__).resolve().parent))

import tilt_guard as tg  # noqa: E402
from test_tilt_guard import make_link  # noqa: E402


def ready_link(rs):
    """A fake-serial link whose tilt guard has its level reference, so the only thing
    that can refuse a drive in these tests is the operator stop."""
    link = make_link(rs)
    t = 0
    for _ in range(tg.REF_SAMPLES + 1):
        t += 20
        link.tilt.update(0.0, 0.0, t, gyro_dps=1.0)
    link.sensors.touch("imu")
    # what the tools read besides send_drive; make_link() only builds the choke point
    link.lock = threading.Lock()
    link.telemetry_snapshot = lambda: {"fake": True, "telemetry_live": True}
    link.last_counts = lambda: None
    link.in_per_tick = lambda: 0.01
    link.range_snapshot = lambda: None

    # Stand in for the pump: the IMU counts as stale after 0.5 s, and a tool call spends
    # about that long settling before it drives, so without this the sensor policy ends
    # every move before the thing under test gets a chance to.
    def keep_live():
        while True:
            time.sleep(0.1)
            link.sensors.touch("imu")
            link.sensors.touch("enc")

    threading.Thread(target=keep_live, daemon=True).start()
    return link


def test_latch_refuses_drive_and_brakes():
    import robot_server as rs
    link = ready_link(rs)
    rs.link = link
    try:
        link.send_drive(60, 60)
        assert link.ser.lines[-1] == "60 60"

        snap = rs.operator_stop("tester", "foot in the way")
        assert snap["latched"] and snap["who"] == "tester", snap
        assert link.ser.lines[-1] == "stop", "latching must brake at once"

        n = len(link.ser.lines)
        try:
            link.send_drive(60, 60)
            assert False, "a latched operator stop must refuse a drive"
        except rs.GuardFault as e:
            assert "operator stop" in str(e) and "foot in the way" in str(e), str(e)
            assert e.info["operator_stop"]["latched"], e.info
        assert link.ser.lines[n:] == ["stop"], link.ser.lines[n:]

        link.send_drive(0, 0)  # zero is always allowed: it is how loops stop
        assert link.ser.lines[-1] == "0 0"
    finally:
        rs.operator_resume("tester")
        rs.link = None


def test_tools_refuse_while_latched_and_no_tool_clears_it():
    import asyncio
    import robot_server as rs
    link = ready_link(rs)
    rs.link = link
    try:
        rs.operator_stop("tester")
        n = len(link.ser.lines)
        for call in (lambda: rs.drive_distance(12, "forward", 50, confirmed_safe=True),
                     lambda: rs.turn_degrees(10, "left", 80, confirmed_safe=True),
                     lambda: rs.jog(50, 50, 0.5, confirmed_safe=True)):
            out = call()
            assert out["ok"] is False and "operator stop" in out["error"], out
            assert "Resume" in out["hint"], out
        assert not [ln for ln in link.ser.lines[n:] if ln != "stop"], link.ser.lines[n:]

        # the tilt-fault release must not double as an operator-stop release
        out = rs.clear_tilt_fault(confirmed_safe=True)
        assert rs.OPSTOP.latched, "clear_tilt_fault released the operator stop: %r" % out
        # and reconnecting must not either (the latch is process-wide, not per link)
        rs.link = ready_link(rs)
        assert rs.link._drive_refusal() and "operator stop" in rs.link._drive_refusal()

        names = {t.name for t in asyncio.run(rs.mcp.list_tools())}
        assert not {n for n in names if "resume" in n or "operator" in n}, names
    finally:
        rs.operator_resume("tester")
        rs.link = None


def test_resume_lets_it_drive_again():
    import robot_server as rs
    link = ready_link(rs)
    rs.link = link
    try:
        rs.operator_stop("tester")
        was, snap = rs.operator_resume("tester")
        assert was is True and snap["latched"] is False, snap
        link.send_drive(40, 40)
        assert link.ser.lines[-1] == "40 40"
        was, _ = rs.operator_resume("tester")
        assert was is False, "resume with nothing latched reports that"
    finally:
        rs.operator_resume("tester")
        rs.link = None


def test_stop_lands_during_a_blocking_move():
    """The real case: a tool is inside its drive loop on a worker thread when the operator
    presses stop. The move must end promptly with a guard error, not run to its target."""
    import robot_server as rs
    link = ready_link(rs)
    rs.link = link
    result = {}

    def run():
        result["out"] = rs.jog(50, 50, 5.0, confirmed_safe=True)

    # Keep the fake sensors fresh for the length of the move, as the pump would: the IMU
    # counts as stale after 0.5 s, and a stale IMU ends the move before the stop can.
    done = threading.Event()

    def keep_live():
        while not done.wait(0.1):
            link.sensors.touch("imu")
            link.sensors.touch("enc")

    threading.Thread(target=keep_live, daemon=True).start()
    th = threading.Thread(target=run)
    try:
        t0 = time.monotonic()
        th.start()
        time.sleep(0.6)  # past the settle, into the keepalive loop
        assert [ln for ln in link.ser.lines if ln == "50 50"], link.ser.lines
        rs.operator_stop("tester", "mid-move")
        th.join(timeout=3.0)
        took = time.monotonic() - t0
        assert not th.is_alive(), "the move kept running after the operator stop"
        assert took < 3.0, "took %.1fs to stop a 5s jog" % took
        out = result["out"]
        assert out["ok"] is False and "operator stop" in out["error"], out
        last_drive = max(i for i, ln in enumerate(link.ser.lines) if ln == "50 50")
        assert "stop" in link.ser.lines[last_drive:], link.ser.lines[last_drive:]
    finally:
        th.join(timeout=6.0)
        done.set()
        rs.operator_resume("tester")
        rs.link = None


def test_http_routes():
    import robot_server as rs
    from starlette.testclient import TestClient
    link = ready_link(rs)
    rs.link = link
    try:
        with TestClient(rs.mcp.streamable_http_app()) as c:
            assert "STOP" in c.get("/").text
            s = c.get("/state").json()
            assert s["connected"] is True and s["operator_stop"]["latched"] is False, s

            r = c.post("/stop", json={"who": "phone", "reason": "test"}).json()
            assert r["ok"] and r["operator_stop"]["latched"], r
            assert link.ser.lines[-1] == "stop"
            assert c.post("/stop").json()["operator_stop"]["who"] == "phone", \
                "a second press keeps the first cause"
            assert c.get("/state").json()["operator_stop"]["latched"] is True

            assert c.get("/stop").status_code == 405, "stop must not fire on a GET"
            assert c.get("/resume").status_code == 405

            r = c.post("/resume").json()
            assert r["was_latched"] is True and r["operator_stop"]["latched"] is False, r

        # with no robot connected the page and the latch still work
        rs.link = None
        with TestClient(rs.mcp.streamable_http_app()) as c:
            s = c.get("/state").json()
            assert s["connected"] is False and "telemetry" not in s, s
            assert c.post("/stop").json()["operator_stop"]["latched"] is True
    finally:
        rs.operator_resume("tester")
        rs.link = None


def _trip(link, t0=10_000):
    """Lean the fake robot past the limit for enough samples to latch the guard."""
    t = t0
    for _ in range(link.tilt.confirm_samples + 2):
        t += 20
        link.tilt.update(40.0, 0.0, t, gyro_dps=1.0)
    assert link.tilt.tripped
    return t


def _settle(link, pitch, t, n=40):
    for _ in range(n):
        t += 20
        link.tilt.update(pitch, 0.0, t, gyro_dps=1.0)
    return t


def test_http_clear_tilt():
    import robot_server as rs
    from starlette.testclient import TestClient
    link = ready_link(rs)
    rs.link = link
    try:
        with TestClient(rs.mcp.streamable_http_app()) as c:
            assert "cleartilt" in c.get("/").text
            page = c.get("/").text
            assert all(k in page for k in ("camaddr", "camconnect", 'data-cam="1"', 'data-cam="2"')), "camera panes"
            assert c.get("/state").json()["tilt_guard"]["tripped"] is False
            assert c.get("/tilt/clear").status_code == 405, "clear must not fire on a GET"

            t = _trip(link)
            assert c.get("/state").json()["tilt_guard"]["tripped"] is True

            # still leaning past limit - margin: refused, stays latched
            t = _settle(link, 22.0, t)
            r = c.post("/tilt/clear", json={"who": "phone"})
            assert r.status_code == 409 and r.json()["ok"] is False, r.json()
            assert link.tilt.tripped

            # back near level and still: clears
            _settle(link, 2.0, t)
            r = c.post("/tilt/clear", json={"who": "phone"})
            assert r.status_code == 200 and r.json()["ok"] is True, r.json()
            assert link.tilt.tripped is False
            assert c.get("/state").json()["tilt_guard"]["tripped"] is False

            # nothing latched: a press is harmless
            assert c.post("/tilt/clear").json()["ok"] is True

        rs.link = None
        with TestClient(rs.mcp.streamable_http_app()) as c:
            r = c.post("/tilt/clear")
            assert r.status_code == 409 and r.json()["ok"] is False
    finally:
        rs.link = None


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print("PASS", fn.__name__)
        except Exception as e:
            failed += 1
            import traceback
            print("FAIL", fn.__name__, "-", repr(e))
            traceback.print_exc()
    print("\n%d/%d passed" % (len(tests) - failed, len(tests)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
