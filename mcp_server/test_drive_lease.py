#!/usr/bin/env python3
"""Hardware-free tests for the drive lease: one client drives at a time.

Run: mcp_server/.venv/bin/python mcp_server/test_drive_lease.py   (or `make test`)
"""

import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("ELLAMA_SERIAL_PORT", "/dev/none")
sys.path.insert(0, str(Path(__file__).resolve().parent))

import drive_lease  # noqa: E402
from test_operator_stop import ready_link  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0
    def __call__(self):
        return self.t


# ---- the lease on its own -------------------------------------------------------------

def test_one_holder_at_a_time():
    c = Clock()
    L = drive_lease.DriveLease(required=True, clock=c)
    ok, a = L.take("claude", "stephen", "demo")
    assert ok and a["lease"], a
    assert L.check(a["lease"]) is None

    ok, b = L.take("find_object.py", "stephen")
    assert not ok and "claude" in b["error"] and "stephen" in b["error"], b
    assert "lease" not in b, "a refused take must not hand out a token"

    assert "held by claude" in L.check(None)
    assert "not the current drive lease" in L.check("deadbeef")


def test_take_needs_a_holder_and_a_watching_operator():
    L = drive_lease.DriveLease()
    assert not L.take("", "stephen")[0]
    ok, info = L.take("claude", "  ")
    assert not ok and "operator" in info["error"], info
    assert not L.snapshot()["held"]


def test_quiet_holder_is_given_up_on_but_only_when_someone_asks():
    c = Clock()
    L = drive_lease.DriveLease(required=True, clock=c)
    _, a = L.take("claude", "stephen", ttl_s=60)
    c.t += 59
    assert not L.take("other", "stephen")[0], "still inside the ttl"

    # past the ttl with nobody else asking: the holder comes back and carries on
    c.t += 300
    assert L.snapshot()["available_to_others"] is True
    assert L.check(a["lease"]) is None
    assert L.snapshot()["quiet_for_s"] == 0.0

    # past the ttl and someone else asks: it changes hands, and the old holder is told why
    c.t += 61
    ok, b = L.take("other", "stephen")
    assert ok and b["lease"] != a["lease"], b
    assert L.snapshot()["holder"] == "other"
    assert L.check(a["lease"]) is not None
    assert L.check(b["lease"]) is None


def test_lapsed_holder_is_told_it_lapsed():
    c = Clock()
    L = drive_lease.DriveLease(required=False, clock=c)   # stdio-style: no lease needed
    _, a = L.take("claude", "stephen", ttl_s=30)
    c.t += 31
    # an abandoned lease must not block a lease-less client for ever
    assert L.check(None) is None
    assert not L.snapshot()["held"]
    msg = L.check(a["lease"])
    assert msg and "lapsed" in msg, msg


def test_never_taken_away_mid_move():
    c = Clock()
    L = drive_lease.DriveLease(required=True, clock=c)
    _, a = L.take("claude", "stephen", ttl_s=10)
    assert L.check(a["lease"]) is None
    L.begin()
    c.t += 500   # a long drive_distance: the holder is blocked in the call and cannot renew
    ok, b = L.take("other", "stephen")
    assert not ok and "move in progress" in b["error"], b
    assert not L.release(a["lease"])[0], "release is refused mid-move"
    L.end()
    assert L.snapshot()["move_in_progress"] is False
    assert L.snapshot()["quiet_for_s"] == 0.0, "the end of a move counts as hearing from the holder"
    assert not L.take("other", "stephen")[0]


def test_ttl_is_clamped():
    L = drive_lease.DriveLease()
    L.take("a", "op", ttl_s=0.01)
    assert L.snapshot()["ttl_s"] == drive_lease.DriveLease.MIN_TTL_S
    L.revoke()
    L.take("a", "op", ttl_s=1e9)
    assert L.snapshot()["ttl_s"] == drive_lease.DriveLease.MAX_TTL_S


def test_release_revoke_and_no_token_in_snapshot():
    L = drive_lease.DriveLease(required=True)
    _, a = L.take("claude", "stephen")
    assert a["lease"] not in repr(L.snapshot()), "the public snapshot must not leak the token"
    assert not L.release("wrong")[0]
    assert L.release(a["lease"])[0] and not L.snapshot()["held"]
    assert "not current" in L.check(a["lease"])

    _, b = L.take("claude", "stephen")
    was, snap = L.revoke("operator page")
    assert was and not snap["held"], snap
    assert L.check(b["lease"]) is not None
    assert L.revoke()[0] is False
    assert [e["event"] for e in snap["recent_events"]][-1] == "revoke"


# ---- through the drive tools ----------------------------------------------------------

def setup(rs, required):
    rs.LEASE.revoke("test")
    rs.LEASE.required = required
    link = ready_link(rs)
    rs.link = link
    return link


def teardown(rs):
    rs.operator_resume("test")
    rs.LEASE.revoke("test")
    rs.LEASE.required = False
    rs.LEASE._clock = time.monotonic
    rs.link = None


def drove(link, since):
    return [ln for ln in link.ser.lines[since:] if ln not in ("stop", "0 0")]


def test_daemon_refuses_to_drive_without_a_lease():
    import robot_server as rs
    link = setup(rs, required=True)
    try:
        n = len(link.ser.lines)
        for call in (lambda: rs.jog(50, 50, 0.1, confirmed_safe=True),
                     lambda: rs.turn_degrees(10, "left", 80, confirmed_safe=True),
                     lambda: rs.drive_distance(12, "forward", 50, confirmed_safe=True),
                     lambda: rs.drive_until_range(300, 50, confirmed_safe=True),
                     lambda: rs.drive_arc(20, 30, "left", 50, confirmed_safe=True)):
            out = call()
            assert out["ok"] is False and "take_drive_lease" in out["error"], out
        assert not drove(link, n), link.ser.lines[n:]

        # stop and reading need no lease
        assert rs.stop()["ok"] is True
        assert rs.guards()["drive_lease"]["held"] is False

        got = rs.take_drive_lease("claude", "stephen", "test")
        assert got["ok"] and got["lease"], got
        out = rs.jog(50, 50, 0.1, confirmed_safe=True, lease=got["lease"])
        assert out["ok"] is True, out
        assert "50 50" in link.ser.lines[n:]
        assert rs.LEASE.snapshot()["move_in_progress"] is False
    finally:
        teardown(rs)


def test_only_the_holder_drives():
    import robot_server as rs
    link = setup(rs, required=True)
    try:
        a = rs.take_drive_lease("claude", "stephen")
        b = rs.take_drive_lease("find_object.py", "stephen")
        assert b["ok"] is False and "claude" in b["error"], b

        n = len(link.ser.lines)
        out = rs.jog(50, 50, 0.1, confirmed_safe=True)
        assert out["ok"] is False and "held by claude" in out["error"], out
        out = rs.jog(50, 50, 0.1, confirmed_safe=True, lease="0000000000000000")
        assert out["ok"] is False and "not the current drive lease" in out["error"], out
        assert not drove(link, n), link.ser.lines[n:]

        # the lease is not permission: the holder still needs confirmed_safe
        out = rs.jog(50, 50, 0.1, confirmed_safe=False, lease=a["lease"])
        assert out["ok"] is False and "confirmed_safe" in out["error"], out
        assert not drove(link, n)
        assert rs.LEASE.snapshot()["move_in_progress"] is False

        assert rs.renew_drive_lease(a["lease"])["ok"] is True
        assert rs.renew_drive_lease("nope")["ok"] is False
        assert rs.release_drive_lease(a["lease"])["ok"] is True
        assert rs.take_drive_lease("find_object.py", "stephen")["ok"] is True
    finally:
        teardown(rs)


def test_single_client_server_drives_as_before():
    import robot_server as rs
    link = setup(rs, required=False)
    try:
        n = len(link.ser.lines)
        out = rs.jog(50, 50, 0.1, confirmed_safe=True)
        assert out["ok"] is True, out
        assert "50 50" in link.ser.lines[n:]
        # ...but once somebody takes a lease, it is honoured there too
        a = rs.take_drive_lease("claude", "stephen")
        out = rs.jog(50, 50, 0.1, confirmed_safe=True)
        assert out["ok"] is False and "held by claude" in out["error"], out
        assert rs.jog(50, 50, 0.1, confirmed_safe=True, lease=a["lease"])["ok"] is True
    finally:
        teardown(rs)


def test_lease_holds_through_a_real_blocking_move():
    import robot_server as rs
    link = setup(rs, required=True)
    clock = Clock()
    rs.LEASE._clock = clock
    a = rs.take_drive_lease("claude", "stephen", ttl_s=10)
    result = {}
    done = threading.Event()

    def keep_live():
        while not done.wait(0.1):
            link.sensors.touch("imu")
            link.sensors.touch("enc")

    threading.Thread(target=keep_live, daemon=True).start()
    th = threading.Thread(target=lambda: result.update(
        out=rs.jog(50, 50, 1.2, confirmed_safe=True, lease=a["lease"])))
    try:
        th.start()
        time.sleep(0.6)
        assert rs.LEASE.snapshot()["move_in_progress"] is True
        clock.t += 500   # far past the ttl, while the holder is blocked inside the move
        b = rs.take_drive_lease("other", "stephen")
        assert b["ok"] is False and "move in progress" in b["error"], b
        th.join(timeout=5)
        assert result["out"]["ok"] is True, result
        snap = rs.LEASE.snapshot()
        assert snap["move_in_progress"] is False and snap["holder"] == "claude", snap
    finally:
        th.join(timeout=5)
        done.set()
        teardown(rs)


def test_operator_stop_outranks_the_lease_and_does_not_take_it():
    import robot_server as rs
    link = setup(rs, required=True)
    try:
        a = rs.take_drive_lease("claude", "stephen")
        rs.operator_stop("stephen", "test")
        out = rs.jog(50, 50, 0.1, confirmed_safe=True, lease=a["lease"])
        assert out["ok"] is False and "operator stop" in out["error"], out
        assert rs.LEASE.snapshot()["holder"] == "claude", "a stop is not a revoke"
        rs.operator_resume("stephen")
        assert rs.jog(50, 50, 0.1, confirmed_safe=True, lease=a["lease"])["ok"] is True

        was, snap = rs.operator_revoke_lease("stephen")
        assert was and not snap["held"]
        out = rs.jog(50, 50, 0.1, confirmed_safe=True, lease=a["lease"])
        assert out["ok"] is False and "take_drive_lease" in out["error"], out
    finally:
        teardown(rs)


def test_tools_and_routes_are_exposed():
    import asyncio
    import robot_server as rs
    from starlette.testclient import TestClient
    link = setup(rs, required=True)
    try:
        tools = {t.name: t for t in asyncio.run(rs.mcp.list_tools())}
        assert {"take_drive_lease", "renew_drive_lease", "release_drive_lease"} <= set(tools)
        for name in ("drive_distance", "turn_degrees", "drive_arc", "drive_until_range", "jog"):
            schema = tools[name].input_schema if hasattr(tools[name], "input_schema") \
                else tools[name].inputSchema
            assert "lease" in schema["properties"], (name, schema)
            assert "lease" not in schema.get("required", []), name
        assert not {n for n in tools if "revoke" in n}, "revoking is the operator's, not a tool"

        a = rs.take_drive_lease("claude", "stephen", "http test")
        with TestClient(rs.mcp.streamable_http_app()) as c:
            s = c.get("/state")
            assert a["lease"] not in s.text, "the token must not appear on the status route"
            L = s.json()["drive_lease"]
            assert L["held"] and L["holder"] == "claude" and L["operator"] == "stephen", L
            assert "Take driving away" not in c.get("/").text, \
                "revoke button was removed from the page; STOP is the override"
            assert 'data-cmd="forward"' in c.get("/").text
            assert c.get("/lease/revoke").status_code == 405
            r = c.post("/lease/revoke").json()
            assert r["was_held"] is True and r["drive_lease"]["held"] is False, r
        assert rs.jog(50, 50, 0.1, confirmed_safe=True, lease=a["lease"])["ok"] is False
    finally:
        teardown(rs)


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
