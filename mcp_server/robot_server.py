#!/usr/bin/env python3
"""MCP server that lets an AI agent drive eLlama over computer_bridge.ino, closing the
loop on the wheel encoders instead of driving open-loop for a fixed duration.

Talks over the same USB-serial link as calibration/live_view.py -- reuses bridge/ for
serial I/O, line parsing and the distance/heading math, so there is exactly one place
that logic lives.

Layout (split from the former single file, 2026-11-05):
  drive_math.py   pure PWM/speed/approach arithmetic and measured constants
  robot_link.py   RobotLink (serial, pump, send_drive choke point), POLICY, TILT
  drive_tools.py  drive_distance, turn_degrees, drive_arc, drive_until_range, jog
  admin_tools.py  connect, telemetry, sensors, policy, guards, recording tools
  robot_server.py this file: the link singleton, safety gates, MCP registration

Nothing here assumes a fixed set of sensors. The firmware's SensorPacket carries the
sender's own name, so a sensor appears when it starts transmitting and disappears when
it stops, and both are normal. This server REPORTS what it has -- sensors() -- and
enforces what it was TOLD to need -- set_required_sensors().

Read DRIVING_POLICY.md before calling anything here. Every tool that can move the
robot requires confirmed_safe=true, set only after asking the human operator in the
moment. Range sensors are not clearance: two narrow cones straight ahead say nothing
about the sides, the floor, a drop, or anything behind.

Setup:
    pip install mcp pyserial
    python3 mcp_server/robot_server.py            # stdio: one client launches and owns it
    python3 mcp_server/robot_server.py --http     # daemon: runs on its own, clients connect

Daemon mode (--http) serves the same tools at http://127.0.0.1:8765/mcp and adds the
operator's side, which is plain HTTP and needs no MCP client (see daemon_http.py):
    GET  /        a page with a STOP button and the robot's state
    POST /stop    latch the operator stop: zero PWM now, every drive refused until...
    POST /resume  ...the operator releases it. No MCP tool can.
    POST /lease/revoke  take driving away from whichever client holds the lease
    GET  /state   connection, guards and telemetry as JSON

Env:
    ELLAMA_SERIAL_PORT       explicit serial port; default auto-detects
    ELLAMA_REQUIRED_SENSORS  sensors the drive tools insist on; default "enc,imu"
    ELLAMA_TILT_LIMIT_DEG / ELLAMA_TILT_LOOKAHEAD_S / ELLAMA_TILT_GUARD (robot_link)
    ELLAMA_HTTP_HOST / ELLAMA_HTTP_PORT   daemon bind address; default 127.0.0.1:8765
    ELLAMA_DRIVE_LEASE       "required" / "optional": whether driving needs a lease when
                             nobody holds one. Default: required under --http, else optional
"""

import argparse
import atexit
import functools
import os
import signal
import sys
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bridge  # noqa: E402
import sensors as sensorlib  # noqa: E402
import tilt_guard  # noqa: E402
from drive_math import *  # noqa: E402,F401,F403
from drive_math import (RANGE_LATENCY_S, TELEMETRY_STALE_S, approach_pwm,  # noqa: E402
                        pwm_for_speed, speed_for_pwm)
from robot_link import (IMU_STALE_FAULT_S, LEASE, OPSTOP, POLICY, TILT,  # noqa: E402
                        GuardFault,
                        RobotLink, TiltFault)

from mcp.server.mcpserver import MCPServer as FastMCP  # noqa: E402  mcp>=2 renamed

# ---------------------------------------------------------------------------
# Connection lifecycle -- lazy/reconnectable so the server can start before the
# ESP32 is plugged in, and so a power-cycle doesn't require restarting it.
# `link` is the singleton every tool (via drive_tools/admin_tools CTX) reads;
# tests assign `robot_server.link = <fake>` to drive the tools without hardware.
# ---------------------------------------------------------------------------

link: Optional[RobotLink] = None
_connect_error: Optional[str] = None


def _connect(port=None):
    global link, _connect_error
    if link is not None:
        link.close()
        link = None
    try:
        link = RobotLink(port)
        _connect_error = None
    except Exception as e:
        _connect_error = str(e)
        raise
    return link


def _shutdown():
    if link is not None:
        link.emergency_stop()


def operator_stop(who="operator", reason=""):
    """Latch the operator's stop and brake. Latch first, then send: a drive command that
    was already past its check goes out before our "stop" and is overridden by it, and
    every one after it is refused at send_drive. Works with no robot connected -- the
    latch is still set, so a robot that connects later starts out stopped."""
    snap = OPSTOP.latch(who, reason)
    if link is not None:
        link.emergency_stop()
    try:
        import trace as _tr
        _tr.event("operator", action="stop", who=who, reason=reason)
    except Exception:
        pass
    return snap


def operator_resume(who="operator"):
    """Release the operator's stop. Only the operator's side calls this (daemon_http)."""
    out = OPSTOP.clear(who)
    try:
        import trace as _tr
        _tr.event("operator", action="resume", who=who)
    except Exception:
        pass
    return out


def operator_clear_tilt(who="operator"):
    """Release a latched tilt fault from the operator page. The person pressing the button
    is standing at the robot, which is the confirmation clear_tilt_fault asks a model for.
    TiltGuard.clear() still refuses while the robot leans within the margin of the limit
    or is still rocking. Returns (ok, message, tilt snapshot or None)."""
    if link is None:
        return False, "no robot connected", None
    with link.lock:
        ok, msg = link.tilt.clear()
        snap = link.tilt.snapshot()
    try:
        import trace as _tr
        _tr.event("operator", action="clear_tilt", who=who, ok=ok,
                  error=None if ok else msg)
    except Exception:
        pass
    return ok, msg, snap


def operator_revoke_lease(who="operator"):
    """Take the drive lease away from whoever holds it. Operator's side only. It does not
    stop a move in progress -- that is operator_stop -- it ends that client's turn."""
    out = LEASE.revoke(who)
    try:
        import trace as _tr
        _tr.event("operator", action="revoke_lease", who=who)
    except Exception:
        pass
    return out


atexit.register(_shutdown)
# os._exit, not sys.exit: SystemExit raised from a handler lands inside the MCP stdio
# loop, which then blocks forever waiting on its worker thread parked in a stdin read --
# the process survived SIGTERM still holding the serial port, so the next run of
# a script that wanted the port could not open it. _shutdown has already sent the stop and
# flushed the port by this line, and atexit would only run the same _shutdown again.
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, lambda signum, frame: (_shutdown(), os._exit(1)))

try:
    _connect(os.environ.get("ELLAMA_SERIAL_PORT"))
except Exception:
    pass  # tools report the connection error; the server itself still starts


mcp = FastMCP("ellama-robot")


def _not_connected():
    return {"ok": False, "error": f"not connected to the robot bridge: {_connect_error}"}


def _tilt_gate():
    """Early, readable refusal for any motion tool; send_drive enforces the same thing as the
    last line of defence."""
    why = link._drive_refusal() if link is not None else None
    if why is None:
        return None
    missing = link.missing_required()
    if OPSTOP.latched:
        hint = ("a person at the robot stopped it. Do not retry and do not look for a way "
                "round this: say what you were doing, ask the operator what they saw, and "
                "wait for them to press Resume.")
    elif missing:
        hint = ("either bring %s back (check the board's power and link, then connect()), or "
                "decide this run does not need it: set_required_sensors() lists what each one "
                "costs. sensors() shows everything that IS reporting."
                % " / ".join(missing))
    elif link.tilt.tripped:
        hint = ("if the robot is upright and still: clear_tilt_fault(confirmed_safe=true) after "
                "asking the operator. On a slope the robot is meant to climb, "
                "set_guard('tilt', limit_deg=...) raises the limit instead.")
    else:
        hint = ("no level reference yet: wait a second with the robot still and the IMU live, "
                "or call set_level_reference")
    return {"ok": False, "error": why, "guards": link._guard_info(), "hint": hint}


def _tilt_guarded(fn):
    """Turn a TiltFault raised mid-move into a normal tool result. The tool's own `finally`
    has already sent the stop by the time this sees it."""
    if getattr(fn, "_ellama_traced", False):
        return fn

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        import time as _t
        start = _t.monotonic()
        try:
            out = fn(*args, **kwargs)
            _trace.event("tool", tool=fn.__name__,
                         ok=bool(isinstance(out, dict) and out.get("ok", True)),
                         elapsed_s=round(_t.monotonic() - start, 3))
            return out
        except GuardFault as e:
            out = {"ok": False, "error": "guard: %s" % e, "guards": e.info,
                   "telemetry": link.telemetry_snapshot() if link is not None else None}
            _trace.event("tool", tool=fn.__name__, ok=False,
                         error=out["error"], elapsed_s=round(_t.monotonic() - start, 3))
            return out
        finally:
            # Pairs with the LEASE.begin() in _confirm_gate. Every drive tool goes through
            # this wrapper, so the lease stops counting a move as in progress however the
            # tool ended; it is a no-op for a call the gate refused.
            LEASE.end()
    wrapper._ellama_traced = True
    return wrapper


def _confirm_gate(confirmed_safe: bool, lease: Optional[str] = None):
    """The gate every drive tool passes before a wheel turns. In order: is the robot
    allowed to move at all (operator stop, sensor policy, tilt guard); is this caller the
    one driving (the lease); did the operator agree to this move (confirmed_safe)."""
    tilt_block = _tilt_gate()
    if tilt_block:
        return tilt_block
    lease_block = LEASE.check(lease)
    if lease_block:
        return {"ok": False, "error": lease_block, "drive_lease": LEASE.snapshot()}
    if confirmed_safe:
        LEASE.begin()   # ended by _tilt_guarded's finally
        return None
    rng = link.range_snapshot() if link is not None else None
    return {
        "ok": False,
        "error": (
            "confirmed_safe must be true, and only after asking the human operator in "
            "this specific moment. The robot's range sensors cover two narrow cones "
            "straight ahead -- they are not clearance: they cannot see the sides, the "
            "floor, a drop, or anything behind. See DRIVING_POLICY.md. State the exact "
            "move (distance/degrees, direction, PWM) and wait for a go-ahead before "
            "calling again with confirmed_safe=true."
        ),
        "range": rng,
    }


# ---------------------------------------------------------------------------
# Tools (registered from drive_tools / admin_tools)
# ---------------------------------------------------------------------------

import admin_tools  # noqa: E402
import drive_tools  # noqa: E402
import trace as _trace  # noqa: E402


def _traced(fn):
    """Wrap a read-only tool so its calls appear in recordings/trace.jsonl.

    Used by admin_tools.register (MCP path) and once more after the re-export
    below (direct-call path). Never raises; never double-wraps.
    """
    if getattr(fn, "_ellama_traced", False):
        return fn

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        import time as _t
        start = _t.monotonic()
        try:
            out = fn(*args, **kwargs)
            _trace.event("tool", tool=fn.__name__,
                         ok=bool(isinstance(out, dict) and out.get("ok", True)),
                         elapsed_s=round(_t.monotonic() - start, 3))
            return out
        except Exception as e:
            _trace.event("tool", tool=fn.__name__, ok=False, error=repr(e))
            raise
    wrapper._ellama_traced = True
    return wrapper


admin_tools.register(mcp, sys.modules[__name__])
drive_tools.register(mcp, sys.modules[__name__])

# Re-exported so `import robot_server as rs; rs.drive_distance(...)` (and the
# test suite) keep working after the split.
from admin_tools import (add_marker, clear_tilt_fault, connect, guards, listen,  # noqa: E402
                         read_range, read_telemetry, sensors, set_guard,
                         release_drive_lease, renew_drive_lease, set_level_reference,
                         set_required_sensors, start_recording, stop, stop_recording,
                         take_drive_lease, vetbook_recent, zero_odometry)
from drive_tools import (drive_arc, drive_distance, drive_until_range, jog,  # noqa: E402
                         turn_degrees)

# Direct-call copies (used by tests and sweep scripts) get the same trace
# wrapper the MCP path got at register time. _traced/_tilt_guarded refuse to
# double-wrap, so this is a no-op for names that already went through them.
for _n in ("read_telemetry", "sensors", "read_range", "guards"):
    globals()[_n] = _traced(globals()[_n])

import daemon_http  # noqa: E402

daemon_http.register(mcp, sys.modules[__name__])

import apriltag_module  # noqa: E402

apriltag_module.register(mcp, sys.modules[__name__])

import tag_turn  # noqa: E402

tag_turn.register(mcp, sys.modules[__name__])

import plan_module  # noqa: E402

plan_module.register(mcp, sys.modules[__name__])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--http", action="store_true",
                    help="run as a daemon over streamable HTTP instead of stdio")
    ap.add_argument("--host", default=os.environ.get("ELLAMA_HTTP_HOST", "127.0.0.1"),
                    help="daemon bind address (default 127.0.0.1: this machine only)")
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("ELLAMA_HTTP_PORT", "8765")))
    args = ap.parse_args(argv)
    import recording as _rec  # noqa: E402
    _rec.VetLog(REPO_ROOT / "recordings").log(
        "server_start", {"mode": "http" if args.http else "stdio",
                         "host": args.host, "port": args.port})
    if args.http:
        # Several clients can connect to a daemon, so driving takes a lease there unless
        # the operator has said otherwise. Under stdio there is exactly one client.
        if os.environ.get("ELLAMA_DRIVE_LEASE", "").lower() not in ("optional", "off", "0",
                                                                    "false", "no"):
            LEASE.required = True
        mcp.run(transport="streamable-http", host=args.host, port=args.port)
    else:
        mcp.run()


if __name__ == "__main__":
    main()
