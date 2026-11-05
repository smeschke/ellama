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
    python3 mcp_server/robot_server.py

Env:
    ELLAMA_SERIAL_PORT       explicit serial port; default auto-detects
    ELLAMA_REQUIRED_SENSORS  sensors the drive tools insist on; default "enc,imu"
    ELLAMA_TILT_LIMIT_DEG / ELLAMA_TILT_LOOKAHEAD_S / ELLAMA_TILT_GUARD (robot_link)
"""

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
from robot_link import (IMU_STALE_FAULT_S, POLICY, TILT, GuardFault,  # noqa: E402
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


atexit.register(_shutdown)
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, lambda signum, frame: (_shutdown(), sys.exit(1)))

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
    if missing:
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
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except GuardFault as e:
            return {"ok": False, "error": "guard: %s" % e, "guards": e.info,
                    "telemetry": link.telemetry_snapshot() if link is not None else None}
    return wrapper


def _confirm_gate(confirmed_safe: bool):
    tilt_block = _tilt_gate()
    if tilt_block:
        return tilt_block
    if confirmed_safe:
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

admin_tools.register(mcp, sys.modules[__name__])
drive_tools.register(mcp, sys.modules[__name__])

# Re-exported so `import robot_server as rs; rs.drive_distance(...)` (and the
# test suite) keep working after the split.
from admin_tools import (add_marker, clear_tilt_fault, connect, guards, listen,  # noqa: E402
                         read_range, read_telemetry, sensors, set_guard,
                         set_level_reference, set_required_sensors, start_recording,
                         stop, stop_recording, zero_odometry)
from drive_tools import (drive_arc, drive_distance, drive_until_range, jog,  # noqa: E402
                         turn_degrees)

if __name__ == "__main__":
    mcp.run()
