#!/usr/bin/env python3
"""Admin tools: connect, telemetry, sensors, policy, guards, odometry,
recording. Registered by robot_server.register_tools(); see drive_tools.py
for the CTX convention.
"""

import sys
import time
from pathlib import Path
from typing import Literal, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import bridge  # noqa: E402
import sensors as sensorlib  # noqa: E402
from drive_math import STOP_SETTLE_S  # noqa: E402

CTX = None


def _L():
    return CTX.link


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


def connect(port: Optional[str] = None) -> dict:
    """(Re)connect to the bridge ESP32 over serial. Call this if the server started
    before the ESP32 was plugged in, or after a power cycle / reconnect. `port` defaults
    to the ELLAMA_SERIAL_PORT env var or auto-detection (e.g. /dev/ttyUSB0)."""
    try:
        l = CTX._connect(port)
        return {"ok": True, "port": l.ser.port}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def read_telemetry() -> dict:
    """Read-only. Current cumulative encoder counts, distance/heading since the last
    zero_odometry(), latest IMU sample and complementary-filter pitch/roll/yaw, and
    whether encoder telemetry is currently live (packets seen within the last couple of
    seconds). Always safe to call -- never sends a drive command. Check telemetry_live
    before trusting any distance/heading number here."""
    if _L() is None:
        return CTX._not_connected()
    return {"ok": True, **_L().telemetry_snapshot()}


def sensors() -> dict:
    """Read-only. Every sensor stream this robot is actually producing, what each guard can
    therefore do, and which sensors the drive tools are currently required to have.

    This is the call to make before deciding whether the robot can run in a given
    situation. Nothing here is a fixed list: a stream appears the first time a packet with
    its name arrives, so a ToF board flashed as "TOF_REAR" shows up on its own.

    Read `streams` carefully -- three states that must not be confused:
      live=true, measuring=true    working
      live=true, measuring=false   reporting, but with nothing to report: a range sensor
                                   with nothing in its cone, or an encoder that is not
                                   responding. `available` says which channels.
      live=false                   silent: board dead, unpowered, unflashed or absent.

    "Nothing is in front of the robot" and "the robot cannot see" are different answers.

    A name that has been seen only once lands in `provisional`, not `streams`: the serial
    link tears, and a torn "IMU" line leaves "MU" looking like a sensor. A provisional name
    never satisfies a required sensor, and is dropped if it stays quiet. A board you have
    just switched on appears there for one packet and then moves to `streams`.

    `guards` says, per guard, whether its sensor is even present -- a guard with no sensor
    reports inactive and does NOT stop the robot; whether to drive in that state is the
    policy's call, not the guard's. `required` is that policy; `missing` is what it asks
    for and is not getting, and `costs` spells out what each absence means.
    """
    if _L() is None:
        return CTX._not_connected()
    now = time.monotonic()
    snap = _L().sensors.snapshot(now)
    missing = _L().policy.missing(_L().sensors, now)
    return {
        "ok": True,
        "streams": snap["streams"],
        "provisional": snap["provisional"],
        "provisional_note":
            "names seen only once. Either a board you just switched on, or a garbled serial "
            "line -- the link tears occasionally and a torn \"IMU\" leaves \"MU\" behind. "
            "They never satisfy a required sensor, and are forgotten if they stay quiet."
            if snap["provisional"] else None,
        "bridge": snap["bridge"],
        "range": _L().range_snapshot(now),
        "guards": _L()._guard_info(),
        "required": list(_L().policy.required),
        "required_set_by": _L().policy.set_by,
        "missing": missing,
        "costs": sensorlib.capability_cost(missing),
        "can_drive": _L()._drive_refusal() is None,
        "why_not": _L()._drive_refusal(),
    }


def read_range() -> dict:
    """Read-only. Distance to the closest thing the forward ToF sensors can see.

    `closest_mm` is a median of recent samples, which is what to steer by; each channel
    also carries `latest_mm`, the raw last reading. Never sends a drive command.

    Two things this does NOT mean. `closest_mm: null` with live=true means every sensor
    reported and found nothing in range -- clear as far as they reach. `live: false` means
    they are not reporting at all, which is blind, not clear; the `note` field says which.
    And range is not clearance either way: these are two narrow cones pointing forward.
    They cannot see the sides, the floor, a drop, or anything behind the robot, so they do
    not replace the operator's confirmed_safe.
    """
    if _L() is None:
        return CTX._not_connected()
    rng = _L().range_snapshot()
    if rng is None:
        return {"ok": True, "range": None,
                "note": "no range sensor has ever reported on this link -- none fitted, or "
                        "none powered. Check sensors() for what is reporting."}
    return {"ok": True, **rng}


def set_required_sensors(names: list[str], confirmed_safe: bool) -> dict:
    """Declare which sensors the drive tools must have live before they will move the robot.
    Does not move the robot. Takes effect immediately and lasts until changed or the server
    restarts.

    This is the judgement call the robot cannot make for itself: whether the situation is
    instrumented well enough for what is about to happen. Examples:
      ["enc", "imu"]   the default -- the robot's own closed-loop feedback
      ["enc"]          the IMU board is dead; drive by encoders, no tilt guard
      []               nothing on the robot is required, because something outside it is
                       watching: a ceiling camera, a marker on the robot, an operator with
                       a hand on the stick. Everything becomes open-loop and says so.
      ["enc", "tof"]   also insist on a range sensor before driving

    "tof" (or "range") matches any ToF board, so it keeps working when TOF_FRONT is joined
    or replaced by another. Unknown names are accepted -- a sensor you are about to flash
    is simply "missing" until it transmits.

    confirmed_safe is required because loosening this widens what the robot will do without
    anything checking it; set it only after the operator has agreed to drive this way. The
    result lists what the new set costs you, and the change is written into any active
    recording.
    """
    if _L() is None:
        return CTX._not_connected()
    if not confirmed_safe:
        return {"ok": False, "error":
                "confirmed_safe must be true: changing the required sensors changes what the "
                "robot will do with no feedback. Tell the operator which sensors you are "
                "dropping and what that costs (see sensors()), and wait for a go-ahead."}
    if not isinstance(names, (list, tuple)):
        return {"ok": False, "error": "names must be a list of sensor names, e.g. [\"enc\"]"}
    before = list(_L().policy.required)
    required = _L().policy.set(names)
    missing = _L().policy.missing(_L().sensors)
    _L().recorder.on_config("required_sensors %s -> %s" % (before, required))
    return {
        "ok": True,
        "required": required,
        "was": before,
        "missing": missing,
        "costs": sensorlib.capability_cost(
            [n for n in sensorlib.SensorPolicy.DEFAULT if n not in required] + missing),
        "can_drive": _L()._drive_refusal() is None,
        "why_not": _L()._drive_refusal(),
        "guards": _L()._guard_info(),
    }


def guards() -> dict:
    """Read-only. The tilt guard: mode, thresholds, whether the IMU it reads is live,
    whether it has tripped and why. Does not move the robot.

    A guard reporting active=false with inactive_because set is not broken -- it has no
    sensor to judge by. That on its own never stops the robot; set_required_sensors()
    decides whether driving without that sensor is acceptable.
    """
    if _L() is None:
        return CTX._not_connected()
    return {"ok": True, **_L()._guard_info()}


def set_guard(
    name: Literal["tilt"] = "tilt",
    mode: Optional[Literal["on", "warn", "off"]] = None,
    limit_deg: Optional[float] = None,
    lookahead_s: Optional[float] = None,
) -> dict:
    """Turn the tilt guard on or off and retune it, while running. Does not move the robot.

    `mode` is "on" (trips and latches), "warn" (records what it would have tripped on and
    never stops the robot -- use this to tune on a real robot) or "off".

    limit_deg (default 25) and lookahead_s are the thresholds. Raise limit_deg for a ramp
    the robot is meant to climb; the guard measures lean from where it was level, so a ramp
    is a real lean to it.

    What this deliberately does NOT do: clear a latched fault. Switching the guard off
    suspends its enforcement, but what it saw stays on the record and comes back if you
    switch it on again. Only clear_tilt_fault releases a latch, and it requires the
    operator to have actually looked at the robot.

    Changes are written into any active recording, so a run's CSV shows the guard config it
    was driven under.
    """
    if _L() is None:
        return CTX._not_connected()
    ok, msg = _L().tilt.configure(mode=mode, limit_deg=limit_deg, lookahead_s=lookahead_s)
    if not ok:
        return {"ok": False, "error": msg, "guards": _L()._guard_info()}
    _L().recorder.on_config("%s guard: %s" % (name, msg))
    return {"ok": True, "message": msg, "guards": _L()._guard_info()}


def zero_odometry() -> dict:
    """Make the robot's current position the origin for distance_*_in and heading_deg in
    read_telemetry(). Does not move the robot -- always safe to call, no confirmation
    needed."""
    if _L() is None:
        return CTX._not_connected()
    with _L().lock:
        _L().enc.zero()
        _L().orient.zero_yaw()
    return {"ok": True, **_L().telemetry_snapshot()}


def stop() -> dict:
    """Immediately command zero PWM (active braking, not coasting -- see
    DRIVING_POLICY.md). Always safe to call with no confirmation, any time, including
    when nothing is currently moving."""
    if _L() is None:
        return CTX._not_connected()
    _L().send_stop()
    time.sleep(STOP_SETTLE_S)
    return {"ok": True, **_L().telemetry_snapshot()}


def clear_tilt_fault(confirmed_safe: bool) -> dict:
    """Release the tilt guard after it tripped (the robot leaned past the limit and every
    drive tool is refused until this is called). Does not move the robot. Set
    confirmed_safe=true only after the human operator has looked at the robot and said it is
    upright and clear of whatever it was climbing. Refused while the IMU still reads the
    robot leaning within 5 deg of the limit or still rocking, so it cannot be used to drive
    out of a lean -- pick the robot up or level it by hand first."""
    if _L() is None:
        return CTX._not_connected()
    if not confirmed_safe:
        return {"ok": False, "error": "confirmed_safe must be true, set only after the operator "
                "confirms the robot is upright and clear",
                "tilt_guard": _L().tilt.snapshot()}
    with _L().lock:
        ok, msg = _L().tilt.clear()
    return {"ok": ok, "message": msg, "tilt_guard": _L().tilt.snapshot()}


def set_level_reference(confirmed_level: bool) -> dict:
    """Tell the tilt guard "this is level": record the robot's current orientation as the
    zero for tilt. It is captured automatically from the first ~0.5 s of a still, live IMU
    after connect(), so call this only if the robot was NOT on level ground then (e.g. the
    server connected while it sat on a ramp), or after the IMU board was remounted. Does not
    move the robot. confirmed_level=true only after the operator says the robot is on level
    ground. Refused if the robot is moving, or if it would move the reference by more than
    half the tilt limit from the one in use (a real mounting change is a few degrees; a
    bigger jump is the robot leaning, and this must not be a way to hide that)."""
    if _L() is None:
        return CTX._not_connected()
    if not confirmed_level:
        return {"ok": False, "error": "confirmed_level must be true, set only after the operator "
                "confirms the robot is sitting on level ground"}
    snap = _L().telemetry_snapshot()
    if not snap["imu_live"]:
        return {"ok": False, "error": "no live IMU", "telemetry": snap}
    with _L().lock:
        t = _L().tilt
        if abs(t.rate_dps) > tilt_guard.REF_MAX_RATE_DPS:
            return {"ok": False, "error": "robot is moving; hold it still first",
                    "tilt_guard": t.snapshot()}
        if t.ready and t.tilt_deg > t.limit_deg / 2.0:
            return {"ok": False, "error": "robot reads %.1f deg off the current reference, more than "
                    "half the %.0f deg limit -- that is a lean, not a mounting change. Level it "
                    "by hand, or restart the server to re-capture from scratch."
                    % (t.tilt_deg, t.limit_deg), "tilt_guard": t.snapshot()}
        t.set_reference(_L().orient.pitch, _L().orient.roll, source="operator")
    return {"ok": True, "tilt_guard": _L().tilt.snapshot()}


def listen() -> dict:
    """Stop and release control back to the handheld stick controller (sends 'listen',
    matching computer_bridge.ino's serial protocol). Use this when done driving so a
    human can pick up the stick without the two fighting on the same ESP-NOW channel.
    Always safe to call, no confirmation needed."""
    if _L() is None:
        return CTX._not_connected()
    _L().send_listen()
    return {"ok": True, **_L().telemetry_snapshot()}


def start_recording(name: Optional[str] = None) -> dict:
    """Read-only. Start recording telemetry to recordings/<name>/<name>.csv (name defaults
    to run_<timestamp>): one row per encoder/IMU sample, in the same format as
    calibration/live_view.py --csv, so analyze_csv.py reads it. Works while the stick
    controller drives -- this server sends nothing to the robot unless a drive tool is
    called. Never moves the robot, no confirmation needed. Use add_marker() during the
    recording, stop_recording() when done.

    Also saves commands.csv: every drive command the stick sent (time_s,left,right), if
    the bridge has the firmware that relays them."""
    if _L() is None:
        return CTX._not_connected()
    try:
        path = _L().recorder.start(name)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return {"ok": True, "csv": str(path), **_L().telemetry_snapshot()}


def add_marker(text: str) -> dict:
    """Read-only. While recording, write a MARK row (with the current distance, heading
    and orientation, and `text` in the note column) into the CSV, e.g. "start of ramp" or
    "wheel slipped". Never moves the robot."""
    if _L() is None:
        return CTX._not_connected()
    with _L().lock:
        t = _L().recorder.mark(_L().enc, _L().orient, "MARK", text)
    if t is None:
        return {"ok": False, "error": "not recording; call start_recording() first"}
    return {"ok": True, "time_s": round(t, 3), "note": text}


def stop_recording() -> dict:
    """Read-only. Stop recording and close the CSV. Returns its path, duration and how
    many samples and markers were saved. Never moves the robot."""
    if _L() is None:
        return CTX._not_connected()
    summary = _L().recorder.stop()
    if summary is None:
        return {"ok": False, "error": "not recording"}
    out = {"ok": True, **summary}
    if summary["commands"] == 0:
        out["note"] = ("no stick commands were captured: either the stick wasn't driven, or "
                       "the bridge still has firmware that doesn't relay them (reflash "
                       "firmware/computer_bridge/computer_bridge.ino)")
    return out



ADMIN_TOOLS = (connect, read_telemetry, sensors, read_range, set_required_sensors,
               guards, set_guard, zero_odometry, stop, clear_tilt_fault,
               set_level_reference, listen, start_recording, add_marker, stop_recording)


def register(mcp, ctx):
    global CTX
    CTX = ctx
    for fn in ADMIN_TOOLS:
        mcp.tool()(fn)
