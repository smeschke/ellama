#!/usr/bin/env python3
"""Drive tools: drive_distance, turn_degrees, drive_arc, drive_until_range, jog.

Registered by robot_server.register_tools(); CTX is the robot_server module,
used for the live `link` singleton and the safety gates (_confirm_gate,
_tilt_guarded), which stay there so tests that set `robot_server.link` keep
working exactly as before.
"""

import math
import sys
import threading
import time
from pathlib import Path
from typing import Literal, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import bridge  # noqa: E402
import sensors as sensorlib  # noqa: E402  (unused here; kept for callers that introspect)
from drive_math import *  # noqa: E402,F401,F403
from drive_math import (approach_pwm, frozen_wheel_fault, pwm_for_speed,  # noqa: E402
                        selected_wheels, speed_for_pwm, traveled_inches)

CTX = None


def _L():
    return CTX.link


def drive_distance(
    distance_inches: float,
    direction: Literal["forward", "reverse"],
    pwm: int,
    confirmed_safe: bool,
    encoder: Literal["both", "left", "right"] = "both",
) -> dict:
    """Closed-loop straight-line move: drives at `pwm` (0-255, clamped to
    MAX_PWM_CEILING=125) until the wheel encoders report `distance_inches` traveled, then
    stops.

    `encoder` is which wheel measures the move: "both" (the default) averages them, which
    is the chassis distance; "left" or "right" measures by that wheel alone, which is the
    same distance on a straight line and the only honest answer when the other channel is
    dead. Pick one deliberately -- an average that includes a dead channel rises at half
    rate and drives twice as far as asked, which is why a selected channel that has not
    ticked within ENCODER_FIRST_TICK_S=0.6s aborts the move and says which wheel. With
    encoder="both" that check applies to each wheel, so a frozen channel now refuses
    instead of silently overshooting. Requires a live encoder feed (refuses otherwise) and confirmed_safe=true, set
    only after asking the human operator about THIS specific move, right before making
    it -- see DRIVING_POLICY.md and this file's module docstring for why that gate can't
    be skipped yet. Always stops first and settles before moving, so you never need to
    worry about reversing direction without a stop in between.

    Pick `pwm` deliberately, don't default to a "clearly visible" value: on any setup you
    haven't personally verified, DRIVING_POLICY.md's floor is 5-10% duty (13-26). There
    is no per-call distance cap: a single call will drive as far as it is asked to, so
    the stop-check-decide-move loop is the operator's to keep, not this tool's.

    direction is which way the robot travels; PWM sign is handled internally
    (FORWARD_PWM_SIGN in this file) but hasn't been verified against a live test yet --
    the first real call on new hardware should be a small one specifically to check the
    robot actually moves the way it was asked.

    blocks the move.
    """
    if _L() is None:
        return CTX._not_connected()
    gate = CTX._confirm_gate(confirmed_safe)
    if gate:
        return gate
    if distance_inches <= 0:
        return {"ok": False, "error": "distance_inches must be positive"}
    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    if encoder not in ENCODER_SELECTIONS:
        return {"ok": False, "error": "encoder must be one of %s"
                % ", ".join(ENCODER_SELECTIONS)}

    snap = _L().telemetry_snapshot()
    start = _L().wheel_counts(encoder)
    # Closed loop needs whichever channels `encoder` named to be reporting. Anything less
    # and this becomes a timed move, which the result says plainly.
    closed_loop = snap["telemetry_live"] and start is not None
    predicted_speed = open_loop_s = None
    if not closed_loop:
        predicted_speed = speed_for_pwm(pwm)
        if predicted_speed is None:
            return {"ok": False, "error":
                    "no encoder feedback, and pwm %d is below the measured deadband of %d -- "
                    "the robot would not move and nothing here could tell. Raise pwm, or "
                    "bring the encoders back." % (pwm, PWM_DEADBAND),
                    "telemetry": snap}
        open_loop_s = distance_inches / predicted_speed

    _L().send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = _L().in_per_tick()
    signed_pwm = pwm * (FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN)
    timeout_s = max(4.0, distance_inches * 0.5 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    try:
        _L().send_drive(signed_pwm, signed_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                _L().send_drive(signed_pwm, signed_pwm)
                last_send = time.monotonic()
            if closed_loop:
                cur = _L().wheel_counts(encoder)
                if cur is None:
                    # An encoder dropped out with the wheels turning. Stop rather than
                    # carry on blind towards a target that can no longer be measured.
                    abort_reason = ("an encoder stopped reporting mid-move -- distance can "
                                    "no longer be measured, so the move was cut short")
                    break
                if traveled_inches(start, cur, encoder, in_per_tick) >= distance_inches:
                    break
                frozen = frozen_wheel_fault(start, cur, encoder, in_per_tick,
                                            time.monotonic() - t_start)
                if frozen:
                    # Only suggest the other wheel when there IS one still reporting --
                    # advising encoder="left" when the left channel is the dead one would
                    # send the next call straight back into this fault.
                    other = {"left": "right", "right": "left"}
                    hint = ((" Or pass encoder=\"%s\" to measure by the wheel that is "
                             "reporting." % other[frozen[0]]) if len(frozen) == 1 else "")
                    abort_reason = (
                        "the %s encoder has not ticked once while driving: it is dead, or "
                        "that wheel is held. Either way this move cannot be measured by "
                        "it, so it was cut short rather than driven to a target nothing "
                        "was measuring. Check the encoder (magnet gap, wiring).%s"
                        % (" and ".join(frozen), hint))
                    break
            elif time.monotonic() - t_start >= open_loop_s:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            time.sleep(0.03)
    finally:
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

    elapsed = time.monotonic() - t_start
    end = _L().wheel_counts(encoder)
    measured = start is not None and end is not None
    actual_inches = (traveled_inches(start, end, encoder, in_per_tick)
                     if measured else None)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_inches": distance_inches,
        "direction": direction,
        "pwm_used": pwm,
        "closed_loop": closed_loop,
        "actual_inches_traveled": None if actual_inches is None else round(actual_inches, 2),
        "estimated_inches_traveled": (None if closed_loop or predicted_speed is None
                                      else round(elapsed * predicted_speed, 2)),
        "measured_by": encoder,
        "distance_source": (("both wheel encoders, averaged" if encoder == "both"
                             else "the %s wheel encoder only" % encoder) if measured else
                            "PREDICTED from the measured PWM-speed curve, not measured -- "
                            "no encoder feedback on this move"),
        "elapsed_s": round(elapsed, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "telemetry": _L().telemetry_snapshot(),
    }


def turn_degrees(
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
) -> dict:
    """Closed-loop spin-in-place: drives the wheels in opposite directions at `pwm`
    (0-255, clamped to MAX_PWM_CEILING=125) until the IMU's gyro-integrated yaw reports
    `degrees` turned, then stops. Same confirmed_safe gate and stop-before-moving
    behavior as drive_distance -- see its docstring and DRIVING_POLICY.md.

    The IMU is the ONLY thing this tool will turn by, and it refuses outright without a
    live one -- there is deliberately no encoder fallback. Verified 2026-09-27 (robot on
    the ground, four 90deg turns each way): the encoder-differential estimate (dr-dl over
    track_width_in) read ~92 deg per commanded 90deg turn while the robot visually rotated
    only ~47 deg -- wheel scrub during an in-place spin makes the wheels turn much more
    than the chassis does. A 2026-09-29 re-run on the same floor measured ~1.7x rather
    than ~2x, which is the point: the error moves with the surface, so no fixed correction
    recovers it. The IMU yaw delta over those turns averaged ~49 deg, matching the ~47 deg
    observed by eye. On this robot the encoders measure arcs and straight lines; they do
    not measure spins. The response still includes encoder_degrees_estimate for reference,
    explicitly labeled unreliable, but nothing reads it.

    There is no per-call cap on how far it will turn. direction "left" is
    counter-clockwise viewed from above, matching bridge/'s heading
    convention.

    blocks the move.
    """
    if _L() is None:
        return CTX._not_connected()
    gate = CTX._confirm_gate(confirmed_safe)
    if gate:
        return gate
    if degrees <= 0:
        return {"ok": False, "error": "degrees must be positive"}
    snap = _L().telemetry_snapshot()
    start = _L().last_counts()
    # The IMU is the only thing allowed to measure a spin in place. The encoders are not a
    # fallback for this: wheel scrub makes them over-read an in-place turn by ~2x, and by a
    # ratio that moves with the surface, so there is no fixed divisor that fixes it.
    if not snap["imu_live"]:
        return {
            "ok": False,
            "error": "no live IMU, so nothing can measure this turn. The wheel encoders are "
            "not a substitute for a spin in place -- scrub makes them over-read it by ~2x, "
            "by a factor that changes with the surface. Bring the IMU back, or use "
            "jog(left_pwm, right_pwm, duration_s) for a timed turn you watch yourself.",
            "telemetry": snap,
        }
    feedback_source = "imu"

    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    if pwm == 0:
        return {"ok": False, "error": "pwm must be > 0"}

    _L().send_stop()
    time.sleep(STOP_SETTLE_S)

    start = _L().last_counts()
    in_per_tick = _L().in_per_tick()
    track_width = _L().cfg["track_width_in"]
    with _L().lock:
        start_yaw = _L().orient.yaw

    # "left" (CCW, nose swings left) needs the right wheel driving real-forward and the
    # left wheel driving real-backward -- both relative to FORWARD_PWM_SIGN, not raw PWM
    # sign, the same correction drive_distance applies.
    sign = 1 if direction == "left" else -1
    left_pwm = -sign * FORWARD_PWM_SIGN * pwm
    right_pwm = sign * FORWARD_PWM_SIGN * pwm
    timeout_s = max(4.0, degrees * 0.1 + 3.0)

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    try:
        _L().send_drive(left_pwm, right_pwm)
        last_send = time.monotonic()
        while True:
            if time.monotonic() - last_send >= DRIVE_KEEPALIVE_S:
                _L().send_drive(left_pwm, right_pwm)
                last_send = time.monotonic()
            with _L().lock:
                current_yaw = _L().orient.yaw
            turned_so_far = abs(current_yaw - start_yaw)
            if turned_so_far >= degrees:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            time.sleep(0.03)
    finally:
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

    end = _L().last_counts()
    with _L().lock:
        end_yaw = _L().orient.yaw
    if end is not None and start is not None:
        dl = (end[0] - start[0]) * in_per_tick
        dr = (end[1] - start[1]) * in_per_tick
        enc_deg = math.degrees((dr - dl) / track_width)
    else:
        dl = dr = enc_deg = None

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_degrees": degrees,
        "direction": direction,
        "pwm_used": pwm,
        "feedback_source": feedback_source,
        "actual_degrees_turned": round(end_yaw - start_yaw, 1),
        "measurement_note": "IMU gyro-integrated yaw -- the only source this tool will "
                            "turn by; the encoder figure below is reported for reference "
                            "and never steers the move",
        "imu_degrees": round(end_yaw - start_yaw, 1),
        "encoder_degrees_estimate": None if enc_deg is None else round(enc_deg, 1),
        "encoder_estimate_unreliable": "wheel scrub during in-place turns inflates this "
        "~2x vs. the IMU/real rotation -- see this tool's docstring",
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "track_width_calibrated": _L().track_width_calibrated,
        "telemetry": _L().telemetry_snapshot(),
    }


def drive_arc(
    radius_inches: float,
    degrees: float,
    direction: Literal["left", "right"],
    pwm: int,
    confirmed_safe: bool,
    max_wheel_pwm: int = MAX_PWM_CEILING,
    travel: Literal["forward", "reverse"] = "forward",
    encoder: Literal["both", "left", "right"] = "both",
) -> dict:
    """Closed-loop curved drive: both wheels roll forward (or backward, see `travel`), the
    outer one faster, so the robot drives an arc of `radius_inches` (measured to the robot's center) until the
    IMU's gyro-integrated yaw reports `degrees` turned, then stops. degrees=360 drives a
    full circle. Same confirmed_safe gate, live-telemetry requirement and
    stop-before-moving behavior as drive_distance -- see DRIVING_POLICY.md.

    `pwm` is the average wheel command; the outer wheel runs at pwm + d (never above
    MAX_PWM_CEILING) and the inner at pwm - d. d starts from the nominal track-width
    geometry, capped so the inner wheel starts at or above zero, then every
    ARC_CONTROL_DT_S the measured curvature (IMU yaw change / signed encoder path length)
    is compared with 1/radius and d is nudged to match, by at most ARC_MAX_STEP_PWM per
    step -- skid-steer wheel scrub makes the geometric split alone wrong, so the loop,
    not the geometry, sets the real radius. For radii tighter than the robot manages with
    the inner wheel stopped (~12.5in), the loop eases the inner wheel through zero into
    reverse, down to -ARC_MAX_INNER_REVERSE_PWM. Aborts immediately if encoder or IMU
    telemetry goes stale mid-move.

    direction uses the same wheel mapping as turn_degrees. Capped at
    MAX_SINGLE_ARC_DEG=360 and MAX_SINGLE_ARC_IN=160 of path per call, radius at least
    MIN_ARC_RADIUS_IN=6.

    `max_wheel_pwm` (default MAX_PWM_CEILING) caps the OUTER wheel's PWM for the whole
    move, including anything the curvature loop asks for -- use it to honor an operator's
    "never above N PWM" limit, since `pwm` is only the average and the outer wheel runs
    above it. The average is reduced if needed so the geometric starting split fits
    under the cap.

    `encoder` is which wheel the curvature loop measures its path with, as in
    drive_distance. One wheel is enough here, but it is not on the arc you asked for: the
    outer wheel runs a radius half a track wider than the robot's centre and the inner one
    half a track tighter, so the loop compares the measured curvature against 1/(radius +-
    half track) for that wheel instead of 1/radius, and the reported path is scaled back to
    the centre. That needs the selected wheel's own radius to be a real arc, so a single
    encoder is refused when the inner wheel would be pivoting inside MIN_ARC_RADIUS_IN.

    `travel="reverse"` drives the same arc backing up: both wheels roll backward and the
    robot's heading still turns `direction` (left = counter-clockwise seen from above), so
    the outer wheel is the LEFT one for "left" (the opposite of a forward arc) and the
    rear sweeps out along the path. Nothing on the robot senses what is behind it, so only
    back up into space the operator has said is clear. Forward then reverse arcs of the same
    direction trace an S-curve: the robot ends facing the opposite way, displaced along its
    original axis by twice the radius and back on the same line laterally.

    blocks the move.
    """
    if _L() is None:
        return CTX._not_connected()
    gate = CTX._confirm_gate(confirmed_safe)
    if gate:
        return gate
    if radius_inches < MIN_ARC_RADIUS_IN:
        return {"ok": False, "error": f"radius_inches must be >= {MIN_ARC_RADIUS_IN} -- "
                "use turn_degrees to spin in place"}
    if not (0 < degrees <= MAX_SINGLE_ARC_DEG):
        return {"ok": False, "error": f"degrees must be > 0 and <= {MAX_SINGLE_ARC_DEG}"}
    arc_len = radius_inches * math.radians(degrees)
    if arc_len > MAX_SINGLE_ARC_IN:
        return {"ok": False, "error": f"arc length {arc_len:.1f}in exceeds the "
                f"{MAX_SINGLE_ARC_IN}in per-call cap -- use a smaller radius or fewer degrees"}

    snap = _L().telemetry_snapshot()
    if not snap["telemetry_live"]:
        return {"ok": False, "error": "no live encoder telemetry -- drive_arc steers by "
                "comparing wheel path against yaw, and without the encoders there is no "
                "path to compare. Use jog(left_pwm, right_pwm, duration_s) for a timed "
                "curve you watch yourself.", "telemetry": snap}
    if not snap["imu_live"]:
        # Not a degraded mode: the whole tool is "curve until the yaw says `degrees`".
        # Without yaw there is no target to drive to and no curvature loop to run, and a
        # fixed wheel split for a fixed time is exactly what jog already does.
        return {"ok": False, "error": "no live IMU -- drive_arc has no way to tell how far "
                "around the arc it has gone, and its curvature loop needs yaw to correct "
                "for wheel scrub. Use jog(left_pwm, right_pwm, duration_s) for a timed "
                "curve, or turn_degrees for a spin in place.", "telemetry": snap}

    wheel_cap = min(abs(int(max_wheel_pwm)), MAX_PWM_CEILING)
    pwm = min(abs(int(pwm)), wheel_cap)
    if pwm == 0:
        return {"ok": False, "error": "pwm and max_wheel_pwm must be > 0"}
    if encoder not in ENCODER_SELECTIONS:
        return {"ok": False, "error": "encoder must be one of %s"
                % ", ".join(ENCODER_SELECTIONS)}

    half_track = _L().cfg["track_width_in"] / 2.0
    travel_sign = 1 if travel == "forward" else -1
    # Forward: "left" = right wheel on the outside, matching turn_degrees' mapping.
    # Reverse: heading turns the same way, so the outside wheel swaps sides.
    outer_is_right = (direction == "left") == (travel == "forward")

    # The wheel doing the measuring runs its own arc, concentric with the robot's but
    # wider (outer) or tighter (inner) by half a track. Measuring one wheel against
    # 1/radius would make the loop chase a curvature that wheel never drives.
    if encoder == "both":
        sel_radius = radius_inches
    else:
        sel_is_outer = (encoder == "right") == outer_is_right
        sel_radius = radius_inches + (half_track if sel_is_outer else -half_track)
        if sel_radius < MIN_ARC_RADIUS_IN:
            return {"ok": False, "error":
                    "radius_inches %.1f puts the %s wheel on a %.1fin arc, inside the "
                    "%.0fin minimum -- that wheel is pivoting, not tracing the curve, so "
                    "its path cannot measure one. Use encoder=\"%s\" (the outer wheel), "
                    "a wider radius, or encoder=\"both\"."
                    % (radius_inches, encoder, sel_radius, MIN_ARC_RADIUS_IN,
                       "right" if encoder == "left" else "left"),
                    "telemetry": snap}

    _L().send_stop()
    time.sleep(STOP_SETTLE_S)

    in_per_tick = _L().in_per_tick()
    diff = pwm * min(1.0, half_track / radius_inches)
    target_curv = 1.0 / sel_radius  # rad per inch of the measuring wheel's path

    def forward_path_in(a, b):
        # Signed along the direction of travel, so a wheel running against it (the inner
        # wheel of a tight arc) subtracts from the path instead of adding.
        idx = selected_wheels(encoder)
        fwd = [(b[i] - a[i]) * ENCODER_FORWARD_SIGN[i] for i in idx]
        return travel_sign * sum(fwd) / len(fwd) * in_per_tick

    # Path of the measuring wheel, scaled to the robot's centre for reporting.
    center_scale = radius_inches / sel_radius

    max_diff = pwm + ARC_MAX_INNER_REVERSE_PWM

    def wheel_cmd(d):
        outer = min(wheel_cap, pwm + d)
        inner = max(-ARC_MAX_INNER_REVERSE_PWM, pwm - d)
        l_real, r_real = (inner, outer) if outer_is_right else (outer, inner)
        return (FORWARD_PWM_SIGN * travel_sign * l_real,
                FORWARD_PWM_SIGN * travel_sign * r_real)

    start = _L().wheel_counts(encoder)
    if start is None:
        # telemetry_live only says a packet arrived; the channel this move measures by can
        # still be unavailable.
        return {"ok": False, "error":
                ("the %s encoder is not reporting -- drive_arc measures its path with it. "
                 "Check encoders_available in read_telemetry(), or pass encoder=\"left\" "
                 "/ \"right\" to measure with the wheel that is."
                 % ("left or right" if encoder == "both" else encoder)),
                "telemetry": snap}
    with _L().lock:
        start_yaw = _L().orient.yaw
    timeout_s = arc_len / 2.0 + 5.0  # generous: ~2 in/s average, well under PWM-50 speed

    t_start = time.monotonic()
    hit_timeout = False
    abort_reason = None
    win_counts, win_yaw = start, start_yaw
    last_ctrl = t_start
    try:
        _L().send_drive(*wheel_cmd(diff))
        while True:
            now = time.monotonic()
            with _L().lock:
                cur_yaw = _L().orient.yaw
                enc_age = None if _L().last_enc_wall is None else now - _L().last_enc_wall
                imu_age = None if _L().last_imu_wall is None else now - _L().last_imu_wall
            if abs(cur_yaw - start_yaw) >= degrees:
                break
            if enc_age is None or enc_age > TELEMETRY_STALE_S or imu_age is None \
                    or imu_age > TELEMETRY_STALE_S:
                abort_reason = "telemetry went stale mid-arc"
                break
            if now - t_start > timeout_s:
                hit_timeout = True
                break
            if now - last_ctrl >= ARC_CONTROL_DT_S:
                cur = _L().wheel_counts(encoder)
                if cur is None:
                    abort_reason = "an encoder stopped reporting mid-arc"
                    break
                path = forward_path_in(win_counts, cur)
                if path > 0.3:  # enough travel for a meaningful curvature estimate
                    curv = math.radians(abs(cur_yaw - win_yaw)) / path
                    step = ARC_GAIN * pwm * (target_curv - curv) / target_curv
                    step = max(-ARC_MAX_STEP_PWM, min(ARC_MAX_STEP_PWM, step))
                    diff = max(0.0, min(float(max_diff), diff + step))
                    win_counts, win_yaw = cur, cur_yaw
                _L().send_drive(*wheel_cmd(diff))  # also the bridge keepalive
                last_ctrl = now
            time.sleep(0.03)
    finally:
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

    end = _L().wheel_counts(encoder)
    with _L().lock:
        end_yaw = _L().orient.yaw
    wheel_path_in = None if (start is None or end is None) else forward_path_in(start, end)
    path_in = None if wheel_path_in is None else wheel_path_in * center_scale
    turned = end_yaw - start_yaw
    eff_radius = (path_in / math.radians(abs(turned))
                  if path_in is not None and abs(turned) > 1 else None)
    final_l, final_r = wheel_cmd(diff)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "requested_radius_in": radius_inches,
        "requested_degrees": degrees,
        "direction": direction,
        "travel": travel,
        "pwm_used": pwm,
        "actual_degrees_turned": round(turned, 1),
        "path_length_in": None if path_in is None else round(path_in, 2),
        "measured_by": encoder,
        "measuring_wheel_radius_in": round(sel_radius, 2),
        "effective_radius_in": None if eff_radius is None else round(eff_radius, 2),
        "final_wheel_pwm": {"left": round(final_l), "right": round(final_r)},
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "telemetry": _L().telemetry_snapshot(),
    }


def drive_until_range(target_mm: float, pwm: int, confirmed_safe: bool,
                      facing: Literal["auto", "forward", "reverse"] = "auto",
                      sensor: Optional[str] = None, tolerance_mm: float = 15.0,
                      min_pwm: int = PWM_DEADBAND + 5) -> dict:
    """Closed-loop approach: drive straight toward whatever a ToF sensor is looking at
    until its median range reads `target_mm`, then stop. The loop closes on the range
    sensor, not the encoders, so it works with the wheel encoders dead.

    It drives in the direction the sensor FACES, because that is the only direction it can
    measure. With facing="auto" that is read off the sensor's name -- TOF_REAR means the
    robot backs up, anything else means it goes forward -- so a board mounted on the back
    but flashed as "TOF_FRONT" will send the robot the wrong way. Pass
    facing="forward"/"reverse" to say which way the board really points; the result always
    reports what it decided and why.

    Two phases, because the sensor is slow relative to the robot:

      1. Continuous, tapering. The number the loop steers by is a median over
         TofState.HISTORY samples at ~5 Hz, so it lags; RANGE_LATENCY_S folds that lag,
         one sample interval and the braking lead together, and the taper never commands
         more speed than stops inside the gap that is left (see approach_pwm).
      2. Pulses, once even min_pwm is too fast to stop in time. Short bursts at min_pwm,
         each followed by a full stop and RANGE_MEDIAN_REFRESH_S of new samples so the
         median is actually about where the robot now is. This is the stop-check-decide
         loop from DRIVING_POLICY.md section 5, which is what the last few centimetres
         need: the shortest burst this drivetrain can make still moves it ~20mm.

    It stops short rather than overshoot. `tolerance_mm` (default 15) is close enough, and
    if the shortest possible pulse would carry it past the target it stops where it is and
    says so in `stopped_short`. Two narrow cones at a fixed height are not clearance --
    they cannot see the floor, a drop, the sides, or anything below the beam -- so
    confirmed_safe still means the operator looked.

    Aborts, and says which: the sensor stops reporting, its median goes out of range (the
    target was lost -- that is blind, not clear), nothing gets closer for a few seconds
    (something is in the way, or the beam is skimming a surface at too shallow an angle),
    or RANGE_MAX_PULSES is reached.
    """
    if _L() is None:
        return CTX._not_connected()
    gate = CTX._confirm_gate(confirmed_safe)
    if gate:
        return gate
    if target_mm < RANGE_MIN_TARGET_MM:
        return {"ok": False, "error":
                "target_mm must be at least %.0f -- closer than that is inside the robot's "
                "own bumper on most mountings, and VL53L0X accuracy falls apart in the "
                "last few centimetres." % RANGE_MIN_TARGET_MM}
    if tolerance_mm < 5:
        return {"ok": False, "error": "tolerance_mm must be at least 5 -- the channels "
                "routinely disagree by more than that about where a surface is"}
    pwm = min(abs(int(pwm)), MAX_PWM_CEILING)
    min_pwm = min(abs(int(min_pwm)), MAX_PWM_CEILING)
    if min_pwm < PWM_DEADBAND:
        return {"ok": False, "error":
                "min_pwm %d is below the measured deadband of %d, where the robot does not "
                "move at all -- the approach would stall out short of the target and look "
                "like an obstruction." % (min_pwm, PWM_DEADBAND)}
    if pwm < min_pwm:
        return {"ok": False, "error": "pwm must be >= min_pwm"}

    rng = _L().range_snapshot()
    if rng is None:
        return {"ok": False, "error": "no range sensor has ever reported -- there is "
                "nothing to close the loop on. sensors() shows what is transmitting.",
                "telemetry": _L().telemetry_snapshot()}

    names = sorted({c["sensor"] for c in rng["channels"].values()})
    if sensor is not None:
        if sensor not in names:
            return {"ok": False, "error": "no range sensor named %r is reporting (have: %s)"
                    % (sensor, ", ".join(names)), "range": rng}
        sensor_name = sensor
    elif len(names) == 1:
        sensor_name = names[0]
    else:
        return {"ok": False, "error":
                "several range sensors are reporting (%s) and they point different ways -- "
                "pass sensor= to say which one this move is closing on." % ", ".join(names),
                "range": rng}

    if facing == "auto":
        direction = "reverse" if sensor_name.upper().endswith("REAR") else "forward"
        facing_source = ("the sensor's name (%s), not a measurement -- a board mounted the "
                         "other way round will drive the robot the wrong way"
                         % sensor_name)
    else:
        direction = facing
        facing_source = "the caller said so"

    def read_gap():
        """(closest median mm on this sensor, is it live) -- the number to steer by."""
        snap = _L().range_snapshot()
        chans = [c for c in snap["channels"].values() if c["sensor"] == sensor_name]
        medians = [c["mm"] for c in chans if c["mm"] is not None]
        return (min(medians) if medians else None), any(c["live"] for c in chans)

    gap, live = read_gap()
    if not live:
        return {"ok": False, "error": "%s is not reporting right now -- that is blind, not "
                "clear." % sensor_name, "range": rng}
    if gap is None:
        return {"ok": False, "error":
                "%s is reporting but has nothing in range, so there is nothing to approach. "
                "Point the robot at the target first (this is 'nothing within reach', not "
                "'the way is clear')." % sensor_name, "range": rng}
    if gap - target_mm <= tolerance_mm:
        return {"ok": True, "moved": False, "target_mm": target_mm, "final_mm": gap,
                "final_in": round(gap / bridge.MM_PER_IN, 1),
                "note": "already within tolerance of the target -- nothing to do",
                "range": _L().range_snapshot(), "telemetry": _L().telemetry_snapshot()}

    sign = FORWARD_PWM_SIGN if direction == "forward" else -FORWARD_PWM_SIGN
    start_gap = gap
    # Generous: the continuous phase at min_pwm plus every pulse and its settle.
    timeout_s = ((start_gap - target_mm) / bridge.MM_PER_IN / speed_for_pwm(min_pwm)
                 + RANGE_MAX_PULSES * (RANGE_MAX_PULSE_S + RANGE_MEDIAN_REFRESH_S) + 5.0)

    _L().send_stop()
    time.sleep(STOP_SETTLE_S)

    start_counts = _L().wheel_counts("both") or _L().wheel_counts("left") \
        or _L().wheel_counts("right")
    in_per_tick = _L().in_per_tick()
    t_start = time.monotonic()
    abort_reason = None
    hit_timeout = stopped_short = False
    pulses = 0
    best_gap = gap
    best_t = t_start
    pwm_log = []

    try:
        # --- phase 1: continuous, tapering as the gap closes ---------------------------
        while True:
            now = time.monotonic()
            gap, live = read_gap()
            if not live:
                abort_reason = "%s stopped reporting mid-approach" % sensor_name
                break
            if gap is None:
                abort_reason = ("%s lost the target mid-approach -- its median went out of "
                                "range, which is blind, not clear" % sensor_name)
                break
            if gap - target_mm <= tolerance_mm:
                break
            if now - t_start > timeout_s:
                hit_timeout = True
                break
            if gap < best_gap - 2:
                best_gap, best_t = gap, now
            elif now - best_t > RANGE_NO_PROGRESS_S:
                abort_reason = ("nothing got closer for %.1fs while driving: something is "
                                "in the way, a wheel is not turning, or the beam is "
                                "skimming the surface too obliquely to read"
                                % RANGE_NO_PROGRESS_S)
                break
            want = approach_pwm(gap - target_mm, pwm, min_pwm)
            if want is None:
                break  # too close to drive continuously; finish in pulses
            pwm_log.append(want)
            _L().send_drive(sign * want, sign * want)
            time.sleep(DRIVE_KEEPALIVE_S)
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

        # --- phase 2: pulse, stop, let the median catch up, look again -----------------
        while abort_reason is None and not hit_timeout:
            time.sleep(RANGE_MEDIAN_REFRESH_S)
            gap, live = read_gap()
            if not live:
                abort_reason = "%s stopped reporting between pulses" % sensor_name
                break
            if gap is None:
                abort_reason = ("%s lost the target between pulses -- its median went out "
                                "of range" % sensor_name)
                break
            remaining = gap - target_mm
            if remaining <= tolerance_mm:
                break
            if time.monotonic() - t_start > timeout_s:
                hit_timeout = True
                break
            if pulses >= RANGE_MAX_PULSES:
                abort_reason = ("still %.0fmm short of the target after %d pulses -- "
                                "stopping rather than nibbling at it indefinitely"
                                % (remaining, pulses))
                break
            creep_in_s = speed_for_pwm(min_pwm)
            burst_s = (remaining / bridge.MM_PER_IN) / creep_in_s
            if burst_s < RANGE_MIN_PULSE_S:
                # The shortest burst this drivetrain makes would carry it past the target.
                # Stopping short is the right error to make.
                stopped_short = True
                break
            pulses += 1
            pwm_log.append(min_pwm)
            burst_s = min(RANGE_MAX_PULSE_S, burst_s)
            burst_start = time.monotonic()
            while time.monotonic() - burst_start < burst_s:
                _L().send_drive(sign * min_pwm, sign * min_pwm)
                time.sleep(min(DRIVE_KEEPALIVE_S,
                               max(0.0, burst_s - (time.monotonic() - burst_start))))
            _L().send_stop()
            time.sleep(STOP_SETTLE_S)
    finally:
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

    time.sleep(RANGE_MEDIAN_REFRESH_S)  # so final_mm is a settled median, not a lagging one
    final_gap, _ = read_gap()
    end_counts = _L().wheel_counts("both") or _L().wheel_counts("left") \
        or _L().wheel_counts("right")
    traveled = None
    if start_counts is not None and end_counts is not None:
        moved = [abs(e - s) for s, e in zip(start_counts, end_counts)
                 if s is not None and e is not None]
        if moved:
            traveled = round(max(moved) * in_per_tick, 2)

    return {
        "ok": not hit_timeout and abort_reason is None,
        "moved": True,
        "target_mm": target_mm,
        "tolerance_mm": tolerance_mm,
        "sensor": sensor_name,
        "direction": direction,
        "facing_from": facing_source,
        "start_mm": start_gap,
        "final_mm": final_gap,
        "final_in": None if final_gap is None else round(final_gap / bridge.MM_PER_IN, 1),
        "within_tolerance": (final_gap is not None
                             and final_gap - target_mm <= tolerance_mm),
        "stopped_short": stopped_short,
        "stopped_short_why": ("the shortest burst this drivetrain can make (%.2fs at pwm "
                              "%d) would have carried it past the target"
                              % (RANGE_MIN_PULSE_S, min_pwm)) if stopped_short else None,
        "pulses": pulses,
        "pwm_used": {"max": max(pwm_log) if pwm_log else None,
                     "min": min(pwm_log) if pwm_log else None},
        "wheel_travel_in": traveled,
        "range_closed_mm": None if final_gap is None else round(start_gap - final_gap, 1),
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "hit_timeout": hit_timeout,
        "abort_reason": abort_reason,
        "range": _L().range_snapshot(),
        "telemetry": _L().telemetry_snapshot(),
    }


def jog(left_pwm: int, right_pwm: int, duration_s: float, confirmed_safe: bool,
        require_telemetry: Optional[bool] = None) -> dict:
    """Raw open-loop command: drive at exactly (left_pwm, right_pwm) for duration_s
    seconds, then stop. No encoder target and no direction-aware sign correction (unlike
    drive_distance/turn_degrees) -- positive does not necessarily mean forward, see
    FORWARD_PWM_SIGN. This is for bench/commissioning tests where a fixed, observable
    duration matters more than hitting a distance or angle (checking direction, deadband,
    turn convention by eye). Same confirmed_safe gate as the other drive tools. Capped at
    MAX_PWM_CEILING=125 per side; duration_s has no upper bound, so an open-loop call runs
    for exactly as long as it is given.

    blocks the move.

    require_telemetry defaults to the server's sensor policy (set_required_sensors): it
    insists on live encoders only if "enc" is in the required set, which it is by default.
    Pass True or False to override for one call -- False ONLY for a bench test the operator
    is watching (e.g. robot on blocks while the encoder link is down): it still needs
    confirmed_safe, and the distances come back as null.
    """
    if _L() is None:
        return CTX._not_connected()
    gate = CTX._confirm_gate(confirmed_safe)
    if gate:
        return gate
    if duration_s <= 0:
        return {"ok": False, "error": "duration_s must be > 0"}

    snap = _L().telemetry_snapshot()
    if require_telemetry is None:
        require_telemetry = "enc" in _L().policy.required
    if require_telemetry and not snap["telemetry_live"]:
        return {
            "ok": False,
            "error": "no live encoder telemetry -- refusing to drive without feedback. "
                     "Pass require_telemetry=false for a watched bench test, or call "
                     "set_required_sensors() to drop \"enc\" for this run.",
            "telemetry": snap,
        }

    _L().send_stop()
    time.sleep(STOP_SETTLE_S)

    start = _L().last_counts()
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < duration_s:
            _L().send_drive(left_pwm, right_pwm)
            time.sleep(min(DRIVE_KEEPALIVE_S, max(0.0, duration_s - (time.monotonic() - t_start))))
    finally:
        _L().send_stop()
        time.sleep(STOP_SETTLE_S)

    end = _L().last_counts()
    in_per_tick = _L().in_per_tick()
    have_counts = start is not None and end is not None
    return {
        "ok": True,
        "left_pwm": left_pwm,
        "right_pwm": right_pwm,
        "duration_s": duration_s,
        "dist_l_in": round((end[0] - start[0]) * in_per_tick, 2) if have_counts else None,
        "dist_r_in": round((end[1] - start[1]) * in_per_tick, 2) if have_counts else None,
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "telemetry": _L().telemetry_snapshot(),
    }



DRIVE_TOOLS = (drive_distance, turn_degrees, drive_arc, drive_until_range, jog)


def register(mcp, ctx):
    global CTX
    CTX = ctx
    for fn in DRIVE_TOOLS:
        mcp.tool()(ctx._tilt_guarded(fn))
