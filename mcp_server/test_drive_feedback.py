#!/usr/bin/env python3
"""Tests for which feedback a drive closes its loop on.

No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_drive_feedback.py

Two things are under test. First, `encoder=` on drive_distance and drive_arc: measuring a
move by one nominated wheel, and refusing a move whose measuring channel is frozen instead
of driving to the timeout (encoder="left") or to twice the distance asked (encoder="both",
where a dead channel halves the average). That second case is not hypothetical -- it is
what this robot did on 2026-10-22 with one encoder frozen at a constant count.

Second, the arithmetic drive_until_range approaches with: the inverse of the PWM-speed
curve, and the taper that keeps the overshoot inside the gap rather than inside the wall.
The ToF median lags by half its window while the robot is moving, which is the whole
reason that tool stops and pulses at the end instead of creeping continuously.
"""

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sensors as sensorlib  # noqa: E402
import robot_server as rs  # noqa: E402
from test_sensors import make_link, feed_enc  # noqa: E402

bridge = rs.bridge
INVALID = -1


def bench_link():
    """A link nothing is required of, so the drive tools run without live sensors."""
    link = make_link()
    link.policy = sensorlib.SensorPolicy([])
    rs.link = link
    return link


def feed_imu(link, ms, yaw_deg=None):
    # last_imu_wall/last_enc_wall are properties over the registry, so observing is
    # what makes a stream recent -- there is nothing to set by hand.
    link.sensors.observe("imu", (0, 0, 16384, 0, 0, 0), ms)
    if yaw_deg is not None:
        with link.lock:
            link.orient.yaw = yaw_deg


class Feeder:
    """Feeds telemetry in the background, like the serial pump does.

    per_packet is (left, right) counts added per packet, so (K, 0) is a robot driving with
    its right encoder frozen -- the shape of a dead channel, which is NOT the same as a
    channel that stops reporting (that is None, tested separately). yaw_dps turns the IMU
    as well, which is the only thing an arc measures its progress by."""

    def __init__(self, link, per_packet=(0, 0), yaw_dps=0.0, period_s=0.02):
        self.link, self.per_packet, self.period_s = link, per_packet, period_s
        self.yaw_dps = yaw_dps
        self.counts = [0, 0]
        self.yaw = 0.0
        self.ms = 0
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _packet(self):
        self.ms += int(self.period_s * 1000)
        feed_enc(self.link, self.counts[0], self.counts[1], self.ms)
        self.yaw += self.yaw_dps * self.period_s
        feed_imu(self.link, self.ms, self.yaw)

    def _run(self):
        while not self._stop.is_set():
            for i in (0, 1):
                self.counts[i] += self.per_packet[i]
            self._packet()
            time.sleep(self.period_s)

    def __enter__(self):
        # Two priming packets: one sample is provisional in the registry (a torn serial
        # line looks like a new sensor), so a stream is not "live" until the second.
        self._packet()
        self._packet()
        self.thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self.thread.join(timeout=2)


def ticks_for(link, inches):
    return int(inches / link.in_per_tick())


# ---- the selection arithmetic ----------------------------------------------------------

def test_both_averages_and_one_wheel_stands_alone():
    # 10 ticks on one wheel, 0 on the other: the average is 5, the left wheel is 10.
    assert rs.traveled_inches((0, 0), (10, 0), "both", 1.0) == 5.0
    assert rs.traveled_inches((0, 0), (10, 0), "left", 1.0) == 10.0
    assert rs.traveled_inches((0, 0), (10, 0), "right", 1.0) == 0.0


def test_travel_is_unsigned_so_a_reversed_encoder_still_counts_up():
    # ENCODER_FORWARD_SIGN has flipped on this robot before; a move measured as negative
    # distance would never reach its target and would run to the timeout.
    assert rs.traveled_inches((0, 0), (-10, -10), "both", 1.0) == 10.0
    assert rs.traveled_inches((0, 0), (-10, 0), "left", 1.0) == 10.0


def test_frozen_wheels_only_reports_the_selected_ones():
    assert rs.frozen_wheels((0, 0), (10, 0), "both") == ["right"]
    assert rs.frozen_wheels((0, 0), (10, 0), "left") == []
    assert rs.frozen_wheels((0, 0), (10, 0), "right") == ["right"]
    assert rs.frozen_wheels((0, 0), (0, 0), "both") == ["left", "right"]


def test_a_frozen_channel_is_a_fault_as_soon_as_the_wheels_disagree():
    # The disagreement trigger: no timer has expired, but one selected wheel has covered
    # FROZEN_COMPARE_IN while the other has not moved at all.
    enough = int(rs.FROZEN_COMPARE_IN / 0.001) + 1
    assert rs.frozen_wheel_fault((0, 0), (enough, 0), "both", 0.001, 0.0) == ["right"]
    # Not yet: a move that has barely started is not a fault.
    assert rs.frozen_wheel_fault((0, 0), (10, 0), "both", 0.001, 0.0) == []


def test_nothing_ticking_at_all_is_caught_by_the_timer():
    # Nothing to compare against, so only elapsed time can tell -- this is the move that
    # would otherwise run to its timeout.
    assert rs.frozen_wheel_fault((0, 0), (0, 0), "left", 0.001, 0.0) == []
    late = rs.ENCODER_FIRST_TICK_S + 0.01
    assert rs.frozen_wheel_fault((0, 0), (0, 0), "left", 0.001, late) == ["left"]
    assert rs.frozen_wheel_fault((0, 0), (0, 0), "both", 0.001, late) == ["left", "right"]


def test_wheel_counts_needs_only_the_channel_it_was_asked_for():
    link = bench_link()
    feed_enc(link, 1000, None, 10)      # right encoder not responding
    assert link.last_counts() is None, "both-wheel reading must still refuse"
    assert link.wheel_counts("left") == (1000, None)
    assert link.wheel_counts("both") is None
    assert link.wheel_counts("right") is None


# ---- the inverse of the PWM-speed curve ------------------------------------------------

def test_pwm_for_speed_inverts_speed_for_pwm():
    for pwm in range(rs.PWM_DEADBAND, rs.MAX_PWM_CEILING + 1):
        back = rs.pwm_for_speed(rs.speed_for_pwm(pwm))
        assert back == pwm, (pwm, back, rs.speed_for_pwm(pwm))


def test_pwm_for_speed_never_exceeds_the_speed_asked_for():
    for speed in (4.5, 6.0, 7.3, 11.1, 14.0):
        pwm = rs.pwm_for_speed(speed)
        assert rs.speed_for_pwm(pwm) <= speed + 1e-9, (speed, pwm)


def test_a_creep_slower_than_the_deadband_has_no_pwm():
    # The useful answer: it means "stop and pulse", not "drive at 1 PWM".
    assert rs.pwm_for_speed(rs.speed_for_pwm(rs.PWM_DEADBAND) - 0.1) is None
    assert rs.pwm_for_speed(0) is None
    assert rs.pwm_for_speed(-5) is None


def test_approach_tapers_as_the_gap_closes():
    wide = rs.approach_pwm(2000, 125, 50)
    mid = rs.approach_pwm(150, 125, 50)
    assert wide is not None and mid is not None
    assert wide > mid, (wide, mid)
    assert wide <= 125 and mid >= 50


def test_approach_hands_over_to_pulses_when_it_cannot_stop_in_time():
    # Close enough that even the slowest continuous speed would overshoot: the loop must
    # be told to stop, not handed a PWM it cannot brake from.
    assert rs.approach_pwm(10, 125, 50) is None


def test_approach_never_commands_a_speed_it_cannot_stop_within():
    for remaining_mm in (60, 120, 300, 900, 2500):
        pwm = rs.approach_pwm(remaining_mm, 125, 50)
        if pwm is None:
            continue
        blind_travel_mm = (rs.speed_for_pwm(pwm) * rs.RANGE_LATENCY_S) * bridge.MM_PER_IN
        assert blind_travel_mm <= remaining_mm, (remaining_mm, pwm, blind_travel_mm)


# ---- drive_distance, closing on the channel it was told to ------------------------------

def test_a_move_measured_by_one_wheel_completes_with_the_other_dead():
    link = bench_link()
    with Feeder(link, per_packet=(ticks_for(link, 0.02), 0)):   # 1 in/s, right frozen
        out = rs.drive_distance(1.0, "forward", 60, True, encoder="left",
                                warn_lights=False)
    assert out["ok"], out
    assert out["measured_by"] == "left", out
    assert "left wheel encoder only" in out["distance_source"], out
    # The fake keeps feeding ticks through the post-stop settle, as a real robot coasting
    # into its brake would, so this is "stopped at the target", not "stopped exactly on it".
    assert 1.0 <= out["actual_inches_traveled"] <= 1.6, out


def test_a_frozen_channel_aborts_instead_of_driving_twice_as_far():
    link = bench_link()
    with Feeder(link, per_packet=(ticks_for(link, 0.02), 0)):
        out = rs.drive_distance(2.0, "forward", 60, True, encoder="both",
                                warn_lights=False)
    assert not out["ok"], out
    assert out["abort_reason"] and "right encoder has not ticked" in out["abort_reason"], out
    assert 'encoder="left"' in out["abort_reason"], out
    # The whole point: it stopped early, not at the 4in the halved average would have
    # called 2in.
    assert out["actual_inches_traveled"] < 1.0, out


def test_a_frozen_selected_channel_names_the_other_wheel_as_the_way_out():
    link = bench_link()
    with Feeder(link, per_packet=(0, ticks_for(link, 0.02))):   # left frozen, right moving
        out = rs.drive_distance(2.0, "forward", 60, True, encoder="left",
                                warn_lights=False)
    assert not out["ok"], out
    assert "left encoder has not ticked" in out["abort_reason"], out
    assert 'encoder="right"' in out["abort_reason"], out
    assert not out["hit_timeout"], "should abort on the frozen channel, not time out"


def test_both_encoders_dead_is_not_told_to_try_the_other_one():
    link = bench_link()
    with Feeder(link, per_packet=(0, 0)):
        out = rs.drive_distance(2.0, "forward", 60, True, encoder="both",
                                warn_lights=False)
    assert not out["ok"], out
    assert "left and right encoder has not ticked" in out["abort_reason"], out
    assert "encoder=" not in out["abort_reason"], out    # there is no good wheel to suggest


def test_an_unknown_encoder_name_is_refused_before_anything_moves():
    link = bench_link()
    feed_enc(link, 0, 0, 0)
    out = rs.drive_distance(2.0, "forward", 60, True, encoder="middle", warn_lights=False)
    assert not out["ok"] and "encoder must be one of" in out["error"], out
    assert link.ser.lines == [], "nothing may be sent to the robot"


# ---- drive_arc, measuring with one wheel on its own radius -----------------------------

def test_a_single_encoder_arc_measures_against_that_wheels_radius():
    link = bench_link()
    # 24in radius, 20in track: the outer wheel runs 34in, the inner 14in. Forward + "left"
    # puts the RIGHT wheel outside, so the left wheel is the inner one.
    with Feeder(link, per_packet=(ticks_for(link, 0.02), ticks_for(link, 0.02)),
                yaw_dps=300):
        out = rs.drive_arc(24.0, 5.0, "left", 50, True, encoder="left", warn_lights=False)
        assert out["measuring_wheel_radius_in"] == 14.0, out
        assert out["measured_by"] == "left", out
        out = rs.drive_arc(24.0, 5.0, "left", 50, True, encoder="right", warn_lights=False)
        assert out["measuring_wheel_radius_in"] == 34.0, out


def test_the_outer_wheel_swaps_sides_on_a_reverse_arc():
    link = bench_link()
    with Feeder(link, per_packet=(ticks_for(link, 0.02), ticks_for(link, 0.02)),
                yaw_dps=300):
        out = rs.drive_arc(24.0, 5.0, "left", 50, True, encoder="left", travel="reverse",
                           warn_lights=False)
    assert out["measuring_wheel_radius_in"] == 34.0, out


def test_an_arc_measured_by_both_wheels_reports_the_centre_radius():
    link = bench_link()
    with Feeder(link, per_packet=(ticks_for(link, 0.02), ticks_for(link, 0.02)),
                yaw_dps=300):
        out = rs.drive_arc(24.0, 5.0, "left", 50, True, warn_lights=False)
    assert out["measuring_wheel_radius_in"] == 24.0, out
    assert out["measured_by"] == "both", out


def test_an_arc_too_tight_for_the_inner_wheel_is_refused_with_the_way_out():
    link = bench_link()
    with Feeder(link, per_packet=(0, 0)):
        out = rs.drive_arc(12.0, 5.0, "left", 50, True, encoder="left", warn_lights=False)
    assert not out["ok"], out
    assert "pivoting" in out["error"] and 'encoder="right"' in out["error"], out
    assert link.ser.lines == [], "a refused call must not touch the hardware"


def test_an_arc_whose_measuring_wheel_is_silent_says_which_one():
    link = bench_link()
    feed_imu(link, 10)
    feed_imu(link, 20)
    feed_enc(link, 1000, None, 30)      # right encoder not responding
    feed_enc(link, 1001, None, 40)
    out = rs.drive_arc(24.0, 5.0, "left", 50, True, encoder="right", warn_lights=False)
    assert not out["ok"], out
    assert "right encoder is not reporting" in out["error"], out


# ---- jog, and what it leaves the motion guard set to ------------------------------------

def test_a_bench_jog_puts_the_motion_guard_back_as_it_found_it():
    link = bench_link()
    link.motion.encoders_required = False   # as set_guard(encoders_required=false) leaves it
    with Feeder(link, per_packet=(10, 0)):
        out = rs.jog(40, 40, 0.05, True, warn_lights=False, require_telemetry=False)
    assert out["ok"], out
    # Forcing this back to True used to re-arm the encoder-fault latch behind the
    # operator's back, so the next move tripped on a channel they had already dismissed.
    assert link.motion.encoders_required is False, "jog must restore, not assume"


def test_a_normal_jog_leaves_the_guard_armed():
    link = bench_link()
    link.motion.encoders_required = True
    with Feeder(link, per_packet=(10, 10)):
        out = rs.jog(40, 40, 0.05, True, warn_lights=False, require_telemetry=True)
    assert out["ok"], out
    assert link.motion.encoders_required is True, out


def test_a_bench_jog_relaxes_the_guard_while_it_runs():
    link = bench_link()
    link.motion.encoders_required = True
    seen = []

    def watch():
        for _ in range(40):
            seen.append(link.motion.encoders_required)
            time.sleep(0.01)

    thread = threading.Thread(target=watch, daemon=True)
    with Feeder(link, per_packet=(10, 0)):
        thread.start()
        rs.jog(40, 40, 0.15, True, warn_lights=False, require_telemetry=False)
    thread.join(timeout=2)
    assert False in seen, "bench mode must relax the latch while the wheels turn"
    assert link.motion.encoders_required is True, "and hand it back afterwards"


# ---- drive_until_range -----------------------------------------------------------------

def feed_tof(link, name, values, ms):
    link.tof.update(name, values, ms)
    link.sensors.observe(name, values, ms, available=[v != INVALID for v in values])


def test_the_drive_direction_comes_from_the_sensors_name():
    link = bench_link()
    feed_tof(link, "TOF_REAR", (1000, 1000), 10)
    feed_tof(link, "TOF_REAR", (1000, 1000), 20)
    out = rs.drive_until_range(900, 60, True, warn_lights=False)
    assert out["direction"] == "reverse", out
    assert "name" in out["facing_from"], out


def test_facing_can_be_overridden_for_a_board_mounted_the_other_way():
    link = bench_link()
    # The real case on 2026-10-22: the board is on the back but flashed "TOF_FRONT".
    feed_tof(link, "TOF_FRONT", (1000, 1000), 10)
    feed_tof(link, "TOF_FRONT", (1000, 1000), 20)
    out = rs.drive_until_range(900, 60, True, facing="reverse", warn_lights=False)
    assert out["direction"] == "reverse", out
    assert out["facing_from"] == "the caller said so", out


def test_two_range_boards_must_be_told_apart_by_the_caller():
    link = bench_link()
    for ms in (10, 20):
        feed_tof(link, "TOF_FRONT", (1000, 1000), ms)
        feed_tof(link, "TOF_REAR", (1000, 1000), ms)
    out = rs.drive_until_range(900, 60, True, warn_lights=False)
    assert not out["ok"], out
    assert "pass sensor=" in out["error"], out
    out = rs.drive_until_range(900, 60, True, sensor="TOF_REAR", warn_lights=False)
    assert out["direction"] == "reverse", out


def test_nothing_in_range_is_refused_as_blind_not_clear():
    link = bench_link()
    feed_tof(link, "TOF_FRONT", (INVALID, INVALID), 10)
    feed_tof(link, "TOF_FRONT", (INVALID, INVALID), 20)
    out = rs.drive_until_range(200, 60, True, warn_lights=False)
    assert not out["ok"], out
    assert "nothing in range" in out["error"], out
    assert link.ser.lines == [], "nothing may be sent to the robot"


def test_already_inside_the_target_does_not_move():
    link = bench_link()
    for ms in (10, 20, 30):
        feed_tof(link, "TOF_FRONT", (180, 190), ms)
    out = rs.drive_until_range(200, 60, True, warn_lights=False)
    assert out["ok"] and out["moved"] is False, out
    assert link.ser.lines == [], "nothing may be sent to the robot"


def test_a_target_inside_the_sensors_usable_range_is_refused():
    link = bench_link()
    feed_tof(link, "TOF_FRONT", (900, 900), 10)
    out = rs.drive_until_range(20, 60, True, warn_lights=False)
    assert not out["ok"] and "target_mm must be at least" in out["error"], out


def test_a_min_pwm_under_the_deadband_is_refused():
    link = bench_link()
    feed_tof(link, "TOF_FRONT", (900, 900), 10)
    out = rs.drive_until_range(200, 60, True, min_pwm=20, warn_lights=False)
    assert not out["ok"] and "deadband" in out["error"], out


def test_an_approach_stops_itself_when_the_range_reaches_the_target():
    link = bench_link()
    stop = threading.Event()
    state = {"mm": 900.0, "ms": 0}

    def closing():
        # Stands in for the robot actually getting closer: the tool is driving, and this
        # is the only thing it can see. Faster than the real robot to keep the test short
        # -- the loop has to cope with the median lagging either way.
        while not stop.is_set():
            state["mm"] = max(120.0, state["mm"] - 15.0)
            state["ms"] += 50
            feed_tof(link, "TOF_FRONT", (int(state["mm"]), int(state["mm"]) + 10),
                     state["ms"])
            time.sleep(0.05)

    thread = threading.Thread(target=closing, daemon=True)
    thread.start()
    try:
        out = rs.drive_until_range(300, 80, True, warn_lights=False)
    finally:
        stop.set()
        thread.join(timeout=2)
    assert out["ok"], out
    assert out["abort_reason"] is None and not out["hit_timeout"], out
    assert out["direction"] == "forward", out
    assert out["final_mm"] <= 300 + out["tolerance_mm"], out
    assert link.ser.lines and link.ser.lines[-1] == "stop", link.ser.lines[-5:]


def test_an_approach_that_stops_getting_closer_aborts_and_says_why():
    link = bench_link()
    stop = threading.Event()
    state = {"ms": 0}

    def stuck():
        # Commands go out, the range never changes: a wheel is held, or the beam is
        # skimming a surface too obliquely to read.
        while not stop.is_set():
            state["ms"] += 50
            feed_tof(link, "TOF_FRONT", (900, 910), state["ms"])
            time.sleep(0.05)

    thread = threading.Thread(target=stuck, daemon=True)
    thread.start()
    try:
        out = rs.drive_until_range(300, 80, True, warn_lights=False)
    finally:
        stop.set()
        thread.join(timeout=2)
    assert not out["ok"], out
    assert out["abort_reason"] and "nothing got closer" in out["abort_reason"], out
    assert out["elapsed_s"] < rs.RANGE_NO_PROGRESS_S + 3, out
    assert link.ser.lines[-1] == "stop", link.ser.lines[-5:]


def test_an_approach_whose_sensor_goes_quiet_aborts_rather_than_driving_blind():
    link = bench_link()
    stop = threading.Event()
    state = {"ms": 0, "quiet": False}

    def fading():
        # Reports for a moment, then the board goes silent mid-approach. "Not reporting"
        # is blind, not clear, so this must stop the robot.
        while not stop.is_set():
            if not state["quiet"]:
                state["ms"] += 50
                feed_tof(link, "TOF_FRONT", (900, 910), state["ms"])
            time.sleep(0.05)

    thread = threading.Thread(target=fading, daemon=True)
    thread.start()
    try:
        time.sleep(0.15)
        state["quiet"] = True
        out = rs.drive_until_range(300, 80, True, warn_lights=False)
    finally:
        stop.set()
        thread.join(timeout=2)
    assert not out["ok"], out
    assert out["abort_reason"] and "stopped reporting" in out["abort_reason"], out
    assert link.ser.lines[-1] == "stop", link.ser.lines[-5:]


def main():
    import traceback
    failures = 0
    for name, fn in sorted(globals().items()):
        if not (name.startswith("test_") and callable(fn)):
            continue
        try:
            fn()
            print("PASS", name)
        except Exception as e:
            failures += 1
            print("FAIL", name, "-", repr(e))
            traceback.print_exc()
    print("\n%s" % ("all passed" if not failures else "%d FAILED" % failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
