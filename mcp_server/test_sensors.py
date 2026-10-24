#!/usr/bin/env python3
"""Tests for sensors.py and the sensor-dependent paths through robot_server.py.

No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none python3 mcp_server/test_sensors.py

These cover the states that are slow and annoying to stage on a real robot -- an encoder
dropping out mid-run, a ToF board that is reporting but has nothing in range, a stream
going quiet -- and the behaviour that used to be hard-coded: that whether a missing sensor
stops the robot is a policy decision, not a constant.
"""

import io
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sensors as sensorlib  # noqa: E402
import recording  # noqa: E402
import tilt_guard as tg  # noqa: E402
import robot_server as rs  # noqa: E402

bridge = rs.bridge

UNAVAILABLE = -2147483648   # SENSOR_VALUE_UNAVAILABLE in LlamaSensors.h
INVALID = -1                # SENSOR_VALUE_INVALID


# ---- bridge.parse_line, the shape everything else is built on --------------------------

def test_encoder_packet_with_one_wheel_out_parses_to_none():
    kind, payload = bridge.parse_line(b"ENC %d 1234 5000" % UNAVAILABLE)
    assert kind == "ENC", (kind, payload)
    left, right, ms = payload
    # None, never a number: the sentinel must not survive as a count, and negating it
    # (encoder_sign) would turn it into an ordinary-looking large value.
    assert left is None and right == 1234 and ms == 5000, payload


def test_tof_packet_parses_generically():
    kind, payload = bridge.parse_line(b"TOF_FRONT 812 -1 48213")
    assert kind == "SENSOR", (kind, payload)
    name, values, ms = payload
    assert name == "TOF_FRONT" and values == (812, -1) and ms == 48213, payload


def test_bridge_chatter_is_not_mistaken_for_a_sensor():
    for line in (b"DROPPED 3", b"FOREIGN 77", b"--- Llama Computer Bridge ---"):
        kind, _ = bridge.parse_line(line)
        assert kind is None, (line, kind)


# ---- the registry ----------------------------------------------------------------------

def test_a_sensor_appears_when_it_transmits():
    reg = sensorlib.SensorRegistry()
    assert reg.streams == {}
    # Asking about a sensor must not invent one -- a stream exists because a packet
    # arrived, never because something looked it up or set a deadline for it.
    reg.set_stale_after("imu", 0.5)
    assert reg.get("imu") is None and reg.streams == {}
    assert reg.missing(["imu"]) == ["imu"]
    reg.observe("imu", (1,) * 6, ms=1)
    assert reg.get("imu").stale_after_s == 0.5   # the deadline was waiting for it
    # A name nothing has ever heard of: no table to add it to, it just shows up.
    reg.observe("TOF_REAR", (300, 400), ms=10)
    assert "tof_rear" in reg.streams
    assert reg.live("tof_rear")


def test_a_quiet_stream_stops_being_live():
    reg = sensorlib.SensorRegistry(stale_after_s=0.5)
    now = time.monotonic()
    reg.observe("imu", (1, 2, 3), ms=1, when=now - 2.0)
    assert not reg.live("imu", now)
    assert reg.age_s("imu", now) > 1.9
    reg.observe("imu", (1, 2, 3), ms=2, when=now)
    assert reg.live("imu", now)


def test_reporting_but_unable_to_measure_is_not_the_same_as_silent():
    reg = sensorlib.SensorRegistry()
    now = time.monotonic()
    # A ToF that is alive and sees nothing in range.
    reg.observe("TOF_FRONT", (INVALID, INVALID), ms=1, available=[False, False], when=now)
    reg.observe("TOF_FRONT", (INVALID, INVALID), ms=2, available=[False, False], when=now)
    snap = reg.snapshot(now)["streams"]["tof_front"]
    assert snap["live"] is True and snap["measuring"] is False, snap
    # vs. a board that has gone away entirely.
    reg.observe("TOF_REAR", (100, 100), ms=1, available=[True, True], when=now - 99)
    reg.observe("TOF_REAR", (100, 100), ms=2, available=[True, True], when=now - 99)
    gone = reg.snapshot(now)["streams"]["tof_rear"]
    assert gone["live"] is False, gone


def test_rate_is_reported():
    reg = sensorlib.SensorRegistry()
    t0 = time.monotonic()
    for i in range(10):
        reg.observe("enc", (i, i), ms=i * 20, when=t0 + i * 0.02)   # 50 Hz
    rate = reg.get("enc").rate_hz()
    assert 45 < rate < 55, rate


def test_policy_groups_match_any_board_of_that_kind():
    reg = sensorlib.SensorRegistry()
    reg.observe("TOF_REAR", (120,), ms=1)
    reg.observe("TOF_REAR", (121,), ms=2)
    # "tof" is satisfied by whichever range board happens to be fitted, so a stored policy
    # survives TOF_FRONT being replaced or joined by another.
    assert reg.missing(["tof"]) == []
    assert reg.missing(["enc"]) == ["enc"]


def test_policy_default_is_todays_behaviour():
    p = sensorlib.SensorPolicy()
    assert p.required == ["enc", "imu"]
    assert p.set(["ENC"]) == ["enc"]          # canonicalised, case-insensitive
    assert p.set([]) == []                    # requiring nothing is a legal answer


# ---- a link with no serial port ---------------------------------------------------------

class FakeSer:
    def __init__(self):
        self.lines = []

    def write(self, b):
        self.lines.append(b.decode().strip())

    def flush(self):
        pass


class NoRecorder:
    """Swallows every recorder call, so make_link() is usable for pump tests too."""
    def __getattr__(self, _name):
        return lambda *a, **kw: None


def make_link():
    link = rs.RobotLink.__new__(rs.RobotLink)
    link.cfg = dict(bridge.DEFAULT_CONFIG)
    link.track_width_calibrated = False
    link._wlock = threading.RLock()
    link.lock = threading.Lock()
    link.ser = FakeSer()
    link.enc = bridge.EncoderState(link.cfg)
    link.orient = bridge.OrientationState()
    link.tof = bridge.TofState()
    link.sensors = sensorlib.SensorRegistry(rs.TELEMETRY_STALE_S)
    link.sensors.set_stale_after("imu", rs.IMU_STALE_FAULT_S)
    link.policy = sensorlib.SensorPolicy()
    link.tilt = tg.TiltGuard(mode="off")      # not under test here
    link.recorder = NoRecorder()
    link.last_imu = None
    return link


def feed_enc(link, left, right, ms):
    """One ENC packet through the same path the pump uses."""
    link.enc.update(left, right, ms)
    link.sensors.observe("enc", (left, right), ms, available=list(link.enc.available))


def encoders_live(link, left=10, right=20):
    """Two ENC packets -- what it takes for the registry to trust the stream (one packet
    could be a torn line, see sensors.py)."""
    feed_enc(link, left, right, 100)
    feed_enc(link, left, right, 120)


def imu_live(link):
    link.sensors.observe("imu", (0,) * 6, 100)
    link.sensors.observe("imu", (0,) * 6, 120)


def test_telemetry_survives_a_dead_encoder():
    link = make_link()
    feed_enc(link, 1000, 2000, 100)
    feed_enc(link, None, 2100, 120)          # left encoder stops responding
    snap = link.telemetry_snapshot()         # must not raise -- it used to, on heading_deg
    assert snap["encoders_available"] == [False, True], snap["encoders_available"]
    assert snap["heading_deg"] is None       # never computed from one live, one frozen wheel
    assert snap["left_ticks"] is None and snap["right_ticks"] == 2100
    # The left wheel's distance is HELD, not zeroed: a missing encoder is not a still wheel.
    assert snap["dist_l_in"] == 0.0, snap["dist_l_in"]
    assert snap["dist_r_in"] > 0


def test_last_counts_is_none_unless_both_wheels_reported():
    link = make_link()
    feed_enc(link, 1000, 2000, 100)
    assert link.last_counts() == (1000, 2000)
    feed_enc(link, None, 2100, 120)
    # Not a tuple with a None in it: every drive loop subtracts these, and None - int
    # would raise mid-move with the motors running.
    assert link.last_counts() is None


def test_range_block_separates_nothing_in_range_from_blind():
    link = make_link()
    link.tof.update("TOF_FRONT", (812, INVALID), 1000)
    link.sensors.observe("TOF_FRONT", (812, INVALID), 1000, available=[True, False])
    rng = link.range_snapshot()
    chans = rng["channels"]
    assert chans["front_left"]["mm"] == 812 and chans["front_left"]["in_range"] is True
    # Reporting, but nothing within reach: in_range False while live stays True.
    assert chans["front_right"]["mm"] is None
    assert chans["front_right"]["in_range"] is False
    assert chans["front_right"]["live"] is True
    assert rng["closest_mm"] == 812 and rng["live"] is True

    # A channel whose last ping missed but which has recent readings still reports a
    # distance: in_range follows the median, latest_in_range follows the raw ping. Seen on
    # the real robot at ~1.4 m, where the right sensor flaps in and out of its limit.
    link.tof.update("TOF_FRONT", (812, 1460), 1100)
    link.tof.update("TOF_FRONT", (812, INVALID), 1200)
    link.sensors.observe("TOF_FRONT", (812, INVALID), 1200, available=[True, False])
    right = link.range_snapshot()["channels"]["front_right"]
    assert right["mm"] == 1460 and right["in_range"] is True, right
    assert right["latest_mm"] is None and right["latest_in_range"] is False, right

    # Same sensor gone quiet: closest is still None, but the note must not read as "clear".
    link.sensors.observe("TOF_FRONT", (812, INVALID), 1000, available=[True, False],
                         when=time.monotonic() - 99)
    blind = link.range_snapshot()
    assert blind["live"] is False and "blind, not clear" in blind["note"], blind["note"]


def test_an_unknown_sensor_needs_no_code_to_appear():
    link = make_link()
    # Exactly what the pump does for a SENSOR line, with a name nothing has heard of.
    link.tof.update("TOF_REAR", (250, 260), 2000)
    link.sensors.observe("TOF_REAR", (250, 260), 2000, available=[True, True])
    link.sensors.observe("TOF_REAR", (251, 259), 2100, available=[True, True])
    snap = link.telemetry_snapshot()
    assert "tof_rear" in snap["sensors"]
    assert "rear_left" in snap["range"]["channels"]
    assert snap["range"]["closest_mm"] == 250


# ---- the policy decides whether a missing sensor stops the robot ------------------------

def test_a_required_sensor_that_is_missing_refuses_and_names_it():
    link = make_link()
    encoders_live(link)                      # encoders live, IMU never seen
    why = link._drive_refusal()
    assert why and "no live IMU" in why, why
    assert "set_required_sensors" in why, why
    assert link.missing_required() == ["imu"]


def test_dropping_a_sensor_from_the_policy_lets_the_robot_drive():
    link = make_link()
    encoders_live(link)
    assert link._drive_refusal() is not None
    link.policy.set(["enc"])                 # "this run does not need the IMU"
    assert link._drive_refusal() is None
    link.send_drive(60, 60)
    assert link.ser.lines[-1] == "60 60"


def test_requiring_nothing_drives_with_no_telemetry_at_all():
    link = make_link()                       # nothing has ever reported
    assert link._drive_refusal() is not None
    link.policy.set([])                      # ceiling camera, marker on the robot
    assert link._drive_refusal() is None
    link.send_drive(-40, -40)
    assert link.ser.lines[-1] == "-40 -40"


def test_requiring_a_sensor_that_is_silent_refuses_even_if_others_are_live():
    link = make_link()
    encoders_live(link)
    imu_live(link)
    link.policy.set(["enc", "imu", "tof"])   # insist on range before moving
    why = link._drive_refusal()
    assert why and "tof not reporting" in why, why
    link.sensors.observe("TOF_FRONT", (500, 500), 110, available=[True, True])
    assert link._drive_refusal() is not None, "one packet is not yet a sensor"
    link.sensors.observe("TOF_FRONT", (501, 499), 120, available=[True, True])
    assert link._drive_refusal() is None


def test_a_torn_serial_line_does_not_invent_a_sensor():
    """Seen on the real robot: 115200 baud tears, and "IMU 194 -122 16028" split across two
    lines leaves "MU 150 -128 ..." -- which parses as a SensorPacket named MU."""
    kind, payload = bridge.parse_line(b"MU 150 -128 16094 -31 -8 10 168501")
    assert kind == "SENSOR" and payload[0] == "MU"   # parse_line cannot tell; the registry must

    reg = sensorlib.SensorRegistry()
    reg.observe("IMU", (1,) * 6, ms=1)
    reg.observe("IMU", (1,) * 6, ms=2)
    reg.observe("MU", (150, -128), ms=3)             # the torn line, once
    snap = reg.snapshot()
    assert "mu" not in snap["streams"], snap["streams"]
    assert "mu" in snap["provisional"]
    assert snap["provisional"]["mu"]["provisional"] is True
    assert "imu" in snap["streams"]
    # ...and it can never satisfy a policy that asks for it.
    assert reg.missing(["mu"]) == ["mu"]
    # Once it stops arriving -- which it does, because nothing is sending it -- it is gone.
    reg.get("mu").last_wall -= 99
    assert "mu" not in reg.snapshot()["provisional"]
    assert reg.get("mu") is None


def test_a_real_new_board_is_trusted_on_its_second_packet():
    reg = sensorlib.SensorRegistry()
    reg.observe("TOF_REAR", (300, 400), ms=1)
    assert "tof_rear" in reg.snapshot()["provisional"]   # could still be a torn line
    reg.observe("TOF_REAR", (305, 402), ms=2)
    assert "tof_rear" in reg.snapshot()["streams"]       # a sensor transmits again
    assert reg.missing(["tof"]) == []


def test_capability_cost_explains_what_was_given_up():
    cost = sensorlib.capability_cost(["imu"])
    assert any("tilt guard" in c for c in cost), cost
    assert any("scrub" in c for c in cost), cost


# ---- guards follow the sensors ----------------------------------------------------------

def test_a_guard_with_no_sensor_reports_inactive_rather_than_refusing():
    link = make_link()
    link.tilt = tg.TiltGuard()               # mode "on", but no IMU has ever reported
    encoders_live(link)
    link.policy.set(["enc"])                 # the operator has accepted no IMU
    info = link._guard_info()
    assert info["tilt"]["active"] is False
    assert "no live IMU" in info["tilt"]["inactive_because"]
    # ...and that on its own does not stop the robot.
    assert link._drive_refusal() is None


def test_guards_can_be_retuned_and_switched_off_at_runtime():
    link = make_link()
    ok, msg = link.tilt.configure(limit_deg=40)
    assert ok and link.tilt.limit_deg == 40, msg


def test_switching_a_guard_off_suspends_it_without_erasing_what_it_saw():
    link = make_link()
    link.tilt = tg.TiltGuard()
    link.tilt.tripped = True
    link.tilt.fault = {"reason": "leaned 30 deg"}
    imu_live(link)
    encoders_live(link)
    assert "tripped" in (link._drive_refusal() or "")
    link.tilt.configure(mode="off")
    # Enforcement stops...
    assert link._drive_refusal() is None
    # ...but the latch and the reason are still on the record, and come back with it.
    assert link.tilt.tripped and link.tilt.fault["reason"] == "leaned 30 deg"
    link.tilt.configure(mode="on")
    assert "tripped" in (link._drive_refusal() or "")


def test_a_tripped_guard_still_blocks_when_every_sensor_is_present():
    link = make_link()
    encoders_live(link)
    imu_live(link)
    link.tilt = tg.TiltGuard()
    link.tilt.tripped = True
    link.tilt.fault = {"reason": "leaned 30 deg"}
    why = link._drive_refusal()
    assert why and "tilt guard tripped" in why, why


# ---- recording --------------------------------------------------------------------------

def test_recording_leaves_an_unknowable_heading_blank_rather_than_crashing(tmp):
    rec = recording.Recorder(tmp)
    rec.start("run")
    link = make_link()
    feed_enc(link, 1000, 2000, 100)
    rec.on_enc(link.enc, link.orient, 1.0, 1.0)
    feed_enc(link, None, 2100, 120)          # left encoder out: heading is unknowable
    rec.on_enc(link.enc, link.orient, None, 1.0)
    rec.on_sensor("TOF_FRONT", (812, -1), link.enc, link.orient)
    rec.on_config("tilt guard: mode=off")
    rec.stop()
    rows = list(io.open(rec.csv_path, encoding="utf-8"))
    head = rows[0].strip().split(",")
    assert "range_closest_mm" in head
    col = head.index("enc_heading_deg")
    cells = [r.strip().split(",") for r in rows[1:]]
    # Both encoders reporting: a real number (0.0 here -- nothing has moved yet), not nan.
    assert cells[0][col] != "nan" and _is_float(cells[0][col]), cells[0][col]
    # One encoder out: nan, NOT a zero -- and still parseable by analyze_csv.py's float().
    assert cells[1][col] == "nan"
    assert all(_floats_ok(r) for r in cells), "every numeric cell must survive float()"
    assert cells[2][1] == "TOF_FRONT"        # the sensor's own name is the source
    assert cells[3][1] == "CONFIG"


def _floats_ok(cells):
    """Every numeric column parses, the way ellama's analyze_csv.py loads these files."""
    head = recording.HEADER
    return all(cells[i] == "" or _is_float(cells[i])
               for i, name in enumerate(head) if name not in ("source", "note"))


def _is_float(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


# ---- speed model -------------------------------------------------------------------------

def test_open_loop_speed_model_matches_the_measured_curve():
    assert rs.speed_for_pwm(40) is None            # below the measured deadband
    assert abs(rs.speed_for_pwm(60) - 6.13) < 0.01  # a measured point
    assert 9.36 < rs.speed_for_pwm(90) < 12.03      # interpolated between two
    assert rs.speed_for_pwm(200) == 14.46           # clamped at the top of the curve


# ---- runner --------------------------------------------------------------------------

def main():
    import tempfile
    import traceback
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for name, fn in sorted(globals().items()):
            if not (name.startswith("test_") and callable(fn)):
                continue
            try:
                if fn.__code__.co_argcount:
                    fn(Path(tmp) / name)
                else:
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
