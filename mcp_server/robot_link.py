#!/usr/bin/env python3
"""RobotLink: the serial connection to computer_bridge, the telemetry pump,
drive-command choke point, sensor policy and tilt guard state.

Extracted from robot_server.py. The module-level POLICY and TILT objects are
shared by every RobotLink for the life of the process: reconnecting must not
wipe a latched tilt fault or re-learn "level" from a leaning robot.
"""

import math
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import serial

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
import bridge  # noqa: E402
import drive_lease  # noqa: E402
import operator_stop  # noqa: E402
import sensors as sensorlib  # noqa: E402
import recording  # noqa: E402
import tilt_guard  # noqa: E402
from drive_math import (DRIVE_SWAP_SIDES, MAX_PWM_CEILING, TELEMETRY_STALE_S,  # noqa: E402
                        TRUE_GEAR_RATIO, selected_wheels)

# ---------------------------------------------------------------------------
_required_env = os.environ.get("ELLAMA_REQUIRED_SENSORS")
POLICY = sensorlib.SensorPolicy(
    [n for n in re.split(r"[,\s]+", _required_env) if n] if _required_env is not None
    else sensorlib.SensorPolicy.DEFAULT)
if _required_env is not None:
    POLICY.set_by = "ELLAMA_REQUIRED_SENSORS"

# ---------------------------------------------------------------------------
# Tilt guard (mcp_server/tilt_guard.py). The robot is low and can run banks up to ~25 deg;
# past that it tips. Every drive command goes through RobotLink.send_drive, which refuses
# (and sends "stop") once the guard has tripped, or while there is no live IMU to judge by.
# This only covers commands that come through THIS server -- the handheld stick is not
# guarded. Env knobs are the operator's, deliberately not tool arguments:
#   ELLAMA_TILT_LIMIT_DEG     trip angle from the level reference (default 25)
#   ELLAMA_TILT_LOOKAHEAD_S   stop-latency look-ahead on the tilt rate (default 0.3)
#   ELLAMA_TILT_GUARD=off     bench use only: never trips and never refuses (still reports tilt)
# ---------------------------------------------------------------------------
TILT_LIMIT_DEG = float(os.environ.get("ELLAMA_TILT_LIMIT_DEG", tilt_guard.DEFAULT_LIMIT_DEG))
TILT_LOOKAHEAD_S = float(os.environ.get("ELLAMA_TILT_LOOKAHEAD_S", tilt_guard.DEFAULT_LOOKAHEAD_S))
TILT_GUARD_ENABLED = os.environ.get("ELLAMA_TILT_GUARD", "on").lower() not in ("off", "0", "false", "no")
# Moving with the IMU silent means flying blind on the one thing this guard watches.
IMU_STALE_FAULT_S = 0.5


# One guard for the life of the server process, shared by every RobotLink. connect() (and a
# robot power cycle) must not be a way to wipe a latched fault, or to re-learn "level" from a
# robot that is leaning -- only restarting the server, which the operator does, resets it.
TILT = tilt_guard.TiltGuard(limit_deg=TILT_LIMIT_DEG, lookahead_s=TILT_LOOKAHEAD_S,
                            enabled=TILT_GUARD_ENABLED)

# The operator's stop latch (mcp_server/operator_stop.py), process-wide for the same reason:
# reconnecting must not release it. Read here at module level rather than through `self`, so
# it holds for every RobotLink however it was built.
OPSTOP = operator_stop.OperatorStop()

# Who is driving (mcp_server/drive_lease.py). Checked at the tool gate in robot_server, not
# in send_drive: a serial line does not know which client asked for it. It lives here so
# that it is process-wide like the rest, and so guards() reports all of them together.
_lease_env = os.environ.get("ELLAMA_DRIVE_LEASE", "").lower()
LEASE = drive_lease.DriveLease(required=_lease_env in ("required", "on", "1", "true", "yes"))


def _trace_event(kind, **fields):
    """Best-effort trace into recordings/trace.jsonl. Never raises."""
    try:
        import trace as _tr
        if not hasattr(_tr, "event"):
            return
        _tr.event(kind, **fields)
    except Exception:
        pass


class GuardFault(Exception):
    """Raised by RobotLink.send_drive when the tilt guard refuses a drive command.
    Drive loops let it propagate: the tool's own `finally` sends the stop, and the tool returns
    the fault (see _tilt_guarded); autonomous threads end with reason "error: ... guard ...".
    `info` carries a snapshot of the guard."""

    def __init__(self, message, info=None):
        super().__init__(message)
        self.info = info or {}


TiltFault = GuardFault  # the original name; tests and callers still use it


def _pick_port(explicit):
    """Like bridge.pick_port, but raises instead of calling sys.exit() -- this runs
    inside a long-lived server, not a one-shot script, so nothing here may exit the
    process."""
    if explicit:
        return explicit
    candidates = bridge.list_candidate_ports()
    if not candidates:
        raise RuntimeError(
            "no serial ports found -- plug in the bridge ESP32, or pass an explicit port"
        )
    return candidates[0]


class RobotLink:
    """Owns the serial connection, the background telemetry pump, and drive commands."""

    _profile_gen = 0   # bumped by switch_robot; class default for links built in tests

    @staticmethod
    def _load_profile():
        """(cfg, track_width_calibrated) for the active robot profile, validated."""
        cfg = bridge.load_config()
        problems = recording.check_config(cfg)
        swap = cfg.get("drive_swap_sides", DRIVE_SWAP_SIDES)
        if not isinstance(swap, bool):
            problems.append("'drive_swap_sides' must be true/false, got %r" % (swap,))
        if problems:
            raise RuntimeError("bad calibration config (%s): %s"
                               % (bridge.ROBOT_NAME, "; ".join(problems)))
        calibrated = cfg["track_width_in"] != bridge.DEFAULT_CONFIG["track_width_in"]
        cfg["gear_ratio"] = TRUE_GEAR_RATIO
        return cfg, calibrated

    def _apply_profile_policy(self, reset_when_unset=False):
        """A profile may preset the sensor policy (e.g. an IMU-less robot runs
        ["enc"]). A profile without one leaves the policy alone at connect (it
        survives reconnects, like the guards) and resets it to the default on a
        robot switch. An explicit ELLAMA_REQUIRED_SENSORS env var always wins."""
        if _required_env is not None:
            return
        prof = self.cfg.get("required_sensors")
        if isinstance(prof, str):
            prof = [prof]
        names = [n for part in (prof or []) for n in re.split(r"[,\s]+", part) if n]
        if names or reset_when_unset:
            POLICY.set(names or list(sensorlib.SensorPolicy.DEFAULT),
                       by="robot profile %s" % bridge.ROBOT_NAME)

    def switch_robot(self, name):
        """Switch to another robot profile without reconnecting. Odometry restarts
        from zero (the encoder sign may have changed). Unknown or invalid profile
        raises and leaves everything as it was. Callers must not switch mid-move."""
        old = bridge.ROBOT_NAME
        with self.lock:
            bridge.set_robot(name)
            try:
                cfg, calibrated = self._load_profile()
            except Exception:
                bridge.set_robot(old)
                raise
            self.cfg = cfg
            self.track_width_calibrated = calibrated
            self.enc = bridge.EncoderState(self.cfg)
            # The pump parses (applying ENC_SIGN) outside this lock; a packet parsed
            # under the old sign must not become the new EncoderState's zero point.
            self._profile_gen += 1
        self._apply_profile_policy(reset_when_unset=True)
        self.recorder.vet.log("robot_profile", {
            "from": old, "to": name,
            "encoder_sign": list(self.cfg["encoder_sign"]),
            "drive_swap_sides": self.cfg.get("drive_swap_sides", DRIVE_SWAP_SIDES),
            "track_width_in": self.cfg["track_width_in"],
        })
        return self.cfg

    def __init__(self, port=None):
        self.cfg, self.track_width_calibrated = self._load_profile()
        self._profile_gen = 0

        port = _pick_port(port)
        # exclusive=True so a second process opening this port fails loudly instead of
        # silently stealing bytes. Nothing locks a tty by default: two readers each consume
        # from the same buffer and both get torn lines, and opening the port a second time
        # resets the ESP32 (see bridge.RESET_SETTLE_S) -- mid-drive, if the robot is moving.
        self.ser = serial.Serial(port, bridge.BAUD_RATE, timeout=0.05, exclusive=True)
        time.sleep(bridge.RESET_SETTLE_S)  # opening the port resets the ESP32
        self.ser.reset_input_buffer()

        self.reader = bridge.SerialReader(self.ser)
        self.reader.start()

        self.enc = bridge.EncoderState(self.cfg)
        self.orient = bridge.OrientationState()
        self.tof = bridge.TofState()
        # Every stream heard from, keyed by the name inside the packet. Nothing registers
        # sensors in advance -- they appear when they transmit.
        self.sensors = sensorlib.SensorRegistry(TELEMETRY_STALE_S)
        # The tilt guard judges lean sample by sample, so a half-second gap in the IMU
        # already means it is guessing. Hold the IMU to a tighter deadline than the rest.
        self.sensors.set_stale_after("imu", IMU_STALE_FAULT_S)
        self.policy = POLICY   # module-level: survives connect(), like the guards
        self._apply_profile_policy()
        self.lock = threading.Lock()
        self.recorder = recording.Recorder(REPO_ROOT / "recordings")
        self.recorder.vet.log("connect", {
            "port": str(port),
            "track_width_in": self.cfg["track_width_in"],
            "track_width_calibrated": self.track_width_calibrated,
            "required_sensors": list(self.policy.required),
        })
        self.last_imu = None
        # Last drive command overheard from another sender (the stick).
        # Updated by the pump; read by the dashboard so hand driving is visible.
        self.last_stick = None
        self.last_stick_wall = None

        # Serial writes come from tool threads AND the pump (the tilt trip sends "stop" from
        # there, ~20 ms after the offending IMU sample), so a line must never interleave.
        self._wlock = threading.RLock()
        self.tilt = TILT  # module-level: a latched fault and the level reference survive connect()

        self._stop_pump = threading.Event()
        self._pump_thread = threading.Thread(target=self._pump, daemon=True)
        self._pump_thread.start()

    # last_*_wall read through the registry so there is one answer to "when did we last
    # hear from this sensor", rather than a field here that could disagree with it.
    @property
    def last_enc_wall(self):
        s = self.sensors.get("enc")
        return None if s is None else s.last_wall

    @property
    def last_imu_wall(self):
        s = self.sensors.get("imu")
        return None if s is None else s.last_wall

    def _pump(self):
        """Drains the serial reader queue into the sensor registry and the enc/orient/tof
        state, forever. This is the only thread that mutates them -- tools only read."""
        while not self._stop_pump.is_set():
            try:
                raw = self.reader.q.get(timeout=0.2)
            except Exception:
                continue
            gen = self._profile_gen   # read BEFORE parse: see switch_robot
            kind, payload = bridge.parse_line(raw)
            tripped = None
            now = time.monotonic()
            with self.lock:
                if kind == "ENC" and gen != self._profile_gen:
                    pass   # parsed under the previous robot's encoder sign: drop it
                elif kind == "ENC":
                    left, right, ms = payload
                    # Either count may be None: that encoder is not responding, which is a
                    # routine state next to a wheel, not an error. EncoderState holds that
                    # wheel's distance rather than reading it as a stationary wheel.
                    sl, sr = self.enc.update(left, right, ms)
                    # Wheels still -> the IMU re-learns its yaw zero-point and holds yaw.
                    both = all(self.enc.available)
                    self.orient.note_wheels(self.enc.dist_l if both else None,
                                            self.enc.dist_r if both else None, now)
                    self.sensors.observe("enc", (left, right), ms,
                                         available=list(self.enc.available), when=now)
                    self.recorder.on_enc(self.enc, self.orient, sl, sr)
                elif kind == "CMD":
                    self.recorder.on_cmd(*payload)  # the stick's command, overheard by the bridge
                    prev_wall = self.last_stick_wall
                    self.last_stick = tuple(payload)
                    self.last_stick_wall = now
                    if prev_wall is None or now - prev_wall > 2.0:
                        _trace_event("stick", action="driving",
                                     left=payload[0], right=payload[1])
                elif kind == "IMU":
                    ax, ay, az, gx, gy, gz, ms = payload
                    agx, agy, agz, dx, dy, dz = self.orient.update(ax, ay, az, gx, gy, gz, ms,
                                                                         now=now)
                    self.sensors.observe("imu", (ax, ay, az, gx, gy, gz), ms, when=now)
                    self.recorder.on_imu(self.enc, self.orient, (agx, agy, agz))
                    self.last_imu = dict(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, ms=ms)
                    tripped = self.tilt.update(self.orient.pitch, self.orient.roll, ms,
                                               gyro_dps=math.hypot(dx, dy))
                elif kind == "SENSOR":
                    # Any SensorPacket sender. Generic on the name on purpose: the name
                    # travels in the packet, so flashing a second ToF board as "TOF_REAR"
                    # needs no change here, in bridge.py, or in the bridge firmware.
                    name, values, ms = payload
                    available = None
                    if sensorlib.canon(name).startswith("tof"):
                        # Range sensors are the ones whose -1 means "could not measure";
                        # TofState applies that reading and keeps the median history.
                        self.tof.update(name, values, ms)
                        available = [v != bridge.SENSOR_VALUE_INVALID for v in values]
                        self.recorder.on_range(self.tof.closest_mm())
                    self.sensors.observe(name, values, ms, available=available, when=now)
                    self.recorder.on_sensor(name, values, self.enc, self.orient)
                elif kind is None:
                    # computer_bridge.ino's own counters. DROPPED is its receive queue
                    # overrunning, which is what used to look like a radio fault.
                    parts = payload.split()
                    if len(parts) == 2 and parts[0] in ("DROPPED", "FOREIGN"):
                        try:
                            self.sensors.note_bridge(parts[0], int(parts[1]))
                        except ValueError:
                            pass
            if tripped is not None:
                # Outside the state lock: the guard already latched, so send_drive refuses
                # from here on. A blocking tool notices on its next send_drive, which
                # raises TiltFault.
                self.emergency_stop()

    def send_line(self, text):
        with self._wlock:
            self.ser.write((text + "\n").encode())
            self.ser.flush()

    def missing_required(self):
        """Required sensors that are not reporting right now (canonical names)."""
        return self.policy.missing(self.sensors)

    def _missing_detail(self, name):
        """Why one required sensor does not count as present, in the operator's terms.

        `name` may be a GROUP ("tof"/"range"), and no stream is ever actually called
        that: the board reports as TOF_FRONT and the registry stores "tof_front". Looking
        the required name up directly therefore answered "never seen" for every group,
        however long the board had been streaming -- so a ToF that had run for eight
        minutes and gone quiet three seconds earlier was described as a board that had
        never existed, which sends the operator hunting for an unplugged cable instead of
        a link that just dropped. Resolve the group first and describe its freshest
        member, naming the stream when it is not the name that was required.
        """
        now = time.monotonic()
        trusted = self.sensors.resolve(name)
        if trusted:
            # The freshest member: with two boards fitted under one group name, the one
            # that went quiet most recently is the one that says what just happened.
            stream = min((self.sensors.get(n) for n in trusted),
                         key=lambda st: st.age_s(now))
            when = "last sample %.1fs ago" % stream.age_s(now)
            if stream.name != sensorlib.canon(name):
                when = "%s %s" % (stream.name, when)
        elif self.sensors.resolve(name, trusted_only=False):
            # One packet is not a sensor: the serial link tears and leaves plausible-looking
            # names behind (see sensors.py). Say so rather than reporting it as silent.
            when = ("seen only once, not yet confirmed -- one packet can be a garbled "
                    "line rather than a sensor")
        else:
            when = "never seen"
        if name == "imu":
            return "no live IMU (%s)" % when
        if name == "enc":
            return "no live encoder telemetry (%s)" % when
        return "%s not reporting (%s)" % (name, when)

    def _drive_refusal(self):
        """Why a nonzero drive command must not go out right now, or None.

        Before anything else: has the operator latched a stop? That is a person at the
        robot saying no, and it outranks every measurement below.

        Then two different questions, asked in this order:

        1. Is a sensor this run was DECLARED to need missing? That is the policy's call
           (set_required_sensors), not this file's opinion. A sensor that is absent but not
           required is reported in the result and never blocks.
        2. Has a guard that is switched on actually tripped? A trip is a measurement that
           something went wrong, so it stays latched until an operator looks at the robot
           and calls clear_tilt_fault.

        A guard whose sensor is absent is NOT a refusal. With no IMU there is no tilt
        guard, which sensors() states as a fact; whether to drive anyway was already
        settled by question 1.
        """
        stopped = OPSTOP.describe()
        if stopped is not None:
            return stopped
        missing = self.missing_required()
        if missing:
            return ("%s -- required by the current sensor policy. Fix the link, or call "
                    "set_required_sensors() to declare this run does not need %s (see what "
                    "that costs in sensors())"
                    % ("; ".join(self._missing_detail(n) for n in missing),
                       " / ".join(missing)))
        t = self.tilt
        if t.mode == "off":
            return None
        if t.tripped:
            return "tilt guard tripped: " + t.fault["reason"]
        # The guard needs a level reference before it can judge anything, and it can only
        # capture one from a live IMU. If the IMU is required, step 1 already passed, so a
        # missing reference is a real "wait a moment". If it is NOT required, the operator
        # has said to drive without an IMU and an unready guard is not news.
        if "imu" in self.policy.required and not t.ready:
            return "tilt guard has no level reference yet (needs ~0.5 s of a still, live IMU)"
        return None

    #: the original name; test_tilt_guard.py and older callers still use it
    _tilt_refusal = _drive_refusal

    def send_drive(self, left, right):
        # Drive side convention: the dashboard and tools speak true left/right;
        # the swap to mirrored motor wiring happens here, so every PC drive
        # path -- dashboard hold, drive_*, turn_*, jog -- agrees. Per robot:
        # cfg "drive_swap_sides" (a real boolean) wins, else the DRIVE_SWAP_SIDES
        # default in drive_math.py. The handheld stick bypasses this server and
        # is not covered by it.
        swap = self.cfg.get("drive_swap_sides", DRIVE_SWAP_SIDES)
        if not isinstance(swap, bool):
            raise RuntimeError("bad calibration config: 'drive_swap_sides' must be "
                               "true/false, got %r" % (swap,))
        if swap:
            left, right = right, left
        left = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(left)))
        right = max(-MAX_PWM_CEILING, min(MAX_PWM_CEILING, int(right)))
        with self._wlock:  # check and write atomically vs. the trip's "stop" (also under it)
            if left or right:  # a zero command is always allowed -- it's how loops stop
                why = self._drive_refusal()
                if why is not None:
                    self.emergency_stop()
                    raise GuardFault(why, self._guard_info())
            self.send_line(f"{left} {right}")

    def _guard_info(self):
        """The tilt guard, annotated with whether the sensor it reads is even present --
        an "off" guard and one with no IMU look identical without this."""
        tilt = self.tilt.snapshot()
        imu_live = self.sensors.trusted_live("imu")
        tilt["sensor"] = "imu"
        tilt["sensor_live"] = imu_live
        tilt["active"] = tilt["mode"] == "on" and imu_live and tilt["ready"]
        if not imu_live:
            tilt["inactive_because"] = "no live IMU: nothing measures lean"
        elif not tilt["ready"]:
            tilt["inactive_because"] = "no level reference captured yet"
        return {"tilt": tilt, "policy": self.policy.snapshot(self.sensors),
                "operator_stop": OPSTOP.snapshot(), "drive_lease": LEASE.snapshot()}

    def send_stop(self):
        self.send_line("stop")

    def send_listen(self):
        self.send_line("listen")

    def emergency_stop(self):
        """Best-effort stop for atexit/signal handlers. Never raises."""
        try:
            self.send_line("stop")
            self.ser.flush()
        except Exception:
            pass

    def last_counts(self):
        """(left, right) cumulative ticks, or None unless BOTH wheels are live.

        Liveness is EncoderState.available (latched per wheel with a timeout), not the
        last packet's shape: two single-encoder boards interleave half-packets, and a
        partial reading must still be reported as no reading, so callers never
        difference a stale wheel mid-move -- the drive tools decide what to do about it."""
        with self.lock:
            if not all(self.enc.available):
                return None
            return tuple(self.enc.last)

    def wheel_counts(self, encoder="both"):
        """(left, right) cumulative ticks, or None unless the channels `encoder` needs
        are both reporting.

        last_counts() insists on both wheels because its callers difference both. A move
        measured by one nominated encoder needs only that one, so this is the same reading
        with a narrower requirement: the unselected slot may come back None, and nothing
        is allowed to difference it. "Reporting" is the latched per-wheel availability,
        so interleaved single-encoder boards stay measurable and a dead one reads None."""
        with self.lock:
            if not all(self.enc.available[i] for i in selected_wheels(encoder)):
                return None
            return tuple(self.enc.last)

    def range_snapshot(self, now=None):
        """Every ToF channel, or None if no range sensor has ever reported.

        `mm` is the median of the last few samples, which is what anything steering should
        use: the firmware deliberately sends any distance inside the sensor's physical
        range without consulting ST's RangeStatus, so occasional short samples arrive and
        the median is what removes them. `latest_mm` is the raw last sample.

        in_range describes `mm`, the median: false means there is no recent usable reading
        on that channel. latest_in_range describes the last raw ping on its own, which flaps
        near the sensor's limit -- a channel can read in_range=true with
        latest_in_range=false, meaning "something is there, the last ping just missed it".

        Neither is the same as live=false, which means the board has gone quiet. A sensor
        that reports "nothing in reach" and a sensor that reports nothing at all call for
        opposite actions, so they are never collapsed.
        """
        now = now or time.monotonic()
        if not self.tof.channels:
            return None
        channels = {}
        for (name, index) in sorted(self.tof.channels):
            entry = self.tof.channels[(name, index)]
            median = self.tof.median_mm(name, index, now=now)
            stream = self.sensors.get(name)
            channels[bridge.TofState.label(name, index)] = dict(
                sensor=name,
                channel=index,
                mm=median,
                inches=None if median is None else round(median / bridge.MM_PER_IN, 1),
                latest_mm=entry["mm"],
                # About `mm` (the median), not the last ping -- so a channel never reports
                # a distance and "nothing in range" in the same breath.
                in_range=median is not None,
                latest_in_range=not entry["invalid"],
                live=bool(stream and stream.live(now)),
            )
        closest = self.tof.closest_mm(now=now)
        live_any = any(c["live"] for c in channels.values())
        return dict(
            channels=channels,
            closest_mm=closest,
            closest_in=None if closest is None else round(closest / bridge.MM_PER_IN, 1),
            live=live_any,
            # Spelled out because "closest_mm: null" is ambiguous on its own, and the two
            # readings call for opposite actions.
            note=("no range sensor is reporting -- the robot is blind, not clear" if not live_any
                  else "nothing within range of any sensor" if closest is None
                  else "closest obstacle %.0f mm (%.1f in) -- forward cones only; this says "
                       "nothing about the sides, the floor or behind"
                       % (closest, closest / bridge.MM_PER_IN)),
        )

    def telemetry_snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = None if self.last_enc_wall is None else now - self.last_enc_wall
            imu_age = None if self.last_imu_wall is None else now - self.last_imu_wall
            left_ticks, right_ticks = self.enc.last
            heading = self.enc.heading_deg   # None while either encoder is missing
            return dict(
                # A stale count presented as current would read as a stopped wheel;
                # report ticks only for live channels (same latch as availability).
                left_ticks=left_ticks if self.enc.available[0] else None,
                right_ticks=right_ticks if self.enc.available[1] else None,
                # [left, right]: which encoder answered in the last packet. A damaged one
                # is routine, so this is reported, not treated as a fault.
                encoders_available=list(self.enc.available),
                dist_l_in=round(self.enc.dist_l, 3),
                dist_r_in=round(self.enc.dist_r, 3),
                dist_avg_in=round(self.enc.dist_avg, 3),
                # None rather than a number computed from one live wheel and one frozen
                # one, which would look plausible and drift without bound.
                heading_deg=None if heading is None else round(heading, 2),
                imu_last=self.last_imu,
                pitch_deg=round(self.orient.pitch, 2),
                roll_deg=round(self.orient.roll, 2),
                yaw_deg=round(self.orient.yaw, 2),
                yaw_zupt=self.orient.zupt_snapshot(),
                tilt=self.tilt.snapshot(),
                telemetry_age_s=None if age is None else round(age, 2),
                telemetry_live=age is not None and age < TELEMETRY_STALE_S,
                imu_age_s=None if imu_age is None else round(imu_age, 2),
                imu_live=imu_age is not None and imu_age < TELEMETRY_STALE_S,
                range=self.range_snapshot(now),
                sensors=self.sensors.summary(now),
                bridge=dict(dropped_frames=self.sensors.dropped,
                            foreign_frames=self.sensors.foreign),
                stick=(None if self.last_stick_wall is None else dict(
                    left=self.last_stick[0], right=self.last_stick[1],
                    age_s=round(now - self.last_stick_wall, 2),
                    active=(now - self.last_stick_wall) < 1.0)),
                required_sensors=list(self.policy.required),
                missing_sensors=self.policy.missing(self.sensors, now),
                gear_ratio_used=self.cfg["gear_ratio"],
                track_width_in=self.cfg["track_width_in"],
                track_width_calibrated=self.track_width_calibrated,
            )

    def in_per_tick(self):
        return bridge.in_per_tick(self.cfg)

    def close(self):
        self.emergency_stop()
        self.recorder.stop()
        self._stop_pump.set()
        self.reader.stop()  # joins bridge.py's own reader thread before the port closes
        try:
            self.ser.close()
        except Exception:
            pass


