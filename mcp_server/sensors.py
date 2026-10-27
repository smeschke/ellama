"""Which sensors this robot actually has right now, and which ones it is required to have.

Pure logic, no serial and no threads of its own -- RobotLink's pump feeds it, tools read it,
so it can be unit-tested without hardware (see test_sensors.py).

Why a registry instead of a fixed list. The firmware's SensorPacket carries the sender's own
name (firmware/libraries/LlamaSensors/LlamaSensors.h), so the bridge relays sensors it
has never heard of and needs no table. This mirrors that on the PC side: a stream appears the
first time a packet with its name arrives, and nothing here enumerates the sensors that exist.
Flashing a second ToF board as "TOF_REAR" needs no change to this file, robot_server.py, or
anything else -- it simply shows up.

Three states, kept distinct everywhere. Collapsing any two of them loses the thing the
operator needs to know:

A fourth state exists because the serial link tears. 115200 baud with ~10 kB/s of text
through it drops the odd byte, so "IMU 194 -122 16028" can arrive split, leaving "MU 150
-128 ..." on the next line -- which looks exactly like a SensorPacket from a sensor named
MU and registered as one. Observed on the real robot. So a stream is PROVISIONAL until it
has been seen SAMPLES_TO_TRUST times: a tear is a one-off, a sensor transmits over and
over. Provisional streams are listed separately, never satisfy a required-sensor policy,
and are forgotten once they go quiet. The cost is that a genuinely new board takes two
packets to be trusted, which at 5 Hz is a third of a second.

  live, measuring         the sensor is reporting usable values
  live, cannot measure    it is reporting, but this channel has nothing to give --
                          a ToF with nothing in range (SENSOR_VALUE_INVALID), or an
                          encoder that is not responding (SENSOR_VALUE_UNAVAILABLE).
                          The board is fine. `available` says which channels.
  silent                  nothing has arrived for stale_after_s: board dead, unflashed,
                          out of range, or never fitted.

"Nothing is in front of the robot" and "the robot cannot see" must never read the same.

Required sensors are policy, not a constant. What the robot needs in order to move depends on
the situation, and that is a judgement for the operator or the driving model, made at the time
-- a walled-off lab with a ceiling camera and a marker on the robot's back is fully
instrumented with no sensors on the robot at all, and a server that insists on an IMU there is
simply wrong. SensorPolicy holds that decision; robot_server.py's drive tools enforce it and
report what each missing sensor costs.
"""

import collections
import threading
import time

#: No packet for this long and a stream is silent. Everything on this robot reports at 5 Hz
#: (ToF, one sensor at a time with dead air between) or 50 Hz (encoders, IMU), so 2 s is many
#: missed packets for any of them. Per-stream overrides via SensorRegistry.set_stale_after.
DEFAULT_STALE_AFTER_S = 2.0

#: Arrival times kept per stream for the rate estimate. ~0.4 s at 50 Hz, 4 s at 5 Hz.
RATE_WINDOW = 20

#: Packets a name must produce before it counts as a real sensor rather than a garbled
#: line. Two is enough: a tear corrupts one line, and the same corruption recurring
#: identically is far less likely than any real sensor's second packet.
SAMPLES_TO_TRUST = 2

#: Name groups a policy can require without naming a specific board, so that replacing
#: TOF_FRONT with TOF_LEFT/TOF_RIGHT doesn't invalidate a stored policy. A group matches any
#: stream whose name starts with the prefix.
GROUP_PREFIXES = {"tof": "tof", "range": "tof"}


def canon(name):
    """Stream names are compared case-insensitively: the firmware sends upper case
    ("TOF_FRONT"), people type lower."""
    return str(name).strip().lower()


class SensorStream:
    """One named telemetry stream and when it was last heard from."""

    def __init__(self, name, stale_after_s=DEFAULT_STALE_AFTER_S):
        self.name = canon(name)
        self.stale_after_s = float(stale_after_s)
        self.values = ()
        self.ms = None            # the sender's own millis(), for dt on the sending side
        self.last_wall = None     # time.monotonic() when we received it
        self.samples = 0
        # Per-channel "this channel produced a usable reading", or None for a sensor that
        # does not distinguish -- an IMU either sends a sample or does not.
        self.available = None
        self._times = collections.deque(maxlen=RATE_WINDOW)

    def observe(self, values, ms, available, when):
        self.values = tuple(values)
        self.ms = ms
        self.available = None if available is None else list(available)
        self.last_wall = when
        self.samples += 1
        self._times.append(when)

    def age_s(self, now):
        return None if self.last_wall is None else now - self.last_wall

    def live(self, now):
        age = self.age_s(now)
        return age is not None and age < self.stale_after_s

    @property
    def trusted(self):
        """Seen often enough to be a sensor rather than a torn line. See the module
        docstring: a single packet from a never-before-seen name is not evidence."""
        return self.samples >= SAMPLES_TO_TRUST

    def rate_hz(self):
        """Packets per second over the recent window, or None before there are two."""
        if len(self._times) < 2:
            return None
        span = self._times[-1] - self._times[0]
        return None if span <= 0 else (len(self._times) - 1) / span

    def snapshot(self, now):
        age = self.age_s(now)
        rate = self.rate_hz()
        return dict(
            name=self.name,
            live=self.live(now),
            provisional=not self.trusted,
            age_s=None if age is None else round(age, 2),
            rate_hz=None if rate is None else round(rate, 1),
            samples=self.samples,
            values=list(self.values),
            available=None if self.available is None else list(self.available),
            # A stream can be live while every channel is unable to measure. That is a
            # working sensor reporting "nothing to see", not a failure.
            measuring=None if self.available is None else any(self.available),
            stale_after_s=self.stale_after_s,
        )


class SensorRegistry:
    """Every stream heard from since the link opened, keyed by the name in the packet."""

    def __init__(self, stale_after_s=DEFAULT_STALE_AFTER_S):
        self._lock = threading.Lock()
        self._default_stale = float(stale_after_s)
        # Per-name deadlines set before that sensor has ever transmitted. Kept apart from
        # `streams` so naming a sensor does not invent one: a stream must appear because a
        # packet arrived, never because something asked about it.
        self._stale_overrides = {}
        self.streams = {}
        # computer_bridge.ino's own counters: DROPPED is its receive queue overrunning (the
        # symptom that used to look like a radio fault), FOREIGN is ESP-NOW traffic from
        # other devices on the channel. Both are cumulative since the bridge booted.
        self.dropped = 0
        self.foreign = 0

    def _stream(self, name):
        key = canon(name)
        s = self.streams.get(key)
        if s is None:
            s = self.streams[key] = SensorStream(
                key, self._stale_overrides.get(key, self._default_stale))
        return s

    def observe(self, name, values=(), ms=None, available=None, when=None):
        """Record one packet. Creates the stream if this name has not been seen before."""
        with self._lock:
            self._stream(name).observe(values, ms, available, when or time.monotonic())

    def touch(self, name, when=None):
        """Assert that this sensor is present and was last heard from at `when`, without
        supplying values. For tests and setup code that need a stream to look live or stale
        at a chosen moment; it marks the stream trusted, since the caller is stating the
        sensor exists rather than offering a packet that might be a torn line."""
        with self._lock:
            st = self._stream(name)
            st.last_wall = time.monotonic() if when is None else when
            st.samples = max(st.samples, SAMPLES_TO_TRUST)

    def set_stale_after(self, name, seconds):
        """Set one stream's silence deadline, whether or not it has ever reported."""
        with self._lock:
            key = canon(name)
            self._stale_overrides[key] = float(seconds)
            if key in self.streams:
                self.streams[key].stale_after_s = float(seconds)

    def note_bridge(self, kind, n):
        """A DROPPED / FOREIGN line from computer_bridge.ino."""
        with self._lock:
            if kind == "DROPPED":
                self.dropped += int(n)
            elif kind == "FOREIGN":
                self.foreign += int(n)

    # -- queries -------------------------------------------------------------------------

    def _prune(self, now):
        """Forget provisional streams that have gone quiet. Caller holds the lock.

        A torn line leaves a name behind that no sensor will ever refresh; without this it
        would sit in sensors() forever looking like a board that died."""
        for key in [k for k, st in self.streams.items()
                    if not st.trusted and not st.live(now)]:
            del self.streams[key]

    def get(self, name):
        with self._lock:
            return self.streams.get(canon(name))

    def live(self, name, now=None):
        s = self.get(name)
        return bool(s and s.live(now or time.monotonic()))

    def age_s(self, name, now=None):
        s = self.get(name)
        return None if s is None else s.age_s(now or time.monotonic())

    def resolve(self, required, now=None):
        """Stream names satisfying one required entry: either that exact name, or, for a
        group ("tof"/"range"), every stream whose name starts with the group's prefix.

        Only trusted streams count. A policy must never be satisfied by a name that one
        garbled line invented."""
        want = canon(required)
        prefix = GROUP_PREFIXES.get(want)
        with self._lock:
            trusted = {n for n, st in self.streams.items() if st.trusted}
            if prefix is None:
                return [want] if want in trusted else []
            return sorted(n for n in trusted if n.startswith(prefix))

    def trusted_live(self, name, now=None):
        """Live AND seen more than once -- what "this sensor is present" means."""
        now = now or time.monotonic()
        st = self.get(name)
        return bool(st and st.trusted and st.live(now))

    def missing(self, required, now=None):
        """Which of `required` has no live, trusted stream. A group entry is satisfied by
        any one live member."""
        now = now or time.monotonic()
        out = []
        for want in required:
            names = self.resolve(want, now)
            if not any(self.trusted_live(n, now) for n in names):
                out.append(canon(want))
        return out

    def summary(self, now=None):
        """One compact line per stream, for results that carry telemetry on every call.
        The full detail is in snapshot(), which the sensors() tool returns."""
        now = now or time.monotonic()
        with self._lock:
            self._prune(now)
            items = sorted((n, st) for n, st in self.streams.items() if st.trusted)
        out = {}
        for n, st in items:
            age, rate = st.age_s(now), st.rate_hz()
            out[n] = dict(live=st.live(now),
                          age_s=None if age is None else round(age, 2),
                          rate_hz=None if rate is None else round(rate, 1),
                          measuring=None if st.available is None else any(st.available))
        return out

    def snapshot(self, now=None):
        now = now or time.monotonic()
        with self._lock:
            self._prune(now)
            items = sorted(self.streams.items())
            streams = {n: st.snapshot(now) for n, st in items if st.trusted}
            # Kept apart rather than mixed in: a name seen once is either a sensor you have
            # just switched on, or a torn line. Both are worth seeing; neither is a sensor
            # you can rely on yet.
            provisional = {n: st.snapshot(now) for n, st in items if not st.trusted}
        return dict(streams=streams, provisional=provisional,
                    bridge=dict(dropped_frames=self.dropped, foreign_frames=self.foreign))


class SensorPolicy:
    """Which sensors the drive tools insist on before they will move the robot.

    Not a safety limit in itself -- it is the operator's or the model's standing answer to
    "is this robot instrumented well enough for what we are about to do". Empty means the
    caller is taking full responsibility, which is the right answer when something outside
    the robot (a ceiling camera, a person watching) is closing the loop.
    """

    #: Both of the robot's own feedback sensors: what the closed-loop drive tools were
    #: written against, so starting here changes nothing until somebody decides otherwise.
    DEFAULT = ("enc", "imu")

    def __init__(self, required=DEFAULT):
        self._lock = threading.Lock()
        self.required = [canon(n) for n in required]
        self.set_by = "default"
        self.set_at = None

    def set(self, names, by="tool"):
        with self._lock:
            self.required = sorted({canon(n) for n in names})
            self.set_by = by
            self.set_at = time.monotonic()
            return list(self.required)

    def missing(self, registry, now=None):
        with self._lock:
            required = list(self.required)
        return registry.missing(required, now)

    def snapshot(self, registry=None, now=None):
        with self._lock:
            out = dict(required=list(self.required), set_by=self.set_by)
        if registry is not None:
            out["missing"] = registry.missing(out["required"], now)
        return out


# ---------------------------------------------------------------------------------------
# What a missing sensor costs. Stated here rather than buried in each tool so that a refusal
# and the sensors() listing give the same answer, and so adding a capability means editing
# one table.
# ---------------------------------------------------------------------------------------

CAPABILITY_COST = {
    "imu": [
        "no tilt guard (nothing measures lean)",
        "no bump or slip detection (both need the accelerometer/gyro)",
        "turn_degrees refuses outright: the encoders are not a fallback for a spin in "
        "place, where wheel scrub makes them over-read by ~2x",
        "drive_arc cannot steer its curvature loop",
    ],
    "enc": [
        "no odometry: distance and wheel speed are unknown",
        "drive_distance and drive_arc cannot close the loop and run open-loop on time",
        "no stall or encoder-fault detection",
    ],
    "tof": ["no range readings: nothing on the robot measures distance to an obstacle"],
}


def capability_cost(missing):
    """Flat list of what is lost with these sensors absent, deduplicated, in order."""
    out = []
    for name in missing:
        for line in CAPABILITY_COST.get(canon(name), CAPABILITY_COST.get("tof", [])
                                        if canon(name).startswith("tof") else []):
            if line not in out:
                out.append(line)
    return out
