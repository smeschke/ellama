"""Show engine for the perform() tool: a MIDI file (or just a tempo) in, light-bar frames and
dance moves out. Pure logic -- no serial, no network -- so it can be tested offline; the
tool in robot_server.py owns the strip, the wheels and the clock.

    show = Show.from_midi("song.mid")        # or Show.from_bpm(103, 60)
    LightRender(show).frame(t)               # -> WLED realtime (DRGB) packet for the 27 LEDs
    Dance(pwm=60, spin_pwm=85).move(k)       # -> what the wheels do on beat k
    synth_wav(show, "out.wav")               # chiptune of the MIDI (optional backing track)

The MIDI parser is deliberately tiny (no dependency): format 0/1 files with PPQ timing, tempo
and time-signature meta events, note on/off, channel 10 as drums. Everything else is skipped.
"""
import bisect
import colorsys
import math
import random
import struct
import wave
from collections import namedtuple
from pathlib import Path

Note = namedtuple("Note", "on off pitch vel chan drum")

KICKS = {35, 36}
SNARES = {37, 38, 39, 40}
HATS = {42, 44, 46, 49, 51, 53, 55, 57, 59}


def _vlq(b, i):
    v = 0
    while True:
        c = b[i]
        i += 1
        v = (v << 7) | (c & 0x7F)
        if not c & 0x80:
            return v, i


class Show:
    """Timeline for one performance, in seconds from the start of the music."""

    def __init__(self, notes, beats, beats_per_bar, length_s, bpm_hint=None):
        self.notes = sorted(notes, key=lambda n: n.on)
        self.beats = beats
        self.beats_per_bar = max(1, beats_per_bar)
        self.length_s = length_s
        self.bpm_hint = bpm_hint

    @classmethod
    def from_bpm(cls, bpm, duration_s, beats_per_bar=4):
        step = 60.0 / bpm
        beats = [k * step for k in range(int(duration_s / step) + 1)]
        return cls([], beats, beats_per_bar, duration_s, bpm)

    @classmethod
    def from_midi(cls, path):
        b = Path(path).read_bytes()
        if b[:4] != b"MThd":
            raise ValueError("not a MIDI file (no MThd header)")
        hlen = struct.unpack(">I", b[4:8])[0]
        _fmt, _ntrk, div = struct.unpack(">HHH", b[8:14])
        if div & 0x8000:
            raise ValueError("SMPTE-timed MIDI is not supported (need ticks per quarter note)")
        ppq = div
        i = 8 + hlen
        tempos, sigs, raw = [], [], []
        while i + 8 <= len(b):
            ln = struct.unpack(">I", b[i + 4:i + 8])[0]
            if b[i:i + 4] != b"MTrk":
                i += 8 + ln
                continue
            trk, i = b[i + 8:i + 8 + ln], i + 8 + ln
            tick, j, status, held = 0, 0, 0, {}
            while j < len(trk):
                d, j = _vlq(trk, j)
                tick += d
                c = trk[j]
                if c == 0xFF:  # meta
                    typ = trk[j + 1]
                    ln2, j2 = _vlq(trk, j + 2)
                    data = trk[j2:j2 + ln2]
                    j = j2 + ln2
                    if typ == 0x51 and ln2 == 3:
                        tempos.append((tick, int.from_bytes(data, "big")))
                    elif typ == 0x58 and ln2 >= 2:
                        sigs.append((tick, data[0], 1 << data[1]))
                    elif typ == 0x2F:
                        break
                    continue
                if c in (0xF0, 0xF7):  # sysex
                    ln2, j = _vlq(trk, j + 1)
                    j += ln2
                    continue
                if c & 0x80:
                    status = c
                    j += 1
                kind, ch = status & 0xF0, status & 0x0F
                n = 1 if kind in (0xC0, 0xD0) else 2
                args = trk[j:j + n]
                j += n
                if len(args) < n:
                    break
                if kind == 0x90 and args[1] > 0:
                    held.setdefault((ch, args[0]), []).append((tick, args[1]))
                elif kind in (0x80, 0x90):
                    stack = held.get((ch, args[0]))
                    if stack:
                        t0, vel = stack.pop(0)
                        raw.append((t0, tick, args[0], vel, ch))
            for (ch, p), lst in held.items():  # notes never released: end with the track
                for t0, vel in lst:
                    raw.append((t0, tick, p, vel, ch))

        tempos.sort()
        if not tempos or tempos[0][0] > 0:
            tempos.insert(0, (0, 500000))
        segs, s = [], 0.0
        for k, (tk, us) in enumerate(tempos):
            if k:
                s += (tk - tempos[k - 1][0]) * tempos[k - 1][1] / 1e6 / ppq
            segs.append((tk, s, us))
        seg_ticks = [x[0] for x in segs]

        def tick_to_s(tk):
            tk0, s0, us = segs[bisect.bisect_right(seg_ticks, tk) - 1]
            return s0 + (tk - tk0) * us / 1e6 / ppq

        notes = [Note(tick_to_s(t0), tick_to_s(t1), p, v, ch, ch == 9) for t0, t1, p, v, ch in raw]
        if not notes:
            raise ValueError("MIDI file has no notes")
        end_tick = max(r[1] for r in raw)
        beats, k = [], 0
        while k * ppq <= end_tick:
            beats.append(tick_to_s(k * ppq))
            k += 1
        num, den = (sigs[0][1], sigs[0][2]) if sigs else (4, 4)
        return cls(notes, beats, round(num * 4 / den), max(n.off for n in notes) + 1.0,
                   60e6 / tempos[0][1])


def _hsv(h, s, v):
    return colorsys.hsv_to_rgb(h % 1.0, s, v)


class LightRender:
    """One 27-LED frame per call. Melody/chord notes drop a coloured spot where their pitch sits
    along the bar (low = left, high = right) that fades; kick = warm burst from the centre,
    snare = white flash at both ends, hats = a sparkle; every beat the whole bar pulses (harder
    on the downbeat) in a colour that changes each bar. With no notes (tempo-only show) you get
    just the beat pulses."""
    BAR_COLORS = (0.92, 0.55, 0.12, 0.76)

    def __init__(self, show, n=27, seed=7):
        self.show, self.n, self.c = show, n, n // 2
        self.rng = random.Random(seed)
        self.on_t = [x.on for x in show.notes]
        melody = sorted(x.pitch for x in show.notes if not x.drum)
        if melody:
            lo = melody[int(len(melody) * 0.03)]
            hi = melody[min(len(melody) - 1, int(len(melody) * 0.97))]
            if hi - lo < 12:
                hi = lo + 12
        else:
            lo, hi = 48, 72
        self.lo, self.hi = lo, hi
        self.buf = [[0.0, 0.0, 0.0] for _ in range(n)]
        self.under, self.under_rgb = 0.0, (1.0, 1.0, 1.0)
        self.seek(0.0)

    def seek(self, t):
        self._t = None
        self._ni = bisect.bisect_left(self.on_t, t)
        self._bi = bisect.bisect_left(self.show.beats, t)
        self.active = []

    def pulse(self, strength, rgb=(1.0, 1.0, 1.0)):
        """Whole-bar flash (also used for the lead-in countdown)."""
        self.under, self.under_rgb = max(self.under, strength), rgb

    def _add(self, i, rgb, amount):
        px = self.buf[i]
        for k in range(3):
            px[k] = min(1.0, px[k] + rgb[k] * amount)

    def _spot(self, pos, rgb, amount, width=1.4):
        for i in range(self.n):
            w = math.exp(-((i - pos) / width) ** 2)
            if w > 0.02:
                self._add(i, rgb, amount * w)

    def _hit(self, x):
        n, c = self.n, self.c
        if x.drum:
            if x.pitch in KICKS:
                for i in range(n):
                    if abs(i - c) <= 5:
                        self._add(i, (1.0, 0.4, 0.05), 1.0 - abs(i - c) / 6)
            elif x.pitch in SNARES:
                for i in range(n):
                    if i < 4 or i >= n - 4:
                        self._add(i, (1.0, 1.0, 1.0), 0.9)
            elif x.pitch in HATS:
                self._add(self.rng.randrange(n), (0.8, 0.9, 1.0), 0.6)
            return
        pos = (min(max(x.pitch, self.lo), self.hi) - self.lo) / (self.hi - self.lo) * (n - 1)
        self._spot(pos, _hsv((x.pitch % 12) / 12, 0.85, 1.0), (x.vel / 127) ** 0.7)
        self.active.append(x)

    def frame(self, t):
        dt = 1 / 30 if self._t is None else max(0.0, t - self._t)
        self._t = t
        keep = 0.80 ** (dt * 30)
        for px in self.buf:
            px[0] *= keep
            px[1] *= keep
            px[2] *= keep
        self.under *= 0.70 ** (dt * 30)
        while self._ni < len(self.on_t) and self.on_t[self._ni] <= t:
            self._hit(self.show.notes[self._ni])
            self._ni += 1
        beats = self.show.beats
        while self._bi < len(beats) and beats[self._bi] <= t:
            down = self._bi % self.show.beats_per_bar == 0
            bar = self._bi // self.show.beats_per_bar
            self.pulse(0.8 if down else 0.4, _hsv(self.BAR_COLORS[bar % 4], 0.9, 1.0))
            self._bi += 1
        self.active = [x for x in self.active if x.off > t]
        for x in self.active:  # a held note keeps its spot lit
            pos = (min(max(x.pitch, self.lo), self.hi) - self.lo) / (self.hi - self.lo) * (self.n - 1)
            self._spot(pos, _hsv((x.pitch % 12) / 12, 0.85, 1.0), 0.10 * (x.vel / 127))
        pkt = bytearray([2, 2])  # WLED realtime DRGB, 2 s timeout
        for px in self.buf:
            for k in range(3):
                v = min(1.0, px[k] + self.under * self.under_rgb[k])
                pkt.append(int(255 * v ** 2.2))
        return bytes(pkt)

    def dark(self):
        return bytes([2, 2]) + bytes(3 * self.n)


class Dance:
    """What the wheels do on each beat. Moves are logical (+ = forward) fractions of pwm /
    spin_pwm, turned into raw wheel PWM with `sign` (robot_server.FORWARD_PWM_SIGN). Eight beats
    per phrase: two sways (an arc each way), forward-and-back rock, a spin each way. Every
    phrase nets out to about where it started. Forward effort is touchy and spins need a lot
    more PWM to beat wheel scrub, so they are separate knobs."""
    PHRASE = ("sway_a", "sway_b", "sway_a", "sway_b", "rock_f", "rock_b", "spin_a", "spin_b")
    SWAY_INNER = 0.4

    def __init__(self, pwm=60, spin_pwm=85, sign=-1):
        self.sign = sign
        s, p, q = self.SWAY_INNER, pwm, spin_pwm
        self.moves = {
            "sway_a": (s * p, p), "sway_b": (p, s * p),
            "rock_f": (p, p), "rock_b": (-p, -p),
            "spin_a": (-q, q), "spin_b": (q, -q),
            "hold": (0, 0),
        }

    def move(self, k):
        """(name, left_raw, right_raw) for beat number k (0-based within the dance)."""
        name = self.PHRASE[k % len(self.PHRASE)]
        left, right = self.moves[name]
        return name, int(round(left * self.sign)), int(round(right * self.sign))


def reverses(prev, new):
    """True if either wheel flips direction between two (left, right) commands. The driving
    policy wants a stop in between, so the runner brakes briefly when this is true."""
    return any(a * b < 0 for a, b in zip(prev, new))


def synth_wav(show, path, sr=22050):
    """Render the show's notes as a small chiptune WAV (square-ish lead + simple drums) so
    perform() can play the MIDI itself when you have no recording."""
    import numpy as np
    out = np.zeros(int((show.length_s + 1.0) * sr), np.float32)
    rng = np.random.default_rng(1)

    def put(i0, x):
        i1 = min(len(out), i0 + len(x))
        if i0 < len(out):
            out[i0:i1] += x[:i1 - i0]

    for nt in show.notes:
        i0 = int(nt.on * sr)
        amp = (nt.vel / 127) * 0.12
        if nt.drum:
            if nt.pitch in KICKS:
                t = np.arange(int(0.18 * sr)) / sr
                ph = 2 * np.pi * (45 * t + 105 * (1 - np.exp(-t / 0.03)) * 0.03)
                put(i0, (np.sin(ph) * np.exp(-t / 0.07) * amp * 5).astype(np.float32))
            elif nt.pitch in SNARES:
                t = np.arange(int(0.15 * sr)) / sr
                put(i0, (rng.standard_normal(len(t)) * np.exp(-t / 0.05) * amp * 3).astype(np.float32))
            elif nt.pitch in HATS:
                t = np.arange(int(0.05 * sr)) / sr
                put(i0, (rng.standard_normal(len(t)) * np.exp(-t / 0.012) * amp * 1.5).astype(np.float32))
            continue
        f = 440.0 * 2 ** ((nt.pitch - 69) / 12)
        dur = max(0.08, nt.off - nt.on)
        rel = 0.10
        t = np.arange(int((dur + rel) * sr)) / sr
        s = np.sin(2 * np.pi * f * t)
        wav = 0.65 * s + 0.35 * np.sign(s)
        env = np.minimum(1.0, t / 0.005) * (0.6 + 0.4 * np.exp(-t / 0.08))
        env *= np.clip((dur + rel - t) / rel, 0.0, 1.0)
        put(i0, (wav * env * amp).astype(np.float32))
    peak = float(np.abs(out).max()) or 1.0
    pcm = (out / peak * 0.8 * 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())
    return len(pcm) / sr
