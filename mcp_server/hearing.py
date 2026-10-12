#!/usr/bin/env python3
"""Listen on the webcam mic and turn one spoken utterance into text.

A background thread keeps `parec` open on a PipeWire/PulseAudio source (the Logitech C310
webcam by default) and holds the last ~30 s of audio in RAM -- nothing is written to disk.
It shuts itself off after IDLE_STOP_S without a capture() call and restarts on the next one.
Keeping the stream open is what makes conversation reliable: speech that starts while the
agent is still deciding what to do next is already in the buffer when capture() begins, and
the noise floor is a rolling quiet-level estimate rather than a one-off snapshot.

Chunks recorded while the robot's own voice is playing (speaking(True) .. speaking(False),
plus a short tail) are marked muted and ignored, so the robot never hears itself.

Utterances are found with an energy gate, then transcribed with faster-whisper (CPU, int8,
model loaded once).

Env:
    ELLAMA_MIC          exact PulseAudio source name (see `pactl list short sources`)
    ELLAMA_MIC_MATCH    substring to pick a source by (default "046d_081b", the C310)
    ELLAMA_WHISPER      faster-whisper model name (default base.en)

Run directly to test one capture:  python3 hearing.py
"""

import math
import os
import subprocess
import threading
import time
from array import array
from collections import deque

RATE = 16000
CHUNK = RATE * 30 // 1000          # 30 ms
CHUNK_S = 0.03
MIC_MATCH = os.environ.get("ELLAMA_MIC_MATCH", "046d_081b")
WHISPER_MODEL = os.environ.get("ELLAMA_WHISPER", "base.en")
IDLE_STOP_S = 300.0
SPEAKER_TAIL_S = 0.35               # room echo after the robot's voice stops

_model = None
_model_lock = threading.Lock()


def find_mic() -> str:
    """PulseAudio source name to record from."""
    if os.environ.get("ELLAMA_MIC"):
        return os.environ["ELLAMA_MIC"]
    out = subprocess.run(["pactl", "list", "short", "sources"], capture_output=True, text=True, timeout=5).stdout
    names = [ln.split("\t")[1] for ln in out.splitlines() if "\t" in ln]
    for n in names:
        if MIC_MATCH in n and not n.endswith(".monitor"):
            return n
    raise RuntimeError(f"no microphone source matching {MIC_MATCH!r}; sources: {names}. "
                       "Set ELLAMA_MIC or ELLAMA_MIC_MATCH.")


def _rms(buf: array) -> float:
    return math.sqrt(sum(x * x for x in buf) / len(buf)) if len(buf) else 0.0


class _Listener:
    def __init__(self):
        self.cv = threading.Condition()
        self.chunks = deque(maxlen=1000)      # (seq, t, muted, rms, samples); ~30 s
        self.seq = 0
        self.floors = deque(maxlen=200)       # rms of recent unmuted chunks, ~6 s
        self.last_floor = 300.0
        self.speaking = False
        self.mute_until = 0.0
        self.last_muted_t = 0.0
        self.consumed_t = 0.0                 # audio up to here has been handed to a capture()
        self.last_use = 0.0
        self.running = False
        self.error = None

    def ensure_running(self):
        with self.cv:
            self.last_use = time.monotonic()
            if self.running:
                return
            self.running = True
            self.error = None
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=preload, daemon=True).start()

    def _run(self):
        proc = None
        try:
            proc = subprocess.Popen(
                ["parec", f"--device={find_mic()}", f"--rate={RATE}", "--channels=1",
                 "--format=s16le", "--latency-msec=30"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            with self.cv:
                self.mute_until = time.monotonic() + 0.25      # start-up click
            while True:
                data = b""
                while len(data) < CHUNK * 2:
                    part = proc.stdout.read(CHUNK * 2 - len(data))
                    if not part:
                        raise RuntimeError("microphone stream ended (is the webcam unplugged?)")
                    data += part
                a = array("h")
                a.frombytes(data)
                r = _rms(a)
                with self.cv:
                    now = time.monotonic()
                    if now - self.last_use > IDLE_STOP_S:
                        return
                    muted = self.speaking or now < self.mute_until
                    if muted:
                        self.last_muted_t = now
                    else:
                        self.floors.append(r)
                    self.seq += 1
                    self.chunks.append((self.seq, now, muted, r, a))
                    self.cv.notify_all()
        except Exception as e:
            self.error = str(e)
        finally:
            if proc:
                proc.kill()
                proc.wait()
            with self.cv:
                self.running = False
                self.cv.notify_all()

    def floor(self) -> float:
        """Quiet level of the room: 20th percentile of recent unmuted chunks (robust to speech)."""
        with self.cv:
            if len(self.floors) >= 10:
                s = sorted(self.floors)
                self.last_floor = s[len(s) // 5]
            return self.last_floor


_listener = _Listener()


def speaking(on: bool):
    """Tell the listener the robot's own voice is (not) playing, so it is not heard."""
    with _listener.cv:
        _listener.speaking = on
        if not on:
            _listener.mute_until = max(_listener.mute_until, time.monotonic() + SPEAKER_TAIL_S)


def mute_for(seconds: float):
    """Ignore the mic briefly (e.g. for our own acknowledgement beep)."""
    with _listener.cv:
        _listener.mute_until = max(_listener.mute_until, time.monotonic() + seconds)


def capture(max_seconds: float = 20.0, wait_seconds: float = 10.0, silence_seconds: float = 0.8,
            on_event=None, on_level=None) -> dict:
    """Wait for someone to talk, then record until they pause.

    Starts from the end of the robot's last utterance (or the last capture), so speech that
    began before this call is still caught. wait_seconds: give up if speech hasn't started
    by then. silence_seconds: pause length that ends the utterance. max_seconds: cap on the
    utterance. on_event("speech_start" | "speech_end"), on_level(rms, threshold) are optional
    callbacks. Returns {"pcm": array('h') or None, "floor", "threshold", "peak", "reason"}.
    """
    L = _listener
    L.ensure_running()
    t0 = time.monotonic()
    start_t = L.last_muted_t if t0 - L.last_muted_t < 30 else t0 - 0.5
    start_t = max(start_t, L.consumed_t)
    with L.cv:
        nxt = next((c[0] for c in L.chunks if c[1] > start_t), L.seq + 1)

    preroll = deque(maxlen=14)             # 0.4 s kept from before speech was detected
    pcm = array("h")
    started, loud, quiet, peak, thr, floor = False, 0, 0.0, 0.0, 700.0, L.floor()
    while True:
        with L.cv:
            while True:
                L.last_use = time.monotonic()
                c = next((x for x in L.chunks if x[0] == nxt), None)
                if c:
                    break
                if L.chunks and L.chunks[0][0] > nxt:      # fell off the back of the buffer
                    nxt = L.chunks[0][0]
                    continue
                if not L.running:
                    raise RuntimeError(L.error or "microphone listener is not running")
                if not started and time.monotonic() - t0 > wait_seconds:
                    L.consumed_t = time.monotonic()
                    return {"pcm": None, "floor": floor, "threshold": thr, "peak": peak, "reason": "no_speech"}
                L.cv.wait(0.1)
        nxt += 1
        _, t, muted, lvl, a = c
        if muted:
            preroll.clear()
            loud = 0
            continue
        peak = max(peak, lvl)
        if not started:
            floor = L.floor()
            thr = max(floor * 2.5, floor + 400, 700)
            if on_level:
                on_level(lvl, thr)
            preroll.append(a)
            loud = loud + 1 if lvl > thr else 0
            if loud >= 3:                                  # ~90 ms of sound in a row, not a click
                started = True
                for p in preroll:
                    pcm.extend(p)
                if on_event:
                    on_event("speech_start")
        else:
            pcm.extend(a)
            if on_level:
                on_level(lvl, thr)
            quiet = quiet + CHUNK_S if lvl < thr * 0.6 else 0.0
            reason = ("pause" if quiet >= silence_seconds
                      else "max_seconds" if len(pcm) / RATE > max_seconds else None)
            if reason:
                L.consumed_t = t
                if on_event:
                    on_event("speech_end")
                return {"pcm": pcm, "floor": floor, "threshold": thr, "peak": peak, "reason": reason}


def transcribe(pcm: array) -> str:
    global _model
    import numpy as np
    with _model_lock:
        if _model is None:
            from faster_whisper import WhisperModel
            _model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
        audio = np.frombuffer(pcm.tobytes(), dtype=np.int16).astype(np.float32) / 32768.0
        segs, _ = _model.transcribe(audio, language="en", vad_filter=True,
                                    condition_on_previous_text=False, beam_size=5)
        return " ".join(s.text.strip() for s in segs).strip()


def preload():
    """Load Whisper now so the first hear() call doesn't pay for it."""
    try:
        transcribe(array("h", [0] * RATE))
    except Exception:
        pass


if __name__ == "__main__":
    print("mic:", find_mic())
    print("Say something...")
    r = capture(on_event=lambda e: print(" ", e))
    print({k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items() if k != "pcm"})
    if r["pcm"] is not None:
        print(f"{len(r['pcm']) / RATE:.1f}s captured")
        print("heard:", repr(transcribe(r["pcm"])))
