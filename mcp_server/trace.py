#!/usr/bin/env python3
"""Append-only trace for the daemon: every tool call and model-side event.

One JSON object per line under recordings/trace.jsonl. The dashboard reads
only the tail, so the file can grow forever. Never raises: tracing must not
break driving.
"""

import json
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PATH = REPO_ROOT / "recordings" / "trace.jsonl"

_lock = threading.Lock()


def _path(path=None):
    return Path(path) if path else DEFAULT_PATH


def event(kind, path=None, **fields):
    """Append one event. Fields must be JSON-serializable; unserializable
    values fall back to repr()."""
    try:
        record = {"t": time.time(), "mono": time.monotonic(), "kind": kind}
        for k, v in fields.items():
            try:
                json.dumps(v)
                record[k] = v
            except (TypeError, ValueError):
                record[k] = repr(v)
        p = _path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        line = json.dumps(record, ensure_ascii=False, default=str)
        with _lock:
            with p.open("a", encoding="utf-8") as f:
                f.write(line + "\n")
    except Exception:
        pass


def tail(n=200, path=None):
    """Last n events, oldest first. Reads only the file tail."""
    try:
        p = _path(path)
        with p.open("rb") as f:
            f.seek(max(0, p.stat().st_size - 256 * 1024))
            lines = f.read().decode(errors="replace").splitlines()[-n:]
        out = []
        for raw in lines:
            try:
                obj = json.loads(raw)
                if isinstance(obj, dict):
                    out.append(obj)
            except ValueError:
                continue
        return out
    except OSError:
        return []
