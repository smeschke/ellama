#!/usr/bin/env python3
"""Focused tests for mcp_server/trace.py: append-only JSONL + tail.

Run: ELLAMA_SERIAL_PORT=/dev/none python3 mcp_server/test_trace.py
"""

import json
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trace


def test_event_then_tail_roundtrips():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "trace.jsonl")
        trace.event("tool", path=p, tool="jog", ok=True, elapsed_s=1.2)
        trace.event("model", path=p, model="m", purpose="chat")
        got = trace.tail(10, path=p)
        assert [e["kind"] for e in got] == ["tool", "model"], got
        assert got[0]["tool"] == "jog" and got[0]["ok"] is True


def test_tail_missing_file_is_empty():
    with tempfile.TemporaryDirectory() as d:
        assert trace.tail(10, path=os.path.join(d, "nope.jsonl")) == []


def test_unserializable_values_do_not_raise():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "trace.jsonl")
        trace.event("tool", path=p, tool="x", raw=object())
        got = trace.tail(10, path=p)
        assert got and got[0]["kind"] == "tool", got


_FUNCS = [v for k, v in sorted(globals().items())
          if k.startswith("test_") and callable(v)]


def main():
    failed = 0
    for fn in _FUNCS:
        try:
            fn()
            print("ok %s" % fn.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL %s: %r" % (fn.__name__, e))
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
