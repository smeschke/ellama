#!/usr/bin/env python3
"""The operator's stop latch: a human said stop, and nothing drives until a human says go.

This exists for the daemon (robot_server.py --http), where the thing driving and the thing
watching are different clients. A tool call can be mid-move, or the model behind it can be
mid-thought, when someone at the robot needs it to stop; they press a button that hits
POST /stop and this latches.

It is deliberately a different thing from the `stop` tool. `stop` is how a drive loop ends
a move, and the next move is one call away. A latched operator stop refuses every nonzero
drive command at RobotLink.send_drive -- the same choke point the tilt guard uses -- and
only the operator's side (POST /resume) releases it. No MCP tool clears it: the client
that was stopped must not be the one that decides it may go again.

One latch for the life of the process, like TILT and POLICY in robot_link.py, so that
connect() or a robot power cycle is not a way round it.
"""

import threading
import time


class OperatorStop:
    #: how many latch/clear events to keep for the status page
    HISTORY = 20

    def __init__(self):
        self._lock = threading.Lock()
        self.latched = False
        self.who = None
        self.reason = None
        self.since_wall = None
        self.events = []

    def _note(self, what, who, reason):
        self.events.append({"event": what, "who": who, "reason": reason,
                            "at": time.strftime("%Y-%m-%dT%H:%M:%S")})
        del self.events[:-self.HISTORY]

    def latch(self, who="operator", reason=""):
        """Latch the stop. Latching while already latched keeps the first cause -- the
        reason someone first stopped the robot is the one worth reading afterwards -- but
        still records that it was pressed again."""
        with self._lock:
            if not self.latched:
                self.latched = True
                self.who = who
                self.reason = reason
                self.since_wall = time.time()
            self._note("stop", who, reason)
            return self._snapshot()

    def clear(self, who="operator"):
        """Release the stop. Returns (was_latched, snapshot)."""
        with self._lock:
            was = self.latched
            self.latched = False
            self.who = self.reason = self.since_wall = None
            self._note("resume", who, "")
            return was, self._snapshot()

    def describe(self):
        """One line for a drive refusal, or None when not latched."""
        with self._lock:
            if not self.latched:
                return None
            why = " (%s)" % self.reason if self.reason else ""
            return ("operator stop latched by %s%s, %.0fs ago -- only the operator can "
                    "release it (the Resume button on the daemon's page, or POST /resume)"
                    % (self.who, why, time.time() - self.since_wall))

    def _snapshot(self):
        return {
            "latched": self.latched,
            "who": self.who,
            "reason": self.reason,
            "latched_for_s": (round(time.time() - self.since_wall, 1)
                              if self.latched else None),
            "recent_events": list(self.events[-5:]),
        }

    def snapshot(self):
        with self._lock:
            return self._snapshot()
