#!/usr/bin/env python3
"""The drive lease: one client drives at a time.

The daemon (robot_server.py --http) lets several clients connect at once -- an AI session,
tools/find_object.py, a dashboard. Reading is harmless from any number of them. Driving is
not: two clients each sending moves produce a robot doing neither thing, with each of them
reasoning from a picture the other has already made stale.

So driving takes a lease. A client names itself and the human who is watching the robot,
gets back a token, and passes that token with every drive call. While the lease is held,
a drive call without that token is refused and told who holds it.

What the lease is NOT:
  * permission to move. confirmed_safe still means the operator was asked; the lease only
    says whose moves these are.
  * a stop. The operator's stop (operator_stop.py) is immediate and outranks it. Revoking a
    lease lets a move already in progress run to its own bounded end; if it must end now,
    that is what STOP is for.
  * needed to stop. `stop`, and every read-only tool, work with no lease from any client.

How it ends: the holder releases it, the operator revokes it (POST /lease/revoke), or the
holder goes quiet. Quiet means no drive call and no renew for ttl_s; from then on another
client may take it. A lease is never taken away while its holder has a move in progress --
the holder is blocked inside that tool call and cannot renew -- and a holder that comes
back before anyone else has asked simply carries on.

`required` decides what happens when nobody holds a lease at all. Under stdio there is one
client, so it is False and driving works as it always did. The daemon sets it True.

One lease for the life of the process, like TILT, POLICY and OPSTOP in robot_link.py.
"""

import secrets
import threading
import time


class DriveLease:
    DEFAULT_TTL_S = 120.0
    MIN_TTL_S = 10.0
    MAX_TTL_S = 600.0
    #: how many take/release/revoke events to keep for the status page
    HISTORY = 20

    def __init__(self, required=False, clock=time.monotonic):
        self.required = required
        self._clock = clock
        self._lock = threading.Lock()
        self._tl = threading.local()
        self._token = None
        self._lapsed_token = None   # so a holder who went quiet is told that, not "unknown"
        self.holder = None
        self.operator = None
        self.purpose = None
        self.ttl_s = self.DEFAULT_TTL_S
        self._taken = None
        self._last_seen = None
        self._active = 0            # the holder's drive calls currently in progress
        self.events = []

    # ---- internals (call with self._lock held) --------------------------------------

    def _note(self, what, who, detail=""):
        self.events.append({"event": what, "who": who, "detail": detail,
                            "at": time.strftime("%Y-%m-%dT%H:%M:%S")})
        del self.events[:-self.HISTORY]

    def _quiet_for(self):
        return self._clock() - self._last_seen

    def _stale(self):
        """Held, but its holder has been quiet past the ttl and is not mid-move."""
        return (self._token is not None and self._active == 0
                and self._quiet_for() > self.ttl_s)

    def _drop(self, what, who, detail=""):
        self._note(what, who, detail or "was held by %s" % self.holder)
        self._lapsed_token = self._token if what == "lapsed" else None
        self._token = None
        self.holder = self.operator = self.purpose = None
        self._taken = self._last_seen = None
        self._active = 0

    def _held_by(self):
        return "%s (operator: %s)" % (self.holder, self.operator)

    def _snapshot(self):
        held = self._token is not None
        return {
            "required": self.required,
            "held": held,
            "holder": self.holder,
            "operator": self.operator,
            "purpose": self.purpose,
            "held_for_s": round(self._clock() - self._taken, 1) if held else None,
            "quiet_for_s": round(self._quiet_for(), 1) if held else None,
            "ttl_s": self.ttl_s if held else None,
            "move_in_progress": self._active > 0,
            "available_to_others": (not held) or self._stale(),
            "recent_events": list(self.events[-5:]),
        }

    # ---- the client's side ---------------------------------------------------------

    def take(self, holder, operator, purpose="", ttl_s=None):
        """Returns (ok, info). On success info carries the token under "lease"."""
        holder = (holder or "").strip()
        operator = (operator or "").strip()
        if not holder:
            return False, {"error": "holder must say who is driving (e.g. \"Claude Code "
                                    "session\", \"find_object.py\")"}
        if not operator:
            return False, {"error": "operator must name the person who is watching the robot "
                                    "and can reach the stop. A lease with nobody watching is "
                                    "exactly the case DRIVING_POLICY.md section 1 forbids."}
        ttl = self.DEFAULT_TTL_S if ttl_s is None else float(ttl_s)
        ttl = max(self.MIN_TTL_S, min(self.MAX_TTL_S, ttl))
        with self._lock:
            if self._token is not None:
                if not self._stale():
                    if self._active:
                        when = "it has a move in progress"
                    else:
                        when = ("it was last heard from %.0fs ago and frees up after %.0fs "
                                "of silence" % (self._quiet_for(), self.ttl_s))
                    return False, {
                        "error": ("the drive lease is held by %s; %s. Wait, or ask the "
                                  "operator to revoke it from the daemon's page. You can "
                                  "still read every sensor and call stop."
                                  % (self._held_by(), when)),
                        "drive_lease": self._snapshot(),
                    }
                self._drop("lapsed", self.holder, "quiet %.0fs; taken over by %s"
                           % (self._quiet_for(), holder))
            self._token = secrets.token_hex(8)
            self._lapsed_token = None
            self.holder, self.operator, self.purpose = holder[:80], operator[:80], purpose[:200]
            self.ttl_s = ttl
            self._taken = self._last_seen = self._clock()
            self._note("take", self.holder, "operator: %s" % self.operator)
            return True, {"lease": self._token, "drive_lease": self._snapshot()}

    def check(self, token):
        """May a drive call carrying `token` go ahead? None if so (and the lease is kept
        alive), else the refusal text. Call begin() after a None when the move starts."""
        with self._lock:
            if self._token is not None and token != self._token and self._stale():
                # An abandoned lease must not block the robot for ever.
                self._drop("lapsed", self.holder, "quiet %.0fs" % self._quiet_for())
            if self._token is None:
                if token is not None and token == self._lapsed_token:
                    return ("your drive lease lapsed: you were quiet for longer than its "
                            "ttl and it was given up. Call take_drive_lease again.")
                if token is not None:
                    return ("that drive lease is not current (released, revoked by the "
                            "operator, or from before a daemon restart). Call "
                            "take_drive_lease again.")
                if self.required:
                    return ("no drive lease. This server lets one client drive at a time: "
                            "call take_drive_lease(holder=..., operator=...) and pass the "
                            "token it returns as lease=... on every drive call.")
                return None
            if token == self._token:
                self._last_seen = self._clock()
                return None
            if token is None:
                return ("the drive lease is held by %s, so a drive call needs its token "
                        "(lease=...). If that is not you, do not drive: read-only tools and "
                        "stop still work." % self._held_by())
            return ("that is not the current drive lease; it is held by %s. Yours was "
                    "released, revoked or lapsed." % self._held_by())

    def begin(self):
        """Mark this thread's drive call as in progress. Only a call that passed check()
        with the holder's token (or with no lease in play) gets here."""
        with self._lock:
            if self._token is not None:
                self._active += 1
                self._tl.began = self._token

    def end(self):
        """Pair of begin(), safe to call when begin() was not: the drive-tool wrapper's
        `finally` runs for refused calls too."""
        began = getattr(self._tl, "began", None)
        if began is None:
            return
        self._tl.began = None
        with self._lock:
            if self._token == began:   # not revoked while the move ran
                self._active = max(0, self._active - 1)
                self._last_seen = self._clock()

    def renew(self, token):
        with self._lock:
            if self._token is not None and token == self._token:
                self._last_seen = self._clock()
                return True, {"drive_lease": self._snapshot()}
            return False, {"error": "that is not the current drive lease",
                           "drive_lease": self._snapshot()}

    def release(self, token):
        with self._lock:
            if self._token is not None and token == self._token:
                if self._active:
                    return False, {"error": "a move made under this lease is still in "
                                            "progress; release it when that returns",
                                   "drive_lease": self._snapshot()}
                self._drop("release", self.holder)
                return True, {"drive_lease": self._snapshot()}
            return False, {"error": "that is not the current drive lease",
                           "drive_lease": self._snapshot()}

    # ---- the operator's side -------------------------------------------------------

    def revoke(self, who="operator"):
        """Take the lease away. A move in progress runs to its own end; nothing after it
        is accepted from that holder. Returns (was_held, snapshot)."""
        with self._lock:
            was = self._token is not None
            if was:
                self._drop("revoke", who)
            return was, self._snapshot()

    def snapshot(self):
        """Public state. Never includes the token."""
        with self._lock:
            return self._snapshot()
