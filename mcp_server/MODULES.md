# Module contract: driving the robot through the daemon

One program owns the serial port: the daemon behind the operator dashboard.
Every driver — script or agent — is a *client* of the daemon. Copy this
pattern; don't open the port, don't steal the lease, don't retry a refusal.

## 1. Connect as a client

MCP over Streamable HTTP, daemon at `http://127.0.0.1:8765`:

- `take_drive_lease(holder, operator, purpose)` → `lease` token.
  `holder` names the script, `operator` the person watching the robot,
  `purpose` the current move (update it per move; the hub shows it).
- Pass `lease=...` plus `confirmed_safe=true` on every drive call
  (`drive_distance`, `turn_degrees`, `drive_arc`, `drive_until_range`, `jog`).
  `confirmed_safe` means the run was approved (see §4).
- `read_telemetry` / `read_range` / `sensors` need no lease.
- Done, killed, or Ctrl-C → `release_drive_lease`. Handle SIGTERM too so a
  killed script doesn't orphan the lease until its 120s silence timeout.

If the daemon is down, fall back to direct serial (`import robot_server`)
only when no dashboard is running — never fight for the port.

## 2. Sensors are discovered, not assumed

On start, read what's live (`telemetry_live`, channel `live` flags,
guard states). Declare per module:

- **required**: without these the module refuses to start, by name
  (e.g. tag-turn without a camera: "no camera, cannot look").
- **absent-but-ok**: degraded behavior stated up front.

A sensor dying mid-run aborts the run naming the sensor, robot stopped.
Never continue blind, never invent zeros, never fill gaps — log them.

## 3. Announce, act, halt

- One MCP tool call at a time. Log one line per move: intent + params +
  result. The daemon also writes every call to `recordings/trace.jsonl`
  (visible at `/api/trace`) — if your log and the trace disagree, the
  trace wins.
- On any refusal (STOP latched, lease lost, guard trip): halt, say why,
  release. No retry loops, no force-take, no revoke — revoking another
  holder's lease is the hub operator's job, on the hub page.

## 4. Confirmation

- Fixed lists: confirm each move, or approve the bounded run up front
  (named list, stated clearances) with the operator watching the hub —
  per module, documented in the module.
- Continuous sense-act loops (centering, following): approve the run up
  front; the hub STOP latch is the live authority and no module can
  release it.

## 5. Minimal skeleton

```python
lease = take_drive_lease(holder="my_module", operator="stephen",
                          purpose="starting")["lease"]
try:
    announce("reverse 12in at pwm 40")
    r = drive_distance(12, "reverse", 40,
                       confirmed_safe=True, lease=lease)
    if not r.get("ok"):
        halt(f"refused: {r}")
finally:
    release_drive_lease(lease)
```
