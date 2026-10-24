#!/usr/bin/env python3
"""Lets a local Ollama model call this repo's MCP tools.

Ollama serves models but is not an MCP client, so this is the missing middle: it starts
robot_server.py over stdio, hands the model the tool schemas, runs the calls the model
asks for, and feeds the results back.

Every tool the server has is offered to the model. Which ones need a human is taken from
the server itself: a tool whose schema has a `confirmed_safe` (or `confirmed_level`)
argument is one the server considers consequential, so the bridge prints the exact call and
blocks on input() before dispatching it. A drive tool added to robot_server.py later is
therefore gated automatically, with no change here.

The approval comes from a person, never from the model's own confirmed_safe -- it will
always set that true, because that is what makes the call succeed. DRIVING_POLICY.md
section 1 requires the ask every time, regardless of how small the move is.

Run:  mcp_server/.venv/bin/python mcp_server/ollama_bridge.py
"""

import asyncio, json, os, sys, time, urllib.request

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

MODEL = os.environ.get("OLLAMA_MODEL", "gpt-oss:20b")
OLLAMA = os.environ.get("OLLAMA_HOST", "http://localhost:11434").rstrip("/")
THINK = os.environ.get("OLLAMA_THINK", "low")

# The bridge's own ceilings, independent of the model and of the server's guards. Raise
# them deliberately: ELLAMA_MAX_PWM=200 ELLAMA_MAX_DURATION_S=6 for a longer or faster run.
MAX_PWM = int(os.environ.get("ELLAMA_MAX_PWM", "120"))
MAX_DURATION_S = float(os.environ.get("ELLAMA_MAX_DURATION_S", "4"))

# A jog result carries a full telemetry snapshot -- ~1200 tokens, which is ~17 s of prefill
# on this CPU every time one lands in the conversation. Cap what goes back to the model and
# say it was cut, so it knows the reading is partial rather than assuming that is all of it.
RESULT_MAX_CHARS = int(os.environ.get("ELLAMA_RESULT_MAX_CHARS", "1200"))

# --simple: the model is a translator, not an agent. One tool, one call, no conversation
# history, nothing fed back. "jog forward slowly for a few seconds" -> jog(60, 60, 3.0),
# you approve, it runs, done. Costs ~300 tokens of schema instead of ~10k, so the first
# turn is seconds rather than minutes, and there is no result-summarising turn to invent
# sensor readings in.
SIMPLE_TOOLS = {"jog", "stop"}

SYSTEM_SIMPLE = """Turn the operator's sentence into exactly one jog() call. Nothing else.

jog(left_pwm, right_pwm, duration_s, confirmed_safe=true). Positive PWM drives a side
forward, negative backward. Both the same drives straight; opposite signs spin in place.

  slowly = 50-60      normally = 70-80      quickly = 90-110
  "a moment" = 1s     "a few seconds" = 3s  no duration given = 1s

Examples:
  "jog forward slowly for a few seconds"  -> jog(60, 60, 3.0, true)
  "back up a little"                      -> jog(-60, -60, 1.0, true)
  "spin right for 2 seconds"              -> jog(70, -70, 2.0, true)
  "turn left slowly"                      -> jog(-50, 50, 1.0, true)
  "stop"                                  -> stop()

Call the tool. Do not explain, do not comment, do not ask questions. If the sentence names
no movement at all, say so in one short line and call nothing."""

# Kept byte-identical every turn: any change to text this early in the prompt costs a
# full re-prefill of the whole block (~88s measured), instead of a cache hit (~0.4s).
# Live state therefore arrives as tool results at the END of the conversation, never here.
SYSTEM = """You are operating a small 4WD robot through tools.

A human must approve every move before it runs, so state the exact call you intend and
why. Work upward from low PWM rather than starting at the target, and never reverse a side
without stopping first. stop() is always safe. Positive PWM drives a side forward, so
jog(60, -60, 1.0) spins in place to the right.

Prefer the tool that measures over the one that guesses: tags() gives a tag's bearing
geometrically, read_range() gives millimetres. describe_view() is a small vision model
writing prose about a photo -- useful for what no sensor reports, like clutter or a
doorway, and not to be trusted as an obstacle sensor.

Call sensors() before trusting any reading. Sensors appear and disappear on their own, so
check what is live rather than assuming. Three states that are NOT the same: live and
measuring (working), live but not measuring (reporting nothing -- an empty ToF cone or a
dead encoder), and not live (silent board). A missing reading is null, never 0.

Range is not clearance: two narrow cones point forward and say nothing about the sides,
the floor, a drop, or anything behind.

NEVER state a sensor reading, a distance, a tick count or an angle unless it came back
from a tool call in this conversation. If you were not told, say you were not told, or call
the tool. Do not reconstruct what probably happened -- a plausible number is worse than no
number, because it cannot be told apart from a real one.

When the tools have done what was asked, do not summarise or interpret the result. Reply
with one short line, or just "done". The operator can read the tool output themselves.

Answer in at most three sentences. No tables, no lists, no preamble."""


def ollama_chat(messages, tools):
    body = json.dumps({
        "model": MODEL, "think": THINK, "stream": False, "keep_alive": -1,
        "options": {"num_ctx": 32768}, "messages": messages, "tools": tools,
    }).encode()
    req = urllib.request.Request(f"{OLLAMA}/api/chat", body,
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=1800) as r:
        return json.load(r)


def as_ollama_tool(t):
    return {"type": "function", "function": {
        "name": t.name,
        "description": (t.description or "").strip(),
        # mcp 2.x calls it input_schema; 1.x called it inputSchema.
        "parameters": (getattr(t, "input_schema", None) or getattr(t, "inputSchema", None)
                       or {"type": "object", "properties": {}}),
    }}


def text_of(result):
    out = []
    for block in getattr(result, "content", []) or []:
        out.append(getattr(block, "text", None) or f"[{type(block).__name__}]")
    body = "\n".join(out) or "(no content)"
    if len(body) > RESULT_MAX_CHARS:
        body = (body[:RESULT_MAX_CHARS]
                + f"\n...[cut after {RESULT_MAX_CHARS} chars. Call the specific read tool "
                  f"if you need a field that is not shown.]")
    # Say so explicitly: a tool that failed must not read as a tool that returned nothing.
    return f"TOOL ERROR: {body}" if getattr(result, "is_error", False) else body


def approve(name, args):
    """Ask the operator about this specific move, in the moment. Returns True only on a
    typed 'yes'. Also clamps the call to the bridge's own ceilings, so a hallucinated
    pwm=255 cannot be approved by a distracted yes."""
    pwm = max([abs(int(v)) for k, v in args.items()
               if "pwm" in k and isinstance(v, (int, float))] or [0])
    dur = float(args.get("duration_s") or 0)
    if pwm > MAX_PWM or dur > MAX_DURATION_S:
        print(f"  !! {name} exceeds bridge limits (pwm {pwm} > {MAX_PWM} or "
              f"{dur}s > {MAX_DURATION_S}s) -- not offered for approval")
        return False
    print(f"\n  MOVE REQUESTED: {name}({json.dumps(args)})")
    print("  The robot will move. Is the area clear, nothing to fall off, no one nearby?")
    try:
        answer = input("  type yes to run it: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    if answer == "yes":
        args["confirmed_safe"] = True   # set HERE, by the human, not by the model
        return True
    print("  not approved\n")
    return False


async def main():
    simple = "--simple" in sys.argv
    here = os.path.dirname(os.path.abspath(__file__))
    params = StdioServerParameters(
        command=sys.executable, args=[os.path.join(here, "robot_server.py")],
        env={**os.environ, "ELLAMA_MOTION_GUARD": "on"},
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            found = (await session.list_tools()).tools
            if simple:
                found = [t for t in found if t.name in SIMPLE_TOOLS]
            tools = [as_ollama_tool(t) for t in found]
            known = {t.name for t in found}
            needs_human = {t["function"]["name"] for t in tools
                           if any(k.startswith("confirmed")
                                  for k in (t["function"]["parameters"].get("properties")
                                            or {}))}
            schema_chars = len(json.dumps(tools))
            print(f"server up: {len(tools)} tools "
                  f"({schema_chars} chars, ~{schema_chars // 4} tokens -- the first turn "
                  f"pays this as prefill, later turns reuse it)")
            print(f"  needs your approval ({len(needs_human)}): "
                  f"{', '.join(sorted(needs_human))}")
            print(f"  ceilings: pwm <= {MAX_PWM}, duration <= {MAX_DURATION_S}s")
            print(f"model: {MODEL} (think={THINK})"
                  f"{'   SIMPLE MODE: one jog call per line' if simple else ''}"
                  f"   ctrl-c or 'quit' to exit\n")

            messages = [{"role": "system", "content": SYSTEM_SIMPLE if simple else SYSTEM}]
            while True:
                try:
                    user = input(">>> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    return
                if user in {"quit", "exit", "/bye"}:
                    return
                if not user:
                    continue
                if simple:
                    # Fresh context every line: the system prompt stays cached, only the
                    # one sentence is new, and nothing accumulates to slow the next turn.
                    t0 = time.monotonic()
                    d = ollama_chat(messages[:1] + [{"role": "user", "content": user}], tools)
                    msg = d["message"]
                    print(f"    [{d.get('eval_count', 0)} tok, "
                          f"{time.monotonic() - t0:.1f}s]")
                    calls = msg.get("tool_calls") or []
                    if not calls:
                        print((msg.get("content") or "(no call)").strip() + "\n")
                        continue
                    for call in calls:
                        name = call["function"]["name"]
                        args = call["function"].get("arguments") or {}
                        if name not in {t.name for t in found}:
                            print(f"  !! no such tool {name}\n")
                        elif name in needs_human and not approve(name, args):
                            pass
                        else:
                            print(f"  -> {name}({json.dumps(args)})")
                            try:
                                print(f"     {text_of(await session.call_tool(name, args))[:200]}\n")
                            except Exception as e:
                                print(f"     ERROR: {e!r}\n")
                    continue

                messages.append({"role": "user", "content": user})

                # Keep going while the model wants tools; it answers in prose when done.
                for _ in range(12):
                    t0 = time.monotonic()
                    d = ollama_chat(messages, tools)
                    msg = d["message"]
                    messages.append(msg)
                    pe = d.get("prompt_eval_count", 0)
                    pd = d.get("prompt_eval_duration", 0) / 1e9
                    ec = d.get("eval_count", 0)
                    print(f"    [ctx {pe} tok, prefill {pd:.1f}s, gen {ec} tok, "
                          f"{time.monotonic() - t0:.1f}s]")

                    calls = msg.get("tool_calls") or []
                    if not calls:
                        print((msg.get("content") or "").strip() + "\n")
                        break
                    for call in calls:
                        fn = call["function"]
                        name, args = fn["name"], fn.get("arguments") or {}
                        if name not in known:            # hallucinated tool name
                            result = (f"NO SUCH TOOL: {name}. Use only the tools you were "
                                      f"given.")
                            print(f"  !! no such tool {name}")
                        elif name in needs_human and not approve(name, args):
                            result = ("REFUSED by the human operator. Do not retry it. Ask "
                                      "what they want changed.")
                        else:
                            print(f"  -> {name}({json.dumps(args)})")
                            try:
                                result = text_of(await session.call_tool(name, args))
                            except Exception as e:
                                result = f"ERROR: {e!r}"
                            print(f"     {result[:200]}")
                        messages.append({"role": "tool", "name": name, "content": result})
                else:
                    print("(gave up after 12 tool rounds)\n")


if __name__ == "__main__":
    asyncio.run(main())
