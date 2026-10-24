#!/usr/bin/env python3
"""The smallest thing that lets a local model drive the robot.

No approvals: whatever the model asks for runs immediately. The tilt guard is on. That is a
controlled-environment setup -- nothing here stands between the model and the motors, so
only run it somewhere a wrong move is harmless.

    mcp_server/.venv/bin/python mcp_server/simple_bridge.py
    >>> jog forward slowly for a few seconds

For the version with the approval prompt and the full toolset, use ollama_bridge.py.
"""

import asyncio, json, os, sys, urllib.request

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

MODEL = "muse-glimmer"   # 27.9B dense: slower per token than gpt-oss:20b (MoE), but it
                         # also does vision and was built for local agent work
TOOLS = ["jog", "turn_degrees", "stop",     # the only tools the model is shown
         "read_range", "sensors", "read_telemetry"]

SYSTEM = """Turn the operator's sentence into exactly one tool call.

jog(left_pwm, right_pwm, duration_s, confirmed_safe=true) -- runs for a time.
  Positive PWM drives a side forward, NEGATIVE drives it backward. Copy the signs the
  operator gives you exactly; a dropped minus sign sends the robot the wrong way.
  Same sign on both sides = straight. Opposite signs = spin in place.

turn_degrees(degrees, direction, pwm, confirmed_safe=true) -- spins by angle and stops
  itself. Use this whenever an angle is named. direction is "left" or "right", and
  degrees is always a positive number.

  slowly = 50-60   normally = 70-80   quickly = 90-110
  "a moment" = 1s   "a few seconds" = 3s   nothing said = 1s

  "forward slowly for a few seconds" -> jog(60, 60, 3.0)
  "back up a little"                 -> jog(-60, -60, 1.0)
  "jog at 90 -90 for 2 seconds"      -> jog(90, -90, 2.0)
  "jog at -90 -90 for 2 seconds"     -> jog(-90, -90, 2.0)
  "spin right for 1 second"          -> jog(70, -70, 1.0)
  "spin left 90 degrees"             -> turn_degrees(90, "left", 70)
  "turn right 45 degrees slowly"     -> turn_degrees(45, "right", 55)
  "stop"                             -> stop()

These read something and never move the robot:
  "read the distance sensors" / "what is in front?" -> read_range()
  "what sensors are live?"                          -> sensors()
  "how far has it driven?"                          -> read_telemetry()

If the sentence asks for none of these -- a greeting, a question about yourself -- then
answer it in one short sentence and call nothing. A question is never a reason to move."""

GUARDS = {
    "ELLAMA_TILT_GUARD": "on",      # keep: it is the one that catches a fall
}


def ask_model(sentence, tools):
    """Send one sentence, get back the model's message (which may contain tool calls)."""
    body = json.dumps({
        "model": MODEL,
        "stream": False,
        # muse-glimmer defaults to think="high", which on a dense 28B CPU model means a long
        # reasoning block at a few tokens/sec. Off is right for translating one sentence.
        # It accepts false/low/medium/high/max; gpt-oss takes low/medium/high; qwen3 only bool.
        "think": False,
        "keep_alive": -1,
        "messages": [{"role": "system", "content": SYSTEM},
                     {"role": "user", "content": sentence}],
        "tools": tools,
    }).encode()
    req = urllib.request.Request("http://localhost:11434/api/chat", body,
                                 {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=600) as r:
            return json.load(r)["message"]
    except urllib.error.HTTPError as e:
        # Ollama explains itself in the body; print that instead of a traceback.
        print(f"  ollama said: {e.code} {e.read().decode()[:300]}")
        return {}


async def main():
    here = os.path.dirname(os.path.abspath(__file__))
    server = StdioServerParameters(
        command=sys.executable,
        args=[os.path.join(here, "robot_server.py")],
        env={**os.environ, **GUARDS},
    )

    async with stdio_client(server) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            # Take the schemas straight from the server, keep only the tools we want.
            tools = [{"type": "function",
                      "function": {"name": t.name,
                                   "description": t.description or "",
                                   "parameters": t.input_schema}}
                     for t in (await session.list_tools()).tools if t.name in TOOLS]

            print(f"{MODEL}, tools: {', '.join(TOOLS)}")
            print("tilt guard ON, NO APPROVALS -- it just runs\n")

            while True:
                try:
                    sentence = input(">>> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    return
                if sentence in ("quit", "exit"):
                    return
                if not sentence:
                    continue

                message = ask_model(sentence, tools)
                calls = message.get("tool_calls") or []

                if not calls:
                    print((message.get("content") or "(no call)").strip() + "\n")
                    continue

                for call in calls:
                    name = call["function"]["name"]
                    args = call["function"].get("arguments") or {}
                    args["confirmed_safe"] = True
                    print(f"  {name}({json.dumps(args)})")
                    result = await session.call_tool(name, args)
                    print(f"  {result.content[0].text[:300]}\n")


if __name__ == "__main__":
    asyncio.run(main())
