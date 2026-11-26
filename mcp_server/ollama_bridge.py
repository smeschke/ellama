#!/usr/bin/env python3
"""Drive-by-chat through llama-router (the same backend Raven uses).

The operator speaks, the model translates every sentence into drive tool
calls, and the calls run. There are no per-move approval prompts: the robot
takes hours to set up and only runs in flat, level lab-like spaces, so the
operator's standing order is that anything they say to this loop is a command
to execute. Every message must therefore produce at least one action call;
a reply with no drive call gets one forced follow-up turn, then gives up.

Standing order, not per-move approval: any tool argument starting with
"confirmed" is set to True here, automatically, and traced. The server's own
guards still apply (tilt guard, drive lease, its PWM ceiling) -- this loop
only skips the human yes/no, nothing on the robot.

Run:  ELLAMA_MCP_URL=http://127.0.0.1:8765 mcp_server/.venv/bin/python mcp_server/ollama_bridge.py
Env:  LLAMA_ROUTER_BASE_URL, LLAMA_ROUTER_API_KEY (same names as Raven),
      ELLAMA_MODEL (else MODEL, else the "chat" router group),
      ELLAMA_MCP_URL (default http://127.0.0.1:8765; --stdio spawns a
      private server instead of using the daemon).
"""

import asyncio, getpass, json, os, sys, time, urllib.request

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# Default member, not the "chat" group: the group flaps between members and
# some members refuse tool calls outright. Override with ELLAMA_MODEL.
MODEL = os.environ.get("ELLAMA_MODEL",
                       os.environ.get("MODEL", "RedHatAI/Qwen3.8-27B-INT4"))

_ROUTER_FALLBACK = None  # (url, key, source) once resolved


def _router_settings():
    """(url, key, source). Environment first; otherwise the same two keys
    from Raven's ~/.env, so a machine that runs Raven needs no extra setup."""
    global _ROUTER_FALLBACK
    url = os.environ.get("LLAMA_ROUTER_BASE_URL", "").rstrip("/")
    key = os.environ.get("LLAMA_ROUTER_API_KEY", "")
    if url and key:
        return url, key, "environment"
    if _ROUTER_FALLBACK is not None:
        return _ROUTER_FALLBACK
    found = {}
    try:
        with open(os.path.expanduser("~/raven/.env")) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                if k.strip() in ("LLAMA_ROUTER_BASE_URL", "LLAMA_ROUTER_API_KEY"):
                    found[k.strip()] = v.strip().strip("\"'")
    except OSError:
        pass
    url = url or found.get("LLAMA_ROUTER_BASE_URL", "").rstrip("/")
    key = key or found.get("LLAMA_ROUTER_API_KEY", "")
    _ROUTER_FALLBACK = (url, key, "~/raven/.env" if found else "environment")
    return _ROUTER_FALLBACK

# Runaway guard, not a task limit: the loop exits as soon as a round has no
# tool calls, so a high cap only binds a model that never stops calling.
# A square needs 8; multi-shape orders need more. Kept high for sequences.
MAX_TOOL_ROUNDS = 25

# A drive result carries a full telemetry snapshot -- ~1200 tokens, which is ~17 s
# of prefill on this CPU every time one lands in the conversation. Cap what goes
# back to the model and say it was cut, so it knows the reading is partial rather
# than assuming that is all of it.
RESULT_MAX_CHARS = int(os.environ.get("ELLAMA_RESULT_MAX_CHARS", "1200"))

# Tools that move (or release) the robot. Anything whose schema has a
# "confirmed*" argument is one the server considers consequential; stop/listen
# change motion state without one. A message counts as obeyed only when at
# least one of these runs.
ALWAYS_ACTION = {"stop", "listen"}

# Lease plumbing the model never sees: the bridge takes one drive lease for
# the whole session and fills the token in itself, so the model just says
# what to do. take/renew/release are therefore not offered as tools.
HIDDEN_TOOLS = {"take_drive_lease", "renew_drive_lease", "release_drive_lease"}

# Who is watching the robot. The server demands a name; the account running
# this loop is the human at the controls, overridable via ELLAMA_OPERATOR.
OPERATOR = os.environ.get("ELLAMA_OPERATOR") or getpass.getuser()


class LeaseFailed(Exception):
    pass

SYSTEM = """Turn the operator's sentence into drive tool calls and make them. Every
sentence is a command: always call at least one action tool per message. Read
tools (sensors, read_telemetry, read_range, guards) are for checking state
before you act, never the whole answer.

Action tools are the drive_* tools plus jog, turn_degrees, drive_arc,
drive_until_range, stop and listen. Their confirmed_* arguments arrive
already true -- do not ask, do not explain, just call. There is no lease
argument on any tool: the bridge holds the drive lease itself. If a drive
call fails because someone else holds the lease, say who holds it and stop.

jog(left_pwm, right_pwm, duration_s): positive PWM drives a side forward,
negative backward. Both the same drives straight; opposite signs spin in place.

Spinning in place needs far more PWM than driving straight: wheel scrub loads
the motors much harder, and below the spin deadband the robot stalls instead
of turning slowly. Two scales -- never use the straight one for a spin.

  straight:       slowly = 50-60   normally = 70-80    quickly = 90-110
  spin in place:  slowly = 70      normally = 80-90    quickly = 100-110
  "a moment" = 1s     "a few seconds" = 3s  no duration given = 1s

Prefer the tool that measures over the one that guesses for exact moves:
drive_distance for inches, turn_degrees for degrees, drive_until_range to
approach something. "stop" -> stop().

Drive FIRST, in the first round, straight from the sentence -- never open with
a read tool to "check state". Only call a read tool when the operator asks
about state, or when a drive call just failed and you need to see why.
Sensors appear and disappear on their own. Three states that are NOT the
same: live and measuring (working), live but not measuring (reporting
nothing), and not live (silent). A missing reading is null, never 0.

NEVER state a sensor reading, a distance, a tick count or an angle unless it
came back from a tool call in this conversation. When the tools have done
what was asked, reply with one short line, or just "done". No tables."""


def _trace_event(kind, **fields):
    """Best-effort model-side trace into the daemon's trace file."""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import trace as _tr
        _tr.event(kind, **fields)
    except Exception:
        pass


def router_chat(messages, tools):
    """One OpenAI-compatible chat call. Returns the assistant message dict."""
    url, key, source = _router_settings()
    if not url or not key:
        raise SystemExit(
            "need LLAMA_ROUTER_BASE_URL and LLAMA_ROUTER_API_KEY in the "
            "environment or ~/raven/.env to reach llama-router")
    body = json.dumps({"model": MODEL, "messages": messages, "tools": tools}).encode()
    req = urllib.request.Request(
        f"{url}/chat/completions", body,
        {"Content-Type": "application/json", "Authorization": f"Bearer {key}"})
    t0 = time.monotonic()
    # llama-router flaps under load (observed transient 404s and 504s on
    # payloads that succeed on retry), so retry those instead of dying.
    last = None
    for attempt in range(3):
        try:
            with urllib.request.urlopen(req, timeout=300) as r:
                data = json.load(r)
            msg = data["choices"][0]["message"]
            _trace_event("model", model=MODEL, purpose="chat",
                         prompt_tokens=(data.get("usage") or {}).get("prompt_tokens"),
                         completion_tokens=(data.get("usage") or {}).get("completion_tokens"))
            print(f"    [{time.monotonic() - t0:.1f}s]")
            content = msg.get("content") or ""
            return {"role": "assistant", "content": content,
                    "tool_calls": msg.get("tool_calls") or []}
        except urllib.error.HTTPError as e:
            last = RuntimeError(f"router HTTP {e.code}: "
                                f"{e.read()[:300].decode(errors='replace')}")
            if e.code not in (404, 429, 500, 502, 503, 504):
                break
            time.sleep(2 * (attempt + 1))
        except Exception as e:
            last = e
            break
    _trace_event("model", model=MODEL, purpose="chat", error=repr(last))
    raise last


def to_openai_tool(t):
    schema = dict(getattr(t, "input_schema", None) or getattr(t, "inputSchema", None)
                  or {"type": "object", "properties": {}})
    # The bridge fills the lease token itself, so the model never sees it.
    props = {k: v for k, v in (schema.get("properties") or {}).items() if k != "lease"}
    schema = {**schema, "properties": props,
              "required": [r for r in (schema.get("required") or []) if r != "lease"]}
    return {"type": "function", "function": {
        "name": t.name,
        "description": (t.description or "").strip(),
        # mcp 2.x calls it input_schema; 1.x called it inputSchema.
        "parameters": schema,
    }}


def is_action(name, props):
    return (name in ALWAYS_ACTION
            or any(k.startswith("confirmed") for k in (props or {})))


def autoconfirm(name, args, props):
    """Standing order: the operator already approved anything they say here."""
    for k in (props or {}):
        if k.startswith("confirmed") and k not in args:
            args[k] = True
    _trace_event("autoconfirm", tool=name, args=args)


def normalize_calls(msg):
    """OpenAI tool_calls -> [(id, name, args)]. Arguments may be a JSON string."""
    out = []
    for call in msg.get("tool_calls") or []:
        fn = call.get("function") or {}
        args = fn.get("arguments") or {}
        if isinstance(args, str):
            try:
                args = json.loads(args) if args.strip() else {}
            except json.JSONDecodeError:
                args = {}
        out.append((call.get("id", ""), fn.get("name", ""), args or {}))
    return out


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


def _payload(result):
    """A tool result as a dict: structured content when the server sends it,
    else the text blocks parsed as JSON."""
    sc = getattr(result, "structured_content", None)
    if isinstance(sc, dict):
        return sc
    try:
        return json.loads("\n".join(getattr(b, "text", None) or ""
                                    for b in (getattr(result, "content", None) or [])))
    except (ValueError, TypeError):
        return {}


def make_lease_manager(session):
    """One drive lease for the session, renewed ahead of every drive call."""
    state = {"token": None}

    async def ensure():
        if state["token"]:
            try:
                if _payload(await session.call_tool(
                        "renew_drive_lease",
                        {"lease": state["token"]})).get("ok"):
                    return state["token"]
            except Exception:
                pass
            state["token"] = None
        try:
            info = _payload(await session.call_tool("take_drive_lease", {
                "holder": "drive-by-chat", "operator": OPERATOR,
                "purpose": "standing order: the operator's commands drive the robot"}))
        except Exception as e:
            raise LeaseFailed(f"could not take the drive lease: {e!r}")
        if info.get("ok") and info.get("lease"):
            state["token"] = info["lease"]
            return state["token"]
        raise LeaseFailed(f"could not take the drive lease: "
                          f"{info.get('error', info) or 'refused'}")

    async def release():
        if state["token"]:
            try:
                await session.call_tool("release_drive_lease",
                                        {"lease": state["token"]})
            except Exception:
                pass
            state["token"] = None

    return ensure, release


async def exec_call(session, call_id, name, args, props, ensure_lease=None):
    if ensure_lease is not None and props is not None and "lease" in props:
        try:
            args["lease"] = await ensure_lease()
        except LeaseFailed as e:
            print(f"  !! {e}")
            return {"role": "tool", "tool_call_id": call_id, "name": name,
                    "content": f"TOOL ERROR: {e}"}
    print(f"  -> {name}({json.dumps(args)})")
    try:
        result = text_of(await session.call_tool(name, args))
    except Exception as e:
        result = f"ERROR: {e!r}"
    print(f"     {result[:200]}")
    return {"role": "tool", "tool_call_id": call_id, "name": name, "content": result}


async def run_rounds(session, chat_fn, messages, tools, props_by_name, max_rounds,
                   ensure_lease=None):
    """Let the model act until a round has no tool calls. Returns True if at
    least one action tool ran."""
    acted = False
    for _ in range(max_rounds):
        msg = chat_fn(messages, tools)
        messages.append({"role": "assistant",
                         "content": msg.get("content") or "",
                         "tool_calls": msg.get("tool_calls") or []})
        calls = normalize_calls(msg)
        if not calls:
            return acted, (msg.get("content") or "").strip()
        for call_id, name, args in calls:
            props = props_by_name.get(name)
            if props is None:
                print(f"  !! no such tool {name}")
                messages.append({"role": "tool", "tool_call_id": call_id,
                                 "name": name,
                                 "content": f"NO SUCH TOOL: {name}. Use only the tools you were given."})
                continue
            autoconfirm(name, args, props)
            messages.append(await exec_call(session, call_id, name, args, props,
                                            ensure_lease))
            acted = acted or is_action(name, props)
    print("(gave up after tool rounds)\n")
    return acted, ""


async def obey(session, chat_fn, messages, tools, props_by_name, rounds,
               ensure_lease=None):
    """One operator message in, at least one action call out."""
    acted, prose = await run_rounds(session, chat_fn, messages, tools,
                                    props_by_name, rounds, ensure_lease)
    if not acted:
        messages.append({"role": "user",
                         "content": "That names a movement. Reply with exactly one "
                                    "drive tool call now, no prose."})
        acted, prose = await run_rounds(session, chat_fn, messages, tools,
                                        props_by_name, 2, ensure_lease)
    if prose:
        print(f"{prose}\n")
    if not acted:
        print("(no drive call -- nothing ran)\n")


async def run_session(read, write, agent):
    async with ClientSession(read, write) as session:
        await session.initialize()
        found = [t for t in (await session.list_tools()).tools
                 if t.name not in HIDDEN_TOOLS]
        tools = [to_openai_tool(t) for t in found]
        props_by_name = {t.name: ((getattr(t, "input_schema", None)
                                   or getattr(t, "inputSchema", None)
                                   or {}).get("properties") or {})
                         for t in found}
        ensure_lease, release_lease = make_lease_manager(session)
        actions = sorted(n for n, p in props_by_name.items() if is_action(n, p))
        print(f"server up: {len(tools)} tools, {len(actions)} action: "
              f"{', '.join(actions)}")
        _rurl, _rkey, _src = _router_settings()
        if not _rurl or not _rkey:
            raise SystemExit(
                "no llama-router credentials: set LLAMA_ROUTER_BASE_URL and "
                "LLAMA_ROUTER_API_KEY in the environment (or ~/raven/.env) before "
                "starting the chat loop")
        print(f"model: {MODEL} via llama-router (router settings from {_src})"
              f"   lease held by the bridge as {OPERATOR}"
              f"   {'AGENT MODE: conversation kept' if agent else 'TRANSLATOR MODE: fresh context per line'}"
              f"   ctrl-c or 'quit' to exit\n")

        history = []
        try:
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
                if agent:
                    history.append({"role": "user", "content": user})
                    await obey(session, router_chat, history, tools, props_by_name,
                               MAX_TOOL_ROUNDS, ensure_lease)
                else:
                    # Fresh context every line: the system prompt stays cached, only
                    # the one sentence is new, and nothing accumulates.
                    await obey(session, router_chat,
                               [{"role": "system", "content": SYSTEM},
                                {"role": "user", "content": user}],
                               tools, props_by_name, MAX_TOOL_ROUNDS,
                               ensure_lease)
        finally:
            await release_lease()


async def main():
    agent = "--agent" in sys.argv
    if "--stdio" in sys.argv:
        here = os.path.dirname(os.path.abspath(__file__))
        params = StdioServerParameters(
            command=sys.executable, args=[os.path.join(here, "robot_server.py")],
        )
        from mcp.client.stdio import stdio_client
        async with stdio_client(params) as (read, write):
            await run_session(read, write, agent)
    else:
        url = os.environ.get("ELLAMA_MCP_URL", "http://127.0.0.1:8765").rstrip("/")
        # Daemon mode: talk to an already-running robot_server.py --http.
        # The daemon owns serial; this loop only translates model <-> tools.
        if not url.endswith("/mcp"):
            url = url + "/mcp"
        print(f"connecting to daemon {url} ...")
        from mcp.client.streamable_http import streamable_http_client
        async with streamable_http_client(url) as (read, write):
            await run_session(read, write, agent)


if __name__ == "__main__":
    asyncio.run(main())
