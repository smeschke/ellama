#!/usr/bin/env python3
"""Does a decision model know when this robot has stopped moving?

Phase 0 of trying llama-router's decision endpoint (/v1/systemone) on eLlama: replay the
already-labelled runs in calibration/*characterization*.csv as sensor states, ask one
noul question, and score the returned probabilities against ground truth. No robot moves.

Ground truth is the `ok` column those sweeps wrote: a run that hit its timeout having
travelled 0.22 in, or spun 2.7 deg, was stalled. The model is asked only what a reflex
layer would see -- the command, the elapsed time and the encoder/IMU readings -- never the
`ok` or `hit_timeout` columns.

What the output tells you:
  AUC ~0.5          the probabilities carry no signal about stalling
  AUC high, acc low the model ranks correctly but leans -- exactly what a calibration fixes
  both high         it works uncalibrated on this question wording

Needs a key from the dashboard ("My API keys"):
  export LLAMA_ROUTER_KEY=lr_...
  tools/decision_test.py --model llama3.2:3b --dump-first
  tools/decision_test.py --model llama3.2:3b --style snapshot --out /tmp/snap.csv
  tools/decision_test.py --model llama3.2:3b --emit-fit /tmp/fit.json   # calibration body
"""

import argparse, csv, glob, json, os, statistics, sys, time, urllib.error, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_URL = "https://llama-router.pdxhackerspace.org"

# Kept byte-identical between the scoring run and the calibration fit: llama-router keys a
# calibration on the model plus the question wording, so changing one character here makes
# any calibration you fitted no longer apply.
QUESTION_ID = "moving"

# Wording matters more than the model here, and llama-router keys a calibration on the
# exact string, so these are fixed presets rather than a free-text flag.
#
#   commanded  the first wording tried, and a trap: "as commanded" asks about command
#              FIDELITY, so a run that overshot 12 in by 4 in scores 0.03 while one that
#              landed on 12.18 in scores 0.94. It ranks a fast overshoot as stuck.
#   turning    motion only, nothing about accuracy -- what a stall detector actually wants.
#   progress   motion in the commanded sense, still not about hitting a target.
#   stuck      the same question negated, to see whether the lean follows the statement
#              or the answer token.
QUESTIONS = {
    "commanded": "The robot is moving as commanded",
    "turning":   "The wheels are turning",
    "progress":  "The robot is making progress",
    "stuck":     "The robot is stuck",
}
# The statement is true when the robot MOVED, except for "stuck", where it is inverted.
INVERTED = {"stuck"}
QUESTION = None   # set from --question

CAUSE_ID = "cause"
CAUSE = {"type": "choice",
         "instructions": "Why is the robot not making progress",
         "criteria": {"stalled": "the motors are loaded but the wheels are not turning",
                      "blocked": "the wheels are turning but the robot is not advancing",
                      "fine": "the robot is moving normally"}}


def num(row, key):
    v = (row.get(key) or "").strip()
    try:
        return float(v)
    except ValueError:
        return None


def truthy(v):
    return (v or "").strip().lower() in {"true", "1", "yes"}


def fmt(v, unit, nd=1):
    """A missing reading is "not reporting", never 0 -- a fabricated zero reads as a
    stationary wheel, which is the one thing this question must not get wrong."""
    return "not reporting" if v is None else f"{v:.{nd}f} {unit}"


def secs(v):
    return "unknown" if v is None else f"{v:.1f} s"


def load_rows(style, snapshot_at=1.5):
    """Every labelled run in calibration/, as (state, label, tag).

    label True  = moved as commanded.
    label False = stalled (the sweep's `ok` column said so).

    style "summary"  the whole run: command, target, elapsed, what the sensors read.
    style "snapshot" what a reflex layer sees mid-move: no target, progress so far only.
      Progress is interpolated linearly to `snapshot_at` seconds, which is honest for a
      stall (zero stays zero) and approximate for a healthy run that was still ramping.
    """
    out = []
    for path in sorted(glob.glob(os.path.join(REPO, "calibration", "*.csv"))):
        base = os.path.basename(path)
        kind = ("turn" if "turn_" in base else
                "speed" if "speed_" in base else
                "drift" if "drift_" in base else None)
        if kind is None:
            continue
        with open(path, newline="") as fh:
            for i, row in enumerate(csv.DictReader(fh)):
                if "ok" not in row:
                    continue
                label = truthy(row["ok"])
                pwm = num(row, "pwm")
                direction = (row.get("direction") or "?").strip()
                if pwm is None:
                    continue
                elapsed = num(row, "elapsed_s")
                tag = f"{base.split('_unit')[0]}:{i}:pwm{pwm:.0f}:{direction}"
                if style == "snapshot" and not elapsed:
                    # No time base, so progress cannot be placed at snapshot_at without
                    # inventing a speed. The drift sweep logged no elapsed_s.
                    continue
                t = min(snapshot_at, elapsed) if elapsed else snapshot_at
                scale = (t / elapsed) if (elapsed and elapsed > 0) else 1.0

                if kind == "turn":
                    imu, enc = num(row, "actual_degrees_turned"), num(row, "encoder_degrees_estimate")
                    if style == "summary":
                        state = (f"Command: spin {direction} in place, pwm {pwm:.0f}, "
                                 f"target {fmt(num(row, 'requested_degrees'), 'deg', 0)}. "
                                 f"Elapsed {secs(elapsed)}. "
                                 f"IMU yaw change: {fmt(imu, 'deg')}. "
                                 f"Encoder-differential heading estimate: {fmt(enc, 'deg')}.")
                    else:
                        state = (f"Command: spin {direction} in place at pwm {pwm:.0f}, "
                                 f"issued {t:.1f} s ago. "
                                 f"IMU yaw change since the command: "
                                 f"{fmt(None if imu is None else imu * scale, 'deg')}. "
                                 f"Encoder-differential heading estimate: "
                                 f"{fmt(None if enc is None else enc * scale, 'deg')}.")
                else:
                    actual = num(row, "actual_in")
                    extra = ""
                    if kind == "drift":
                        l, r, yaw = num(row, "left_in"), num(row, "right_in"), num(row, "imu_yaw_deg")
                        extra = (f" Left wheel {fmt(l, 'in')}, right wheel {fmt(r, 'in')}. "
                                 f"IMU yaw change: {fmt(yaw, 'deg')}.")
                    if style == "summary":
                        tgt = num(row, "requested_in")
                        state = (f"Command: drive {direction}, pwm {pwm:.0f}"
                                 + (f", target {tgt:.1f} in" if tgt is not None else "") + ". "
                                 + (f"Elapsed {secs(elapsed)}. " if elapsed else "")
                                 + f"Encoder distance travelled: {fmt(actual, 'in', 2)}.{extra}")
                    else:
                        state = (f"Command: drive {direction} at pwm {pwm:.0f}, issued "
                                 f"{t:.1f} s ago. Encoder distance since the command: "
                                 f"{fmt(None if actual is None else actual * scale, 'in', 2)}.")
                out.append((state, label, tag))
    return out


def ask(url, key, model, state, want_cause, timeout):
    questions = {QUESTION_ID: QUESTION}
    if want_cause:
        questions[CAUSE_ID] = CAUSE
    body = json.dumps({"model": model, "state": state, "questions": questions}).encode()
    req = urllib.request.Request(f"{url}/v1/systemone", body, {
        "Content-Type": "application/json", "API-KEY": key,
        "Authorization": f"Bearer {key}",
    })
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r), time.monotonic() - t0, None
    except urllib.error.HTTPError as e:
        return None, time.monotonic() - t0, f"HTTP {e.code}: {e.read()[:300].decode('utf-8', 'replace')}"
    except Exception as e:
        return None, time.monotonic() - t0, repr(e)


def probability(payload):
    """P(statement is true) out of the answer, without assuming one field name.

    The README documents a noul answer as carrying `noul`, but gives no literal response
    body, so accept the nearby shapes rather than crashing on the real one.
    """
    ans = ((payload.get("answers") or {}).get(QUESTION_ID)) if isinstance(payload, dict) else None
    if ans is None:
        ans = payload
    if isinstance(ans, (int, float)):
        return float(ans)
    if not isinstance(ans, dict):
        return None
    for k in ("noul", "probability", "p", "true", "value"):
        v = ans.get(k)
        if isinstance(v, (int, float)):
            return float(v)
        if isinstance(v, dict):                       # e.g. {"noul": {"true": 0.9}}
            for kk in ("true", "probability", "p"):
                if isinstance(v.get(kk), (int, float)):
                    return float(v[kk])
    probs = ans.get("probabilities")
    if isinstance(probs, dict):
        for kk in ("true", "True", "yes"):
            if isinstance(probs.get(kk), (int, float)):
                return float(probs[kk])
    return None


def auc(pairs):
    """P(a moving run scores above a stalled one), ties counted as half. 0.5 = no signal."""
    pos = [p for p, y in pairs if y]
    neg = [p for p, y in pairs if not y]
    if not pos or not neg:
        return None
    wins = sum(1.0 if a > b else 0.5 if a == b else 0.0 for a in pos for b in neg)
    return wins / (len(pos) * len(neg))


def best_threshold(pairs):
    cuts = sorted({p for p, _ in pairs} | {0.0, 1.0})
    best = (0.0, 0.5)
    for c in cuts:
        acc = sum(1 for p, y in pairs if (p >= c) == y) / len(pairs)
        if acc > best[0]:
            best = (acc, c)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help="model id as the dashboard lists it")
    ap.add_argument("--question", choices=sorted(QUESTIONS), default="turning",
                    help="which statement to ask (default: turning)")
    ap.add_argument("--url", default=os.environ.get("LLAMA_ROUTER_URL", DEFAULT_URL))
    ap.add_argument("--style", choices=["summary", "snapshot"], default="summary")
    ap.add_argument("--snapshot-at", type=float, default=1.5,
                    help="seconds into the move that --style snapshot describes")
    ap.add_argument("--cause", action="store_true",
                    help="also ask the choice question (not scored; costs nothing extra "
                         "in round trips)")
    ap.add_argument("--limit", type=int, help="only the first N runs")
    ap.add_argument("--timeout", type=float, default=180,
                    help="high by default: every provider is idle, so the first call pays "
                         "the model load")
    ap.add_argument("--out", help="write per-run probabilities to this CSV")
    ap.add_argument("--emit-fit", metavar="PATH",
                    help="write a /api/decisions/calibrations/fit body from these runs "
                         "instead of fitting (the calibration API wants a dashboard "
                         "session, not an lr_ key)")
    ap.add_argument("--dump-first", action="store_true",
                    help="print the first raw response -- the response shape is not in "
                         "the docs, so look at it once")
    args = ap.parse_args()

    global QUESTION
    QUESTION = {"type": "noul", "instructions": QUESTIONS[args.question]}
    inverted = args.question in INVERTED

    rows = load_rows(args.style, args.snapshot_at)
    if inverted:
        # Keep `label` meaning "the statement is true", so the scoring below is unchanged.
        rows = [(st, not y, tag) for st, y, tag in rows]
    if args.limit:
        rows = rows[:args.limit]
    if not rows:
        sys.exit("no labelled runs found in calibration/*.csv")

    if args.emit_fit:
        body = {"model": args.model, "question": QUESTION,
                "examples": [{"state": s, "answer": y} for s, y, _ in rows]}
        with open(args.emit_fit, "w") as fh:
            json.dump(body, fh, indent=2)
        print(f"{len(rows)} labelled examples -> {args.emit_fit}")
        print(f"  POST it to {args.url}/api/decisions/calibrations/fit")
        print("  Hold runs back for testing by hand if you want a clean held-out score.")
        return

    key = os.environ.get("LLAMA_ROUTER_KEY") or os.environ.get("LLAMA_ROUTER_API_KEY")
    if not key:
        sys.exit("set LLAMA_ROUTER_KEY=lr_... (dashboard -> My API keys)")

    n_pos = sum(1 for _, y, _ in rows if y)
    print(f"{len(rows)} labelled runs: {n_pos} where the statement is true, "
          f"{len(rows) - n_pos} where it is false"
          + ("  (statement inverted: true = stalled)" if inverted else ""))
    print(f"model {args.model}  style {args.style}  -> {args.url}/v1/systemone")
    print(f'question: "{QUESTION["instructions"]}"\n')

    pairs, lat, failures, records = [], [], 0, []
    for i, (state, label, tag) in enumerate(rows, 1):
        payload, dt, err = ask(args.url, key, args.model, state, args.cause, args.timeout)
        lat.append(dt)
        if err:
            failures += 1
            print(f"[{i:3}/{len(rows)}] {tag:28} FAILED {dt:6.1f}s  {err}")
            if failures >= 3 and not pairs:
                sys.exit("\nthree failures and no successes -- stopping. Check the key, "
                         "the model id, and whether this provider exposes logprobs.")
            continue
        if args.dump_first and not pairs:
            print("--- first raw response ---")
            print(json.dumps(payload, indent=2)[:2000])
            print("--- end ---\n")
        p = probability(payload)
        if p is None:
            failures += 1
            print(f"[{i:3}/{len(rows)}] {tag:28} no probability found in: "
                  f"{json.dumps(payload)[:200]}")
            continue
        pairs.append((p, label))
        records.append(dict(tag=tag, label=label, p=p, latency_s=round(dt, 2), state=state))
        truth = "TRUE " if label else "false"
        flag = "  <-- wrong side of 0.5" if (p >= 0.5) != label else ""
        print(f"[{i:3}/{len(rows)}] {tag:28} truth {truth}  p={p:.3f}  {dt:5.1f}s{flag}")

    if args.out and records:
        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(records[0]))
            w.writeheader()
            w.writerows(records)
        print(f"\nper-run results -> {args.out}")

    if not pairs:
        sys.exit("\nno usable answers.")

    pos = [p for p, y in pairs if y]
    neg = [p for p, y in pairs if not y]
    a = auc(pairs)
    acc, cut = best_threshold(pairs)
    brier = statistics.fmean((p - (1.0 if y else 0.0)) ** 2 for p, y in pairs)

    print(f"\n{'-' * 62}\nscored {len(pairs)} runs" + (f", {failures} failed" if failures else ""))
    if pos:
        print(f"  moving  mean p {statistics.fmean(pos):.3f}  "
              f"range {min(pos):.3f}-{max(pos):.3f}  n={len(pos)}")
    if neg:
        print(f"  stalled mean p {statistics.fmean(neg):.3f}  "
              f"range {min(neg):.3f}-{max(neg):.3f}  n={len(neg)}")
    print("  AUC n/a (one class only)" if a is None else f"  AUC {a:.3f}   (0.5 = no signal)")
    print(f"  accuracy at p>=0.5: {sum(1 for p, y in pairs if (p >= 0.5) == y) / len(pairs):.3f}")
    print(f"  best accuracy {acc:.3f} at threshold {cut:.3f}")
    print(f"  Brier {brier:.3f}   (lower is better; 0.25 = always saying 0.5)")
    print(f"  latency median {statistics.median(lat):.2f}s  min {min(lat):.2f}s  "
          f"max {max(lat):.2f}s")
    if a is not None and a > 0.75 and abs(cut - 0.5) > 0.15:
        print("\n  Ranks well but the useful threshold is not 0.5 -- that gap is the lean a\n"
              "  calibration removes. Next: --emit-fit.")


if __name__ == "__main__":
    main()
