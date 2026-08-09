#!/usr/bin/env python3
"""Per-burst report for the encoder-shed-lag validation campaign.

Inputs: a sideport jsonl recording (socat|jq -c capture of :8300) and the
campaign driver log (BURST n ON/OFF mono= stamps from shedlag_campaign.sh).
The sideport's t_ms and the driver's mono= are both CLOCK_MONOTONIC, so the
stamps index straight into the recording with no offset arithmetic.

For each burst, reports the leg-2 mechanism end to end:

  cascade_ms   first demote -> lowest rung reached (ctl.last_event)
  floor_rung   lowest rung reached
  shed_lag_ms  first demote -> drone.enc.cmd_kbps reaching the floor rung's
               target (1 Hz telemetry: true value is up to ~1s earlier;
               reported value is the UPPER edge, the pre/post comparison is
               still valid because both builds carry the same quantization)
  txq_drops    drone TxQueue drop-oldest counter delta over the burst window
               (drone-side truth: unaffected by GS-injected loss)
  fps_min      lowest link.video.fps sample in the window
  freeze_ms    time fps spent below 20 (sideport is 2 Hz -> 500ms steps)
  trunc/drop   link.video truncated/dropped AU counter deltas
  s1_abn_pk    peak streams[1].abandoned_s (residual FEC failures/s)

Acceptance for the fix (drone-side shed on decrease + txq flush + self-IDR):
shed_lag_ms bounded by one telemetry period, txq_drops == 0, no fps sample
below 20, freeze_ms == 0. The baseline run establishes the pre-fix numbers.

Usage:
  shedlag_report.py recording.jsonl campaign.log [--window-s 12]
"""

import argparse
import json
import re
import sys


def load_bursts(log_path):
    bursts = []
    on_re = re.compile(r"BURST (\d+) ON mono= (\d+)")
    off_re = re.compile(r"BURST (\d+) OFF mono= (\d+)")
    for line in open(log_path):
        m = on_re.search(line)
        if m:
            bursts.append({"n": int(m.group(1)), "on": int(m.group(2))})
            continue
        m = off_re.search(line)
        if m and bursts and bursts[-1]["n"] == int(m.group(1)):
            bursts[-1]["off"] = int(m.group(2))
    return [b for b in bursts if "off" in b]


def load_samples(jsonl_path):
    out = []
    for line in open(jsonl_path):
        try:
            j = json.loads(line)
        except json.JSONDecodeError:
            continue  # partial first/last line of a live capture
        t = j.get("t_ms")
        if t is None:
            continue
        L = j.get("link") or {}
        d = j.get("drone") or {}
        ctl = L.get("ctl") or {}
        v = L.get("video") or {}
        s1 = next((s for s in (L.get("streams") or []) if s.get("stream") == 1), {})
        ev = ctl.get("last_event") or {}
        out.append({
            "t": t,
            "rung": (ctl.get("rung") or {}).get("idx"),
            "ev_t": ev.get("t_ms"), "ev_to": ev.get("to"),
            "ev_reason": ev.get("reason"),
            "cmd": (d.get("enc") or {}).get("cmd_kbps"),
            "txq_drops": (d.get("txq") or {}).get("drops"),
            "fps": v.get("fps"), "trunc": v.get("truncated"),
            "drop": v.get("dropped"), "abn_s": s1.get("abandoned_s"),
        })
    out.sort(key=lambda s: s["t"])
    return out


def in_window(samples, a, b):
    return [s for s in samples if a <= s["t"] <= b]


def burst_report(samples, on_ms, window_ms, floor_targets):
    w = in_window(samples, on_ms - 2000, on_ms + window_ms)
    if not w:
        return None
    # demote events inside the window, from the last_event stamps
    evs = {}
    for s in w:
        if s["ev_t"] and s["ev_t"] >= on_ms - 500:
            evs[s["ev_t"]] = (s["ev_to"], s["ev_reason"])
    demotes = sorted((t, to, r) for t, (to, r) in evs.items()
                     if r not in ("promote",))
    if not demotes:
        return None
    t_first = demotes[0][0]
    floor = min(to for _, to, _ in demotes)
    t_floor = next(t for t, to, _ in demotes if to == floor)
    # cmd_kbps reaching the floor target (1 Hz telemetry upper edge)
    target = floor_targets.get(floor)
    t_shed = None
    if target is not None:
        for s in w:
            if s["t"] >= t_first and s["cmd"] is not None and s["cmd"] <= target:
                t_shed = s["t"]
                break
    def delta(key):
        vals = [s[key] for s in w if s[key] is not None]
        return (vals[-1] - vals[0]) if len(vals) >= 2 else None
    fps = [s["fps"] for s in w if s["fps"] is not None and s["t"] >= t_first]
    freeze_ms = 500 * sum(1 for f in fps if f < 20.0)
    abn = [s["abn_s"] for s in w if s["abn_s"] is not None]
    return {
        "cascade_ms": t_floor - t_first, "floor": floor,
        "shed_lag_ms": (t_shed - t_first) if t_shed else None,
        "txq_drops": delta("txq_drops"),
        "fps_min": min(fps) if fps else None, "freeze_ms": freeze_ms,
        "trunc": delta("trunc"), "drop": delta("drop"),
        "s1_abn_pk": max(abn) if abn else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("campaign_log")
    ap.add_argument("--window-s", type=float, default=12.0,
                    help="analysis window after burst ON (default 12)")
    ap.add_argument("--floor-target", action="append", default=[],
                    metavar="RUNG=KBPS",
                    help="cmd_kbps target per floor rung (default 0=1300)")
    args = ap.parse_args()

    floor_targets = {0: 1300}
    for spec in args.floor_target:
        r, k = spec.split("=")
        floor_targets[int(r)] = int(k)

    bursts = load_bursts(args.campaign_log)
    if not bursts:
        sys.exit("no BURST ON/OFF stamps found in campaign log")
    samples = load_samples(args.jsonl)
    if not samples:
        sys.exit("no parsable samples in recording")

    cols = ("burst", "cascade_ms", "floor", "shed_lag_ms", "txq_drops",
            "fps_min", "freeze_ms", "trunc", "drop", "s1_abn_pk")
    print(("{:>9} " * len(cols)).format(*cols))
    for b in bursts:
        r = burst_report(samples, b["on"], int(args.window_s * 1000),
                         floor_targets)
        if r is None:
            print(f"{b['n']:>9} " + "no demote observed")
            continue
        vals = [b["n"]] + [r[c] for c in cols[1:]]
        print(("{:>9} " * len(cols)).format(
            *[("-" if v is None else (f"{v:.1f}" if isinstance(v, float) else v))
              for v in vals]))


if __name__ == "__main__":
    main()
