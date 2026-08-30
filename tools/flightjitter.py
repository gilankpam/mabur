#!/usr/bin/env python3
# Post-flight jitter/stutter attribution over the GS flight instrument's
# recordings (tools/gs/flightrec.py): au-NNNN.log per-AU rows, plus the
# matching flight-NNNN.jsonl 1 Hz sideport recording when present.
#
# Method (docs/airtime-model.md §4 "decomposition trick"): inter-arrival
# intervals are predicted purely from AU size deltas (iv ≈ frame_period +
# slope·Δlen, slope fitted from the data itself); whatever the size model
# explains is encoder/scene-driven, the residual is transport. Each stutter
# event (inter-arrival excess > threshold) is individually classified:
#
#   gap           frame_id discontinuity — the frame never reached the ring
#   rung-change   ladder rung / commanded-bitrate step within ±2 s (the CBR
#                 convergence sawtooth) — needs the jsonl
#   size          the late frame is proportionally big: implied slope
#                 (iv-per)/Δlen sits in a physical band (scene/encoder burst)
#   loss-recovery sizes flat but the sideport recorded pre-FEC loss that
#                 second (FEC repair wait) — needs the jsonl
#   transport     none of the above (USB batching, scheduling, queueing)
#
# Usage: flightjitter.py au-0001.log [flight-0001.jsonl]
import bisect
import json
import statistics
import sys

FLAG_IDR, FLAG_DISCONT, FLAG_COMPLETE = 0x01, 0x02, 0x80
FRAME_US_DEFAULT = 1_000_000.0 / 60.0
EMA_SHIFT = 16          # player's jitter EMA constant
WARMUP = 64             # EMA samples ignored before summarizing
RUNG_WINDOW_MS = 2000
LOSS_WINDOW_MS = 1500
LOSS_MIN = 0.005
SLOPE_BAND = (0.2, 3.0)  # plausible µs/byte serialization slopes
MIN_DLEN = 500           # bytes; below this a frame can't "explain" a stall


def _ema_series(vals):
    ema, out = 0.0, []
    prev = None
    for v in vals:
        if prev is not None:
            ema += (abs(v - prev) - ema) / EMA_SHIFT
        out.append(ema)
        prev = v
    return out


def _median_after_warmup(series):
    tail = series[WARMUP:] if len(series) > WARMUP else series
    return statistics.median(tail) if tail else 0.0


def _fit_slope(dlens, excesses):
    """Least-squares slope of interval excess (µs) vs Δlen (bytes)."""
    sxx = sum(d * d for d in dlens)
    if sxx == 0:
        return 0.0
    return sum(d * e for d, e in zip(dlens, excesses)) / sxx


def _jsonl_walk(jsonl):
    """(t_ms, rung_idx, cmd_kbps, pre_fec) per row, rows without them skipped."""
    out = []
    for o in jsonl or []:
        try:
            # "drone" (and its members) serialize as null until drone
            # telemetry arrives — `or {}` covers key-present-but-null.
            enc = (o.get("drone") or {}).get("enc") or {}
            rung = o["link"]["ctl"]["rung"]
            out.append((o["t_ms"],
                        rung["idx"],
                        enc.get("cmd_kbps"),
                        o["link"]["ctl"].get("pre_fec_loss") or 0.0,
                        rung.get("mcs")))
        except (KeyError, TypeError):
            continue
    out.sort()
    return out


def analyze(rows, jsonl=None, fps=60.0, stutter_excess_ms=25.0, offset_us=0):
    """rows: dicts with t_us, pts, sid, fid, len (flags/nal0 optional),
    sorted by arrival. jsonl: parsed sideport rows. offset_us aligns the two
    clocks: t_ms*1000 + offset_us ≈ t_us."""
    per = 1_000_000.0 / fps
    js = _jsonl_walk(jsonl)
    change_times = [js[i][0] for i in range(1, len(js))
                    if js[i][1] != js[i - 1][1] or js[i][2] != js[i - 1][2]]

    def near_rung_change(ev_ms):
        return any(abs(ev_ms - c) <= RUNG_WINDOW_MS for c in change_times)

    def lossy_second(ev_ms):
        return any(abs(ev_ms - t) <= LOSS_WINDOW_MS and pre > LOSS_MIN
                   for t, _, _, pre, _ in js)

    js_times = [e[0] for e in js]

    def rung_mcs_at(ev_ms):
        i = bisect.bisect_right(js_times, ev_ms) - 1
        if 0 <= i < len(js) and ev_ms - js_times[i] <= 2500:
            return js[i][4]
        return None

    events = []
    ivs, dlens, iv_rungs = [], [], []
    disconts = damaged = 0
    if rows and not rows[0]["flags"] & FLAG_COMPLETE:
        damaged += 1
        events.append({"t_us": rows[0]["t_us"], "fid": rows[0]["fid"],
                       "kind": "damaged", "excess_ms": 0.0, "dlen": 0})
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        ev_ms = (b["t_us"] - offset_us) // 1000
        if not b["flags"] & FLAG_COMPLETE:
            damaged += 1
            events.append({"t_us": b["t_us"], "fid": b["fid"],
                           "kind": "damaged", "excess_ms": 0.0,
                           "dlen": b["len"] - a["len"]})
        if b["flags"] & FLAG_DISCONT:
            # maburgs-marked seam (drone restart, fid namespace jump):
            # neither an interval nor a gap — break the chain here.
            disconts += 1
            continue
        iv = b["t_us"] - a["t_us"]
        dlen = b["len"] - a["len"]
        ivs.append(iv)
        dlens.append(dlen)
        iv_rungs.append(rung_mcs_at(ev_ms))
        fid_jump = b["fid"] - a["fid"] - 1
        if fid_jump > 0:
            events.append({"t_us": b["t_us"], "fid": a["fid"] + 1,
                           "kind": "gap", "missing": fid_jump,
                           "excess_ms": round((iv - per) / 1000.0, 1)})
            continue
        excess = iv - per
        if excess <= stutter_excess_ms * 1000.0:
            continue
        if near_rung_change(ev_ms):
            kind = "rung-change"
        elif dlen > MIN_DLEN and SLOPE_BAND[0] <= excess / dlen <= SLOPE_BAND[1]:
            kind = "size"
        elif lossy_second(ev_ms):
            kind = "loss-recovery"
        else:
            kind = "transport"
        events.append({"t_us": b["t_us"], "fid": b["fid"], "kind": kind,
                       "excess_ms": round(excess / 1000.0, 1),
                       "dlen": dlen})

    excesses = [iv - per for iv in ivs]
    slope = _fit_slope(dlens, excesses)
    if not SLOPE_BAND[0] <= slope <= SLOPE_BAND[1]:
        slope = max(min(slope, SLOPE_BAND[1]), 0.0)
    resid = [e - slope * d for e, d in zip(excesses, dlens)]
    jitter_ms = _median_after_warmup(_ema_series([v / 1000.0 for v in ivs]))
    resid_ms = _median_after_warmup(_ema_series([v / 1000.0 for v in resid]))
    frac = 0.0
    if jitter_ms > 1.0:
        frac = max(0.0, min(1.0, 1.0 - resid_ms / jitter_ms))
    summary = {
        "aus": len(rows),
        "duration_s": round((rows[-1]["t_us"] - rows[0]["t_us"]) / 1e6, 1)
        if len(rows) > 1 else 0.0,
        "jitter_ema_ms": round(jitter_ms, 2),
        "residual_jitter_ms": round(resid_ms, 2),
        "size_explained_frac": round(frac, 3),
        "slope_us_per_byte": round(slope, 3),
        "gaps": sum(e.get("missing", 0) for e in events if e["kind"] == "gap"),
        "stutters": sum(1 for e in events
                        if e["kind"] not in ("gap", "damaged")),
        "disconts": disconts,
        "damaged": damaged,
    }
    # Per-rung breakdown ("where does the jitter live"): |Δinterval| samples
    # bucketed by the rung mcs active that second. A plain percentile, not
    # the player EMA — segments interleave, an EMA would smear across rungs.
    per_rung = {}
    for i in range(1, len(ivs)):
        r = iv_rungs[i]
        if r is None:
            continue
        per_rung.setdefault(r, {"djit": [], "dwell_us": 0})
        per_rung[r]["djit"].append(abs(ivs[i] - ivs[i - 1]) / 1000.0)
        per_rung[r]["dwell_us"] += ivs[i]
    ev_rungs = {}
    for e in events:
        r = rung_mcs_at((e["t_us"] - offset_us) // 1000)
        if r is not None and e["kind"] not in ("gap", "damaged"):
            ev_rungs.setdefault(r, 0)
            ev_rungs[r] += 1
    summary["per_rung"] = {
        r: {"dwell_s": round(d["dwell_us"] / 1e6, 1),
            "n": len(d["djit"]),
            "djit_p50_ms": round(statistics.median(d["djit"]), 2),
            "djit_p90_ms": round(sorted(d["djit"])[int(0.9 * (len(d["djit"]) - 1))], 2),
            "stutters": ev_rungs.get(r, 0)}
        for r, d in sorted(per_rung.items()) if d["djit"]}
    return {"summary": summary, "events": events}


def load_au_log(path):
    rows, resyncs, syncs = [], 0, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line.split()
                if parts[:2] == ["#", "sync"] and len(parts) >= 4:
                    syncs.append((int(parts[2]), int(parts[3])))
                elif parts[:2] == ["#", "resync"]:
                    resyncs += 1
                continue
            t, pts, sid, fid, ln, flags, nal0 = line.split()
            rows.append(dict(t_us=int(t), pts=int(pts), sid=int(sid),
                             fid=int(fid), len=int(ln), flags=int(flags, 0),
                             nal0=int(nal0)))
    offset = 0
    if syncs:
        offset = int(statistics.median(t_us - t_ms * 1000
                                       for t_us, t_ms in syncs))
    return rows, offset, resyncs


def load_jsonl(path):
    out = []
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: flightjitter.py au-NNNN.log [flight-NNNN.jsonl]")
    rows, offset, resyncs = load_au_log(sys.argv[1])
    jsonl = load_jsonl(sys.argv[2]) if len(sys.argv) > 2 else None
    if not rows:
        sys.exit(f"{sys.argv[1]}: no AU rows")
    rep = analyze(rows, jsonl=jsonl, offset_us=offset)
    s = rep["summary"]
    s["ring_resyncs"] = resyncs
    print(json.dumps(s, indent=2))
    kinds = {}
    for e in rep["events"]:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    print(f"\n{len(rep['events'])} events: " +
          ", ".join(f"{k}={v}" for k, v in sorted(kinds.items())))
    for e in rep["events"][:200]:
        t_s = (e["t_us"] - rows[0]["t_us"]) / 1e6
        extra = (f"missing={e['missing']}" if e["kind"] == "gap"
                 else f"dlen={e['dlen']:+d}")
        print(f"  t+{t_s:8.2f}s fid={e['fid']:<8d} {e['kind']:<13s} "
              f"excess={e['excess_ms']:6.1f}ms {extra}")
    if len(rep["events"]) > 200:
        print(f"  ... {len(rep['events']) - 200} more")


if __name__ == "__main__":
    main()
