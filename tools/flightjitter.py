#!/usr/bin/env python3
# Post-flight jitter/stutter attribution over a debug-log session directory
# (docs/observability.md): au.log per-AU rows, written in-process by
# maburgs itself since the 2026-09-06 debug-log consolidation (formerly
# tools/gs/flightrec.py, an external ring reader, deleted that day), plus
# the matching flight.jsonl 1 Hz sideport recording when present.
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
#   fec-wait      direct evidence on the stutter frame itself: its own
#                 t_complete − t_first (SlotHdr v2) exceeds 20 ms — the AU
#                 sat waiting on FEC repair inside the ring, not in transit.
#                 Checked BEFORE loss-recovery so direct per-frame evidence
#                 wins over the jsonl-inferred class below.
#   loss-recovery sizes flat but the sideport recorded pre-FEC loss that
#                 second (FEC repair wait) — needs the jsonl
#   transport     none of the above (USB batching, scheduling, queueing)
#
# Row format: v1 (7 columns, no header marker), v2 (11 columns behind a
# "# aulog 2" marker — SlotHdr v2's t_first/t_complete/enc_us/dq_ms
# appended, 2026-08-31 latency-accounting task 13), v3 (12 columns behind
# "# aulog 3" — a trailing air_ms column, 2026-09-06 air clock, ignored
# here), or v4 (same 12 columns behind "# aulog 4" — the debug-log
# consolidation: t_us now runs on ONE session-wide CLOCK_MONOTONIC shared
# with ctl/probe/lat, so it needs no wall-clock bridge; "# sync" lines are
# ignored for v4+ even if present). load_au_log keys its parsing on the
# marker; v1 rows get t_first=t_complete=0 (arrival/jitter math falls back
# to the read-time t_us column, as before). When t_complete is nonzero it
# is the arrival-time basis for inter-AU intervals (writer-stamped ring
# completion time, not flightrec's read-poll stamp) — same fix as
# aucadence.py's t_complete switch, see docs/data-provenance.md.
#
# Usage: flightjitter.py [session-dir | au-0001.log [flight-0001.jsonl]]
#        (no argument -> the highest-numbered session under session.py's
#        DEFAULT_ROOT)
import bisect
import json
import os
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
FEC_WAIT_US = 20_000     # t_complete-t_first threshold for the fec-wait class


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


def _arr_us(r):
    """Arrival-time basis for interval math: SlotHdr v2's writer-stamped
    t_complete when present (nonzero), else the row's read-time t_us
    (v1 rows, or a v2 slot whose caller never passed AuLatMeta)."""
    return r.get("t_complete") or r["t_us"]


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
    disconts = damaged = holes = 0
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
        iv = _arr_us(b) - _arr_us(a)
        dlen = b["len"] - a["len"]
        ivs.append(iv)
        dlens.append(dlen)
        iv_rungs.append(rung_mcs_at(ev_ms))
        fid_jump = b["fid"] - a["fid"] - 1
        # Holes: frames that never reached display — fid gaps (burned an id,
        # vanished) plus same-sid seams without one (shed enh burns no fid).
        # A gap explains its own seam, so count one or the other per boundary.
        if fid_jump > 0:
            holes += fid_jump
        elif a["sid"] == b["sid"]:
            holes += 1
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
        elif b.get("t_complete") and b.get("t_first") and \
                b["t_complete"] - b["t_first"] > FEC_WAIT_US:
            kind = "fec-wait"
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
    # Perceptual metrics — what eyes notice, which the EMA underweights
    # (0029 A/B: EMA +32% read as "much worse" because holes/min +117%).
    dur_min = summary["duration_s"] / 60.0
    if dur_min > 0 and ivs:
        s_iv = sorted(ivs)
        summary["holes_per_min"] = round(holes / dur_min, 1)
        summary["stalls50_per_min"] = round(
            sum(1 for v in ivs if v > 50_000) / dur_min, 1)
        summary["stalls80_per_min"] = round(
            sum(1 for v in ivs if v > 80_000) / dur_min, 1)
        summary["iv_p99_ms"] = round(s_iv[int(0.99 * (len(s_iv) - 1))] / 1000.0, 1)
        summary["iv_max_ms"] = round(s_iv[-1] / 1000.0, 1)
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
    """Parses au-NNNN.log. Row format is keyed off a "# aulog N" marker line
    (absent = v1, 7 columns; N>=2 = v2+, 11+ columns — SlotHdr v2's
    t_first/t_complete/enc_us/dq_ms appended; v3/v4 add a trailing air_ms
    column, ignored here). v1 rows get t_first=t_complete=enc_us=dq_ms=0
    so downstream code (_arr_us, the fec-wait class) can treat "present"
    as "nonzero" uniformly. v4+ t_us is already on one session-wide
    CLOCK_MONOTONIC (debug-log consolidation), so no wall-clock/mono
    offset is derived from "# sync" lines even when present; only v<=3
    logs still bridge through them.

    "# resync N" only ever appears in a pre-v4 log (the deleted flightrec's
    external-reader epoch resync marker). "# dropped N" is the v4 writer's
    own hole-signal (gs/src/log_writer.h: the shared per-session LogWriter's
    ring overflowed and it counted the lines it had to drop) — summed here
    across every marker in the file, since a rejoined session can flush more
    than one."""
    rows, resyncs, dropped, syncs = [], 0, 0, []
    version = 1
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line.split()
                if parts[:2] == ["#", "aulog"] and len(parts) >= 3:
                    version = int(parts[2])
                elif parts[:2] == ["#", "sync"] and len(parts) >= 4:
                    syncs.append((int(parts[2]), int(parts[3])))
                elif parts[:2] == ["#", "resync"]:
                    resyncs += 1
                elif parts[:2] == ["#", "dropped"] and len(parts) >= 3:
                    dropped += int(parts[2])
                continue
            fields = line.split()
            t, pts, sid, fid, ln, flags, nal0 = fields[:7]
            if version >= 2 and len(fields) >= 11:
                t_first, t_complete, enc_us, dq_ms = fields[7:11]
            else:
                t_first = t_complete = enc_us = dq_ms = 0
            rows.append(dict(t_us=int(t), pts=int(pts), sid=int(sid),
                             fid=int(fid), len=int(ln), flags=int(flags, 0),
                             nal0=int(nal0), t_first=int(t_first),
                             t_complete=int(t_complete), enc_us=int(enc_us),
                             dq_ms=int(dq_ms)))
    offset = 0
    # v4+ t_us is already monotonic (no wall-clock/mono bridge needed);
    # only v<=3 logs derive an offset from their "# sync" lines.
    if version <= 3 and syncs:
        offset = int(statistics.median(t_us - t_ms * 1000
                                       for t_us, t_ms in syncs))
    return rows, offset, resyncs, dropped


def load_jsonl(path):
    # Binary read: a power-off mid-write leaves the DVR's tail as garbage
    # bytes (flight-0023, 2026-09-05), and a text-mode iterator raises
    # UnicodeDecodeError before json ever sees the line.
    out = []
    with open(path, "rb") as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
    return out


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import session as _session
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    if arg is None or os.path.isdir(arg):
        s = _session.resolve(arg)
        if s.au is None:
            sys.exit("no au log in that session")
        au_path, jsonl_path = s.au, s.flight
    else:
        au_path = arg
        jsonl_path = sys.argv[2] if len(sys.argv) > 2 else None
    rows, offset, resyncs, dropped = load_au_log(au_path)
    jsonl = load_jsonl(jsonl_path) if jsonl_path else None
    if not rows:
        sys.exit(f"{au_path}: no AU rows")
    rep = analyze(rows, jsonl=jsonl, offset_us=offset)
    s = rep["summary"]
    # ring_resyncs is the pre-v4 hole-signal (the deleted flightrec's
    # external-reader epoch resync) and is structurally always 0 for a v4
    # log written by maburgs itself — kept here only because an old
    # recording can still carry it. log_dropped is the v4 replacement: the
    # in-process LogWriter's own ring-overflow counter (gs/src/log_writer.h,
    # "# dropped N"), which IS live on current recordings.
    s["ring_resyncs"] = resyncs
    s["log_dropped"] = dropped
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
