#!/usr/bin/env python3
"""Post-flight analysis of a mabur sideport flight.jsonl (schema v1 + link.ctl)
or a maburgs ctl-NNNN_<date>.log (see gs/src/ctl_log.h; parses ctllog v1-v11,
warns on pre-v4, pre-v7, pre-v8, pre-v9 and pre-v10; prints a u/u3
definition label -- arrival-booked vs completion-booked -- at the v11
boundary). Format is auto-detected from the first line.
Usage: flightreport.py <session-dir> | flight.jsonl | ctl-*.log | probe-*.log [au-*.log]

A ctl or probe log also gets the probe-stream report; the optional second
argument names the flightrec au-NNNN.log to join probe rows to (otherwise
the one in the same directory or ./log whose mono-time range overlaps).

Note: last_event is a single overwritten struct on the wire; multiple rung transitions
inside one 500ms export window surface only as the LAST transition. Reported counts are
a lower bound due to this schema limitation."""
import glob, json, math, os, re, statistics, sys

# Same clamp sentinel CtlLog writes at the source (mirrors StatsExporter's
# clamp_util(): u3/u_pred/E's u carry a 1e9 zero-guard sentinel from
# LadderController when the divisor budget is 0, clamped to <=1e3 at the
# write site). Anything at or above this is "unmeasurable", not a reading.
SENTINEL = 1000.0


def is_sentinel(v):
    return isinstance(v, float) and v >= SENTINEL


def load(path):
    # Binary read: a power-off mid-write leaves the DVR's tail as garbage
    # bytes (flight-0023, 2026-09-05), and a text-mode iterator raises
    # UnicodeDecodeError before json ever sees the line.
    rows = []
    with open(path, "rb") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: rows.append(json.loads(line))
            except (ValueError, UnicodeDecodeError): continue
    if not rows: sys.exit("no parseable datagrams")
    return rows


def _parse_ladder_token(raw):
    """Parse the ctllog header's `ladder=` value into a list of
    {"mcs": int, "bw": int, "ov_base": float, "ov_enh": float} rungs.

    v1-v7 wrote a single per-rung overhead x100 (`mcs/ov`, e.g. "5/25");
    v8 (Task 5, same-rate-fixed-pairs) splits it into a base/enh pair
    (`mcs/ovb:ove`, e.g. "5/25:50"). A single (pre-v8) value is treated as
    both base and enh -- that rung had no split to lose. v12 (2026-09-24,
    40 MHz rungs) prefixes the width (`bw:mcs/ovb:ove`, e.g. "40:3/50:25");
    older tokens are 20 MHz."""
    rungs = []
    if not raw:
        return rungs
    for entry in raw.split(","):
        if "/" not in entry:
            continue
        mcs_s, ov_s = entry.split("/", 1)
        # v12 (40 MHz rungs): bw:mcs. Older tokens carry mcs alone = 20 MHz.
        if ":" in mcs_s:
            bw_s, mcs_s = mcs_s.split(":", 1)
        else:
            bw_s = "20"
        try:
            mcs, bw = int(mcs_s), int(bw_s)
        except ValueError:
            continue
        if ":" in ov_s:
            ovb_s, ove_s = ov_s.split(":", 1)
        else:
            ovb_s = ove_s = ov_s
        try:
            ov_base, ov_enh = float(ovb_s) / 100.0, float(ove_s) / 100.0
        except ValueError:
            continue
        rungs.append({"mcs": mcs, "bw": bw, "ov_base": ov_base, "ov_enh": ov_enh})
    return rungs


def load_ctllog(path):
    """Parse a maburgs ctl log (gs/src/ctl_log.cpp formats).

    Returns {"header": {...}, "S": [...], "E": [...], "P": [...], "N": [...], "R": [...]}.
    S lines are NOT strictly 1 Hz (SIGUSR1 emits off-cadence extras) -- callers
    must key everything off t_ms, never assume uniform spacing. float() parses
    "nan" natively, which the pre-session rung-0 warm-up samples rely on.
    """
    header = {}
    S, E, P, N, R = [], [], [], [], []
    with open(path) as f:
        first = f.readline().strip()
        toks0 = first.split()
        header["_version"] = int(toks0[1]) if len(toks0) > 1 and toks0[1].isdigit() else 0
        # ctllog 6 ladder=0/100,2/50,... down_util=0.35 up_util=0.15
        # ctllog 8 ladder=0/100:100,2/50:50,... down_util=0.35 up_util=0.15
        for tok in first.split()[2:]:
            if "=" not in tok: continue
            k, v = tok.split("=", 1)
            header[k] = v
        # Structured ladder rungs (Task 5, same-rate-fixed-pairs): v8 splits
        # each rung's overhead into a base/enh pair (mcs/ovb:ove); v1-v7
        # wrote a single value per rung (mcs/ov) -- treated as both, since a
        # pre-split rung had no base/enh distinction to lose. Stored under
        # a "_"-prefixed key like _version so the raw header-field print
        # loop (which skips "_"-prefixed keys) doesn't also print this.
        header["_ladder"] = _parse_ladder_token(header.get("ladder", ""))

        for line in f:
            line = line.strip()
            if not line: continue
            toks = line.split()
            tag = toks[0]
            try:
                if tag == "S" and len(toks) >= 8:
                    S.append({
                        "t_ms": float(toks[1]), "rung": int(toks[2]),
                        "u": float(toks[3]), "snr_db": float(toks[4]),
                        "resid": float(toks[5]), "u3": float(toks[6]),
                        "resid3": float(toks[7]),
                        # 2026-08-10 EVM label; absent on older logs.
                        "evm_db": float(toks[8]) if len(toks) >= 9 else float("nan"),
                        # 2026-08-14 attribution (ctllog 2); absent on older logs.
                        "resid_cur": float(toks[9]) if len(toks) >= 10 else float("nan"),
                        # 2026-08-14 fade deltas (ctllog 3); absent on older logs.
                        "drssi": float(toks[10]) if len(toks) >= 12 else float("nan"),
                        "dsnr": float(toks[11]) if len(toks) >= 12 else float("nan"),
                        # 2026-08-15 label RSSI (ctllog 5); absent on older logs.
                        "rssi": float(toks[12]) if len(toks) >= 13 else float("nan"),
                        # 2026-09-04 probe gate (ctllog 10); absent on older logs.
                        "probe_rung": int(toks[13]) if len(toks) >= 16 else -1,
                        "probe_u": float(toks[14]) if len(toks) >= 16 else float("nan"),
                        "probe_n": int(toks[15]) if len(toks) >= 16 else 0,
                    })
                elif tag == "E" and len(toks) >= 7:
                    E.append({
                        "t_ms": float(toks[1]), "from": int(toks[2]),
                        "to": int(toks[3]), "reason": toks[4],
                        "u": float(toks[5]), "snr_db": float(toks[6]),
                        # 2026-08-10 EVM label; absent on older logs.
                        "evm_db": float(toks[7]) if len(toks) >= 8 else float("nan"),
                    })
                elif tag == "P" and len(toks) >= 7:
                    P.append({
                        "t_ms": float(toks[1]), "rung": int(toks[2]),
                        "outcome": toks[3], "snr_db": float(toks[4]),
                        "u_pred": float(toks[5]), "dur_ms": float(toks[6]),
                        # 2026-08-10 EVM label; absent on older logs.
                        "evm_db": float(toks[7]) if len(toks) >= 8 else float("nan"),
                    })
                elif tag == "N" and len(toks) >= 5:
                    N.append({
                        "t_ms": float(toks[1]), "rung": int(toks[2]),
                        "k": int(toks[3]), "until_ms": float(toks[4]),
                    })
                elif tag == "R" and len(toks) >= 13:
                    R.append({
                        "t_ms": float(toks[1]), "rung": int(toks[2]),
                        "u": float(toks[3]), "resid": float(toks[4]),
                        "u3": float(toks[5]), "resid3": float(toks[6]),
                        "evm_db": float(toks[7]), "evm_sd_db": float(toks[8]),
                        "n": int(toks[9]), "age_s": float(toks[10]),
                        "probe_u": float(toks[11]), "probe_n": int(toks[12]),
                    })
            except ValueError:
                continue  # malformed record; skip rather than abort the report

    return {"header": header, "S": S, "E": E, "P": P, "N": N, "R": R}


def wall_fit(records):
    """Per-rung pass/fail SNR summary + outlier-aware wall estimate.

    records: list of P dicts (one rung's worth). nan-snr probes are excluded
    from every stat but still counted. A FAIL whose snr exceeds the max PASS
    snr is an outlier (loss at high SNR is not an SNR wall -- see
    docs/mcs6-bench-anomaly.md) and is excluded from the fit. Suggested wall
    is the midpoint between the max inlier-fail snr and the min pass snr;
    "insufficient data" when either side is empty.
    """
    # v10 (probe-stream) renamed the P-line vocabulary from probe outcomes
    # (pass/fail/abort) to gate EDGES (clean/lossy/noinfo); map the new
    # names onto the old ones so the rest of this function -- and every
    # caller -- doesn't need to know which log version it's reading.
    def _oc(r):
        return {"clean": "pass", "lossy": "fail", "noinfo": "abort"}.get(
            r["outcome"], r["outcome"])

    passes = [r for r in records if _oc(r) == "pass"]
    fails = [r for r in records if _oc(r) == "fail"]
    aborts = [r for r in records if _oc(r) == "abort"]

    def valid(rs): return [r["snr_db"] for r in rs if not math.isnan(r["snr_db"])]

    pass_snrs, fail_snrs = valid(passes), valid(fails)
    nan_n = sum(1 for r in records if math.isnan(r["snr_db"]))
    # u_pred is a separate reading from snr_db -- a probe can be a clean
    # pass/fail on SNR while its predicted util still saturated the 1e9
    # zero-guard sentinel (clamped to <=1e3 at the write site). Counted for
    # visibility only; the SNR-based pass/fail/outlier/wall logic above is
    # unaffected.
    u_pred_sat_n = sum(1 for r in records if is_sentinel(r["u_pred"]))

    max_pass = max(pass_snrs) if pass_snrs else None
    outlier_snrs = [s for s in fail_snrs if max_pass is not None and s > max_pass]
    inlier_fail_snrs = [s for s in fail_snrs if s not in outlier_snrs]

    wall = None
    if inlier_fail_snrs and pass_snrs:
        wall = (max(inlier_fail_snrs) + min(pass_snrs)) / 2.0

    return {
        "n_pass": len(passes), "n_fail": len(fails), "n_abort": len(aborts),
        "nan_snr": nan_n, "u_pred_sat": u_pred_sat_n,
        "pass_snrs": pass_snrs, "fail_snrs": fail_snrs,
        "outlier_snrs": outlier_snrs, "inlier_fail_snrs": inlier_fail_snrs,
        "wall": wall,
    }


def print_rung_store_report(R):
    """Per-rung EWMA store summary from the FINAL R snapshot per rung, plus
    a report-only inversion callout (spec 2026-08-13: analyzer prototype of
    rung auto-skip — thresholds are defaults to be tuned on recordings)."""
    if not R:
        return
    last = {}
    for r in R:          # file order is time order; last line per rung wins
        last[r["rung"]] = r
    print("RUNG STORE (final R snapshot per rung)")
    print("  rung      u  resid     u3 resid3    evm evm_sd      n  age_s"
          "  probe_u probe_n")
    for i in sorted(last):
        r = last[i]
        print(f"  {i:4d} {r['u']:6.3f} {r['resid']:6.3f} {r['u3']:6.3f}"
              f" {r['resid3']:6.3f} {r['evm_db']:6.1f} {r['evm_sd_db']:6.2f}"
              f" {r['n']:6d} {r['age_s']:6.1f} {r['probe_u']:8.3f}"
              f" {r['probe_n']:7d}")
    MIN_N = 300
    for lo_i in sorted(last):
        for hi_i in sorted(last):
            if hi_i <= lo_i:
                continue
            lo, hi = last[lo_i], last[hi_i]
            if lo["n"] < MIN_N or hi["n"] < MIN_N:
                continue
            resid_inv = hi["resid"] >= max(2 * lo["resid"], lo["resid"] + 0.02)
            util_inv = lo["u"] > 0 and hi["u"] >= 1.5 * lo["u"]
            if resid_inv or util_inv:
                key = "resid" if resid_inv else "u"
                print(f"  !! INVERSION rung {hi_i} worse than rung {lo_i}"
                      f" ({key}: {hi[key]:.3f} vs {lo[key]:.3f},"
                      f" n {hi['n']}/{lo['n']})")


def bw40_summary(ctllog, probation_ms=3000.0):
    """Time held on 40 MHz rungs and promotes onto them, from the ctllog 12
    header's per-rung bw and the E lines. None when no rung is 40 MHz (every
    pre-v12 recording). A promote onto 40 MHz counts as held unless the link
    LEAVES the 40 MHz region (reaches a 20 MHz rung) within probation_ms --
    a climb 40/3 -> 40/4 stays on 40 and still holds. probation_ms is the
    bundle's link.probation_ms, 3 s; the log does not carry it."""
    rungs = ctllog["header"].get("_ladder") or []
    if not any(r.get("bw") == 40 for r in rungs):
        return None
    S, E = ctllog.get("S", []), ctllog.get("E", [])
    ts = [s["t_ms"] for s in S] + [e["t_ms"] for e in E]
    if not ts:
        return {"held_s": 0.0, "total_s": 0.0, "promotes": 0, "held_past_probation": 0}
    t0, t1 = min(ts), max(ts)

    def bw_of(idx):
        return rungs[idx]["bw"] if 0 <= idx < len(rungs) else 20

    # Initial rung: the first E line's `from`; with no E lines, the first S
    # line's rung (S carries the live rung); else rung 0.
    if E:
        cur = E[0]["from"]
    elif S:
        cur = min(S, key=lambda s: s["t_ms"])["rung"]
    else:
        cur = 0
    t_prev, held, promotes, held_past = t0, 0.0, 0, 0
    for i, e in enumerate(E):
        if bw_of(cur) == 40:
            held += e["t_ms"] - t_prev
        if bw_of(e["from"]) != 40 and bw_of(e["to"]) == 40:
            promotes += 1
            # First time the link reaches a 20 MHz rung after this promote;
            # the end of the recording when it never does.
            left = next((x["t_ms"] for x in E[i + 1:] if bw_of(x["to"]) != 40), t1)
            if left - e["t_ms"] >= probation_ms:
                held_past += 1
        cur, t_prev = e["to"], e["t_ms"]
    if bw_of(cur) == 40:
        held += t1 - t_prev
    return {"held_s": held / 1000.0, "total_s": (t1 - t0) / 1000.0,
            "promotes": promotes, "held_past_probation": held_past}


def print_bw40_report(ctllog, probation_ms=3000.0):
    b = bw40_summary(ctllog, probation_ms)
    if b is None:
        return
    print("BW40 (40 MHz rungs; per-rung width from the ctllog 12 header)")
    print(f"  held {b['held_s']:.1f} s of {b['total_s']:.1f} s on 40 MHz rungs;"
          f" promotes onto 40 MHz: {b['promotes']},"
          f" held past {probation_ms / 1000.0:g} s: {b['held_past_probation']}")
    if b["promotes"] and b["held_past_probation"] == 0:
        print("  !! every promote onto 40 MHz fell straight back: suspect a busy"
              " secondary -- the scout cannot see it (docs/bw40.md)")


def print_wall_report(ctllog):
    header, S, E, P, N = (ctllog[k] for k in ("header", "S", "E", "P", "N"))

    print("CTL LOG HEADER")
    ver = header.get("_version", 0)
    if ver and ver < 10:
        print("  NOTE: ctllog v%d -- P lines are discrete probe outcomes "
              "(one per 2 s probe); v10+ P lines are gate EDGES and R/S "
              "probe_u is a continuous EWMA -- do not pool probe_u across "
              "this line." % ver)
    if ver and ver < 9:
        print("  NOTE: ctllog v%d -- resid/resid_cur (S) and resid (R) come "
              "from the PACKET-level delivery window, which counted a late "
              "sliding-window FEC repair as loss. A v%d resid > 0 does NOT "
              "mean video was lost: on the 2026-09-02 bench that measure "
              "fired 200 spurious residual demotes in 57 min with the FEC "
              "decoder's abandonment counter frozen. Do NOT pool per-rung "
              "resid with v9+ recordings, and read the inversion callout "
              "below as reorder rate (which scales with packet rate, hence "
              "with rung) rather than as loss." % (ver, ver))
    if ver and ver < 8:
        print("  NOTE: ctllog v%d -- ladder rungs are single-overhead; pair "
              "semantics from v8 (same-rate-fixed-pairs, 2026-08-30) split "
              "each rung's overhead into a base/enh pair. The header's "
              "`ladder=` token here carries one value per rung, parsed as "
              "both base and enh." % ver)
    if ver and ver < 7:
        print("  NOTE: ctllog v%d -- predates the 2026-08-29 airtime-balance-uep "
              "collapse from 4 UEP streams to 2 (BASE sid0 + ENH sid1). "
              "s3_residual/s3_util and the S/R lines' u3/resid3/evm_db read "
              "the OLD 4-stream layout (stream 3 = the T2 canary layer) here; "
              "on v7+ the SAME reason strings/fields read sid1 (ENH) instead, "
              "and the ordinary s1 quantities (u, resid) read sid0 (BASE) "
              "instead of the old stream 1. Not directly comparable with "
              "v7+ recordings." % ver)
    if ver and ver < 6:
        print("  NOTE: ctllog v%d -- the S line's rung is the LIVE rung, so any "
              "sample with loss that coincides with a demote is filed against "
              "the rung the link demoted TO, not the one that caused it." % ver)
    if ver and ver < 4:
        print("  NOTE: ctllog v%d -- snr_db/evm_db are s1-class only, and "
              "drssi/dsnr were zeroed on ~25%% of ticks by the card-hop "
              "re-baseline. Not comparable with v4+ recordings." % ver)
    for k, v in header.items():
        if k.startswith("_"): continue  # internal, e.g. _version -- not a header field
        print(f"  {k}={v}")
    if ver >= 11:
        print("  u/u3: arrival-booked pre-FEC loss (ArrivalTracker, ctllog 11+)")
    elif ver:
        print("  u/u3: completion-booked pre-FEC loss (lags the air; transition-inflated) -- pre-ctllog-11")

    print("DWELL (S records)")
    # The rung's mcs/bw from the ctllog header's ladder (ctllog 6+); a bare
    # index when the header carries none or the index is outside it.
    ladder = header.get("_ladder") or []
    by_rung = {}
    for s in S: by_rung.setdefault(s["rung"], []).append(s)
    for rung in sorted(by_rung):
        samples = by_rung[rung]
        snrs = [s["snr_db"] for s in samples if not math.isnan(s["snr_db"])]
        nan_n = len(samples) - len(snrs)
        sentinel_n = sum(1 for s in samples if is_sentinel(s["u3"]))
        snr_str = f"{min(snrs):.1f}..{max(snrs):.1f} dB" if snrs else "n/a"
        evms = [s["evm_db"] for s in samples if not math.isnan(s["evm_db"])]
        evm_str = f" evm={min(evms):.1f}..{max(evms):.1f} dB" if evms else ""
        extra = ""
        if nan_n: extra += f" nan_snr={nan_n}"
        if sentinel_n: extra += f" u3_sentinel={sentinel_n}"
        label = (f" (mcs{ladder[rung]['mcs']}/{ladder[rung]['bw']})"
                 if 0 <= rung < len(ladder) else "")
        print(f"  rung {rung}{label}: n={len(samples)} snr={snr_str}{evm_str}{extra}")

    print("EVENTS")
    reason_counts = {}
    for e in E:
        reason_counts[e["reason"]] = reason_counts.get(e["reason"], 0) + 1
        # u carries u3 (layer-3 util), not the s1 util, whenever reason
        # starts with "s3_" -- label accordingly rather than mislabeling it.
        label = "u3" if e["reason"].startswith("s3_") else "u"
        u_str = "sentinel" if is_sentinel(e["u"]) else f"{e['u']:.4f}"
        evm_str = f" evm={e['evm_db']:.1f}" if not math.isnan(e["evm_db"]) else ""
        print(f"  t={e['t_ms']:.0f} rung {e['from']}->{e['to']} "
              f"reason={e['reason']} {label}={u_str} snr={e['snr_db']:.1f}{evm_str}")

    print("EVENT SUMMARY (count per reason)")
    for reason in sorted(reason_counts):
        print(f"  {reason}: {reason_counts[reason]}")

    print("PENALTIES")
    for n in N:
        print(f"  t={n['t_ms']:.0f} rung {n['rung']} k={n['k']} until={n['until_ms']:.0f}")

    print("WALL REPORT")
    p_by_rung = {}
    for p in P: p_by_rung.setdefault(p["rung"], []).append(p)
    for rung in sorted(p_by_rung):
        fit = wall_fit(p_by_rung[rung])
        line = f"  rung {rung}: pass={fit['n_pass']} fail={fit['n_fail']}"
        if fit["n_abort"]: line += f" abort={fit['n_abort']}"
        if fit["nan_snr"]: line += f" nan_snr={fit['nan_snr']}"
        if fit["u_pred_sat"]: line += f" u_pred saturated: {fit['u_pred_sat']}"
        print(line)
        if fit["pass_snrs"]:
            print(f"    pass snr: {min(fit['pass_snrs']):.1f}..{max(fit['pass_snrs']):.1f} dB")
        if fit["fail_snrs"]:
            print(f"    fail snr: {min(fit['fail_snrs']):.1f}..{max(fit['fail_snrs']):.1f} dB")
        for s in fit["outlier_snrs"]:
            print(f"    outlier fail at snr={s:.1f} dB (loss at high SNR -- "
                  f"not an SNR wall; see docs/mcs6-bench-anomaly.md)")
        if fit["wall"] is not None:
            print(f"    suggested wall: {fit['wall']:.1f} dB "
                  f"(between fail@{max(fit['inlier_fail_snrs']):.1f} "
                  f"and pass@{min(fit['pass_snrs']):.1f})")
        else:
            print("    suggested wall: insufficient data")

    print_rung_store_report(ctllog.get("R", []))


def merge_consecutive_residuals(residuals):
    """Merge consecutive residual-positive samples into episodes.

    Each element is (t, residual_loss, traj, drone_state, rssi, snr).
    Consecutive = no clean (non-residual) sample between. At the real 500ms
    sideport cadence (2 Hz), use 750ms gap threshold (1.5× sample interval)
    to detect breaks.
    Returns merged list where consecutive samples become one episode.
    """
    if not residuals:
        return []

    merged = []
    episode_start_t, episode_max_rl, episode_trajs = residuals[0][0], residuals[0][1], [residuals[0][2]]
    episode_ds, episode_rssi, episode_snr = residuals[0][3], residuals[0][4], residuals[0][5]
    last_sample_t = residuals[0][0]

    for i in range(1, len(residuals)):
        t, rl, traj, ds, rssi, snr = residuals[i]

        # Check if consecutive: gap from PREVIOUS sample > 750ms means a clean sample(s) between
        if t - last_sample_t > 750:
            # Gap detected: finish current episode and start new one
            merged.append((episode_start_t, episode_max_rl, episode_trajs, episode_ds, episode_rssi, episode_snr))
            episode_start_t, episode_max_rl, episode_trajs = t, rl, [traj]
            episode_ds, episode_rssi, episode_snr = ds, rssi, snr
        else:
            # Consecutive: extend current episode
            episode_max_rl = max(episode_max_rl, rl)
            episode_trajs.append(traj)
            episode_ds = ds
            episode_rssi = rssi
            episode_snr = snr

        last_sample_t = t

    # Finalize last episode
    merged.append((episode_start_t, episode_max_rl, episode_trajs, episode_ds, episode_rssi, episode_snr))
    return merged


def find_episodes(E, gap_ms=3000):
    """Cluster demote E-lines (to < from) into episodes. A promote or a
    gap > gap_ms between consecutive demotes closes the episode. Returns
    dicts: t0, first_reason, path (from0, toN), steps, duration_ms,
    fade_lead_ms (leading-fade episodes only), false_fade, repromote_ms
    (false fades: delay to the next promote, None if log ends first)."""
    eps, cur = [], None

    def close(after_idx):
        nonlocal cur
        if cur is None:
            return
        e = cur
        e["duration_ms"] = e["_last_t"] - e["t0"]
        e["false_fade"] = all(r == "fade" for r in e["_reasons"])
        lead = None
        if e["_reasons"][0] == "fade":
            for t, r in zip(e["_ts"][1:], e["_reasons"][1:]):
                if r != "fade":
                    lead = t - e["_ts"][0]
                    break
        e["fade_lead_ms"] = lead
        e["repromote_ms"] = None
        if e["false_fade"]:
            for later in E[after_idx:]:
                if later["to"] > later["from"]:
                    e["repromote_ms"] = later["t_ms"] - e["_last_t"]
                    break
        for k in ("_reasons", "_ts", "_last_t"):
            del e[k]
        eps.append(e)
        cur = None

    for i, ev in enumerate(E):
        demote = ev["to"] < ev["from"]
        if not demote:
            close(i)
            continue
        if cur is not None and ev["t_ms"] - cur["_last_t"] > gap_ms:
            close(i)
        if cur is None:
            cur = {"t0": ev["t_ms"], "first_reason": ev["reason"],
                   "path": (ev["from"], ev["to"]), "steps": 0,
                   "_reasons": [], "_ts": [], "_last_t": ev["t_ms"]}
        cur["steps"] += 1
        cur["path"] = (cur["path"][0], ev["to"])
        cur["_reasons"].append(ev["reason"])
        cur["_ts"].append(ev["t_ms"])
        cur["_last_t"] = ev["t_ms"]
    close(len(E))
    return eps


def attribution_misses(E, window_ms=200):
    """Residual demotes firing within window_ms of ANY earlier E line.
    Attribution is unconditional since 2026-08-15 (link.attrib was removed
    and now fails boot; before that date it was a default-on kill switch),
    so on any recording this should be ~zero; hits mean the transition
    watermark missed a debris class (spec 2026-08-14 fade-demote §5). This
    canary only scores `reason == "residual"` (s1); it does not cover the
    s3-residual sliding-window mechanism described in the 2026-08-15 spec
    §9 / CLAUDE.md — see find_episodes() for that split instead."""
    out = []
    for i, ev in enumerate(E):
        if ev["reason"] != "residual" or ev["to"] >= ev["from"]:
            continue
        if any(0 < ev["t_ms"] - p["t_ms"] <= window_ms for p in E[:i]):
            out.append(ev)
    return out


def s3_settle_refires(E, settle_ms=300, slack_ms=60):
    """Canary for the 2026-09-05 s3 debris double-step (flights 20/21: 13 of
    14 s3_residual cascades took a second step at exactly s3_settle_ms and
    were promoted straight back). A demote whose predecessor is a demote of
    reason s3_residual landing settle_ms..settle_ms+slack_ms earlier is the
    window re-firing on old-rung debris the instant the controller's gate
    opens, not a second measurement. Should be ~0 once transition_edge.h
    blanks s3_resid_cur; a nonzero count on a new recording means the
    settle blank regressed or s3_settle_ms was tuned below the horizon lag."""
    out = []
    for prev, ev in zip(E, E[1:]):
        if ev["to"] >= ev["from"] or prev["to"] >= prev["from"]:
            continue
        if prev["reason"] != "s3_residual":
            continue
        dt = ev["t_ms"] - prev["t_ms"]
        if settle_ms <= dt <= settle_ms + slack_ms:
            out.append(ev)
    return out


def print_episode_report(ctllog):
    E = ctllog.get("E", [])
    eps = find_episodes(E)
    if not eps:
        print("\nepisodes: none")
        return
    print(f"\nepisodes ({len(eps)}):")
    for e in eps:
        lead = "-" if e["fade_lead_ms"] is None else f"{e['fade_lead_ms']:.0f}ms"
        tag = " FALSE-FADE" if e["false_fade"] else ""
        rp = ("" if e["repromote_ms"] is None
              else f" repromote+{e['repromote_ms']/1000:.1f}s")
        print(f"  t={e['t0']/1000:.1f}s {e['path'][0]}->{e['path'][1]}"
              f" steps={e['steps']} dur={e['duration_ms']:.0f}ms"
              f" first={e['first_reason']} fade_lead={lead}{tag}{rp}")
    misses = attribution_misses(E)
    print(f"  attribution-miss canary (residual <=200ms after a transition): "
          f"{len(misses)}" + ("" if not misses else " ⚠"))
    refires = s3_settle_refires(E)
    print(f"  s3-settle-refire canary (demote 300-360ms after an s3_residual demote): "
          f"{len(refires)}" + ("" if not refires else " ⚠"))


def probe_lead(E, P, horizon_ms=10000):
    """Per demote episode: time from the FIRST `lossy` gate edge since the
    rung was entered (the last E line before the episode, any direction) to
    the episode's first demote; None if no lossy edge fell inside that hold.
    Edges before the entry transition were scored while a different rung
    was held -- typically the pre-promote flapping that the 2 s clean
    streak then overrode -- and are not a warning about this hold, so they
    never count (flight-0020, 2026-09-05: the unbounded search reported
    5-19 s "leads" that predated the promote by that much). `edges` is the
    number of lossy edges inside the hold. false_alarms = lossy edges whose
    next E line is not a demote within horizon_ms: a promote in between
    means the gate went clean and the ladder moved on, so a demote of the
    NEXT hold does not vindicate the edge. Input for the v2 probe-demote
    threshold."""
    eps = find_episodes(E)
    lossy = [p for p in P if p["outcome"] == "lossy"]
    out = []
    for e in eps:
        entry = max((ev["t_ms"] for ev in E if ev["t_ms"] < e["t0"]), default=-math.inf)
        inside = [p["t_ms"] for p in lossy if entry < p["t_ms"] <= e["t0"]]
        out.append({"t0": e["t0"], "first_reason": e["first_reason"],
                    "lead_ms": (e["t0"] - min(inside)) if inside else None,
                    "edges": len(inside)})
    false_alarms = 0
    for p in lossy:
        nxt = next((ev for ev in E if ev["t_ms"] >= p["t_ms"]), None)
        hit = (nxt is not None and nxt["to"] < nxt["from"]
               and nxt["t_ms"] - p["t_ms"] <= horizon_ms)
        false_alarms += not hit
    return {"episodes": out, "false_alarms": false_alarms, "lossy_edges": len(lossy)}


def load_probelog(path):
    """probe-NNNN_<date>.log / probe.log: 'probelog <v> bpb=<n>' then
    't_ms seq mcs enh_fid blocks_ok card_mask snr_c0 snr_c1 evm_c0 evm_c1'
    plus, from probelog 2 (2026-09-05), 'first_ms': the radio's arrival
    stamp of the body's first sight (mono ms, µs fraction) -- None on v1
    rows, whose t_ms is the ~10 ms finalize tick and useless for timing.
    probelog 3 (2026-09-24, 40 MHz rungs) inserts 'bw' after mcs; older
    rows are 20 MHz."""
    rows, bpb, version = [], 4, 1
    with open(path) as f:
        first = f.readline().split()
        if len(first) >= 2 and first[0] == "probelog":
            version = int(first[1])
        for tok in first[2:]:
            if tok.startswith("bpb="): bpb = int(tok[4:])
        for line in f:
            t = line.split()
            if version >= 3:
                if len(t) < 12: continue
                bw, t = t[3], t[:3] + t[4:]   # drop bw: the rest is the v2 layout
            else:
                bw = "20"
            if len(t) < 10: continue
            try:
                rows.append({"t_ms": float(t[0]), "seq": int(t[1]), "mcs": int(t[2]),
                             "bw": int(bw),
                             "enh_fid": int(t[3]), "blocks_ok": int(t[4]),
                             "card_mask": int(t[5]),
                             "snr": [float(t[6]), float(t[7])],
                             "evm": [float(t[8]), float(t[9])],
                             "first_ms": float(t[10]) if version >= 2 and len(t) >= 11 else None})
            except ValueError:
                continue
    return {"bpb": bpb, "version": version, "rows": rows}


def load_aulog(path):
    """flightrec's au-NNNN.log (docs/observability.md): '# aulog N' marker
    then 't_us pts sid fid len flags nal0 [t_first t_complete enc dq]'.
    Only what the probe join needs: sid, fid, t_first/t_complete (mono µs).
    v1 rows (no marker) have no completion stamp and are skipped."""
    rows, version = [], 1
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p: continue
            if p[0] == "#":
                if len(p) >= 3 and p[1] == "aulog": version = int(p[2])
                continue
            if version < 2 or len(p) < 11: continue
            try:
                rows.append({"sid": int(p[2]), "fid": int(p[3]),
                             "t_first": int(p[7]), "t_complete": int(p[8])})
            except ValueError:
                continue
    return rows


def find_aulog_for(probe_path, pl):
    """The au log is written by flightrec under ITS OWN index (max+1 in
    /media/dvr/log), not the ctl/probe NNNN, and into a different directory
    (<ctl_log_dir>/log/). Both stamp CLOCK_MONOTONIC, which restarts at
    boot, so the au log from the same boot is the one whose t_complete
    range overlaps the probe rows' first_ms range the most."""
    stamps = [r["first_ms"] * 1000 for r in pl["rows"] if r["first_ms"] is not None]
    if not stamps: return None
    lo, hi = min(stamps), max(stamps)
    d = os.path.dirname(os.path.abspath(probe_path))
    # Overlap alone does not discriminate: every boot's mono clock starts
    # near zero, so on a DVR holding many flights a dozen au logs overlap
    # the probe span within seconds of each other (probe-0353 vs
    # au-0020..0032 picked au-0001, 2026-09-06). Among the overlapping
    # candidates take the one whose enh fids actually JOIN the probe rows;
    # overlap only breaks ties (and is the fallback when nothing joins).
    best, best_key = None, (0, 0)
    for cand in sorted(glob.glob(os.path.join(d, "au-*.log")) +
                       glob.glob(os.path.join(d, "log", "au-*.log"))):
        au = load_aulog(cand)
        ts = [r["t_complete"] for r in au if r["t_complete"]]
        if not ts: continue
        ov = min(hi, max(ts)) - max(lo, min(ts))
        if ov <= 0: continue
        key = (len(probe_au_offset_rows(pl, au)), ov)
        if key > best_key: best, best_key = cand, key
    return best


def probe_au_offset_rows(pl, au_rows, max_gap_ms=1000.0):
    """(probe row, offset_ms) for every probe row with an arrival stamp:
    first_ms - t_complete of the ENH AU (sid 1) it rode behind. Joined on
    enh_fid, a 16-bit id that wraps every ~36 min: of the AUs sharing a
    fid, the one whose completion is nearest in time (and within
    max_gap_ms) is taken. This IS the tail the RCF slotter has to wait out
    after an enh completion before the burst is actually off air."""
    by_fid = {}
    for r in au_rows:
        if r["sid"] == 1 and r["t_complete"]:
            by_fid.setdefault(r["fid"], []).append(r["t_complete"])
    out = []
    for r in pl["rows"]:
        if r["first_ms"] is None: continue
        cands = by_fid.get(r["enh_fid"])
        if not cands: continue
        t = r["first_ms"] * 1000
        tc = min(cands, key=lambda c: abs(t - c))
        if abs(t - tc) <= max_gap_ms * 1000:
            out.append((r, (t - tc) / 1000.0))
    return out


def probe_au_offsets(pl, au_rows, max_gap_ms=1000.0):
    return [o for _, o in probe_au_offset_rows(pl, au_rows, max_gap_ms)]


def _pct(v, q):
    if not v: return float("nan")
    s = sorted(v); i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


PROBE_RESYNC_MS = 3000.0      # link.failsafe_ms: a longer silence is a starve, not loss
PROBE_RESYNC_BODIES = 10000   # ~5 min of enh AUs: a bigger seq jump is a new random seed


def probelog_summary(pl):
    """Per (mcs, bw): received bodies, lost bodies (seq gaps, attributed to
    the NEXT received body's (mcs, bw)), surviving blocks, per-card body counts.
    A seq gap is loss only inside a live link: ProbeSource seeds a RANDOM
    initial seq per daemon start (the SwEncoder restart contract), and
    across a starve the GS is deaf for > failsafe_ms while the drone keeps
    counting -- both read as billions of "lost" bodies otherwise
    (probe-0352 mcs1, 2026-09-06). A backwards seq, a > PROBE_RESYNC_MS
    silence or a > PROBE_RESYNC_BODIES jump re-anchors instead, counted in
    `resyncs` on the mcs of the body that re-anchored."""
    out = {}
    prev_seq = None; prev_t = None
    for r in pl["rows"]:
        s = out.setdefault((r["mcs"], r.get("bw", 20)), {"bodies": 0, "lost_bodies": 0, "blocks_ok": 0,
                                       "card0": 0, "card1": 0, "resyncs": 0})
        if prev_seq is not None:
            gap = r["seq"] - prev_seq - 1
            if (r["seq"] <= prev_seq or gap > PROBE_RESYNC_BODIES or
                    r["t_ms"] - prev_t > PROBE_RESYNC_MS):
                s["resyncs"] += 1
            elif gap > 0:
                s["lost_bodies"] += gap
        prev_seq = r["seq"]; prev_t = r["t_ms"]
        s["bodies"] += 1; s["blocks_ok"] += r["blocks_ok"]
        if r["card_mask"] & 1: s["card0"] += 1
        if r["card_mask"] & 2: s["card1"] += 1
    return out


def print_probe_report(ctllog, probelog, au_rows=None):
    lead = probe_lead(ctllog.get("E", []), ctllog.get("P", []))
    print(f"\nPROBE GATE (lossy edges={lead['lossy_edges']}, "
          f"false alarms (no demote before the next transition / within 10 s)="
          f"{lead['false_alarms']}; lead = first lossy edge since the rung was entered)")
    for e in lead["episodes"]:
        l = "-" if e["lead_ms"] is None else f"{e['lead_ms']:.0f}ms"
        print(f"  demote t={e['t0']/1000:.1f}s first={e['first_reason']} "
              f"probe lead={l} edges={e['edges']}")
    if probelog:
        bpb = probelog["bpb"]
        print("PROBE LOG (per mcs/bw)")
        for (mcs, bw), s in sorted(probelog_summary(probelog).items()):
            tot = s["bodies"] + s["lost_bodies"]
            body_loss = s["lost_bodies"] / tot if tot else float("nan")
            blk_loss = 1 - s["blocks_ok"] / (tot * bpb) if tot else float("nan")
            print(f"  mcs{mcs}/{bw}: bodies={s['bodies']} lost={s['lost_bodies']} "
                  f"body_loss={body_loss:.3f} block_loss={blk_loss:.3f} "
                  f"c0={s['card0']} c1={s['card1']}"
                  + (f" resyncs={s['resyncs']}" if s.get("resyncs") else ""))
        if au_rows is not None:
            pairs = probe_au_offset_rows(probelog, au_rows)
            offs = [o for _, o in pairs]
            if offs:
                print(f"  completion->probe (ms, enh AU t_complete -> probe first sight): "
                      f"n={len(offs)} p10={_pct(offs, .1):.2f} p50={_pct(offs, .5):.2f} "
                      f"p90={_pct(offs, .9):.2f} p99={_pct(offs, .99):.2f} "
                      f"max={max(offs):.2f} min={min(offs):.2f}")
                by_mcs = {}
                for r, o in pairs:
                    by_mcs.setdefault((r["mcs"], r.get("bw", 20)), []).append(o)
                for (mcs, bw), v in sorted(by_mcs.items()):
                    print(f"    mcs{mcs}/{bw}: n={len(v)} p50={_pct(v, .5):.2f} p90={_pct(v, .9):.2f} "
                          f"p99={_pct(v, .99):.2f}")
            else:
                print("  completion->probe: no joinable rows (probelog v1, or no "
                      "overlapping au-NNNN.log v2 next to it / in ./log)")


def sniff_probelog(path):
    """True if `path` is a maburgs probe log (first line starts 'probelog ')."""
    with open(path) as f:
        return f.readline().startswith("probelog ")


def sniff_feclog(path):
    """True if `path` is a maburgs fec log (first line starts 'feclog ')."""
    with open(path) as f:
        return f.readline().startswith("feclog ")


FEC_COLS = ("t_ms", "sid", "mcs", "bw", "ov", "first_seq", "span", "m", "rec",
            "aband", "stale", "r", "w")
FEC_CANDIDATE_OV = (0.25, 0.35, 0.50, 0.75, 1.00)


def load_feclog(path):
    """fec.log (gs/src/fec_log.h): one row per loss episode a video layer's
    decoder closed. A rejoined session re-states the marker partway
    through; `# dropped N` is the LogWriter's gap marker. Both skipped,
    everything else is a row. feclog 2 (2026-09-24, 40 MHz rungs) inserts
    `bw` after mcs; feclog 1 rows carry no bw and are 20 MHz."""
    rows = []
    version = 1
    with open(path) as f:
        for line in f:
            if line.startswith("feclog "):
                tok = line.split()
                if len(tok) >= 2 and tok[1].isdigit():
                    version = int(tok[1])
                continue
            if line.startswith("#"):
                continue
            tok = line.split()
            if version < 2:
                tok = tok[:3] + ["20"] + tok[3:]
            if len(tok) != len(FEC_COLS):
                continue
            r = {}
            for k, v in zip(FEC_COLS, tok):
                r[k] = float(v) if k in ("t_ms", "ov") else int(v)
            rows.append(r)
    return rows


def fec_ov_req(m, r, ov):
    """The overhead this episode would have needed. At overhead x the same
    lost air carries m*(1+ov)/(1+x) sources (an aggregate is a fixed number
    of envelopes, fewer of them repairs) against r*x/ov covering repairs, and
    the decoder needs repairs >= sources: x*(1+x) = m*ov*(1+ov)/r, so
    x = (sqrt(1+4c)-1)/2 with c = m*ov*(1+ov)/r. inf when no repair
    covered the episode at all."""
    if r <= 0:
        return float("inf")
    c = m * ov * (1.0 + ov) / r
    return (math.sqrt(1.0 + 4.0 * c) - 1.0) / 2.0


def print_fec_report(rows):
    """FEC EPISODES: per (sid, mcs, bw, ov) group -- the rung and layer the
    episode flew on -- how many episodes, how many fell inside a transition
    (stale > 0: excluded from the counterfactual), how many actually failed
    (aband > 0), the missing-count and ov_req distributions, and how many
    non-stale episodes would have failed at each candidate overhead
    (ov_req > candidate). This is the input to a static rung-table retune
    (docs/observability.md, fec.log)."""
    if not rows:
        return
    groups = {}
    for r in rows:
        groups.setdefault((r["sid"], r["mcs"], r.get("bw", 20), r["ov"]), []).append(r)
    print("FEC EPISODES (fec.log: runs of sources never delivered; "
          "ov_req = overhead the episode would have needed)")
    for key in sorted(groups):
        sid, mcs, bw, ov = key
        g = groups[key]
        live = [r for r in g if r["stale"] == 0]
        stale = len(g) - len(live)
        failed = sum(1 for r in g if r["aband"] > 0)
        ms = [r["m"] for r in live]
        reqs = [fec_ov_req(r["m"], r["r"], ov) for r in live]
        print(f"  sid {sid} mcs {mcs}/{bw} ov {ov:.2f}: n={len(g)} stale={stale} "
              f"failed={failed}")
        if not live:
            continue
        print(f"    m p50/p90/max={_pct(ms, 0.5)}/{_pct(ms, 0.9)}/{max(ms)}  "
              f"r p50={_pct([r['r'] for r in live], 0.5)}  "
              f"w={_pct([r['w'] for r in live], 0.5)}")
        print(f"    ov_req p50/p90/p99/max={_pct(reqs, 0.5):.2f}/"
              f"{_pct(reqs, 0.9):.2f}/{_pct(reqs, 0.99):.2f}/{max(reqs):.2f}")
        fails = " ".join(f"{c:.2f}:{sum(1 for x in reqs if x > c)}"
                         for c in FEC_CANDIDATE_OV)
        print(f"    would fail at ov {fails}  (of {len(live)} non-stale)")


def sniff_ctllog(path):
    """True if `path` is a maburgs ctl log (first line starts 'ctllog ')."""
    with open(path) as f:
        first = f.readline()
    return first.startswith("ctllog ")


def drone_rx_samples(rows):
    """drone.radio.rx (cca-on 2026-09-23) is a PER-TELEMETRY-PERIOD count
    the exporter repeats on every record until the next Telem, so a sample
    is one distinct drone.tlm_seq, never one record. Returns a list of
    (own, foreign, crcfail) tuples in order; empty on recordings that
    predate the key."""
    out = []
    last_seq = None
    for r in rows:
        d = r.get("drone") or {}
        e = (d.get("radio") or {}).get("rx")
        if not isinstance(e, dict):
            continue
        seq = d.get("tlm_seq")
        if seq is not None and seq == last_seq:
            continue
        last_seq = seq
        out.append((e.get("own") or 0, e.get("foreign") or 0, e.get("crcfail") or 0))
    return out


def print_drone_rx_report(rows):
    """DRONE RX: what the drone's own receiver saw on the channel, per
    telemetry period -- the altitude view the GS cards cannot measure. With
    carrier sense ON (2026-09-23) `foreign` (CRC-clean, not ours) and
    `crcfail` (preamble heard, payload undecodable) are the frames the
    drone's transmitter deferred to; the hop verdict only reacts above
    hop.verdict.foreign_pps (50), so this is the number that says whether
    deferral below that bar is costing air. Silent on old recordings."""
    s = drone_rx_samples(rows)
    if not s:
        return
    own = [x[0] for x in s]
    foreign = [x[1] for x in s]
    crcfail = [x[2] for x in s]
    print()
    print(f"DRONE RX (per telemetry period, once per tlm_seq): n={len(s)}")
    print(f"  foreign: p50={_pct(foreign, .5):.0f} p90={_pct(foreign, .9):.0f} max={max(foreign)}"
          f"   crcfail: p50={_pct(crcfail, .5):.0f} p90={_pct(crcfail, .9):.0f} max={max(crcfail)}"
          f"   own: p50={_pct(own, .5):.0f} p90={_pct(own, .9):.0f} max={max(own)}")


def print_salvage_report(rows):
    """SALVAGE: what rx.keep_corrupted (2026-09-08) bought. The sideport's
    per-card crc_fail and per-stream corrupt/salvaged/sub_fail are
    cumulative; sum their per-interval deltas over the recording and
    attribute each interval's movement to the rung the ladder reports at
    the interval's end, next to that interval's abandoned symbols (the
    post-FEC loss the salvage is competing with). A maburgs restart
    rejoins the session directory, so one file can carry a counter reset
    (sideport `session` changes, counters restart from 0): a reset
    interval contributes the post-reset value, never a negative delta.
    Silent on recordings that predate the counters -- old jsonl on the DVR
    must still report cleanly."""
    srows = [r for r in rows
             if any("corrupt" in (s or {})
                    for s in ((r.get("link") or {}).get("streams") or []))]
    if len(srows) < 2:
        return

    def card_map(r):
        return {c.get("id"): c.get("crc_fail") or 0 for c in (r.get("cards") or [])}

    def stream_map(r):
        return {s.get("stream"): s for s in ((r.get("link") or {}).get("streams") or [])}

    def delta(cur, prev, reset):
        cur, prev = cur or 0, prev or 0
        return cur if (reset or cur < prev) else cur - prev

    keys = ("corrupt", "salvaged", "salvage_only", "sub_fail", "abandoned")
    # salvage_only (2026-09-09) is younger than the section: print it only
    # when the recording carries the key, so older jsonl reads unchanged.
    have_so = any("salvage_only" in (s or {})
                  for r in srows for s in stream_map(r).values())

    def so(v):
        return f" salvage_only={v['salvage_only']}" if have_so else ""
    card_tot, stream_tot, by_rung = {}, {}, {}
    prev = None
    for r in srows:
        cards, streams = card_map(r), stream_map(r)
        if prev is not None:
            pr, pcards, pstreams = prev
            reset = r.get("session") != pr.get("session")
            for cid, v in cards.items():
                card_tot[cid] = card_tot.get(cid, 0) + delta(v, pcards.get(cid), reset)
            rung = (((r.get("link") or {}).get("ctl") or {}).get("rung") or {}).get("idx")
            acc = by_rung.setdefault(rung, {k: 0 for k in keys})
            for sid, b in streams.items():
                a = pstreams.get(sid) or {}
                st = stream_tot.setdefault(sid, {k: 0 for k in keys})
                for k in keys:
                    dv = delta(b.get(k), a.get(k), reset)
                    st[k] += dv
                    acc[k] += dv
        prev = (r, cards, streams)

    print("SALVAGE (rx.keep_corrupted: FCS-corrupt bodies delivered, "
          "CRC16-clean sub-blocks salvaged)")
    for cid in sorted(card_tot):
        print(f"  card {cid}: crc_fail={card_tot[cid]}"
              "  (mabur + foreign; foreign junk lands here too)")
    for sid in sorted(stream_tot):
        v = stream_tot[sid]
        print(f"  stream {sid}: corrupt={v['corrupt']} salvaged={v['salvaged']}{so(v)} "
              f"sub_fail={v['sub_fail']} abandoned={v['abandoned']}")
    print("  PER RUNG (all streams; interval attributed to the rung at its end)")
    for rung in sorted(by_rung, key=lambda x: (x is None, x)):
        v = by_rung[rung]
        print(f"    rung {rung}: corrupt={v['corrupt']} salvaged={v['salvaged']}{so(v)} "
              f"sub_fail={v['sub_fail']} abandoned={v['abandoned']}")


def load_scanlog(path):
    """Parse a maburgs scan.log (gs/src/scan_log.cpp record formats,
    'scanlog N' marker). Only the record kinds the HOP report needs are
    tokenised -- V (per-window verdict), H (hop controller event), D (scout
    dwell) and M (channel-plan move). C (adapter caps) and K (scan pick) are
    skipped.

    V's per-card block REPEATS once per card (2 cards on this hardware
    today, but the parser must not assume that): 'V t verdict evidence_hex
    ref_rung|- link_loss_pct recovered [card foreign fa cca crc rssi snr
    drssi]...'.

    H's kind field is single-token snake_case today ("hold_cap" /
    "hold_exhausted" included, fixed at the emitter -- they used to be the
    literal two-word C++ strings "hold cap" / "hold exhausted", a
    space-delimited field containing the field delimiter). The parser still
    anchors epoch/target/score/elapsed_ms from the END of the line rather
    than a fixed column count (kind = everything between t_ms and those
    four always-single-token fields): harmless now that kind is always one
    word, and it costs nothing to keep it robust against a future kind that
    isn't.

    Returns {"version": int, "V": [...], "H": [...], "D": [...], "M": [...]}.
    Silent on lines that don't parse (older/newer record shapes) rather
    than aborting the report -- CLAUDE.md: recordings outlive the code that
    wrote them."""
    version = 0
    V, H, D, M = [], [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            toks = line.split()
            tag = toks[0]
            # A GS restart REJOINS the session directory, so one scan.log can
            # carry several 'scanlog N' markers -- after a deploy the first
            # session starts under the old binary's header with the new
            # binary's marker (and all its V/H/D records) further down. The
            # highest marker seen decides the version; the old sections hold
            # no record kind this report reads. (Bench 2026-09-15: reading
            # only line 1 skipped the HOP section on the session with the
            # first real hop.)
            if tag == "scanlog" and len(toks) >= 2 and toks[1].isdigit():
                version = max(version, int(toks[1]))
                continue
            try:
                if tag == "V" and len(toks) >= 7:
                    n_extra = len(toks) - 7
                    if n_extra % 8 != 0:
                        continue  # malformed card block; skip rather than misparse
                    ref_rung = None if toks[4] == "-" else int(toks[4])
                    cards = []
                    for i in range(7, len(toks), 8):
                        cards.append({
                            "card": int(toks[i]), "foreign": int(toks[i + 1]),
                            "fa": int(toks[i + 2]), "cca": int(toks[i + 3]),
                            "crc_fail": int(toks[i + 4]), "rssi_dbm": float(toks[i + 5]),
                            "snr_db": float(toks[i + 6]), "d_rssi_db": float(toks[i + 7]),
                        })
                    V.append({
                        "t_ms": float(toks[1]), "verdict": toks[2],
                        "evidence": int(toks[3], 16), "ref_rung": ref_rung,
                        "link_loss_pct": float(toks[5]), "recovered": int(toks[6]),
                        "cards": cards,
                    })
                elif tag == "H" and len(toks) >= 7:
                    kind = " ".join(toks[2:-4])
                    H.append({
                        "t_ms": float(toks[1]), "kind": kind,
                        "epoch": int(toks[-4]), "target": int(toks[-3]),
                        "score": int(toks[-2]), "elapsed_ms": float(toks[-1]),
                    })
                elif tag == "D" and len(toks) >= 17:
                    igi = None if toks[10] == "-" else int(toks[10])
                    floor_dbm = None if toks[11] == "nan" else int(toks[11])
                    D.append({
                        "t_ms": float(toks[1]), "card": int(toks[2]), "ch": int(toks[3]),
                        "round": int(toks[4]), "observe_ms": int(toks[5]),
                        "cca": int(toks[6]), "fa": int(toks[7]), "own": int(toks[8]),
                        "foreign": int(toks[9]), "igi": igi, "floor_dbm": floor_dbm,
                        "flags_hex": int(toks[12], 16), "sess": int(toks[13]),
                        "to_us": int(toks[14]), "read_us": int(toks[15]),
                        "back_us": int(toks[16]),
                        "bw": int(toks[17]) if len(toks) >= 18 else 20,
                    })
                elif tag == "M" and len(toks) >= 6:
                    card = None if toks[2] == "all" else int(toks[2])
                    M.append({
                        "t_ms": float(toks[1]), "card": card,
                        "from": int(toks[3]), "to": int(toks[4]), "reason": toks[5],
                    })
            except ValueError:
                continue  # malformed record; skip rather than abort the report
    return {"version": version, "V": V, "H": H, "D": D, "M": M}


def sniff_scanlog(path):
    """True if `path` is a maburgs scan log (first line starts 'scanlog ')."""
    with open(path) as f:
        return f.readline().startswith("scanlog ")


# HopController's own kinds (gs/src/hop_controller.cpp), stripped of any
# "would_" prefix (cfg_.enable=false: the FSM still runs and still logs,
# every action just suppressed -- see HopController::tick / log_event).
# "order" and "verify_fail" are the only two that place a NEW HopAction::Order
# (idle_tick's fresh trigger, and verifying_tick's retry-after-fail); the
# rest either continue an open attempt (lead_confirm, one_card_retune) or
# close one (verify_pass, withdraw, and the two hold variants).
_HOP_ORDER_KINDS = {"order", "verify_fail"}
# A hold is a STATE, and HopController logs only its EDGES: hold_cap /
# hold_exhausted / verify_fail on the way in, "hold_end" on the way out,
# whose elapsed_ms is how long the episode lasted (holds used to re-log
# every ~10 ms control tick instead). "hold_end" is TERMINAL rather than
# informational, defensively: the controller does not currently emit one
# while an attempt row is open -- a hold entry closes the row first, and
# the terminal "verify_fail" carries an UNBUMPED epoch so build_hop_rows'
# epoch-match branch reads it as the outcome, not a retry -- but an
# unrecognised kind arriving with a row open falls through to
# "unterminated", i.e. "the log ends mid-attempt", which would be a lie
# about a flight that ended in a hold. Recordings from before the
# edge-logging fix still parse: their first per-tick hold line closes the
# row exactly as the single one does now.
# "session_lost" (2026-09-24): HopController::on_session_lost withdrew an
# order that was still waiting for its confirm when the link's session
# dropped -- the attempt is over, with its own outcome.
_HOP_TERMINAL_ONLY_KINDS = {"withdraw", "session_lost", "hold_cap", "hold_exhausted",
                            "hold_end"}
_HOP_RESTORE_WINDOW_MS = 5000.0  # generous: production fires E hop_restore
                                 # essentially in the same tick as H order/
                                 # verify_fail (main.cpp calls
                                 # VrxController::restore_rung() synchronously
                                 # while handling HopAction::Order, well
                                 # before any lead_confirm/verify outcome) --
                                 # the window only needs to reject a restore
                                 # line that belongs to some OTHER hop.


def _strip_would(kind):
    return kind[len("would_"):] if kind.startswith("would_") else kind


def _find_nearest_unused(candidates, used, anchor_ms, window_ms):
    """Index of the closest not-yet-`used` candidate['t_ms'] to `anchor_ms`
    within `window_ms`, or None. Marks it used."""
    best_i, best_d = None, None
    for i, c in enumerate(candidates):
        if used[i]:
            continue
        d = abs(c["t_ms"] - anchor_ms)
        if d <= window_ms and (best_d is None or d < best_d):
            best_i, best_d = i, d
    if best_i is not None:
        used[best_i] = True
        return candidates[best_i]["t_ms"]
    return None


def build_hop_rows(H, restores):
    """One row per hop ATTEMPT (every event that places a fresh
    HopAction::Order: HopController's "order" and retry-triggering
    "verify_fail"), real or shadow ("would_"-prefixed, hop.enable=false).

    `restores` is ctl.log's E rows already filtered to reason=='hop_restore'.
    Each is consumed by at most one row, matched to that row's OWN order/
    retry timestamp (the real causal anchor: LadderController::restore()
    fires synchronously inside main.cpp's HopAction::Order handling, not at
    lead_confirm or verify_pass) -- never reused, so a hop that never got a
    restore (e.g. withdrawn, or a shadow row, which never restores at all)
    correctly prints blank instead of stealing a different hop's line.

    A "verify_fail" H line does double duty in the source: HopController::
    order() logs it while ALSO bumping the epoch, so it is simultaneously
    the outcome of the failed attempt and the start of the retry -- except
    when there is no retry candidate, when it is logged directly (epoch
    unchanged) as a true terminal outcome. The two are told apart here by
    epoch: unchanged epoch = terminal, bumped epoch = retry (a new row)."""
    used = [False] * len(restores)
    rows = []
    open_row = None

    def close(outcome_kind, t_ms):
        open_row["outcome"] = outcome_kind
        open_row["outcome_ts"] = t_ms
        rows.append(open_row)

    for h in H:
        kind = h["kind"]
        shadow = kind.startswith("would_")
        base = _strip_would(kind)

        if base in _HOP_ORDER_KINDS:
            if (base == "verify_fail" and open_row is not None
                    and h["epoch"] == open_row["order_epoch"]):
                close(kind, h["t_ms"])   # terminal: no retry follows
                open_row = None
                continue
            if open_row is not None:
                # A retry (base=="verify_fail", bumped epoch): the failed
                # verify IS why this new order was placed. A fresh "order"
                # while one was already open should never happen per the
                # FSM (strictly one attempt in flight) -- defensive only.
                close(kind if base == "verify_fail" else "interrupted", h["t_ms"])
            open_row = {
                "shadow": shadow, "order_ts": h["t_ms"], "order_epoch": h["epoch"],
                "target": h["target"], "video_ts": None,
                "outcome": None, "outcome_ts": None, "restore_ts": None,
            }
            if not shadow:
                # Shadow rows never call restore() at all (main.cpp's Order
                # case, and restore_rung() inside it, only runs when the
                # HopAction actually kind != None -- forced None whenever
                # cfg_.enable is false) -- searching would only risk
                # stealing a real hop's restore line.
                open_row["restore_ts"] = _find_nearest_unused(
                    restores, used, h["t_ms"], _HOP_RESTORE_WINDOW_MS)
            continue

        if open_row is None:
            continue   # a hold with no attempt in progress: not a row

        if base == "lead_confirm":
            if open_row["video_ts"] is None:
                open_row["video_ts"] = h["t_ms"]
            continue
        if base == "one_card_retune":
            continue   # informational only; doesn't end the attempt
        if base == "verify_pass" or base in _HOP_TERMINAL_ONLY_KINDS:
            close(kind, h["t_ms"])
            open_row = None
            continue
        # Unrecognised kind (a future addition): leave the row open rather
        # than guess at its meaning.

    if open_row is not None:
        # Log ends mid-attempt (DVR truncation, or the recording was cut).
        close("unterminated", open_row["order_ts"])
    return rows


def find_hop_onset(V, order_ts):
    """t_ms of the FIRST V in the contiguous non-healthy run immediately
    preceding `order_ts` -- the moment things started going bad, not just
    the last sample before the order. None if the nearest V at/before
    order_ts is healthy (or there is no V at all yet)."""
    before = sorted((v for v in V if v["t_ms"] <= order_ts), key=lambda v: v["t_ms"])
    if not before or before[-1]["verdict"] == "healthy":
        return None
    onset = before[-1]["t_ms"]
    for v in reversed(before[:-1]):
        if v["verdict"] == "healthy":
            break
        onset = v["t_ms"]
    return onset


def verdict_histogram(V):
    """{"healthy": n, ...} in the canonical hop_verdict.h order, zero counts
    omitted."""
    counts = {}
    for v in V:
        counts[v["verdict"]] = counts.get(v["verdict"], 0) + 1
    return {k: counts[k] for k in ("healthy", "fade", "interfered", "unknown") if counts.get(k)}


# gs/src/hop_verdict.h's five independent evidence bits, in the same order
# they're OR'd together in HopVerdict::window(). A window's verdict can be
# Healthy while some of these are still set (only `impaired` gates the top-
# level Healthy/not split; weak/fading/contended/raised are computed and
# OR'd in unconditionally) -- so the bit tally is over ALL windows, not
# just non-healthy ones, matching what the bitmask itself actually counts.
_EVIDENCE_BITS = (("impaired", 1), ("weak", 2), ("fading", 4),
                  ("contended", 8), ("raised", 16))


def evidence_bit_tally(V):
    """{"impaired": n, "weak": n, ...} -- how many windows set each bit."""
    return {name: sum(1 for v in V if v["evidence"] & mask) for name, mask in _EVIDENCE_BITS}


def evidence_card_medians(V):
    """Per card, over NON-healthy windows only: median foreign, fa,
    rssi_dbm, snr_db (median, not mean -- these are counter deltas with
    outliers) and the sample count. This is the calibration read: how far
    each threshold (hop_verdict.h's foreign_pps/fa_pps/weak_rssi_dbm/
    weak_snr_db) actually sat from what tripped it."""
    by_card = {}
    for v in V:
        if v["verdict"] == "healthy":
            continue
        for c in v["cards"]:
            d = by_card.setdefault(c["card"], {"foreign": [], "fa": [], "rssi_dbm": [], "snr_db": []})
            d["foreign"].append(c["foreign"]); d["fa"].append(c["fa"])
            d["rssi_dbm"].append(c["rssi_dbm"]); d["snr_db"].append(c["snr_db"])
    return {card: {"n": len(d["foreign"]),
                   **{k: statistics.median(vals) for k, vals in d.items()}}
            for card, d in by_card.items() if d["foreign"]}


def dwell_cost_summary(D):
    """Per card: count and median(to_us+read_us+back_us) of scout dwells
    taken with sess==1 -- INSIDE the live session, i.e. dwells that
    actually cost airtime (the brief's 'per-card dwell cost summary').
    Dwells outside a session (sess==0, off-air scanning) are free and
    excluded."""
    by_card = {}
    for d in D:
        if d.get("sess") != 1:
            continue
        by_card.setdefault(d["card"], []).append(d["to_us"] + d["read_us"] + d["back_us"])
    return {card: {"count": len(vals), "median_us": statistics.median(vals)}
            for card, vals in by_card.items()}


def _fmt_delta_ms(a, b):
    return "-" if a is None or b is None else f"{b - a:.0f} ms"


def _print_hop_table(rows, V):
    for r in rows:
        onset_ts = find_hop_onset(V, r["order_ts"])
        print(f"  t={r['order_ts']:.0f} target={r['target']}"
              f"{' [SHADOW]' if r['shadow'] else ''}"
              f"  onset->order {_fmt_delta_ms(onset_ts, r['order_ts'])}"
              f"  order->video {_fmt_delta_ms(r['order_ts'], r['video_ts'])}"
              f"  video->restore {_fmt_delta_ms(r['video_ts'], r['restore_ts'])}"
              f"  outcome {r['outcome']}")


def print_hop_report(scanlog, ctllog):
    """HOP section (Task 13): one row per hop attempt (onset->order->video
    ->restore, outcome), the per-card dwell cost summary, and -- on a
    session with zero REAL hops -- the verdict histogram, which for a
    hop.enable=false flight (every hop this plan's first flight will ever
    log) is the entire signal on whether the verdict engine is classifying
    right. hop.enable=false does not mean zero H events: the controller
    still runs and still logs every decision with a "would_" prefix, so a
    log full of would_order/would_withdraw pairs is handled as SHADOW rows
    (see build_hop_rows) printed in their own table, in addition to the
    histogram -- not silently collapsed into "zero hops". Alongside the
    histogram: the evidence bitmask decoded per bit, and per-card medians
    of foreign/fa/rssi/snr over non-healthy windows -- a verdict name alone
    can't tell contention from a weak signal from fading."""
    V, H, D = scanlog.get("V", []), scanlog.get("H", []), scanlog.get("D", [])
    restores = [e for e in ctllog.get("E", []) if e.get("reason") == "hop_restore"]
    rows = build_hop_rows(H, restores)
    real_rows = [r for r in rows if not r["shadow"]]
    shadow_rows = [r for r in rows if r["shadow"]]

    if real_rows:
        print(f"\nHOP REPORT ({len(real_rows)} hop(s))")
        _print_hop_table(real_rows, V)
    elif shadow_rows:
        print(f"\nHOP REPORT -- hop.enable=false: {len(shadow_rows)} SHADOW hop(s) "
              "(every action suppressed; these are what the controller WOULD have "
              "ordered). video never confirms while disabled -- nothing actually "
              "retunes -- so the outcome is structurally almost always a "
              "would_withdraw timeout at confirm_ms; that is expected, not a bug.")
        _print_hop_table(shadow_rows, V)
    if not real_rows:
        hist = verdict_histogram(V)
        print("verdicts: " + " ".join(f"{k} {n}" for k, n in hist.items()) if hist
              else "verdicts: (none)")
        # Calibration instrument (spec Open Items: "the observe-only
        # flights are the calibration") -- a verdict name alone can't tell
        # contention from a weak signal from fading, nor which card drove
        # it, so decode the evidence bitmask and show how close each
        # per-card threshold actually sat to tripping.
        tally = evidence_bit_tally(V)
        print("evidence bits: " + " ".join(f"{k}={n}" for k, n in tally.items()))
        medians = evidence_card_medians(V)
        for card in sorted(medians):
            m = medians[card]
            print(f"  card {card} (non-healthy, n={m['n']}): "
                  f"foreign={m['foreign']:.0f} fa={m['fa']:.0f} "
                  f"rssi={m['rssi_dbm']:.1f} snr={m['snr_db']:.1f}")

    dsum = dwell_cost_summary(D)
    if dsum:
        print("DWELL COST (sess=1: airtime actually spent scouting inside the live session)")
        for card in sorted(dsum):
            s = dsum[card]
            print(f"  card {card}: n={s['count']} median(to+read+back)={s['median_us']:.0f}us")


def main(path, aulog=None, probelog_path=None, scanlog_path=None):
    if sniff_feclog(path):
        print_fec_report(load_feclog(path))
        return
    if sniff_probelog(path):
        # A probe log on its own (bench use): just the per-body report and
        # the completion->probe join.
        probelog = load_probelog(path)
        au = load_aulog(aulog) if aulog else None
        if au is None:
            found = find_aulog_for(path, probelog)
            au = load_aulog(found) if found else []
            if found: print(f"au log: {found}")
        print_probe_report({"E": [], "P": []}, probelog, au)
        return
    if sniff_ctllog(path):
        ctllog = load_ctllog(path)
        print_wall_report(ctllog)
        print_episode_report(ctllog)
        print_bw40_report(ctllog)
        probelog = None
        probe_src = None
        if probelog_path:
            # Session mode: session.resolve() already paired ctl.log with
            # probe.log structurally (they are siblings in the session
            # directory), so the legacy filename-glob heuristic below is
            # not consulted at all.
            probelog = load_probelog(probelog_path)
            probe_src = probelog_path
        else:
            # Sibling probe body log: same NNNN as the ctl log
            # (ctl-NNNN_<date>.log / probe-NNNN_<date>.log), written alongside it
            # by the same GS session (Task 11, probe-stream). Absent on older
            # recordings and on ctl logs from a probe-less session.
            m = re.search(r"ctl-(\d+)_", os.path.basename(path))
            if m:
                matches = sorted(glob.glob(os.path.join(
                    os.path.dirname(path), f"probe-{m.group(1)}_*.log")))
                if matches:
                    probelog = load_probelog(matches[0])
                    probe_src = matches[0]
        au = None
        if probelog:
            found = aulog or find_aulog_for(probe_src, probelog)
            au = load_aulog(found) if found else []
        print_probe_report(ctllog, probelog, au)
        if scanlog_path:
            # scan.log is session-mode-only (session.py finds it as a
            # sibling of this same ctl.log; there is no legacy filename
            # heuristic for it, the feature postdates the ctl-NNNN_ layout).
            # Absent entirely, or a marker version older than the V/H/D
            # record shapes this parses (none shipped yet, but the wire is
            # not additive-only -- CLAUDE.md), skips the HOP section rather
            # than printing a misparsed or empty one.
            scanlog = load_scanlog(scanlog_path)
            if scanlog["version"] >= 2:
                print_hop_report(scanlog, ctllog)
        return

    rows = load(path)

    # SNR scale. There are TWO half-dB problems in one datagram and they
    # need saying separately, because only one of them is in the past.
    #
    # (1) cards[].classes[].snr changed 2026-08-04 (half-dB -> dB).
    #     Recordings straddling that change are not comparable; flag it
    #     rather than quietly averaging two scales together.
    #
    #     ⚠ THIS CHECK IS A BACKSTOP, NOT A DETECTOR, and it must not be
    #     read as one. It fires only above 60, i.e. only when the old
    #     half-dB scale pushed a value somewhere no real dB reading goes. A
    #     normal link at 10-25 dB reads 20-50 on the old scale and sails
    #     straight through in silence. There is no fixing that by making the
    #     threshold smarter: a pre-fix file reading 48 (24 dB) and a
    #     post-fix file reading 48 (a perfectly ordinary strong bench link)
    #     are the SAME NUMBER, and nothing in the datagram distinguishes
    #     them -- the schema is additive-only under v:1 and carries no scale
    #     tag. Silence here means "not obviously old", never "confirmed dB".
    #     Date the recording instead.
    snrs = [k["snr"] for r in rows for c in r.get("cards", [])
            for k in c.get("classes", {}).values()
            if isinstance(k.get("snr"), (int, float))]
    if snrs and max(snrs) > 60.0:
        print("WARNING: cards[].classes[].snr exceeds 60 -- this recording predates "
              "the 2026-08-04 half-dB fix; divide those values by 2 to compare with "
              "newer files.", file=sys.stderr)

    # (2) drone.uplink.snr_a/snr_b had the SAME bug and were fixed the same
    #     day, at the same place (the exporter) -- the drone's own receiver
    #     reads the uplink through the same devourer RxAtrib.snr and
    #     telemetry.cpp forwards it raw. So this is now the same backstop as
    #     (1), with the same limits, and not a live-bug warning.
    #
    #     It gets its own check rather than being folded into (1) because
    #     the two came from different senders and a recording can in
    #     principle straddle only one of them (a GS updated before its
    #     drone's telemetry was being logged). Same >60 threshold, same
    #     caveat: silence means "not obviously old", never "confirmed dB".
    up_snr = [s for r in rows
              for u in [((r.get("drone") or {}).get("uplink") or {})]
              for key in ("snr_a", "snr_b")
              for s in [u.get(key)] if isinstance(s, (int, float))]
    if up_snr and max(up_snr) > 60.0:
        print("WARNING: drone.uplink.snr_a/snr_b exceeds 60 -- this recording "
              "predates the 2026-08-04 half-dB fix; divide those values by 2 to "
              "compare with newer files. Saw max %.1f (= %.1f dB)."
              % (max(up_snr), max(up_snr) / 2.0), file=sys.stderr)

    # (3) link.streams changed shape 2026-08-29 (4 UEP layers -> 2: BASE
    #     sid0 + ENH sid1), in the SAME commit that changed the meaning of
    #     the wire's overhead field itself (RC_VERSION 4): the old field was
    #     a per-layer uep_layer_overhead fraction, the new one is the LITERAL
    #     FEC command overhead, and an old cmd value reads HALF of what the
    #     same nominal air overhead reads as post-break (old cmd x2 = new
    #     actual). This is a DETECTOR, not a converter (data-provenance
    #     policy, CLAUDE.md): a 4-entry streams array is the signature of a
    #     pre-break recording, and every overhead-shaped field in one --
    #     link.op.overhead, link.ctl.rung.ov, link.ctl.ladder[].ov,
    #     link.streams[].ov, drone.applied.overhead (renamed
    #     overhead_base/overhead_enh post-break) -- is cmd-scale (x0.5 air)
    #     there. Flag it and let the analyst read the numbers as what they
    #     are; never silently rescale historical data.
    old_shape = any(len((r.get("link") or {}).get("streams") or []) == 4
                     for r in rows)
    if old_shape:
        print("WARNING: link.streams has 4 entries -- this recording predates "
              "the 2026-08-29 overhead scale break (4-layer UEP, RC_VERSION <4). "
              "Every overhead value in this file (op.overhead, ctl.rung.ov, "
              "ctl.ladder[].ov, streams[].ov, drone.applied.overhead) is "
              "cmd-scale (x0.5 air) -- HALF the actual air overhead on the "
              "post-2026-08-29 scale. Read them as cmd-scale (x0.5 air); do "
              "not compare directly with a post-break recording.",
              file=sys.stderr)

    trans, in_rung, u_by_rung, residuals = [], {}, {}, []
    prev_ev_t, prev = None, None

    for d in rows:
        l = d.get("link") or {}
        ctl = l.get("ctl")
        if not ctl: continue

        t, idx = d.get("t_ms", 0), ctl["rung"]["idx"]
        u_by_rung.setdefault(idx, []).append(ctl.get("util", 0.0))

        if prev is not None:
            in_rung[prev[1]] = in_rung.get(prev[1], 0) + (t - prev[0])
        prev = (t, idx)

        ev = ctl.get("last_event") or {}
        if ev.get("t_ms") and ev["t_ms"] != prev_ev_t:
            prev_ev_t = ev["t_ms"]
            # Context join: extract drone state and RSSI/SNR from cards
            drone_state = (d.get("drone") or {}).get("state", "unknown")
            rssi_s1 = None
            snr_s1 = None
            cards = d.get("cards") or []
            if cards:
                classes = cards[0].get("classes") or {}
                s1 = classes.get("s1") or {}
                rssi_s1 = s1.get("rssi")
                snr_s1 = s1.get("snr")
            trans.append((ev, drone_state, rssi_s1, snr_s1))

        # Residual loss detection and context join
        if (l.get("residual_loss") or 0) > 0:
            tail = [r for r in rows if 0 <= t - r.get("t_ms", 0) <= 5000]
            traj = [round((r["link"]["ctl"].get("util", 0)), 2)
                    for r in tail if (r.get("link") or {}).get("ctl")]
            drone_state = (d.get("drone") or {}).get("state", "unknown")
            rssi_s1 = None
            snr_s1 = None
            cards = d.get("cards") or []
            if cards:
                classes = cards[0].get("classes") or {}
                s1 = classes.get("s1") or {}
                rssi_s1 = s1.get("rssi")
                snr_s1 = s1.get("snr")
            residuals.append((t, l["residual_loss"], traj, drone_state, rssi_s1, snr_s1))

    # Merge consecutive residuals
    residuals = merge_consecutive_residuals(residuals)

    print("TRANSITIONS")
    for ev, drone_state, rssi_s1, snr_s1 in trans:
        rssi_str = f" rssi={rssi_s1:.1f}" if rssi_s1 is not None else ""
        snr_str = f" snr={snr_s1:.1f}" if snr_s1 is not None else ""
        print(f"  t={ev['t_ms']} rung {ev['from']}->{ev['to']} reason={ev['reason']} u={ev.get('u',0):.2f} drone_state={drone_state}{rssi_str}{snr_str}")

    print("TIME IN RUNG")
    total = sum(in_rung.values()) or 1
    for r in sorted(in_rung): print(f"  rung {r}: {in_rung[r]/1000:.1f}s ({100*in_rung[r]/total:.0f}%)")

    print("U PER RUNG (p50/p95/max)")
    for r in sorted(u_by_rung):
        us = sorted(u_by_rung[r])
        p = lambda q: us[min(len(us)-1, int(q*len(us)))]
        print(f"  rung {r}: {p(.5):.2f}/{p(.95):.2f}/{us[-1]:.2f}  n={len(us)}")

    print(f"RESIDUAL EPISODES: {len(residuals)}")
    for t, rl, trajs, drone_state, rssi_s1, snr_s1 in residuals:
        rssi_str = f" rssi={rssi_s1:.1f}" if rssi_s1 is not None else ""
        snr_str = f" snr={snr_s1:.1f}" if snr_s1 is not None else ""
        # Flatten list of trajectory lists
        flat_traj = [u for traj in trajs for u in traj]
        print(f"  t={t} residual={rl:.4f} u[-5s..]={flat_traj} drone_state={drone_state}{rssi_str}{snr_str}")

    print_salvage_report(rows)
    print_drone_rx_report(rows)

    # link.attrib.suppressed was removed from the sideport 2026-09-02 with
    # the packet-level delivery window it was defined against. Old
    # recordings still carry it; report it there and say what it means.
    sup = [((r.get("link") or {}).get("attrib") or {}).get("suppressed")
           for r in rows]
    sup = [s for s in sup if isinstance(s, (int, float))]
    if sup:
        print(f"attrib suppressed delta over flight: {int(sup[-1] - sup[0])}"
              "  (pre-2026-09-02 recording: counted windows where the "
              "packet-level total and attributed views disagreed; the key no "
              "longer exists)")


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import session as _session
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    if arg is None or os.path.isdir(arg):
        s = _session.resolve(arg)
        if s.dir is None:
            sys.exit("no session found; pass a session dir or a log file")
        primary = s.ctl or s.probe or s.flight
        if primary is None:
            sys.exit(f"{s.dir}: no ctl.log, probe.log or flight.jsonl")
        # In session mode both the au log and the ctl<->probe pairing are
        # structural (session.resolve() found them as siblings in the
        # session directory) -- find_aulog_for's index-overlap guess and
        # the ctl-NNNN_ filename-glob heuristic are not consulted at all.
        probe_arg = s.probe if primary == s.ctl else None
        scan_arg = s.scan if primary == s.ctl else None
        main(primary, s.au, probe_arg, scan_arg)
        # The ctl/probe branches of main() return before the jsonl analysis,
        # so in session mode read the sibling flight.jsonl for the SALVAGE
        # section too -- it is a flight's post-flight command, not a ctl
        # viewer. Silent when the recording predates the counters.
        if primary != s.flight and s.flight:
            print_salvage_report(load(s.flight))
            print_drone_rx_report(load(s.flight))
        # fec.log (2026-09-15) is a sibling too: the FEC EPISODES section
        # rides along whichever primary the session offered.
        if s.fec:
            print_fec_report(load_feclog(s.fec))
    else:
        main(arg, sys.argv[2] if len(sys.argv) > 2 else None)
