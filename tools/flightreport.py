#!/usr/bin/env python3
"""Post-flight analysis of a mabur sideport flight.jsonl (schema v1 + link.ctl)
or a maburgs ctl-NNNN_<date>.log (see gs/src/ctl_log.h; parses ctllog v1-v10,
warns on pre-v4, pre-v7, pre-v8, pre-v9 and pre-v10). Format is auto-detected
from the first line.
Usage: flightreport.py flight.jsonl | ctl-0001_20260805.log | probe-0001_20260905.log [au-NNNN.log]

A ctl or probe log also gets the probe-stream report; the optional second
argument names the flightrec au-NNNN.log to join probe rows to (otherwise
the one in the same directory or ./log whose mono-time range overlaps).

Note: last_event is a single overwritten struct on the wire; multiple rung transitions
inside one 500ms export window surface only as the LAST transition. Reported counts are
a lower bound due to this schema limitation."""
import glob, json, math, os, re, sys

# Same clamp sentinel CtlLog writes at the source (mirrors StatsExporter's
# clamp_util(): u3/u_pred/E's u carry a 1e9 zero-guard sentinel from
# LadderController when the divisor budget is 0, clamped to <=1e3 at the
# write site). Anything at or above this is "unmeasurable", not a reading.
SENTINEL = 1000.0


def is_sentinel(v):
    return isinstance(v, float) and v >= SENTINEL


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: rows.append(json.loads(line))
            except ValueError: continue
    if not rows: sys.exit("no parseable datagrams")
    return rows


def _parse_ladder_token(raw):
    """Parse the ctllog header's `ladder=` value into a list of
    {"mcs": int, "ov_base": float, "ov_enh": float} rungs.

    v1-v7 wrote a single per-rung overhead x100 (`mcs/ov`, e.g. "5/25");
    v8 (Task 5, same-rate-fixed-pairs) splits it into a base/enh pair
    (`mcs/ovb:ove`, e.g. "5/25:50"). A single (pre-v8) value is treated as
    both base and enh -- that rung had no split to lose."""
    rungs = []
    if not raw:
        return rungs
    for entry in raw.split(","):
        if "/" not in entry:
            continue
        mcs_s, ov_s = entry.split("/", 1)
        try:
            mcs = int(mcs_s)
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
        rungs.append({"mcs": mcs, "ov_base": ov_base, "ov_enh": ov_enh})
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

    print("DWELL (S records)")
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
        print(f"  rung {rung}: n={len(samples)} snr={snr_str}{evm_str}{extra}")

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


def probe_lead(E, P, horizon_ms=10000):
    """Per demote episode: time from the last `lossy` gate edge before its
    first demote (None if none). false_alarms = lossy edges with no demote
    within horizon_ms after them. Input for the v2 probe-demote threshold."""
    eps = find_episodes(E)
    lossy = [p for p in P if p["outcome"] == "lossy"]
    out = []
    for e in eps:
        prior = [p["t_ms"] for p in lossy if p["t_ms"] <= e["t0"]]
        out.append({"t0": e["t0"], "first_reason": e["first_reason"],
                    "lead_ms": (e["t0"] - max(prior)) if prior else None})
    demote_ts = [ev["t_ms"] for ev in E if ev["to"] < ev["from"]]
    false_alarms = sum(1 for p in lossy
                       if not any(0 <= t - p["t_ms"] <= horizon_ms for t in demote_ts))
    return {"episodes": out, "false_alarms": false_alarms, "lossy_edges": len(lossy)}


def load_probelog(path):
    """probe-NNNN_<date>.log: 'probelog <v> bpb=<n>' then
    't_ms seq mcs enh_fid blocks_ok card_mask snr_c0 snr_c1 evm_c0 evm_c1'
    plus, from probelog 2 (2026-09-05), 'first_ms': the radio's arrival
    stamp of the body's first sight (mono ms, µs fraction) -- None on v1
    rows, whose t_ms is the ~10 ms finalize tick and useless for timing."""
    rows, bpb, version = [], 4, 1
    with open(path) as f:
        first = f.readline().split()
        if len(first) >= 2 and first[0] == "probelog":
            version = int(first[1])
        for tok in first[2:]:
            if tok.startswith("bpb="): bpb = int(tok[4:])
        for line in f:
            t = line.split()
            if len(t) < 10: continue
            try:
                rows.append({"t_ms": float(t[0]), "seq": int(t[1]), "mcs": int(t[2]),
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
    best, best_ov = None, 0
    for cand in sorted(glob.glob(os.path.join(d, "au-*.log")) +
                       glob.glob(os.path.join(d, "log", "au-*.log"))):
        ts = [r["t_complete"] for r in load_aulog(cand) if r["t_complete"]]
        if not ts: continue
        ov = min(hi, max(ts)) - max(lo, min(ts))
        if ov > best_ov: best, best_ov = cand, ov
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


def probelog_summary(pl):
    """Per mcs: received bodies, lost bodies (seq gaps, attributed to the
    NEXT received body's mcs), surviving blocks, per-card body counts."""
    out = {}
    prev_seq = None
    for r in pl["rows"]:
        s = out.setdefault(r["mcs"], {"bodies": 0, "lost_bodies": 0, "blocks_ok": 0,
                                       "card0": 0, "card1": 0})
        if prev_seq is not None and r["seq"] > prev_seq + 1:
            s["lost_bodies"] += r["seq"] - prev_seq - 1
        prev_seq = r["seq"]
        s["bodies"] += 1; s["blocks_ok"] += r["blocks_ok"]
        if r["card_mask"] & 1: s["card0"] += 1
        if r["card_mask"] & 2: s["card1"] += 1
    return out


def print_probe_report(ctllog, probelog, au_rows=None):
    lead = probe_lead(ctllog.get("E", []), ctllog.get("P", []))
    print(f"\nPROBE GATE (lossy edges={lead['lossy_edges']}, "
          f"false alarms (no demote within 10 s)={lead['false_alarms']})")
    for e in lead["episodes"]:
        l = "-" if e["lead_ms"] is None else f"{e['lead_ms']:.0f}ms"
        print(f"  demote t={e['t0']/1000:.1f}s first={e['first_reason']} probe lead={l}")
    if probelog:
        bpb = probelog["bpb"]
        print("PROBE LOG (per mcs)")
        for mcs, s in sorted(probelog_summary(probelog).items()):
            tot = s["bodies"] + s["lost_bodies"]
            body_loss = s["lost_bodies"] / tot if tot else float("nan")
            blk_loss = 1 - s["blocks_ok"] / (tot * bpb) if tot else float("nan")
            print(f"  mcs{mcs}: bodies={s['bodies']} lost={s['lost_bodies']} "
                  f"body_loss={body_loss:.3f} block_loss={blk_loss:.3f} "
                  f"c0={s['card0']} c1={s['card1']}")
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
                    by_mcs.setdefault(r["mcs"], []).append(o)
                for mcs, v in sorted(by_mcs.items()):
                    print(f"    mcs{mcs}: n={len(v)} p50={_pct(v, .5):.2f} p90={_pct(v, .9):.2f} "
                          f"p99={_pct(v, .99):.2f}")
            else:
                print("  completion->probe: no joinable rows (probelog v1, or no "
                      "overlapping au-NNNN.log v2 next to it / in ./log)")


def sniff_probelog(path):
    """True if `path` is a maburgs probe log (first line starts 'probelog ')."""
    with open(path) as f:
        return f.readline().startswith("probelog ")


def sniff_ctllog(path):
    """True if `path` is a maburgs ctl log (first line starts 'ctllog ')."""
    with open(path) as f:
        first = f.readline()
    return first.startswith("ctllog ")


def main(path, aulog=None):
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
        probelog = None
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
        au = None
        if probelog:
            found = aulog or find_aulog_for(matches[0], probelog)
            au = load_aulog(found) if found else []
        print_probe_report(ctllog, probelog, au)
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
    if len(sys.argv) not in (2, 3): sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else None)
