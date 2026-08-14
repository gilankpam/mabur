#!/usr/bin/env python3
"""Post-flight analysis of a mabur sideport flight.jsonl (schema v1 + link.ctl)
or a maburgs ctl-NNNN_<date>.log (see gs/src/ctl_log.h; parses ctllog v1-v4,
warns on pre-v4). Format is auto-detected from the first line.
Usage: flightreport.py flight.jsonl | ctl-0001_20260805.log

Note: last_event is a single overwritten struct on the wire; multiple rung transitions
inside one 500ms export window surface only as the LAST transition. Reported counts are
a lower bound due to this schema limitation."""
import json, math, sys

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
        # ctllog 4 ladder=0/100,2/50,... down_util=0.35 up_util=0.15
        for tok in first.split()[2:]:
            if "=" not in tok: continue
            k, v = tok.split("=", 1)
            header[k] = v

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
    passes = [r for r in records if r["outcome"] == "pass"]
    fails = [r for r in records if r["outcome"] == "fail"]
    aborts = [r for r in records if r["outcome"] == "abort"]

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
    With link.attrib on this should be ~zero; hits mean the transition
    watermark missed a debris class (spec 2026-08-14 fade-demote §5)."""
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


def sniff_ctllog(path):
    """True if `path` is a maburgs ctl log (first line starts 'ctllog ')."""
    with open(path) as f:
        first = f.readline()
    return first.startswith("ctllog ")


def main(path):
    if sniff_ctllog(path):
        ctllog = load_ctllog(path)
        print_wall_report(ctllog)
        print_episode_report(ctllog)
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

    sup = [((r.get("link") or {}).get("attrib") or {}).get("suppressed")
           for r in rows]
    sup = [s for s in sup if isinstance(s, (int, float))]
    if sup:
        print(f"attrib suppressed delta over flight: {int(sup[-1] - sup[0])}")


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit(__doc__)
    main(sys.argv[1])
