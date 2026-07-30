#!/usr/bin/env python3
"""Post-flight analysis of a mabur sideport flight.jsonl (schema v1 + link.ctl).
Usage: flightreport.py flight.jsonl | --calib [--confirm-ms N] run1.jsonl [run2 ...]

Note: last_event is a single overwritten struct on the wire; multiple rung transitions
inside one 500ms export window surface only as the LAST transition. Reported counts are
a lower bound due to this schema limitation."""
import json, sys


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


def _percentile(sorted_vals, q):
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


def calib(paths, confirm_ms=250):
    """Threshold calibration over one or more recordings (spec 2026-07-30):
    clean-u percentiles per rung from stress-off residual-free runs, and a
    candidate down_util sweep scoring lead time from sustained crossing to
    first residual. Caveat printed with the output: the sideport samples u at
    2 Hz while the controller ticks at ~20 Hz, so leads are +/-500 ms and a
    sub-500ms confirm window is treated as one sample."""
    candidates = [round(0.05 * i, 2) for i in range(1, 13)]  # 0.05 .. 0.60
    clean_u = {}
    episodes = []
    for path in paths:
        rows = load(path)
        samples = []
        for d in rows:
            link = d.get("link") or {}
            ctl = link.get("ctl")
            if not ctl:
                continue
            samples.append((d.get("t_ms", 0),
                            ctl.get("util", 0.0),
                            (ctl.get("rung") or {}).get("idx"),
                            (link.get("op") or {}).get("offset_qdb") or 0,
                            (link.get("residual_loss") or 0) > 0))
        if not samples:
            continue
        has_resid = any(s[4] for s in samples)
        stress_on = any(s[3] != 0 for s in samples)
        if not has_resid and not stress_on:
            for _t, u, rung, _off, _r in samples:
                clean_u.setdefault(rung, []).append(u)
        prev_resid = False
        for i, (t, _u, rung, off, resid) in enumerate(samples):
            if resid and not prev_resid:
                leads = {}
                for c in candidates:
                    fire = None       # first completed confirm window above c
                    run_start = None
                    for t2, u2, _rg, _of, _rs in samples:
                        if t2 >= t:
                            break
                        if u2 > c:
                            if run_start is None:
                                run_start = t2
                            if fire is None and t2 - run_start >= confirm_ms - 500:
                                fire = t2  # 2 Hz sampling: one sample above c
                                           # already spans >= any confirm <= 500
                        else:
                            run_start = None
                    leads[c] = (t - fire) if fire is not None else None
                episodes.append((path, t, rung, off, leads))
            prev_resid = resid

    print("CLEAN U PER RUNG (stress-off, residual-free runs)")
    for rung in sorted(k for k in clean_u if k is not None):
        us = sorted(clean_u[rung])
        print(f"  rung {rung}: p50 {_percentile(us, .5):.3f} "
              f"p95 {_percentile(us, .95):.3f} p99 {_percentile(us, .99):.3f} n={len(us)}")
    print(f"CANDIDATE down_util SWEEP (episodes={len(episodes)}, confirm_ms={confirm_ms}, "
          f"leads +/-500ms at 2 Hz sampling)")
    for c in candidates:
        caught = [e[4][c] for e in episodes if e[4][c] is not None]
        med = sorted(caught)[len(caught) // 2] / 1000.0 if caught else 0.0
        print(f"  {c:.2f}: caught {len(caught)}/{len(episodes)}, median lead {med:.1f}s")
    print("EPISODES")
    for path, t, rung, off, leads in episodes:
        ls = " ".join(f"{c:.2f}->{leads[c]/1000.0:.1f}s" for c in candidates
                      if leads[c] is not None)
        print(f"  {path} t={t} rung={rung} offset={off} {ls or 'uncaught at all candidates'}")
    print("SELECTION RULE (spec 2026-07-30): up_util > clean p99 x >=2; "
          "down_util = highest candidate catching >=80% of SLOW-ramp episodes "
          "with median lead >= 2s; confirm_ms <= half that median lead; "
          "fast-ramp episodes are informational only.")


def main(path):
    rows = load(path)
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


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--calib":
        args = args[1:]
        confirm = 250
        if args and args[0] == "--confirm-ms":
            confirm = int(args[1]); args = args[2:]
        if not args: sys.exit(__doc__)
        calib(args, confirm)
    elif len(args) == 1:
        main(args[0])
    else:
        sys.exit(__doc__)
