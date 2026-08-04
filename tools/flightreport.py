#!/usr/bin/env python3
"""Post-flight analysis of a mabur sideport flight.jsonl (schema v1 + link.ctl).
Usage: flightreport.py flight.jsonl

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


def main(path):
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

    # (2) drone.uplink.snr_a/snr_b were NOT fixed. They are drone-sourced,
    #     forwarded raw from devourer by the T_TELEM frame, and are half-dB
    #     on every recording ever made, including one taken today. So this
    #     one needs no threshold and gets none: presence of the key is the
    #     proof, and a value-based guess would only be able to miss. Warning
    #     unconditionally is the point -- the GS-side check above is a
    #     backstop for a bug that is fixed, and this is a live one.
    up_snr = [s for r in rows
              for u in [((r.get("drone") or {}).get("uplink") or {})]
              for key in ("snr_a", "snr_b")
              for s in [u.get(key)] if isinstance(s, (int, float))]
    if up_snr:
        print("WARNING: drone.uplink.snr_a/snr_b are HALF-dB on every recording -- "
              "that path was never converted, so unlike cards[].classes[].snr they "
              "are wrong regardless of date; divide by 2. Saw max %.1f (= %.1f dB)."
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


if __name__ == "__main__":
    if len(sys.argv) != 2: sys.exit(__doc__)
    main(sys.argv[1])
