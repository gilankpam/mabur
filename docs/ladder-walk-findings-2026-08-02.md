# Ladder controller walk campaign — findings, down_util 0.35 adoption

Date 2026-08-02. Three walk recordings (house + two open-field), analyzed
with `tools/flightreport.py` (master) and the `--calib` sweep from
`feat/ladder-stress-calib`. This campaign supersedes the BLOCKED bench
stress calibration (docs/ladder-stress-calibration.md): real fades finally
produced the episodes the bench's ≥10 dB margin never could.

## Recordings

| file (repo root, untracked) | environment | span | notes |
|---|---|---|---|
| `flight-walk-20260802.jsonl` | house→out, GS indoors, obstructions | 8.3 min | manual statsrec; tail ≥595 s = drone powered off |
| `flight-field-20260802.jsonl` | open field, smooth distance fades | 2.6 min | boot-recorder's first field file; down_util 0.6 |
| `flight-field2-du035-20260802.jsonl` | open field, same route | 1.7 min | **down_util 0.35** |

Infrastructure added this session: the GS now records the stats sideport
from boot — `/etc/init.d/S97statsrec` → `/usr/local/bin/statsrec.py` →
`/media/dvr/flight-NNNN_<date>.jsonl` (59 G SD, survives power cut, one
file per boot, DVR-style index because the RTC is wrong at boot; :8301
re-emit kept for a live `maburtop --port 8301`). Reboot-verified. Field
recordings need no manual step.

## Result 1 — util is coincident under obstruction fades, mildly leading
under smooth fades

The spec's binding selection rule (down_util = highest candidate catching
≥80% of slow episodes with ≥2 s median lead) is **unreachable on real
data**, but for a different reason than the bench block:

- **House walk (obstruction cliffs, ~20 dB in ~2 s):** 19 residual
  episodes; u jumps 0.01→0.5 in the same 2 Hz sample loss appears. Best
  candidate (0.05) catches 9/19 at ~1 s lead. No threshold makes the util
  path *leading* here.
- **Field walks (smooth distance fades):** u builds 0.33→0.35 over the
  1–2 samples before loss. Candidates 0.05–0.35 catch 2/2 baseline
  episodes at 0.5–1.5 s lead; the old 0.6 catches 0/2 (house: 2/19).

Clean-side margin: clean u p95 ≤0.06 at rungs 1–3 (house mid-zone) but
0.30–0.33 at rungs 1/3 under genuine field stress → 0.35 is the floor-safe
pick; do not go below 0.30.

## Result 2 — down_util 0.6 → 0.35 ADOPTED and field-validated

Deployed via GS config (`link.down_util` in `/etc/maburgs.json`; GS-only
key, drone unaffected). A/B on the same field route:

| | baseline du=0.6 | du=0.35 |
|---|---|---|
| util demotes | 1 | 2 — u=0.38 @ −67 dBm, u=0.46 @ −83, both pre-loss, neither reachable at 0.6 |
| residual episodes | 2 | **0** (caveat: walk2 never went below −85; baseline's episodes were at −89) |
| drop rate, −60..−75 dBm band | 2.1 f/s | **0.9 f/s** |
| false util demotes at clean signal | 0 | 0 (no flap) |

Verdict: keep 0.35.

## Result 3 — the rung failure wall is portable across environments

Every rung ≥3 failure in all three walks sat at **s1 RSSI −73±2**
(SNR 31–42); rung 5 parks clean above ≈ −60. Same numbers indoors with
multipath and in the open field → an RSSI-gated promote (don't retry
rungs ≥3 below ~−70, rung 5 below ~−62) has write-downable thresholds
without further calibration. The mcs6 inversion history
(docs/mcs6-bench-anomaly.md) is the standing caution against assuming
full SNR→rung monotonicity; the gate should be a *cap*, not a table.

## Result 4 — controller behavioral findings

**Good:** clean park at rung 5 (0 loss over minutes); correct mid-zone
settling; link held alive at −89 dBm/SNR 4–10 at rung 0 with fps ~58–60;
smooth-fade tracking 5→4→3→2→1→0 with only 198 dropped frames and 0
stalls/RTP gaps (field baseline).

**Bad, ranked by user-visible cost:**

1. **Signal-blind escalating penalties** (house walk): one residual blip
   (u=0.01, −68 dBm) on the walk-back earned rung 3 an escalated **60 s**
   penalty; the ladder sat at rung 2 for 30+ s while RSSI improved
   −54→−26. The penalty ledger ignores that conditions changed since the
   fail. Fix: release/halve penalties when SNR improves ≥ ~10 dB over the
   value recorded at penalty time.
2. **Climb-crash yo-yo under obstruction** (house): 4× promote-into-wall
   cycles (0→3→crash) at −73 dBm before escalated penalties settled it at
   rung 2; 9 probation fails, 796 frames dropped over the walk. The RSSI
   promote gate (Result 3) prevents this class entirely.
3. **Cold-start shed** (both field walks): the encoder pushes 8 Mbps from
   the first frame while the ladder is still at rung 0 (mcs0/ov1.0 ≈
   3 Mbps usable) and climbs at 5 s/rung → ~25 s of shedding (86 frames in
   one burst, walk2, at −29 dBm). Fix ideas: shorter promote hold during
   initial climb, or SNR-informed starting rung.
4. **s3-only bleed off-bench** (house, deep edge): buckets with s1
   abandoned=0 but s3 abandoning 122–156 syms/20 s while u≈0.08 — the
   known s3 blindness, now seen in the wild. NOT reproduced under smooth
   field fades (s1/s3 abandons equal) → looks obstruction-specific;
   priority drops below items 1–3.

Also: rungs 3/4 earned ~1–8 % dwell across all walks — at these ranges
they are mostly transit/crash rungs, worth rechecking after the promote
gate lands.

## Improvement queue (evidence-ranked)

1. Signal-conditioned penalty release (item 4.1)
2. ~~down_util 0.35~~ — DONE this session
3. Faster cold start (item 4.3)
4. RSSI-gated promotes (Result 3)
5. s3-abandonment input (item 4.4, obstruction environments only)

## Open config gap

The current GS SD image carries `"mcs": 6, "overhead": 0.15` in the
ladder — the ov 0.25 adoption from the mcs6+STBC investigation
(docs/mcs6-bench-anomaly.md) is missing from it. Until edited, a field
park at mcs6 will re-expose the enhance-tail bleed.
