# Probe stream — bench deploy + acceptance, 2026-09-04

First deployment of the always-on probe stream (spec
`2026-09-04-probe-stream-design.md`, branch `probe-stream`, PR #44) to the
bench pair. RC_VERSION 6 flag day: GS config first (five flat `probe_*`
keys deleted, `link.probe` added), then `maburgs` + `maburtop.py`, then
`maburd`. Rollbacks kept: `maburgs.pre-probe` / `maburd.pre-probe`
(pre-branch), `maburgs.pre-aufix` (branch, pre-`bb8d5d7`),
`/etc/maburgs.json.pre-probe`.

Deployed GS config carries the bench's tuned `link.probe.max_util: 0.2`
(carried over from the deleted `probe_max_util`); everything else default.

## What passed

| gate | result |
|---|---|
| `ausniff` (standing gate) | 60.3 / 60.5 fps, 0 `frame_id_gaps`, 0 resyncs |
| `aucadence` base−enh offset | probe ON 2.72 ms vs OFF 2.38 ms — Δ0.33 ms, inside the ±0.5 ms repeatability |
| cold climb | rung 0→5, **every** promote `promote_probed`, ~3.1 s/rung |
| air cost @mcs5 (flight-0018 jsonl) | ON 59.28 % (n 2890) vs OFF 59.35 % (n 352) — no measurable cost |
| refusal (§8.3 step 3, loss-sim `s5 eff=60`) | link **held at rung 1 for 95 s**, gate `lossy`, `u_probe` 2.36→1.69, `loss` 0.73, `probe_holds` 1, no promote; injection off → 1→5 all `promote_probed` within 30 s |
| observability | `ctllog 10` (S probe columns, P gate edges), `probe-NNNN.log`, `link.probe`, `classes.probe`, `promotes_probed`/`probe_holds` all populate; `flightreport.py` reads both files |

`probe_holds` counted **1** for a 95 s hold — one count per hold EPISODE,
as designed.

## Finding 1 — AU/body finalize phase lag (FIXED, `bb8d5d7`)

First deploy showed `probe_u` pinned at exactly **0.2** in 23.6 % of S
samples while the per-body probe log showed 1 lost body in 469.
0.2 = (4 blocks) / (15 bodies × 4) / budget_enh 0.333 — a **one-body
deficit**, not loss: an AU expectation finalized at `t_au + 100 ms` while
its probe body finalized at `t_body + 100 ms`, and the body arrives one
AU-airtime after the AU begins. The 500 ms window caught the gap.

Fixed by finalizing a matched AU together with its body (`ProbeTrack`,
match by `enh_fid` at `on_body`, book both in the same `tick()`).
Re-measured on the same bench with a long-dwell config (1350 samples at
probe rungs 1-2): nonzero samples **23.6 % → ~12 %**. The residual is
genuine loss, see Finding 2.

## Finding 2 — the RCF slotter concentrates GS TX blanking onto the probe (OPEN)

The probe body is the **last** body of every enh burst, and `RcfSlotter`
releases GS control frames at **AU completion** — which, since this
branch, is one probe body before the burst actually ends. The GS's own
transmit blanks both RX cards for ~180 µs
(`docs/gs-uplink-self-blanking-findings-2026-09-02.md`), so the slotter
now steers that blast onto the probe.

Pinned mcs4 / probe mcs4, 91 s per segment, probe loss from the per-body
log's seq span, enh from the sideport's sid-1 counters:

| segment | GS ctrl sends | probe body loss | enh pre-FEC |
|---|---|---|---|
| `feedback_ms` 50 | 25/s | 0.149 % | 0.073 % |
| `feedback_ms` 200 | 5/s | **0.000 %** | 0.037 % |
| slotter OFF (`rcf_slot_hold_ms` 0) | 20/s | 0.074 % | 0.106 % |
| `feedback_ms` 50 (repeat) | 24.8/s | 0.111 % | 0.081 % |

Two things follow. Probe loss **scales with the control-send rate** and
goes to exactly zero at 5 sends/s — it is GS-inflicted, not RF. And
turning the slotter OFF **halves probe loss (0.149 → 0.074 %) while
raising enh loss (0.073 → 0.106 %)**: with the slotter on, sends are
deliberately aimed at the completion instant, which is where the probe
now sits. The slotter is doing its job for video at the probe's expense.

Consequence for the gate: a lost probe body puts `u_probe` at 0.2 (or
0.214 / 0.4) for the ~10 consecutive 50 ms samples the 500 ms window
covers, which trips `lossy` against the bench's `max_util: 0.2` and
resets the clean streak. Promotes still happen (loss events are ~7-15 s
apart, streaks of 2 s fit between them), but the gate flaps and the
measurement reads ~2× the real video loss at the same MCS.

**Recommended fix (not made — `RcfSlotter` affects all video delivery and
wants its own bench validation):** teach the slotter that the burst ends
one probe body after the AU completes — add the probe's serialization
(1403 B at the probe MCS, ~0.2-1.1 ms) to `lead_ms`/`guard_ms` when a
probe profile is commanded, so releases land after the probe rather than
on it. Expected: probe loss → the `feedback_ms 200` floor, enh unchanged.

## Notes for the next bench session

- The GS's `/root/losssim.py` was **stale** (pre-dates this work) and
  defaults to `--port 8302`; the `S96maburgs.losssim` wrapper starts the
  daemon with `--loss-sim 8390`. Pass `--port 8390` or the tool times out
  with "is maburgs running with --loss-sim?" — the in-repo tool has the
  same 8302 default. Fresh copy now deployed as `/root/losssim.py`
  (old kept as `losssim.py.pre-probe`).
- The loss-sim GS binary is `/usr/local/bin/maburgs.losssim`
  (`-DMABUR_LOSS_SIM=ON` cross-build); production is the plain `maburgs`.
- At the **top rung** the S line's probe columns read `-1 nan 0` and the
  gate is `off` — there is nothing above to probe. To sample `u_probe`,
  either catch the climb or hold the ladder below top (this session used
  `link.clean_ms: 60000`).
- Not yet done: the flight (spec §8.3 step 6) and the `probe-lead` report
  over flight logs, which is the input for the v2 probe-driven demote.
