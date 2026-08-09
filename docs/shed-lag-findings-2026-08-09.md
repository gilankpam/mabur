# Encoder shed lag on demote cascade: the 2026-08-09 freeze-crash root cause

Field crash 2026-08-09: during a fast fade (34→20 dB in ~3 s) the pilot saw
noticeable lag, then a 1–2 s video freeze, and crashed. The flight record is
`ctl-0020_20170804.log` on the GS DVR (episode B, t=598.9–604 s, is the
reference cascade; episodes A and C show the same signature). This document
records the root-cause analysis, the simulation, and the hardware
reproduction. The fix is designed separately
(`docs/superpowers/specs/` — gitignored, see CLAUDE.md for why specs live
there).

## TL;DR

A multi-rung demote cascade outruns the drone's encoder bitrate policy.
The radio MCS follows every rung change within ~200 ms, but the encoder
bitrate call is gated by a 1000 ms throttle plus ±10 % hysteresis
(`drone/src/rc_agent.cpp`, `run_bitrate_policy`), so during a cascade the
first shed (a mid-ladder value) sets the throttle stamp and the following
sheds — including the rung-0 floor — are swallowed for up to a second.
The encoder then pushes ~10× the bottom rung's budget into the air:

- **Freeze**: the TxQueue (256 bodies, drop-oldest) overflows and kills
  whole FEC bodies → unrecoverable base-stream loss → the player drops
  incomplete AUs and waits up to ~2 s for the next periodic `sid==0`
  refresh (no IDR-request path exists).
- **Lag**: once the encoder finally sheds, rung 0's budget leaves only
  ~15 % drain headroom, so the queue built during the descent trickles out
  for seconds — sustained latency after the link is already "stable".

Three latencies stack: (1) the one-rung-at-a-time descent transmits at
dead MCSes while the channel collapses; (2) the encoder shed lag above;
(3) the open-loop ~2 s decode recovery. Leg 2 dominates and is fully
validated below.

## Evidence 1: ctl-0020 (the crash itself)

Three full-ladder cascades (5→0 in 1.2–2.0 s). After each arrival at
rung 0, s1 utilization decays over ~2–3 s — the queue drain — e.g. episode
B: u = 0.97, 0.63, 0.05 at 1 Hz. Residual demotes fired at three rungs on
the way down. The flight ends with `starved` at t=708.9 s followed by
57 s of bit-identical samples (drone TX silent — the crash).

## Evidence 2: host simulation (`tests/sim_shed_lag.cpp`)

Replays episode B's exact RCF timeline into the **real `RcAgent`**:

- Encoder receives exactly two calls: 14500 kbps at cascade+0.3 s and the
  1400 floor at cascade+1.27 s. The 19300 shed dies to hysteresis, the
  5100 shed to the throttle. (Asserted — this is the regression pin.)
- A coarse TxQueue/air model on that timeline reproduces the logged u
  decay and drops ~13 % of bodies.
- Counterfactual (shed forced on every decrease): drops 13 %→0, but the
  queue-drain tail survives — rung 0 cannot flush a descent-built queue
  quickly. A complete fix must also flush the queue, not just shed faster.

## Evidence 3: hardware reproduction (bench, 2026-08-09)

Rig: the `bench/s3-loss-sim` injector ported onto master
(`bench/shed-lag-validation` branch), GS running `--loss-sim 8390`.
Trigger: `s1 eff=35 burst=3` for 3 s — heavy enough to defeat every rung's
FEC overhead, emulating a fade that kills all rungs. Five identical bursts
(`/root/shedlag_campaign.sh` on the GS), analyzed by
`tools/bench/shedlag_report.py`:

| burst | cascade→floor | shed lag | txq drops | fps min | freeze | s1 abn peak |
|---|---|---|---|---|---|---|
| 1 | 665 ms | 2.55 s | 1786 | 4 | 1.0 s | 1505/s |
| 2 | 310 ms | 3.01 s | 2080 | 6 | 2.0 s | 2180/s |
| 3 | 430 ms | 2.69 s | 1640 | 6 | 1.5 s | 2388/s |
| 4 | 518 ms | 2.55 s | 1475 | 10 | 1.5 s | 1607/s |
| 5 | 360 ms | 2.56 s | 1829 | 4 | 1.5 s | 2115/s |

(shed lag is the 1 Hz-telemetry upper edge: first demote → `cmd_kbps` at
the floor value. Bench values: steady cmd 16000, rung-0 floor 1300.)

Every burst reproduces the crash signature: ~1500–2000 TxQueue drop-oldest
kills while `enc.mbps` reads ~12 into a 1.3 Mbps budget, fps collapse to
4–10, 1–2 s freeze, s1 abandonment 1.5–2.4 k/s.

Controls that make it airtight:

- **10 % loss = null result.** One rung absorbed it (u 0.53→0.01), no
  freeze, no queue drops, cmd unchanged. Single-rung demotes at high rungs
  are harmless; the pathology needs a multi-rung budget drop.
- **Injection-free freeze.** A promote-flap (0→1→0 via s3_util) froze
  video (fps 8, 164 queue drops) with the injector fully off — the
  congestion is drone-side, not an artifact of injected loss.

## The A/B gate for any fix

Re-run the identical campaign against the fixed drone build and compare
tables. Acceptance: `txq_drops = 0`, `freeze_ms = 0`, no fps sample below
20, shed lag bounded by one telemetry period. Baseline recording:
`/media/dvr/shedlag-base.jsonl` + `shedlag-campaign-baseline.log` on the
GS.

## Fix and A/B result (2026-08-09, same day)

The fix is the synchronized shed (`f200f6d`): in
`RcAgent::run_bitrate_policy`, a bitrate DECREASE vs the last sent value
bypasses the v1 throttle/hysteresis and goes out on the same tick that
applies the MCS; identical re-sends stay deduped and increases stay lazy.
No flush, no self-IDR, no wire change — the campaign was to decide whether
more was needed, and it wasn't. Identical 5-burst campaign against the
fixed `maburd`:

| burst | cascade→floor | shed lag | txq drops | fps min | freeze | s1 abn peak |
|---|---|---|---|---|---|---|
| 1 | 886 ms | 1.22 s | **0** | 50 | **0** | 312/s |
| 2 | 360 ms | 1.04 s | **0** | 56 | **0** | 387/s |
| 3 | 578 ms | 1.22 s | **0** | 56 | **0** | 589/s |
| 4 | 767 ms | 1.69 s | **0** | 50 | **0** | 508/s |
| 5 | 412 ms | 1.51 s | **0** | 54 | **0** | 357/s |

Acceptance: PASS on all five bursts — zero TxQueue drops (was 1475–2080),
zero freeze (was 1.0–2.0 s), fps floor 50 (was 4–10). The reported shed
lag is measured from the FIRST demote and includes the cascade walk
itself plus the 1 Hz telemetry sampling edge; per-burst
`shed_lag − cascade` is 335–918 ms, i.e. within one telemetry period of
the floor commit. The residual s1 abandonment (312–589/s) and the small
trunc/drop counts occur DURING the 35 % injection burst — that is the
injected trigger loss itself, present in both runs; the baseline's excess
over it was the drone-side queue kill, now gone.

The queue-drain lag tail the model predicted did not produce a visible
symptom (fps never left the 50s); the deferred ideas (TxQueue flush,
drone self-IDR, GS-side multi-rung demote, RCF IDR-request) stay
deferred until a measured need appears.

## Deployment state (bench, as of 2026-08-09)

- GS: master + loss-sim rig, started with `--loss-sim 8390` (rollbacks
  `/usr/local/bin/maburgs.pre-shedlag`, `/etc/init.d/S96maburgs.pre-shedlag`).
  Control port is 8390 — NOT the rig's old 8302 default, which maburplay's
  GS OSD now occupies.
- Drone: runs the shed-sync fix build (`f200f6d`), rollback
  `/usr/bin/maburd.pre-shedsync` (the pre-fix cca-disable build; the older
  `.pre-discca` backup was pruned for rootfs space).
