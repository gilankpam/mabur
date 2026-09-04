# TX/RX timing: who transmits when, and what actually coordinates it

Written 2026-09-05 after the probe-blanking fix
(`docs/probe-blanking-fix-findings-2026-09-05.md`), because the question
"does mabur orchestrate the half-duplex radio?" kept coming up with the
answer spread across six findings docs. This page is the holistic view;
each claim points at where it was measured. Numbers marked *bench* are
from the 2026-09-05 rig (pinned mcs4, 60 AU/s, `feedback_ms` 50,
agg 6, two GS cards) unless stated otherwise.

## 1. The physics that everything else follows from

- **Every card is half-duplex.** A Realtek 8822E/Jaguar3 card cannot
  receive while it transmits. The drone has one card. The GS has two,
  both receiving video, one of them (the `TxSelector`'s pick) also
  sending the uplink.
- **Carrier sense is OFF on both daemons** (since 2026-08-05,
  `dev_cfg.tuning.disable_cca = true` in `drone/src/main.cpp` and
  `gs/src/radio_frontend.cpp`; `docs/data-provenance.md`). Neither radio
  defers to the other or to co-channel traffic. The one thing the chip
  still does on its own: it will not *start* a TX while an RX PPDU is
  in progress — it fires the moment that PPDU ends
  (`docs/gs-uplink-self-blanking-findings-2026-09-02.md` "Mechanism").
- **A GS send blinds both GS cards** for ~180 µs: the transmitting card
  is deaf by definition and the sibling is saturated at −4 dBm. Any
  drone PPDU whose preamble starts inside that window is lost on both
  cards, whole (one A-MPDU aggregate). Same doc, evidence 4–6.
- **A drone burst deafens the drone** for its whole duration. An RCF
  that lands inside a burst is gone; delivery to the drone tracks
  `1 − air_pct` (`docs/rcf-uplink-loss-findings-2026-08-14.md` §3).
- **The two clocks free-run.** The drone's sensor/encoder cadence and the
  GS's `feedback_ms` timer are independent oscillators. Nothing in mabur
  syncs them (no TSF exchange, no beacon-derived schedule).

So on one channel there are two talkers, no listen-before-talk, and the
only hardware arbitration is "finish receiving the current PPDU before
keying up". Everything mabur does about timing is software on top of
that.

## 2. Who transmits what

Drone → GS (the downlink, ~99 % of airtime):

| what | cadence | size / air (bench, mcs4) | shaped by |
|---|---|---|---|
| video bodies, base (sid 0) + enh (sid 1) | one AU per frame, 60 AU/s alternating base/enh at 30 fps each; each AU is one burst | AU ~29–37 kB p50 → 20–27 bodies of 1403 B → 5–7 aggregates of 4 (agg 6 fills 4 on this feed); burst 12.7 ms mean, max 21–28 | encoder bitrate (`run_bitrate_policy`), FEC overhead pair per rung, MCS, `feed_batch` 6 + `ampdu.max_num` 6 |
| probe stream (sid 5) | one body after every ENH AU, 30/s | 1403 B, own PPDU right behind the burst's last aggregate; lands 0.9 ms p50 / 4 ms p99 after the AU's completion stamp | MCS of rung `current + 1`; ≈1 % of air at mcs4, 2.7 % at mcs1 (`docs/airtime-model.md` §6) |
| MSP OSD (sid 4) | `msp.update_rate_hz` 3, ~5–8 bodies/s | 1343 B | not slotted; shares the chip's hardware seq counter with video (was the phantom-loss source, fixed) |
| T_TELEM | 1 Hz | ~80 B | not slotted |
| DISC_ACK / beacons | rendezvous only | small | not slotted |

GS → drone (the uplink, low duty cycle, but every send is a blast):

| what | cadence | notes |
|---|---|---|
| RCF (rate-control feedback) | every `feedback_ms` (default 100; bench 50) → 10–20/s | ~20–27 B at HT MCS0, ~180 µs incl. preamble |
| RCF repeat burst | `rcf_repeat_copies` 3 × `rcf_repeat_ms` 10 on an op-changing RCF only | the fix for the 30–50 % uplink loss (`rcf-uplink-loss` §4.3); adds sends exactly when the link is changing |
| DISC keepalive | `beacon_keepalive_ms` 1000 | slotted like any RCF since 2026-09-03 |

Bench totals: GS 20 sends/s, drone hears 18.5/s (93 %) *with the slotter
on*; before slotting, delivery was 51–69 % depending on rung
(`rcf-uplink-loss` §3, flight-confirmed §7).

## 3. What mabur does orchestrate

There is **no TDMA and no shared schedule**. There are three
independent mechanisms, each on one side, each reacting to what it can
observe locally.

### 3.1 Drone: keep the burst shorter than the period (open loop)

`RcAgent::run_bitrate_policy()` commands the encoder at

    kbps = phy_rate(op) × airtime_budget / [ f0·(1+ov_base) + (1−f0)·(1+ov_enh) ]

with `encoder.airtime_budget` 0.6, `f0` 0.60 fixed, clamped to
`bitrate_min/max_kbps` (1000/16000). The intent is that video occupies
≤ 60 % of each 16.7 ms period, leaving ~6.7 ms of idle per AU for the
GS to talk in. It is **open loop**: nothing measures the burst and
feeds it back. Measured duty runs above budget — bench `link.air_pct`
66 % at a commanded 15.6 Mb/s, and the burst span (first body → last
body) is 12.7 ms mean of 16.7, i.e. ~76 % including preambles and
inter-aggregate gaps — because of framing excess (~5 %, deliberately
uncompensated since 2026-09-01, `docs/airtime-model.md` §1), the
encoder overshooting CBR by up to 40 % on scene bursts
(`handover-venc-overshoot-2026-09-03.md`), and the probe's 1 %.

Two backstops when the burst does not fit the period:

- **TxQueue** (`drone/src/tx_queue.h`, cap 256 bodies ≈ 150 ms):
  drop-oldest, so an overrun becomes bounded latency plus
  FEC-recoverable erasures rather than an ever-growing backlog. The GS
  cannot tell a queue drop from an RF loss, which is why
- **congestion shed** (`docs/link-adaptation.md` "Drone congestion
  shed"): at half the queue cap the drone sheds the enh layer before it
  is queued, which the GS scores as silence, not loss.

Neither is a timing mechanism; they bound the damage when the open-loop
budget is wrong. **The drone never gates its own TX start against the
GS** — with CCA off it will key up into an RCF that is on air, which is
the aligned-collision mode that made GS-only CCA *worse* than nothing
(`rcf-uplink-loss` §6).

### 3.2 GS: slot the uplink into the drone's idle (`RcfSlotter`)

`gs/src/rcf_slot.h`, `docs/link-adaptation.md` "RCF slotting". Every
control send is held until the GS believes the drone is idle:

- the **next burst is predicted** from the first-body cadence (EMA of
  first-body intervals; flat to < 1 ms, unlike completions);
- a **base-AU completion** releases at once if `now + lead_ms(3) <
  next_first − guard_ms(1)`;
- an **ENH completion releases nothing** — the probe body's arrival
  does, because the probe is the last PPDU of the burst (2026-09-05);
  a lost probe falls back to a learned deadline (`tail_ub_ms`, 4–6 ms
  on the bench);
- a frame offered within `grace_ms` 2 after a burst end goes out at once;
- a hold older than `rcf_slot_hold_ms` 30 is released **regardless**
  (timeout — a random-phase blast).

What it achieves (bench, mcs5 park, `rcf-slot` A/B): video PPDU loss
1.03 → 0.11 events/s, RCF delivery at the drone 51–69 % → ~93 %, at the
cost of up to one AU period of control latency (`close_ms` 5 → 22 ms).

What it cannot do: manufacture idle. It needs `idle ≥ lead + guard`
= 4 ms per period. At the bench's 76 % duty that margin is ~0, and
**22–43 % of sends end as timeouts** (`probe-blanking-fix-findings` §2).
The real send latency is 1–1.5 ms, so `lead_ms` 3 is ~2× conservative;
retuning it is the cheapest lever and is not done yet.

### 3.3 Drone: hear the RCF at all

The drone's RX thread runs continuously (`dev_cfg.rx.enable_with_tx`),
but physically it hears nothing during its own bursts. Three things
mitigate the resulting 30–50 % uplink loss:

1. the GS slotter (3.2) — sends now land in the drone's gaps, 93 %
   delivery on the bench;
2. the **repeat burst** — an op-changing RCF goes out 3× 10 ms apart,
   so a lost commit costs 10 ms, not a full `feedback_ms`;
3. **failsafe timing** — `link.failsafe_ms` 3000 without any RCF drops
   the drone to the max-range op, `rendezvous_ms` 30000 to
   re-rendezvous. These are the only drone-side reactions to not hearing
   the GS.

Both-sides CCA was measured to add +15–22 delivery points on a clean
channel but re-exposes the downlink to co-channel deferral; it is
**not shipped** pending a congested-channel A/B (`rcf-uplink-loss` §6).

## 4. How the variables interact

Per AU period `T = 1/fps_AU` (16.7 ms at 60 AU/s):

    burst  = bodies × serialization(MCS) + (n_agg − 1) × inter-agg gap
           ≈ AU_bytes × (1 + ov) × (1 + framing) / phy_rate + ~0.5 ms × n_agg
    idle   = T − burst − probe_body(MCS)
    needs: idle ≥ lead_ms + guard_ms  (GS can slot)   and
           idle ≥ ~0.5 ms              (drone can hear a ~180 µs RCF at all)

| variable | moves | effect on the idle |
|---|---|---|
| fps | `T` | 30 fps enh-only would double `T`; SVC-T alternation halves the per-AU period but halves each AU |
| bitrate | AU_bytes | linear; open-loop budget targets 60 % but measured 66–76 % |
| FEC overhead pair | ×(1+ov) per stream | base 1.0 / enh 0.5 → base bursts carry 2× payload air, enh 1.5× |
| MCS | serialization | the ladder's whole point: each rung re-targets bitrate so `burst/T` stays ≈ budget; the *idle in ms* is therefore roughly rung-independent by design |
| probe | +1 body/ENH AU | 0.3 ms at mcs4, 2 ms at mcs0 — always at the burst end, now the slotter's release signal |
| `feedback_ms` | sends/s | more sends = more blasts; **50 = 3 × T phase-locks timeouts to the burst** |
| repeat burst | sends during rung changes | three sends 10 ms apart inside one period: at most one of them can be slotted |

The design supports the variables **in the sense that the idle is
re-targeted per rung** (bitrate policy) and the GS **re-learns the
cadence continuously** (first-body EMA, probe-arrival release, learned
`tail_ub`). It does not support them in the sense of a guarantee: when
the encoder overshoots, or a rung's budget is mis-set, the idle simply
disappears and the slotter degrades to timeouts (random-phase blasts,
video and probe loss both rise), while the drone loses the RCFs that
would have demoted it. That is a positive-feedback corner and the
congestion shed is what currently breaks it.

## 5. Known gaps, ranked by measured cost

1. **Timeouts at high duty** — 22–43 % of sends on the bench, each a
   random-phase blast; phase-locked to the burst at `feedback_ms` 50.
   Levers: `lead_ms` 3 → ~2 (measured latency + margin), `rcf_slot_hold_ms`
   30 → 50, and/or a `feedback_ms` that is not a small multiple of `T`.
   Not done. (`probe-blanking-fix-findings` §6.)
2. **Open-loop airtime budget** — measured 66–76 % against a 60 %
   target. A closed loop exists in pieces (`link.air_pct` on the GS,
   `au_tail` span gauge) but nothing feeds it back into
   `airtime_budget`. Lever: measure `burst/T` on the drone (it knows
   every body's push→pop time) and trim the command; or lower the
   config budget. Cost: bitrate.
3. **Drone TX start is unconditional** — it never waits for a GS send
   to finish. Both-sides CCA would fix exactly this and measured no
   video cost on a clean channel, but is unproven congested.
4. **MSP/telemetry unslotted on the drone** — 6–9 PPDUs/s outside the
   AU cadence. Measured not to correlate with losses (self-blanking
   evidence list), so cosmetic today; they do consume the shared
   hardware seq counter.
5. **No clock sync** — the chip TSF is read (rx_pace `tsfl_d`) but never
   used for scheduling. A TSF-anchored slot (drone publishes its burst
   phase in T_TELEM; GS sends at phase + offset) would replace the
   cadence predictor with a measurement and make mid-gap aiming
   possible. Nothing built; the predictor is within 1 ms already.

## 6. How to check the timing on a live pair

- `link.rcf_slot.{au,probe,timeout,passthru,tail_ub_ms}` on the sideport
  (grace sends count under `au`) — the timeout share is the single best
  health number for "is there idle".
- `link.air_pct` vs `encoder.airtime_budget` — the open-loop error.
- `drone.rcf.rx_pps` vs `cards[].tx_pps` — uplink delivery; ≥ 90 % means
  sends are landing in gaps.
- `MABUR_GAPLOG=1` on the GS + `tools/bench/probesend.py` — every send
  placed against every burst end; `tools/bench/ausniff.py` for the video
  cost. `flightreport.py probe-NNNN.log` gives the completion→probe
  offsets, i.e. how late the burst really ends.
