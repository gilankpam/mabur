# TX/RX timing: who transmits when, and what actually coordinates it

The holistic view of mabur's half-duplex radio timing:
- who keys up when;
- the three local mechanisms that keep the uplink and the downlink apart;
- how the variables interact;
- where the gaps are.

Each claim points at the code or at the bench record that measured it.

## 1. The physics that everything else follows from

- **Every card is half-duplex.** A card cannot receive while it transmits.
  The drone has one card. The GS has one or more receiving video, and one
  of them (the `TxSelector`'s pick, `gs/src/tx_selector.h`) also sends the
  uplink.
- **Carrier sense is ON on both daemons.**
  `dev_cfg.tuning.disable_cca = false` in `drone/src/main.cpp` and
  `gs/src/radio_frontend.cpp`. Each radio defers to any decodable 802.11
  preamble (the other end's and anyone else's), then waits AIFS + backoff.
  The two ends only ever flip CCA together: CCA on the GS alone keys the
  drone up into an RCF already on air, which is worse than neither.
  Record: `docs/findings/2026-09-23-cca-on.md` (mechanism of the CCA-off
  era: `docs/findings/2026-09-02-gs-uplink-self-blanking.md`).
- **A GS send blinds the GS cards** for about 180 µs. The transmitting card
  is deaf by definition, and a sibling card is saturated at about −4 dBm.
  A drone PPDU whose preamble starts inside that window is lost on every
  card, whole (one A-MPDU aggregate). The chip holds a TX until the PPDU it
  is receiving ends, so the damage is to the NEXT PPDU starting right
  behind the blast.
- **A drone burst deafens the drone** for its whole duration. An RCF that
  lands inside a burst is gone, so delivery to the drone tracks
  `1 − air duty` (`docs/findings/2026-08-14-rcf-uplink-loss.md` §3).
- **The two clocks free-run.** The drone's sensor/encoder cadence and the
  GS's `feedback_ms` timer are independent oscillators. Nothing in mabur
  syncs them: there is no TSF exchange and no beacon-derived schedule.

On one channel there are two talkers. The only arbitration is carrier
sense plus "finish receiving the current PPDU before keying up". Everything
else mabur does about timing is software on top of that.

## 2. Who transmits what

Drone → GS (the downlink, nearly all of the airtime):

| what | cadence | size / air | shaped by |
|---|---|---|---|
| video bodies, base (sid 0) + enh (sid 1) | one AU per frame; 60 AU/s at 60 fps SVC-T, alternating base/enh; each AU is one burst | 1405 B bodies (332/bpb 4), aggregated by `[ampdu]` (`max_num` 6, from `min_mcs_20`/`min_mcs_40`) | encoder bitrate (`run_bitrate_policy`), the per-rung FEC overhead pair, MCS, `fec.feed_batch` (bundle 1) |
| probe stream (sid 5) | one body after every video AU while a probe is commanded, i.e. the AU rate | one 1405 B body, its own PPDU right behind the burst's last aggregate | MCS of rung `current + link.probe.rung_offset` (`docs/airtime-model.md` §6) |
| NACK retransmits (sid 0 symbols) | on a verified `T_NACK` only | pushed at the TxQueue HEAD (`TxQueue::push_front`) | `[nack] air_pct`/`burst_ms` token bucket (`docs/fec-nack.md`) |
| MSP OSD (sid 4) | `msp.update_rate_hz` 3 | `msp.symbol_size` 1312 B units | not slotted |
| T_TELEM | 1 Hz | 54 B payload (`TELEM_LEN`) | not slotted |
| T_MAV (MAVLink tunnel, untagged) | ≤ 1 per 50 ms | ≤ 600 B payload (`kMavMaxPayload`) | not slotted |
| T_MAV calibration replies (ack, sweep done, applied) | `maburcal` runs only: one per command, a sweep-done every 200 ms until the next command | one MAVLink packet each | not slotted, unpaced, from the TX writer thread |
| DISC_ACK | rendezvous only | small | not slotted |

GS → drone (the uplink: a low duty cycle, but every send is a blast):

| what | cadence | path |
|---|---|---|
| RCF (rate-control feedback) | every `link.feedback_ms`: code default 100, bundle 50 | slotted (§3.2); about 180 µs on air incl. preamble |
| DISC keepalive | `beacon_keepalive_ms` 1000 | slotted like any RCF |
| T_NACK (base selective repeat) | at most one per poll, when `link.nack.enable` (bundle: on) | **not slotted**: `send_control_frame` directly, mid-burst (`docs/fec-nack.md` "Packing and send") |
| T_MAV (MAVLink tunnel, tagged) | on demand | slotted like RCF |
| T_MAV calibration lane (sweep, apply, abort) | `maburcal` runs only: every 200 ms until answered, three aborts at the end | **not slotted**, not behind the radio-silence gate: the session offers a message only between sweeps, when the air is the GS's (`docs/mavlink.md` "The cal lane") |

Every uplink send is gated by `ChannelCore::may_send`, and every one but
the calibration lane by the calibration radio-silence gate.

## 3. What mabur does orchestrate

There is **no TDMA and no shared schedule**. There are three independent
mechanisms, each on one side, each reacting to what it can observe
locally.

### 3.1 Drone: keep the burst shorter than the period (open loop)

`RcAgent::run_bitrate_policy()` commands the encoder at

    kbps = 1000 × airtime_budget / [ 0.60·(1+ov_base)/rate_b + 0.40·(1+ov_enh)/rate_e ]

The terms:
- `rate_*` is each layer's DELIVERED rate: nominal PHY rate ×
  `air_clock.efficiency_<bw>[mcs]` (`drone/src/air_rate.h`).
- The result is clamped to `encoder.bitrate_min/max_kbps`.
- Bundle values are `encoder.airtime_budget` 0.65 and 1000–24000 kbps.

Full description: `docs/link-adaptation.md` "RcAgent owns the encoder".

The intent is that FEC-protected video occupies about `airtime_budget` of
what the link can really move in each AU period. The rest is idle in which
the GS can talk.

It is **open loop**: nothing measures the burst and feeds it back. Duty
can exceed the budget because:
- framing excess is deliberately uncompensated (`docs/airtime-model.md`
  §1);
- the encoder overshoots CBR on scene bursts
  (`docs/findings/2026-09-03-handover-venc-overshoot.md`);
- the probe body is not a term in the formula.

Three backstops when the burst does not fit the period. None of them is a
timing mechanism; they bound the damage when the open-loop budget is wrong.

- **TxQueue** (`drone/src/tx_queue.h`, `kTxQueueCap` 256 bodies, about
  150 ms). It drops the oldest body on overflow, so an overrun becomes
  bounded latency plus FEC-recoverable erasures rather than an
  ever-growing backlog. The GS cannot tell a queue drop from an RF loss.
- **Congestion shed.** At half the queue cap, the drone sheds the enh layer
  before it is queued, which the GS scores as silence, not loss
  (`docs/link-adaptation.md` "Drone congestion shed").
- **Air clock.** This sheds enh AUs when the modelled air backlog reaches
  `air_clock.shed_ms` (bundle 25) (`docs/link-adaptation.md` "Drone air
  clock").

With carrier sense on, the drone's MAC also defers its TX start to an RCF
that is already on air.

### 3.2 GS: slot the uplink into the drone's idle (`RcfSlotter`)

`gs/src/rcf_slot.h`, wired in `gs/src/main.cpp` and `web/src/web_gs.cpp`.
Every RCF and DISC keepalive is offered to the slotter and held until the
GS believes the drone is between bursts:

- **Predicting the next burst.** The slotter predicts the next burst from
  the first-body cadence: an EMA of first-body intervals
  (`on_au_first`). First-body arrival is flat to under 1 ms, whereas
  completion moves with frame size and FEC repair.
- **Release at AU completion.** A completion (`on_au_complete`) releases
  the hold at once if `now + lead_ms(3) < next_first − guard_ms(1)`. A
  completion too close to the next burst is skipped, and the frame waits
  for the next one.
- **Probe arrival is the release while a probe is commanded.** Every video
  AU is then trailed by a probe body, the last PPDU of the burst, which
  lands up to a few ms after the completion stamp. A release at the
  completion would put the blast exactly on the probe. So a completion
  releases nothing by itself. The probe sink's `on_probe_tail` is the
  release (`SlotReason::Probe`), with the idle-ahead check made at that
  instant.
- **Lost-probe fallback.** If the probe is lost, the release falls back to
  a deadline at completion + `tail_ub_ms`. That value is a decaying max of
  observed completion→probe offsets, floored at the probe body's airtime,
  plus 1 ms.
- **Grace.** A frame offered within `grace_ms` (2) after the burst end goes
  out at once.
- **Timeout.** A hold older than `link.rcf_slot_hold_ms` (default and
  bundle 30; 0 = slotter off) is released **regardless**. That is a
  random-phase blast.
- **No video.** With no AU in the last 100 ms (`video_recent_ms`:
  rendezvous, stalled video), everything passes through. DISC therefore
  needs no bypass.
- **Order.** Frames offered while others are held queue behind them, so
  every slotted sender keeps its order: a grace, no-video or pass-through
  offer that finds the queue non-empty is held and released with it.

`lead_ms`, `guard_ms`, `grace_ms` and `video_recent_ms` are compiled in
(`RcfSlotCfg{hold, 100, 2, 3, 1}`). Only the hold is config.

**Cost.** A commanded op change reaches the drone up to one AU period
later (`link.attrib.close_ms`).

**What it cannot do: manufacture idle.** It needs
`idle ≥ lead + guard` = 4 ms per period. When the duty leaves less than
that, sends end as timeouts. The real send latency is 1–1.5 ms, so
`lead_ms` 3 is about 2× conservative.

Sideport: `link.rcf_slot.{au,probe,timeout,passthru,tail_ub_ms}`. Grace
sends count under `au`.

Bench records:
- `docs/findings/2026-09-02-gs-uplink-self-blanking.md` (the slotter A/B)
- `docs/findings/2026-09-04-handover-probe-blanking.md`
- `docs/findings/2026-09-05-probe-blanking-fix.md` (probe-arrival release,
  timeout share)

### 3.3 Drone: hear the RCF at all

The drone's RX thread runs continuously (`dev_cfg.rx.enable_with_tx`), but
physically it hears nothing during its own bursts. What mitigates the
resulting uplink loss:

1. **The GS slotter (3.2).** Sends land in the drone's gaps.
2. **Carrier sense on both ends.** Neither end keys up into the other's
   PPDU.
3. **Failsafe timing.** After `link.failsafe_ms` with no RCF (code default
   1000, bundle 3000), the drone drops to the max-range op. After
   `link.rendezvous_ms` (30000) it re-rendezvouses. These are the only
   drone-side reactions to not hearing the GS.

An op-changing RCF is sent once. A lost one costs one `feedback_ms`
(`docs/findings/2026-09-05-switch-loss.md`). The co-channel deferral that
carrier sense exposes the downlink to is measured in flight by the drone's
`drone.radio.rx` (`own`/`foreign`/`crcfail`) telemetry.

## 4. How the variables interact

Per AU period, `T = 1/fps_AU` (16.7 ms at 60 AU/s):

    burst  = bodies × serialization(MCS) + (n_agg − 1) × inter-agg gap
           ≈ AU_bytes × (1 + ov) × (1 + framing) / phy_rate + ~0.5 ms × n_agg
    idle   = T − burst − probe_body(MCS)
    needs: idle ≥ lead_ms + guard_ms  (GS can slot)   and
           idle ≥ ~0.5 ms              (drone can hear a ~180 µs RCF at all)

| variable | moves | effect on the idle |
|---|---|---|
| fps | `T` | SVC-T alternation halves the per-AU period but also halves each AU |
| bitrate | AU_bytes | linear; the open-loop budget targets `airtime_budget` of delivered capacity |
| FEC overhead pair | ×(1+ov) per stream | bundle base 0.5 / enh 0.25 → base bursts carry 1.5× payload air, enh 1.25× |
| MCS / width | serialization | the ladder's whole point: each rung re-targets bitrate so `burst/T` stays near the budget, so the *idle in ms* is roughly rung-independent by design |
| probe | +1 body per AU | always at the burst end; it is the slotter's release signal |
| `feedback_ms` | sends/s | more sends = more blasts; a `feedback_ms` that is a small multiple of `T` (50 = 3 × T) phase-locks timeouts to the burst |
| NACK | +1 unslotted send per poll, + retransmits at the TxQueue head | each T_NACK is a blast at a random phase; retransmits lengthen the burst |

The design supports these variables **in the sense that the idle is
re-targeted per rung** (the bitrate policy). The GS also **re-learns the
cadence continuously**: the first-body EMA, the probe-arrival release and
the learned `tail_ub`.

It does not support them in the sense of a guarantee. When the encoder
overshoots, or a rung's budget is mis-set, the idle disappears. The
slotter then degrades to timeouts (random-phase blasts, so video and probe
loss both rise), while the drone loses the RCFs that would have demoted
it. That is a positive-feedback corner. The congestion shed and the air
clock are what break it.

## 5. Known gaps

1. **Timeouts at high duty.** Each timeout is a random-phase blast, and at
   `feedback_ms` 50 they are phase-locked to the burst. The levers are
   still unpulled:
   - `lead_ms` 3 → about 2 (measured latency + margin);
   - `rcf_slot_hold_ms` 30 → 50;
   - a `feedback_ms` that is not a small multiple of `T`.

   Record: `docs/findings/2026-09-05-probe-blanking-fix.md` §6.
2. **Open-loop airtime budget.** Nothing feeds measured duty back into
   `airtime_budget`. A closed loop exists only in pieces: the drone's air
   clock (`au.log` `air_ms`), GS `link.air_pct`, and the per-AU arrival
   span in `au.log` (`t_last − t_first`). One lever is to measure `burst/T` on the drone, which knows
   every body's push→pop time, and trim the command. The other is to lower
   the config budget, at a bitrate cost.
3. **The NACK uplink is unslotted.** T_NACK goes out mid-burst by design,
   because a slotted NACK arrived too late to help
   (`docs/findings/2026-10-05-fec-nack-spike.md`). Each one is a blast
   that can cost a downlink aggregate on every GS card. With
   `link.nack.enable` on, its rate adds directly to the GS-inflicted loss.
4. **MSP/telemetry unslotted on the drone.** Their PPDUs fall outside the
   AU cadence. They were measured not to correlate with losses
   (self-blanking evidence list), so this is cosmetic today.
5. **No clock sync.** The chip TSF is read (`rx_pace` `tsfl_d`, gap log),
   but never used for scheduling. A TSF-anchored slot would replace the
   cadence predictor with a measurement and make mid-gap aiming possible:
   the drone publishes its burst phase in T_TELEM, and the GS sends at
   phase + offset. Nothing is built.

## 6. How to check the timing on a live pair

- **Is there idle?** `link.rcf_slot.{au,probe,timeout,passthru,tail_ub_ms}`
  on the sideport (grace sends count under `au`). The timeout share is the
  single best health number for this.
- **Open-loop error.** Compare `link.air_pct` against
  `encoder.airtime_budget × air_clock.efficiency_<bw>[mcs]`, not against
  the budget alone. `link.air_pct` is injected rate over NOMINAL PHY rate
  for streams 0/1 (`gs/src/stats_exporter.cpp`), while the budget is a
  fraction of DELIVERED rate. `link.air_pct` does not include the probe.
- **Uplink delivery.** Compare `drone.rcf.rx_pps` against the TX card's
  `cards[].tx_pps`. At or above 90 %, sends are landing in gaps.
- **Every send against every burst.** Run the GS with `MABUR_GAPLOG=1`
  plus `tools/bench/probesend.py`; that places each send against each
  burst end. Use `tools/bench/ausniff.py` for the video cost.
- **How late the burst really ends.** `flightreport.py <session-dir>` (or
  its `probe.log`) gives the completion→probe offsets.
