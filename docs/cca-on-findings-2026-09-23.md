# Carrier sense back ON, both ends — 2026-09-23

Branch `cca-on` (off master `7fe5a30`). `dev_cfg.tuning.disable_cca`
flipped `true → false` in `drone/src/main.cpp` and
`gs/src/radio_frontend.cpp`, so devourer leaves the 8822E's MAC
carrier-sense gate (0x520[14] primary CCA, [15] EDCCA) at the chip
default. Plus `RC_VERSION` 9 → 10: `Telem` gains the drone's RX-side
channel view (`rx_own`, `rx_foreign`, `rx_crcfail`, per telemetry
period) so a flight records what the drone's transmitter actually defers
to at altitude — the one number the ground cards cannot measure.

## Why

CCA was switched off on both ends on 2026-08-05 on the argument that the
downlink owns its channel and CSMA only stutters it (devourer measured
injection deferring 41–45 % to a co-channel 802.11 transmitter with its
`txdemo` injector). Three things on the 2026-09-23 bench overturned that:

1. **With both ends blind, the GS's own uplink send kills a drone PPDU on
   BOTH ground cards** ~1.5 times a second (two bodies, no CRC-failed
   frame — the sibling's receiver is saturated by the −4 dBm neighbour).
   Master's AU-completion slotter happened to land those on the parity
   tail (0.3 repairs/s), the `tx-windows` branch landed them on the next
   AU's source bodies (5–7 repairs/s per stream, +2 ms fec p99). Either
   way the loss is real and only avoided by luck of placement.
2. **Carrier sense on both ends removes it outright.** Same bench, same
   130 s `MABUR_GAPLOG=1` battery, master + the flip:

   | metric (rung 5, singles, scout ON, 1080p60) | master CCA off | master CCA on |
   |---|---|---|
   | both-card holes within 4 ms after a GS send (97 s) | 137 (1.42/s) | **23 (0.24/s)** |
   | … inside an AU source span | 11 | **0** |
   | fec.log loss episodes | 12 | **0** |
   | `streams[*].recovered` per s | 0.23 / 0.33 | **0.00 / 0.00** |
   | `video.lat.fec` p50 / p99 ms | 9.6 / 13.2 | **9.9 / 12.7** |
   | `video.lat.air` p99 ms | 5.9 | **4.1** |
   | `video.jitter_ms` | 3.3 | 3.3 |
   | drone `rcf.rx_pps` (GS sends 20.8/s) | 18.1 | **19.35** (90.5 → 96.8 %) |
   | `link.air_pct` / `cmd_kbps` / fps / drops | 54 / 17100 / 60 / 0–1 | 54 / 17100 / 60 / 0 |
   | ausniff / aucadence | clean / +1.10 ms | clean / −0.05 ms |

   No deferral cost was visible: same air, same commanded bitrate, same
   fps; the drone logged one drop and a transient `txq=25` over the arm.
   This reproduces the 2026-08-14 both-sides arm
   (`docs/rcf-uplink-loss-findings-2026-08-14.md` §6: +15–22 pts RCF
   delivery, no video cost) and adds the decoder-level proof.
3. **The deferral being avoided was never there.** All 99 usable sessions
   on the GS DVR (bench and flights, ch136/120) read `cards[].foreign_pps`
   p90 = 0 (max 5) and `cca − own` ≤ 14: the site is 802.11-quiet at
   ground level. The vendor kernel driver (`../rtl88x2eu-20230815`) never
   disables this gate for monitor injection, so a stock wfb-ng deployment
   has always run this way.

**GS-only carrier sense is not a fallback.** It measured *worse* than
blind on 2026-08-14 (rung-3 RCF delivery 51 → 23 %): the GS defers to
the burst edge, which is exactly where a blind drone restarts. The two
ends move together.

## The measurement that found it: branch vs master, A/B/A

Same bench, same afternoon (2026-09-23), rung 5 adaptive, singles, scout
ON, 1080p60; sequence tx-windows (A1) → master `7fe5a30` (B) → tx-windows
(A2), each arm a 130 s `MABUR_GAPLOG=1` run analysed with the tx-windows
branch's `gapan.py` plus the sideport session split at the restarts.

| metric (rung 5 steady) | tx-windows A1 | master B | tx-windows A2 |
|---|---|---|---|
| `streams[*].recovered` per s (FEC repairs, s0 / s1) | 5.5 / 6.0 | 0.23 / 0.33 | 4.7 / 7.2 |
| fec.log episodes, 96.6 s window | 149 | 12 | 153 |
| `video.lat.fec` p50 / p99 ms | 10.2 / 15.4 | 9.6 / 13.2 | 10.0 / 15.5 |
| `video.jitter_ms` | 4.44 | 3.31 | 4.91 |
| both-card holes within 4 ms after a GS send (card-0 view) | 151 | 137 | 153 |
| … inside an AU source span | 127 | 11 | 131 |
| GS sends inside an AU source span | 1445 / 2006 | 27 / 2005 | 1502 / 2003 |
| send − t_complete of the current AU, p50 | −8.9 ms | +0.97 ms | −8.8 ms |
| `video.fps` / drops per run | 60 / 1 | 60 / 0–1 | 60 / 1 |

Both builds lose one drone PPDU (two ~1400 B bodies, no CRC-failed frame
on either card) about 1.5 times a second to the GS's own send. Master's
AU-completion slotter released the RCF ~1 ms after the last source symbol
of an AU, so its holes fell in the parity tail — free. The TX-window
release landed the send 8.8 ms BEFORE the current AU completed, i.e. on
the first source bodies of the next AU — eight source symbols per hit,
hence the 15–20x repairs and +2 ms fec p99. The gapan zero-loss contract
was met by neither; carrier sense on both ends (table above) is what
removed the collision instead of relocating it. Not comparable across
those two builds and deliberately absent from the table: `loss_pct`
(tx-windows excludes scout-deaf cards), `pre_fec_loss` / `arr_late`
(guard 64 vs 32), `crc_fail` (deaf suppresses it), gapan's
"send-attributed" (chance model differs with the gap count).

## What is still open

- Carrier sense reacts to a lower bar than the hop verdict: any decodable
  preamble, versus `hop.verdict.foreign_pps` 50/s. Below 50/s the drone
  defers but the link never hops. At 50 typical frames/s that is a bounded
  ~3–5 % of air, but it is not zero, and a drone at altitude sees more
  access points than a receiver on a table. That is what the new Telem
  fields exist to measure: fly it and read `flightreport.py`'s `DRONE RX`
  section (`foreign` and `crcfail` per period, sampled once per
  `tlm_seq`). Tens per second is nothing; hundreds is the number to act
  on (lower the verdict threshold, or revisit).
- Only 802.11 preambles trigger primary CCA; EDCCA stays at never-trigger
  (`adaptivity` off), so analog VTX, DJI and other non-WiFi energy never
  defer the drone. Another wfb-ng/mabur pilot on the same channel does —
  and with carrier sense off that case was a collision, not a deferral.
- `gapan --assert-zero` still exits 1 with carrier sense on: 25 both-card
  holes per 97 s remain, all in the parity tail or idle, plus the
  dwell-attributed rows master logs. None cost a repair.

## Wire and sideport

- `Telem` +6 bytes (`TELEM_LEN` 89 → 95): `rx_own`, `rx_foreign`,
  `rx_crcfail` = every frame the drone's monitor-mode RX callback saw in
  the period, split into CRC-clean RC from the GS, CRC-clean not-ours, and
  CRC-failed (preamble heard, payload undecodable). Saturating u16.
  **Software counts on purpose.** The first cut read the chip's OFDM
  CCA/FA registers (`GetRxEnergyScout`) once a second; a register read is
  a control-plane transfer and must take the TX gate exclusive like a
  retune, and that stalled the USB TX pool: 795 TxQueue drops in 20 min,
  air p99 4 → 9 ms, one gap-log run with 23 video drops. The RX-path
  counts cost nothing and answer the same question — with EDCCA at
  never-trigger, the MAC only defers to preambles, and every preamble
  that reaches the callback is counted here (only a PLCP that fails
  outright is missed).
- Sideport `drone.radio.rx = {own, foreign, crcfail}`; per period,
  repeated on every record until the next Telem. `maburtop` shows them
  on the drone `radio` row (`rx own N foreign N crc N`); `flightreport`
  prints `DRONE RX` (once per `tlm_seq`, silent on older recordings).
- Both bring-up lines now print the requested state either way
  (`maburd radio: MAC carrier sense (CCA+EDCCA) requested ON ...`,
  `maburgs radio card N: ... requested ON ...`).
- Deploy: `docs/deploy.md` "2026-09-23 RC_VERSION 10". No config change
  on either end.

## Follow-up the same evening: the OSD's constant small loss

With carrier sense on the compact OSD still showed a steady ~1 % on its
LOSS row. That row is `link.pre_fec_loss`: the ArrivalTracker books a seq
missing once a later seq arrives 32 symbols ahead of it, and a symbol
heard after that counts `arr_late` and is never un-booked. On this bench
`arr_late` (39k/36k per stream) exceeded missing-at-line (18k/17k) while
FEC repaired 126/127 symbols in 1.6 M (0.008 %): every "lost" symbol was
heard late. The drone's USB TX pool (4 threads × 3-frame URBs, ~12
bodies = 48 symbols in flight) reorders bodies on air; the guard was one
FEC window. The tx-windows branch had already made it a config key
(`link.arrival_guard_syms`, cherry-picked plumbing `8a68cc8`); sweep at
rung 5, 70 s steady per arm, config-only restarts:

| guard | OSD pre-FEC loss | missing-at-line s0/s1 | `arr_late` per s | repairs per s |
|---|---|---|---|---|
| 32 (old compile-time) | ~1.1–1.4 % | 1.08 / 1.08 % | 73 / 73 | 0.07–0.19 |
| 64 | 0.67 % | 0.65 / 0.47 % | 47 / 27 | 0.26 / 0.00 |
| 96 | 0.23 % | 0.23 / 0.09 % | 16 / 5 | 0.17 / 0.23 |
| 128 | 0.09 % | 0.08 / 0.00 % | 4.7 / 0.3 | 0.11 / 0.00 |
| **192** | **0.02 %** | 0.02 / 0.01 % | 0.2 / 0.0 | 0.59 / 0.41 |

fps 60 and 0–2 drops in every arm. Shipped default 192. Cost: the
ladder's util input is booked ~60 ms later at rung 5 (~165 ms at rung 0),
inside the feedback period plus probation. The root cause — bodies
airing out of submission order — is drone-side and untouched; a single
in-order sender would remove the need for the guard, at the ~26 Mbps
single-URB throughput cap the pool exists to beat.

## The fix, as shipped on `cca-on`

Four commits off master `7fe5a30`, host suite 151/151 at each, tests
written first:

1. `e2ead1c` — carrier sense ON both ends (`disable_cca = false`,
   drone/src/main.cpp + gs/src/radio_frontend.cpp); `RC_VERSION` 10,
   `Telem` +`rx_own`/`rx_foreign`/`rx_crcfail`; sideport
   `drone.radio.rx`; maburtop radio row; flightreport `DRONE RX`.
2. `7ff6473` — cherry-pick of the tx-windows ArrivalTracker-guard
   plumbing (decoder constructors, `UepDecoder::arrival_guard`).
3. `1b82be6` — `link.arrival_guard_syms` config key, default 192 (the
   OSD's "constant ~1 % loss", section above).
4. `13c5eae` — `link.pre_fec_loss` pools base + enh; maburplay reads it
   first, `link.ctl.pre_fec_loss` (base-only, the ladder's input) only as
   the null-window fallback. OSD row shape unchanged: `loss:<pre>/<post>`.

**What the OSD's LOSS row means now.** Left: the share of source symbols
on EITHER video layer the GS had not heard on any card within 192
symbols of a later one — on the bench this tracks the decoder's repair
count (0.02 % vs 0.008–0.02 %), i.e. real pre-FEC erasures. Right:
`residual_loss`, what FEC could not rebuild — the number that costs
frames. Both are sliding windows, booked ~60 ms late at rung 5 (~165 ms
at rung 0). Not on the row: which layer an erasure hit (sideport
`streams[*].recovered` per stream says), and anything the ladder did
about it (`link.ctl.*`).

**Deploy state (2026-09-23 evening).** Drone: `maburd` = commit 1
build (later commits are GS-only), config `mabur.toml` = the master
shape (no `[tx_window]`), rollbacks `maburd.master` + `maburd.txwin` on
the rootfs, `maburd.pre-txwin` / `maburd.master-cca` in tmpfs only. GS:
`maburgs` + `maburplay` at the branch head, config `maburgs.toml` =
master shape + `arrival_guard_syms = 192`; rollbacks `maburgs.{master,
txwin,master-cca,guard}`, `maburgs.toml.{master,txwin,pre-guard}`,
`maburplay.pre-pooled`. The `docs/deploy.md` "2026-09-23 RC_VERSION 10"
entry is the procedure for any other device.

## Pending: flight validation

Nothing above has flown. The bench cannot measure the one cost the flip
re-exposes — deferral to 802.11 neighbours the drone hears at altitude —
so the flight is the gate for merging `cca-on` to master. Fly the usual
profile (climb through the rungs, a range leg, a hover at the far point)
with the DVR recording, then:

1. **Deferral exposure** — `tools/flightreport.py <session-dir>`, section
   `DRONE RX`: `foreign` and `crcfail` per telemetry period (once per
   `tlm_seq`). Tens per second at p90 is nothing. Hundreds means the drone
   was deferring to traffic the hop verdict ignores
   (`hop.verdict.foreign_pps` 50): lower that threshold or revisit the
   flip. Cross-read `drone.air_backlog_max_ms`, `txq_wait_ms` and
   `link.air_pct` on the same records — deferral shows as backlog at
   unchanged commanded bitrate before it shows as loss.
2. **The collision stays gone** — `streams[*].recovered` per second at
   the mcs5 park should sit near the bench's ~0.1–0.6, not the blind
   pair's 0.3 or the tx-windows 5–7; fec.log episodes per minute in
   single digits at park; `drone.rcf.rx_pps` ≥ 19 at `feedback_ms` 50.
3. **No TX-side cost** — drone `stats:` line `txq_drop=0` and `drops`
   flat over the flight (the register-read regression was 795 drops in
   20 min; the shipped build reads 0). `link.video.lat.fec` p99 at park
   within ~1 ms of the last blind flight's; the bench read 12.7–14.6 ms
   across arms with no build-attributable trend.
4. **Ladder behaviour under the 192 guard** — `flightreport` transitions
   and `U PER RUNG`: demotes should still fire on real loss (util reacts
   ~60 ms later than before at rung 5, ~165 ms at rung 0). A promote
   that bounces where the same site used to hold, or a demote that
   arrives a rung late, is the guard costing reaction time — 128 is the
   fallback value (0.09 % artefact) and it is config-only.
5. **OSD sanity** — the LOSS row should read ~0.0/0.0 in the near field
   and rise with `streams[*].recovered` at range, not sit at a constant
   1 % as it did before the guard.

Pass = 1 in the tens, 2–3 as stated, 4 no regression, 5 as expected →
merge `cca-on` to master (ff), retire the `tx-windows` branch (its deaf
accounting and `hop.fastretune_fw` remain candidates on their own
merits), and delete the `*.master` / `*.txwin` / `*.master-cca` rollback
binaries on both devices. Fail on 1 → the congested case is real at this
site's altitude: options are lowering `hop.verdict.foreign_pps`, a
drone-side EDCCA/primary-CCA split (keep the RCF-collision fix, ignore
weak neighbours), or the tx-windows design with the window moved to the
parity tail. Fail on 3 or 4 → roll the GS config to `arrival_guard_syms =
128` first, the drone binary to `maburd.master` + `mabur.toml.master`
second, as separate steps, so the two changes are not confounded.
