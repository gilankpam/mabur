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
