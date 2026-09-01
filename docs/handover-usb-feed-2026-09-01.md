# Handover — the USB feed is the fec lever now

> **⚠ SUPERSEDED (2026-09-01, sessions 2–3): read findings §14–§17
> instead of this page.** The investigation closed the question this
> handover posed: the USB feed is NOT the fec lever (no wall to 2600
> bodies/s, §15), and neither is any other single lever — §17's unified
> model is `fec ≈ max(production spread, air spread) + ~2.2 ms GS tail`,
> with the ~360 µs/body invariant measured ON THE AIR (rx_pace tsfl) and
> the anatomy split by the au_tail gauge. The pinned-rung retest showed
> A-MPDU is feed-limited (2-deep, nets zero) and only pays combined with
> feed bunching (~−1.6 ms). Ranked levers in §17; #1 is 656-symbol
> bodies (attacks every term, hole-sweep gated). Probes 3–5 below are
> retired.

2026-09-01, bench (drone `192.168.10.152`, GS `10.18.0.1`), branch
`ampdu`. Continues `docs/handover-fec-latency-2026-08-31.md` and
`docs/dq-spike-findings-2026-08-31.md` §11–§12 (the lab notebook — read
§12 for the measurements; this page is the map for the next session).

## One-paragraph state

The A-MPDU lever is **dead** (§12): aggregates form on the 8822E but a
depth sweep (off/6/31) left fec p50 flat at ~16 ms, and aggregate PHY
reports poison the GS RF telemetry, so aggregation is off by default.
Elimination across three sessions now points one place: **the ~151 µs
fixed per-body cost is host/USB-side**, and cutting it is the only
remaining way to shrink `fec` (bodies serializing to the GS) short of a
slice-granular redesign. Nothing about the USB path has been touched yet;
this is a fresh front with instruments already in the tree.

## The evidence stack (why USB, not air)

- **Fixed 151 µs/body across a 4× MCS swing** (§9) — per-frame cost,
  rate-independent, so not duty or serialization.
- **tx_threads 1/2/4/8 flat** (§8) — parallelism above the USB layer does
  not help; something serializes below the thread pool.
- **EDCA/CW null** (§10) and **A-MPDU null** (§12) — both air-side
  medium-access levers measured as no-ops. maburd also runs
  `disable_cca`, so the original "DIFS+backoff" attribution never fit.
- **The ceiling tracks the feed, not the air**: maburd tops out ~1826
  bodies/s (≈20.4 Mb/s effective at mcs5) while txdemo pushed 2346
  singles/s through the same chip at the same rate with a different URB
  pattern.
- **Numerology**: the documented 8822E bulk-OUT acceptance handshake
  (~0.4 ms/URB, linkbench bisect 2026-07-14) ÷ 3-frame URBs ≈ 133 µs/body
  ≈ the fixed 151 µs.

## ⚠ Contradiction to resolve FIRST

Two prior findings disagree about the TX submission model:

- devourer `docs/aggregation.md`: HalMAC TX is **synchronous bulk** —
  "each sender thread blocks on its own transfer".
- dq-spike §8: `tx.send_bodies` returns in **~20 µs** at every
  tx_threads setting incl. 1 — "async libusb, UsbTransport.cpp".

Both cannot be right as stated. Read `../devourer/src/UsbTransport.cpp`
(and how `send_packets` drives it) and pin down where a URB actually
waits: libusb submit? completion callback? an internal devourer queue?
The answer determines which probe below matters.

## Where the code is

- maburd feed: `drone/src/usb_tx_pool.h` (UsbTxPool, tx_threads
  workers), `DevourerSink` + pool wiring `drone/src/main.cpp:763-781`,
  `RadioTx::send_bodies` (3-frame batches via `send_many`).
- devourer TX: `send_packets` (`IRtlDevice`), URB packing rules
  `../devourer/src/TxAggPlan.h` (3-descriptor HalMAC cap **per bulk
  transfer** — NOT a cap on transfers in flight), USB layer
  `../devourer/src/UsbTransport.cpp`, jaguar3 descriptor build
  `../devourer/src/jaguar3/RtlJaguar3Device.cpp` (`build_tx_block`).
- Existing gauges: `tx_send` (wall time + body count in the tx thread,
  commit 407d9ee), dq_split sideport line, `tools/bench/`:
  fecdump flow described in handover-fec-latency §"Bench + tree state".

## Probe sequence (cheapest → most committal)

1. **Resolve the sync/async contradiction** (read-only, ~30 min). Trace
   one body from `UsbTxPool` worker → `send_packets` → libusb. Name the
   blocking point.
2. **Instrument URB acceptance** (small drone-side gauge, bench A/B).
   Timestamp around each `send_packets` call per worker; export
   count/wall-time (extend the existing `tx_send` gauge). Questions:
   does acceptance really cost ~0.4 ms per URB? Do 4 workers' URBs
   pipeline or serialize? Distribution, not just mean — §9's fixed cost
   should reappear here if the attribution is right.
3. **Feed-shape sweep on the bench** (config/env only if possible).
   txdemo reached 2346/s; find what shape does it: 1-frame URBs vs
   3-frame, more in-flight URBs, submit cadence. If a shape change alone
   raises the ~1826 ceiling, the lever is confirmed and cheap.
4. **Async/multi-URB rework** (devourer project, only if 2-3 confirm).
   If acceptance serializes in devourer's sync bulk path, move to
   overlapped async bulk-OUT (multiple URBs in flight per endpoint).
   Note jaguar1's async path exists but "does not parallelize" per
   aggregation.md — understand why before copying it.
5. **Retest A-MPDU after the feed is fixed** (config-only:
   `ampdu.max_num > 0`). If the chip-side queue was starved (shallow
   aggregates), a fixed feed may finally let aggregation engage — but
   the RF-report poisoning (§12 addendum) must be solved before flying
   with it (third-adapter depth measurement, or accept sparse RF).

## Success criterion + gates

Raise the saturated drain ceiling above ~1826 bodies/s / cut the fixed
per-body gap, measured as: fecdump drain slope (fec p50 at a fixed
bitrate/scene), `tools/bench/ausniff.py` (standing gate), aucadence,
RCF close_ms. At the 11 M cap the link is unsaturated — for ceiling
measurements either raise the cap temporarily or measure slope on
per-AU bursts.

## Bench state (verified 2026-09-01, end of ampdu session)

- Both ends run the `ampdu` builds (drone `maburd` + GS `maburgs`,
  rollbacks `*.pre-ampdu`). Aggregation OFF (`ampdu{max_num:0}` in
  `/etc/mabur.json`, also the compiled default since 0a682e1).
- Drone config: ch136, `bitrate_max_kbps` **11000** (was 14000 — the
  14 M cap saturated the ceiling and caused the operator-visible ~100 ms
  `air` queue; see §12 finding 3). Rollback of the cap is a one-line
  edit; the underlying `airtime_budget` non-enforcement is still open.
- GS binaries require devourer branch **`jgr3-physt`** (../devourer is
  checked out on it; f18bf1b off f3b76ea) — upstreaming pending.
- Healthy at cut: ausniff 59.5 fps / 0 gaps, s0 RSSI −52.0 sd 0.25 dB,
  SNR 32.4 sd 0.6, air p50 1.1 ms, ladder parked mcs5.
- Scratchpad datasets (this session): `fec_pre_ampdu.csv`, `fec_post2.csv`,
  `fec_a0/a31/b11.csv`, `sideport_pre/post/post2.jsonl`, spike logs
  `ampdu_e_logs/` — the §12 numbers derive from these.
