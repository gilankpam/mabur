# Source Truncation & the Drone Drain Ceiling — Findings + Future Work

2026-07-17 glitch-hunt findings, written for the next session that meets
"video glitching with clean link stats." Companion docs:
`docs/txagc-calibration.md` (power axis), `docs/handover-video-delivery.md`
(the original 2026-07-13/15 campaign this incident rhymed with).

## TL;DR

Video glitched at 100% link delivery because the GS op pin
(`mcs5/ov0.375` × the drone's 8000 kbps encoder clamp = **11 Mbps on air**)
exceeded this drone's **~9–10 Mbps air drain ceiling**: waybeam's venc ring
overwrote slice tails before maburd drained them. The result is
**seq-invisible**: transport metrics are perfect, but ~58% of FU chains
arrive born-truncated and the decoder glitches. Fixed by pinning an
in-budget op (`mcs7/ov0.10` = 8.8 Mbps air → 0 truncations, 59.6 fps at
full wall-parked power). **The offset-power stack was fully exonerated** by
A/B bisection — binaries, wall diffs, offset mode, per-op register writes,
and PA draw were each ruled out individually.

## The failure signature (recognize it fast next time)

| Layer | What it shows | Verdict it tempts you into |
|---|---|---|
| GS link stats (`cf`, `snr`, `mis`) | perfect | "link is fine" ✓ (true) |
| GS reorder (`ord[skip/late]`) | frozen | "stream is fine" ✗ (WRONG — ord cannot see source truncation) |
| `rtpsniff.py` default | 4.7% loss, 62% bad frames | "transport catastrophe" ✗ (WRONG — its capture buffer drops; over-counts) |
| `tools/bench/fu_probe.py` | 0.02% loss, **58% hard-truncated FU chains** | source truncation ✓ |

The one reliable measurement is `fu_probe.py` on the GS (dedup + unwrap +
8 MB capture buffer + marker-classified chain ends). `end_HARD > ~2%` with
`missing ≈ 0` = slices are being destroyed **on the drone before
transmission**. Complete-frame fps ≈ 60 × (1 − truncation rate) — a
"30 fps" stream from a 60 fps encoder is half the frames losing their tails,
not an fps configuration problem.

## Root cause mechanics

- waybeam encodes at the bitrate maburd commands (CBR, 60 fps) into the
  shm venc ring (512 × 4012 B).
- maburd drains the ring at whatever the radio path sustains for the
  commanded op: `air_bytes/s = video_bitrate × (1 + fec_overhead)` at the
  op's MCS airtime.
- When air cost exceeds the platform ceiling, the ring wraps over unread
  slice tails. waybeam does not count this visibly; maburd's `drops`
  counter does NOT capture it (it happens inside slices); the seq space
  stays contiguous. Hence "seq-invisible" (same mechanism as the
  2026-07-15 campaign; that fix reduced TX cost — the lesson was never
  encoded into the policy, so a config pin re-triggered it).

Measured operating points (this drone, ch149/20 MHz, 8000 kbps clamp):

| op | air cost | result |
|---|---|---|
| mcs7 / ov0.10 | 8.8 Mbps | clean (0 truncations, 59.5 fps) |
| mcs5 / ov0.375 | 11.0 Mbps | 51–67% truncation, ~20–29 complete fps |

The proven-clean historical point was 9.3 Mbps air (2026-07-13). Ceiling is
therefore between ~9.3 and 11 Mbps for this unit; treat **≤ 9 Mbps air** as
the safe envelope until measured tighter.

## The bisection (proof the new stack is innocent)

All arms measured with `fu_probe.py`, 15 s each:

| maburd | maburgs | GS op | drone power | hard trunc | fps |
|---|---|---|---|---|---|
| old | old (adaptive → mcs7/ov0.10) | 8.8 M air | none | 0 | 59.5 |
| **new** | old (adaptive → mcs7/ov0.10) | 8.8 M air | none | 0 | 59.5 |
| new | new (pin mcs5/ov0.375) | 11 M air | none | 464 | 28.9 |
| new | new (pin mcs5/ov0.375) | 11 M air | offset −16 | 449 | 28.5 |
| new | new (pin mcs5/ov0.375) | 11 M air | offset 0 | 603 | 19.8 |
| new | new (pin **mcs7/ov0.10**) | 8.8 M air | offset 0 (full) | **0** | **59.6** |

Truncation follows the op's air cost and nothing else.

## Verified-good deployed configuration (as left on the rig)

- Drone: offset-power maburd, `power_mode: "offset"`, walls
  `[91,91,91,91,73,56,51,49]`, margin 1 dB (`/etc/mabur.json`).
- GS: offset-power maburgs, pin `static_mcs 7, static_overhead 0.10,
  static_offset_qdb 0` (`/etc/maburgs.json`).
- Result: 59.6 fps, 0 truncations, 0.06% seq loss, SNR ~61 dB — at full
  wall-parked power.
- Rollback artifacts: `*.pre-offset` binaries/configs on both devices.

## Future work (in priority order)

1. **Encode the drain ceiling into the drone bitrate policy.** Today
   `run_bitrate_policy` (drone/src/rc_agent.cpp) computes
   `kbps = T0_rate × airtime_budget / (1 + overhead)` and clamps to
   `waybeam.bitrate_max_kbps` — nothing in that math knows the platform's
   real USB/ring drain limit, so it happily commands over-ceiling ops
   (mcs5/ov0.375@8000 passed all its checks while destroying the video).
   Candidate shapes: a `radio.max_air_kbps` config (measured per platform,
   like the walls) that caps `kbps × (1 + total_stream_overhead)`; or make
   `airtime_budget` honest against measured drain instead of PHY airtime.
   Note the overhead mismatch too: the policy uses
   `uep_layer_overhead(1, ov)` (T0's share) while the air cost that killed
   us is the whole-stream `(1 + ov)` — audit that discrepancy.
2. **GS-side guard**: the controller's op table could reject ops whose
   `src_bitrate × (1 + overhead) / phy_rate_eff` exceeds a configured
   drone drain fraction — same table, one more feasibility term, mirroring
   how walls cap the power axis (docs/txagc-calibration.md).
3. **maburd death cycle (unresolved)**: ~34 restarts in 52 min during the
   overload era (proxy: waybeam `[venc_config] Config saved` lines — each
   maburd LINKED-entry forces a bitrate set → config save; ALSO fix that:
   a runtime bitrate set should not rewrite flash every time). Each
   rebirth spent ~3.5 min RX-deaf in RENDEZVOUS (`rx_beat=0` — the "RX
   bring-up lottery"). Deaths are plausibly overload-driven and may be
   gone with in-budget ops — host-side watchers (`stall_watch.sh`,
   `deathwatch.sh`, session scratchpad) were left running to catch dying
   words. If deaths persist at 8.8 Mbps air, hunt independently. The
   drone respawn loop truncates `/tmp/mabur.log` on restart, destroying
   crash evidence — make S96mabur rotate (e.g. `mv` to `.prev`) instead.
4. **One USB stall observed, unexplained**: 19 × `bulk_send EP 5 FAIL
   rc=-7` (timeouts, partial transfers) in one ~1.5 s cluster, 1021
   TxQueue drops, self-recovered, never recurred in ~1 h. Possibly
   overload-era; possibly VBUS/interference. Watchers will timestamp a
   recurrence.
5. **Adaptive re-enable interaction**: the old adaptive controller
   incidentally picked in-budget ops (its energy ranking prefers cheap
   airtime — mcs7/ov0.10). When re-enabling adaptive on the offset stack,
   items 1–2 become mandatory first: an adaptive controller exploring
   toward heavy-FEC/slow-MCS rows would walk straight into the ceiling.

## Method notes (for the next debugging session)

- `fu_probe.py` (tools/bench/) is the loss/truncation ground truth on the
  GS; default-buffer `rtpsniff.py` numbers are NOT trustworthy on this SBC
  (its per-gap log remains useful).
- The drone SoC's load average sits ~14 from D-state SDK kernel threads —
  it is NOT a CPU-famine signal on this platform; check `%idle` instead.
- GS `ord[]` counters measure maburgs' own output ordering — they can
  never show source truncation. Sniff the 5600 stream.
- waybeam's `[venc_config] Config saved` line count is a free proxy
  counter for maburd restarts/link-flaps across maburd log truncations.
