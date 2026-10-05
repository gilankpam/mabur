# fec-nack bench gate, 2026-10-06

The bench gate for the as-built software NACK (`docs/fec-nack.md`, spec §8),
run on branch `fec-nack` at 9417274: drone `out/arm/maburd`, GS
`out/arm64/maburgs` (md5 e926abd1…), both cross-built from that HEAD,
RC_VERSION 15 on both ends. Every arm ran the as-built binary staged as
`/usr/local/bin/maburgs.fecnack` with a config from `tools/bench/nack/cfg/`
under `/tmp/cfg/` on the GS. Sessions are `/media/dvr/log/00NN` on the GS.

**Build provenance.** The GS binary used in every arm (e926abd1…) was built
with `MABUR_LOSS_SIM=ON`, a stale CMake cache entry from the spike. The
injector does nothing without `--loss-sim`, and only the loss-sim arm passed
that flag. After the gate, `tools/build-arm64.sh` was changed to force
`-DMABUR_LOSS_SIM=OFF`, and a clean rebuild (md5 0d08cf66…) replaced
production maburgs and `maburgs.fecnack` on the GS.

**Summary.** The base layer gets what it was built for: under the jammer the
NACK arm had 0 base truncations, 0 base drops and 0 base abandoned episodes
(after the start-up warm-up) against 10 / 2 / 12 in the control, with air
and cadence unchanged. Three criteria fail or cannot be shown: **wasted
requests run at 35–37 %** under real interference (gate 25 %), the **fade
arm's demote-timing criterion** (one feedback period) is outside what this
bench can resolve and reads +295 ms on the mean, and **at rung 0 no request
was ever filled**. Total AU truncations barely move (225 → 210), because
the jammer's truncations are almost all enh, and the NACK covers base only.

## Rig and jammer

- Jammer: `tools/bench/benchjam.sh` on the host 8822EU, channel 144 (the op
  channel, 40 MHz pair 140+144), 1000 B QoS-Data at 6 Mbit/s.
- Calibration (pinned mcs2/40, control config, 60 s): **180 fps → base
  abandoned 49 symbols/min, enh 919, frames trunc 40 / drop 8**. That is
  inside the 30–100 target, so 180 was used, the same as the 2026-10-05 spike.
  The jammer reported 12 910 frames in 75 s (≈172 fps actual).
- Loss arm: `PPS=180 ARMS="A0_control A1_nack" tools/bench/nack/runjam.sh`.
  Each run is 300 s under the jammer plus about 20 s of link-up before it and
  about 8 s after it. The session numbers below cover the whole session.
- Fade arm: `cfg/F0_control.toml` / `cfg/F1_nack.toml` (the A configs with
  `static_mcs = -1`), 40 s of link-up (the ladder climbs to rung 5,
  mcs4/40), then
  `benchjam.sh --pps 250 --burst-on 10000 --burst-off 20000 --secs 95`. That
  gives three 10 s pulses, and a fourth 5 s pulse at the end that never
  demoted in any run. Each pulse edge was read from the cards'
  `foreign_pps` in flight.jsonl, which is on the same clock as ctl.log.
  The pair ran twice (runs 1 and 2) to measure control-vs-control noise.
- Loss-sim arm: the production build tree `build-arm64` still had
  `MABUR_LOSS_SIM=ON` cached from the spike, so the as-built binary already
  contains the injector (see Concerns). The arm used the same binary with
  `--loss-sim 8303` and `lossctl.py 's0 eff=1.5 burst=4'` (base only). No
  separate build was needed.

| arm | control session | NACK session |
|---|---|---|
| loss (jammer 180 fps, pinned mcs2/40) | 0018 | 0019 |
| fade run 1 (bursty 250 fps, adaptive) | 0020 | 0021 |
| fade run 2 | 0022 | 0023 |
| loss-sim (s0 eff 1.5 burst 4, pinned) | 0024 | 0025 (cut at 252 s, see below) |

## Loss arm (jammer, pinned mcs2/40)

| | control 0018 (330 s) | NACK 0019 (347 s) |
|---|---|---|
| truncated AUs, base / enh | 10 / 215 | **0** / 210 |
| dropped AUs (fid gaps), base / enh | 2 / 35 | **0** / 43 |
| fec abandoned episodes, base / enh | 13 / 249 | 1 / 250 |
| base abandoned excl. the t+3 s start-up warm-up episode | 12 episodes | **0** |
| gsdelta 300 s window: s0 abn syms / frames trunc / drop | 457 / 225 / 75 | **0** / 210 / 86 |
| complete first→finish p50 / p90 / p99 ms | 9.1 / 12.8 / 43.3 | 9.1 / 12.9 / 40.5 |
| `link.air_pct` median | 52.53 | 52.56 |
| drone cpu median | 34.7 | 34.5 |
| NACK requests / repeats / syms_requested / tail_requests | – | 296 / 21 / 3 692 / 256 |
| filled / late_fill / wasted / dropped_deadline / suppressed | – | 2 007 / 134 / **1 305 (35.3 %)** / 0 / 0 |
| fill ms (200 ms windows): median of window p90 / p90 of window p90 / max | – | 10 / 16 / 27 |
| settle_ms (last) / late_ms_max | – | 9 / 32 |
| drone.nack rx / retx_syms / retx_refused | 0 / 0 / 0 | 249 / 3 134 / **0** |
| uplink delivery (drone rx / (requests + repeats)) | – | 79 % |
| `drone.auth_reject` true rows | 0 of 1 644 | 0 of 1 707 |

Base/enh split of the dropped AUs: AU parity maps 1:1 to class in both
pinned sessions (fid even = base, one exception in 0019).

Readings:

- Base truncated + dropped: **12 → 0 (−100 %)**. Every base hole the jammer
  caused was refilled.
- The total AU loss is unchanged because the jammer's truncations are enh
  (215 of 225). The spike's 256 → 14 truncations (findings 2026-10-05)
  were measured with a per-layer tracker that also covered enh. The as-built
  NACK is base-only by design, so that headline does not carry over.
- Wasted is 35 %: a FEC repair recovered the symbol before the copy landed.
  The adaptive settle sat at 9 ms (`late_ms_max` 32). That matches the
  spike's settle-6 arm (34 % wasted) and not its settle-12 arm (19 %).
  86 % of requests were tail requests (256 of 296).
- `auth_reject` was never true, including at pair adoption. Both sessions
  open with one row of drone `failsafe` (left over from the previous arm's
  maburgs) and are `linked` 0.4 s later. No ≥1 s dropout happened inside
  either session, so the post-dropout case was not exercised here.

## Fade arm (adaptive ladder, bursty jammer 250 fps 10 s on / 20 s off)

First demote after each pulse onset (ms; `feedback_ms` = 50):

| run | arm | pulse 1 | pulse 2 | pulse 3 | reach rung 0 (pulses 1/2/3) |
|---|---|---|---|---|---|
| 1 | control 0020 | 580 | 471 | 308 | 7 088 / 7 113 / 3 165 |
| 1 | NACK 0021 | 1 166 | 774 | 441 | 8 364 / 4 011 / 5 061 |
| 2 | control 0022 | 222 | 1 349 | 140 | 2 725 / 5 378 / 3 596 |
| 2 | NACK 0023 | 351 | 76 | 2 035 | 5 036 / 3 242 / 4 533 |

- First demote: control mean 512 / median 390 ms, NACK mean 807 / median
  608 ms (+295 ms on the mean). Time to reach rung 0: control mean 4 844,
  NACK 5 041 ms (+197 ms).
- The two control runs alone differ by 117 ms on their means, and single
  pulses range from 140 to 1 349 ms. The spec's "within one feedback period"
  is below this bench's noise with three pulses per run, so the criterion
  can be neither met nor refuted here. As literally stated it fails.

NACK counters (whole session):

| | run 1 (0021) | run 2 (0023) |
|---|---|---|
| requests / repeats / syms_requested / tail | 92 / 33 / 845 / 41 | 57 / 17 / 543 / 7 |
| filled / wasted / dropped_deadline | 296 / 311 (37 %) / 29 | 189 / 196 (36 %) / 19 |
| suppressed | 2 (all at rung 0, pulse 1) | 3 (all at rung 0, pulse 2) |
| drone rx / retx_syms / retx_refused | 79 / 675 / **80** | 48 / 449 / **34** |
| syms requested at rung 0 → filled / wasted | 277 → **0** / 169 | 190 → **0** / 120 |
| filled by rung 1/2/3/4/5 | 3/89/63/141/0 | 19/0/115/8/47 |
| base abandoned episodes (control → NACK, incl. warm-up) | 9 → 1 | 5 → 1 |
| frames trunc / drop at end (control vs NACK) | 25/14 vs 24/20 | 15/19 vs 24/22 |
| `link.air_pct` median (control vs NACK) | 45.8 vs 45.7 | 45.6 vs 45.3 |
| `drone.auth_reject` true rows | 0 | 0 |

Rungs are taken from the ctl.log E lines at each 200 ms sideport row, so the
attribution is approximate at transitions. The drone's counters arrive with
Telem lag. NACK activity happens only inside the pulses. Every off segment
has zero requests, so nothing runs between pulses.

- `suppressed` rises, but only at rung 0 and in one pulse per run (2 and 3).
- `retx_refused` stays bounded: 80 and 34 symbols, 10.6 % and 7.0 % of
  what the drone was asked for. The refusals fall at rungs 1–3.
- **No fill ever lands at rung 0** (mcs0/20), where the ladder parks for
  the rest of each pulse. The drone did send at rung 0 (rx 46 / 32 NACKs,
  retx 251 / 173 symbols in Telem attributed to rung 0), but every request
  there ended wasted or past its deadline. Fills do show at rungs 1–5 on the
  way down and back up.

## Loss-sim arm (secondary; pinned mcs2/40, `s0 eff=1.5 burst=4`)

The drone's bench plug was switched off externally at about 03:59:55, 252 s
into the NACK session (both cards dropped to 0 pps at once). This bench
never touched the plugs. The NACK arm therefore has about 232 s under loss
against the control's 297 s. Rates are per minute of loss.

| | control 0024 (317 s) | NACK 0025 (252 s usable) |
|---|---|---|
| truncated AUs (all base) | 19 (3.6/min) | 1 (0.2/min) |
| dropped AUs | 1 | 0 |
| base abandoned episodes excl. warm-up / syms | 18 / 586 | **0** / 0 |
| injected per-card s0 drops (`LOSS-SIM`) | 81 500 | 64 344 |
| requests / repeats / syms_requested / tail | – | 326 / 9 / 3 025 / 142 |
| filled / late_fill / wasted / dropped_deadline / suppressed | – | 2 274 / 12 / 618 (**20.4 %**) / 24 / 0 |
| fill ms: median window p90 / p90 of window p90 / max | – | 8 / 14 / 28 |
| drone rx / retx_syms / retx_refused | – | 321 / 2 933 / **2** |
| uplink delivery | – | 96 % |
| `link.air_pct` median | 53.25 | 52.52 |
| complete first→finish p99 ms | 35.1 | 27.3 |

Fills per unit of loss: about 588 fills/min against 118 control-abandoned
base symbols/min. That is about 5 fills per symbol the control lost, because
most fills land on symbols the FEC would have recovered anyway. It is 3.5 %
of the per-card injected drops.

## Standing gates

`tools/bench/ausniff.py` and `tools/bench/aucadence.py` ran on the GS against
`/dev/shm/mabur-au` during the loss arm. Each started about 50–70 s into the
jammer: 120 s for ausniff, then 25 s for aucadence.

| | control 0018 | NACK 0019 |
|---|---|---|
| ausniff AUs / fps | 7 198 / 60.0 | 7 205 / 60.0 |
| complete base / enh, incomplete base / enh | 3 603 / 3 512, 2 / 81 | 3 609 / 3 507, 0 / 89 |
| frame_id_gaps / resyncs / dropped_oversize | 16 / 0 / 0 | 13 / 0 / 0 |
| aucadence base−enh offset (n 750 / ~746) | +1.15 ms | +0.74 ms |

ausniff is clean: no resyncs and no oversize drops. The gaps and incomplete
AUs are the jammer's real losses and match au.log. The aucadence offset sits
at the recorded pinned-mcs2 baseline of +1.1 ms, well inside the
`--gate-ms 4.0` envelope. Before the arms, a 20 s ausniff on production
maburgs with the new drone (no jammer, mcs4/40) read 1 216 AUs, 0 gaps,
0 resyncs. The fade and loss-sim arms were not gated with ausniff.

## Verdict against spec §8

| criterion | result |
|---|---|
| loss: base truncated + dropped ≤ 10 % of control | **PASS**: 12 → 0 |
| loss: fill p90 < 25 ms | **PASS**: window p90 median 10, 90th pct 16, max fill 27 |
| loss: wasted < 25 % of syms_requested | **FAIL**: 35.3 % (adaptive settle 9 ms) |
| loss: drone `retx_refused` 0 | **PASS**: 0 |
| loss: `air_pct` within 1 pt | **PASS**: 52.53 vs 52.56 |
| ausniff clean | **PASS** |
| aucadence within per-rung baseline | **PASS**: +1.15 / +0.74 ms vs +1.1 baseline |
| fade: demote timing within one feedback period | **FAIL as stated / unresolvable**: +295 ms mean, control-vs-control noise 117 ms |
| fade: `suppressed` rises during pulses | **PARTIAL**: 2 and 3, rung 0 only, one pulse per run |
| fade: `retx_refused` bounded | **PASS**: 80 / 34 symbols (≤ 11 %) |
| fade: fills resume at the lower rung | **FAIL at rung 0**: 0 of 467 requested symbols filled; fills at rungs 1–5 only |
| loss-sim: same criteria | base −100 %, fill p90 ok, wasted 20 % ok, air ok, **retx_refused 2** (not 0) |
| `auth_reject` stays false | **PASS**: 0 rows in all 8 sessions, including pair adoption |

## Concerns carried forward

- **Wasted at 35–37 % under real interference.** The adaptive settle runs
  at 8–9 ms, below the 12 ms the spike recommended. Under the jammer,
  `late_ms_max` reads 32, so a settle floor or a different adaptation input
  is the lever.
- **Rung 0 never fills.** At mcs0/20 the copies always lose the race to the
  repair or the deadline. Requests there spend uplink and drone air for
  nothing, so suppressing the NACK at rung 0 is worth considering.
- **The enh layer dominates AU loss under a co-channel jammer.** The
  base-only NACK does not move the visible truncation count in this
  geometry.
- The loss-sim injector in the production build is fixed: see Build
  provenance above.
- The loss-sim NACK arm was cut short by the external drone power-off, and
  the drone's state after that was not observed. Right after the session
  dropped, maburgs logged one `REFUSING video: peer session did not
  advertise CAP_FRAME_WIRE (chip_caps=0x0007)` with both cards at 0 pps.
  Its origin was not investigated.
