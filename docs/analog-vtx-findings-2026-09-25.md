# Analog VTX vs the in-flight channel hop — bench findings, 2026-09-25

Fills the "non-802.11 (O4/analog) interferer — not run" row of the bench
matrix in `docs/inflight-channel-hop.md`. Short version: **the hop never
fires on a co-channel analog carrier, even while the carrier is degrading
the link**, because every non-802.11 term the verdict has is derived from
the preamble detector, which an FM carrier never trips. The ranker has the
same blind spot in the other direction: the carrier's own channels score as
the cleanest candidates.

Raw data: `analog-vtx-2026-09-25.tgz` in the repo root (untracked): two
maburgs debug-log sessions (`log/0000` = bare-sensor scan + first live run,
`log-hop/0000` = live run after the drone restart), three chanscout JSONL
sweeps, the two summarisers, and both `flightreport.py` outputs.

## Rig

- GS = **this host**, one RTL8822EU (`0bda:a81a`, usb 5-1) driven by the
  host-built `maburgs` at branch `hop-split-fix` (`54e4f95`, bw40 rungs +
  hop-split fix; wire-compatible with the drone's 2026-09-24 `maburd`).
  Config = `gs/bundle/maburgs.default.toml` with home **128**, candidates
  `[144, 40, 136]`, `hop.enable = true`, debug log on. The real GS was off.
- Drone on the bench (`192.168.10.152`), `/etc/mabur.toml` with
  `channel = 128` (backup `mabur.toml.pre-analog`), later also the TX wall
  table at its floor (`rate_walls_rel` −40 ×8, `legacy_wall_rel` −40,
  `wall_margin_db` 6.0). Disarmed on the bench, so ~30 fps.
- Interferers: an analog 5.8 GHz VTX on "E4" (table value 5645 MHz), and
  briefly a second one near 5770 MHz (Raceband R4 / Band-B B3). Placement
  was hand-held: ~1 m from the dongle, then "a few cm".
- Gates: `tools/bench/ausniff.py` on the AU ring; `ctl.log`/`scan.log`
  (`V`/`E`/`H`/`D` lines); `flightreport.py` HOP section.

## 1. What the card sees (bare sensor, drone off)

`maburgs` alone stays in its boot scan and logs one `D` line per 277 ms
dwell on home 128 and every candidate half. VTX off, then on:

| ch | VTX off: FA/s med (max) | floor | VTX on: FA/s med (max) | floor |
|---|---|---|---|---|
| 124 | 0 (4) | −94 | 0 (0) | invalid |
| 128 | 14 (54) | −95 | 0 (0) | invalid |
| 132 | 0 (51) | −96 | 0 (0) | invalid |
| 136 | 0 (18) | −95 | 0 (0) | invalid |
| 140 | 0 (18) | −96 | 487 (763) | −76 |
| 144 | 0 (14) | −95 | 28 800 (29 700) | −96 |
| 40 (802.11 neighbour) | 141 (282), foreign 83/s | −92 | 686 (1162) | −96 |

devourer `chanscout` over the whole 5 GHz grid (100 ms dwells, adds the NHM
histogram the `D` record lacks), VTX on:

| ch (MHz) | FA/s med (max) | CCA/s | NHM busy % |
|---|---|---|---|
| 112 (5560) | 2060 (11 190) | 2020 | 3 |
| 116–132 (5580–5660) | **0** | **0** | **100** |
| 136 (5680) | 0 | 0 | 0 |
| 140 (5700) | 0 (530) | 0 | 0–99 |
| 144 (5720) | 220 (2500) | 210 | 0–10 |

Same shape for the second VTX: 149/153/157 at 100 % busy with FA 0 on
153/157, storms of 395/s on 149, 235/s on 161 and 13 400/s on 169.

Reading: the **occupied channels read as silence** — zero false alarms,
zero CCA, the absolute-floor read discarded as invalid (every idle sample
above the −70 dBm top threshold, `NoiseFloorMath.h`), NHM 100 % busy. The
false-alarm storms the 2026-09-14 spike recorded (`FA 250–2650/s`) live on
the **shoulders**, 20–60 MHz off the carrier, where its edge leaks in at a
level noise-like enough to trip the OFDM correlator by chance. The E4
carrier's footprint (116–132 busy, 136 clean) centres nearer 5620 than the
table's 5645; 20 MHz bins can't resolve it further (5 MHz plan tokens are
retuned at 20 by chanscout).

## 2. Live link, co-channel (128 @ 40 MHz = pair 124+128)

### VTX ~1 m away

Link untouched: rung 6 (mcs4/40), RSSI −57 dBm, SNR 31 dB, 30 fps, 0 gaps in
every 30–45 s ausniff window. One 150 ms window at switch-on read FA
120/s → `interfered` with `raised`; no persist, no order. One 0.5 s loss
burst ~80 s later (rung 6→3, re-promoted in 10 s, 0 gaps) — cause unknown,
possibly the VTX stepping up from pit power. In-session FA was 0 on every
other window; CCA = own frames. Drone RX telemetry: foreign 0, crcfail 0
throughout.

Dropping the drone TX to the wall-table floor bought only ~9 dB (RSSI −66,
SNR 28); still untouched. **The floor of that knob is not enough
attenuation for a co-channel test at desk range.**

### VTX a few cm from the dongle (session `log-hop/0000`, onset ≈ t 4900 s)

| | before | carrier on | after (off at ≈ t 5199 s) |
|---|---|---|---|
| RSSI at the card | −58 | −70 … −79 | −58 within 5 s |
| SNR | 31 | 18–21 | 31 |
| fps | 31 | 19–20 | 30 |
| ausniff gaps / 45 s | 0 | 4–6 (+1–2 incomplete) | 0 |
| card CRC-fail counter | 11 | → 125 | → 233 (cumulative) |
| ladder | rung 6 steady | **5 util cascades 6→3**, each re-promoted in ~10 s | one last cascade at the off-transient, then rung 6 |

Verdict over the 176 windows of the degraded stretch (`flightreport.py`):

```
verdicts: healthy 45 unknown 131
evidence bits: impaired=131 weak=0 fading=7 contended=0 raised=0
  card 0 (non-healthy, n=131): foreign=0 fa=1 rssi=-77.6 snr=21.3
```

**0 `interfered`, 0 `fade`, no `H` line, no hop.** `raised` needs
FA > 100/s and FA was 0; `weak` needs RSSI < −78 **and** SNR < 12 and SNR
never left 18–21; `fading` tripped on 7 windows but only gates
`interfered`. Every impaired window fell through to `unknown`, the verdict
that by design "leaves the ladder to it". The ladder did its job (shed and
recovered five times), which is exactly the pre-hop behaviour the spike
called the response today.

One-card note: this GS ran no periodic in-session dwells, and that is by
design — `scout_loop` in `gs/src/main.cpp` gates on `n_cards >= 2`; a
one-card GS picks its target in a synchronous burst at order time. The rig
was valid for the one-card hop path; the absence of an order is the
verdict's.

## 3. How an analog carrier hurts this link

Not as a blocker. An FM video carrier occupies ~10–15 MHz of the 40 MHz
pair; the OFDM link loses that slice of subcarriers and LDPC covers it while
the margin is large (SNR 31 → 28 with no loss). Only when the carrier is
strong enough to **desense the front end** does RSSI fall and CRC failures
start, and then the ladder sheds on `util`. From the verdict's inputs that
is a fade with SNR mysteriously intact — RSSI drops 12–20 dB while SNR
drops ~10 — and nothing tests for that combination.

## 4. What detection needs

**Done 2026-09-25**, same day: the NHM airtime evidence feature (branch
`nhm-airtime`, spec `docs/superpowers/specs/2026-09-25-nhm-airtime-design.md`,
bench gate `docs/nhm-airtime-spike-findings-2026-09-25.md`) shipped
everything scoped below — a `blocked` evidence bit on the verdict
(`kEvBlocked`, beats `fading`, not `weak`) and a blocked tier on both
rankers, fed by `ArmNhmBusy`/`ReadNhmBusy` on the same register pass this
section describes. Detail in `docs/inflight-channel-hop.md` §2/§3. Not
done by that branch: the level sweep this page's §5 still calls out
(desk-range only) and the two-card exercise.

The card already reports the right sensor on the same register pass: the
NHM histogram (`NhmReader.h`), raw idle-air power binned by level,
independent of the preamble detector. Both VTXs put it at 100 % busy on
every occupied channel, ambient 802.11 neighbours at 1–30 %.

- **Verdict:** a `blocked` bit = NHM busy above a fraction on the op card
  with CCA ≈ own, feeding `interfered` like `raised`. Report the bucket-11
  mass explicitly, not just the existing "floor invalid" flag, so
  "everything above the top threshold" is distinguishable from "no idle
  samples on a busy channel" (`nhm_abs_floor_dbm` returns false for both).
- **Rankers:** `HopRanker::score` (= fa + (cca − own) + 4·foreign) and
  `ChannelRanker::busy` (= (cca − own) + fa + foreign) score a saturated
  channel **0 = best**; an invalid floor is skipped, not penalised. Both
  need the busy fraction as a worst-case term.
- **Cost:** as coded, one NHM read is ~27 USB transfers plus 1 ms poll
  sleeps ≈ 8–10 ms — the whole dwell today (~12 ms). Avoidable: program
  the thresholds once (absolute table is fixed, IGI is pinned at 32),
  trigger at the start of the 5 ms observe (2 writes), read ready + 3
  result words with the FA/CCA pass at the end → ~6 transfers, ~1.5 ms per
  dwell. Raise `period` to cover the observe so histogram and counters
  describe the same air.

## 5. Caveats and open items

- **Level regime.** Everything here is desk-range. A VTX on another quad
  at tens of metres arrives weaker; the in-band case may sit in the
  shoulder-like FA regime at that level, which the existing bit would
  catch. A level sweep (walk-out or attenuator on the VTX) is the missing
  measurement. The drone-side power floor is not a substitute.
- **Bench margin.** The drone at −57 dBm with 31 dB SNR shrugs off a
  carrier that reads 100 % busy above −70 dBm. Co-channel tests need the
  drone path attenuated, not the drone's TX index lowered.
- **Two-card GS** was not exercised (the real GS was off). The verdict
  result does not depend on card count; the ranker consequence does
  (periodic dwells would rank the carrier's channels as best targets).
- The one `interfered` window at switch-on and the 80 s-later loss burst
  are unexplained; the VTX's power-up behaviour was not observed.

## Restore

Drone config restored from `mabur.toml.pre-analog` (home 136, normal wall
table) and `maburd` restarted; host maburgs stopped; drone plug left on.
