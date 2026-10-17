# Does demand refresh stay quiet on a clean link and heal a damaged base on the bench?

**Status:** bench record; the flight is pending. The mechanism is
described in `docs/fec-nack.md` "Demand refresh"; the `fec.log` I/H rows,
`link.refresh` and the REFRESH section in `docs/observability.md`. The
GOP ceiling it runs on is `docs/findings/2026-10-17-sdk-gop-ceiling.md`;
the 10 s GOP baseline is `docs/findings/2026-10-17-long-gop-spike.md`
(session 0133). A loss-sim rerun on the fixed build follows the
Answer (session 0139).

## Question

With the drone at `venc.gop_s = 0` (GOP 65535 frames, intra refresh off),
the base layer heals only when maburgs' `RefreshRequester` asks for an
IDR. On a clean link, does it ask for nothing beyond the player's start
report? Under injected base loss, do the requests follow the damage and
heal in about one RCF period? Do the player's reports and an outage
behave as designed? And does the GS still pass its standing gates
(ausniff, aucadence) and judder no more than the 10 s GOP did?

## Setup

Bench drone and bench Radxa GS, rung 4 (MCS 4, 40 MHz), static indoor
scene, 4 row slices, SVC-T, NACK on at the GS. The drone ran the maburd
from the GOP ceiling run (branch `demand-refresh`, `VENC_GOP_NONE` 65535;
the MI proc node read `CBR65535`), with `gop_s` 0 and
`intra_refresh_frames` 0 and no other config change. The GS ran
`maburgs` and `maburplay` (glibc Radxa build, deployed together) built
from the tree at 5fb9cde0. The build finished at the moment 6c34cf13
landed. Both commits change only docs and the drone bundle on top of
642b2353, so the GS code is as of 642b2353. No GS config changed.

**The benched GS build predates the review fix wave 030487d0..974ffc36**,
which landed on the branch during the bench. That wave books slice-salvaged
base AUs as `salvaged` instead of `hole` (4af5a88c), retries a refresh
request only while maburgs is in a session (4af5a88c), corrects the
drone's IDR pacer comments and tests (030487d0), and rewrites
`docs/fec-nack.md` "Across an outage". Where a result below depends on
that, it says so.

| session | GS binary | what ran |
|---|---|---|
| 0136 | maburgs (glibc) | link-up, player start, 326 s clean window, ausniff, aucadence, player restart, two record presses |
| 0137 | maburgs, musl `MABUR_LOSS_SIM` build, `--loss-sim 8303` | 3 % then 10 % base loss, then a 3 s full loss of both layers |
| 0138, 0139 | maburgs (glibc) | ausniff ×2, aucadence, then an 8 s drone power cut (0139 is the rotation at the drone's restart) |

- Loss: `tools/bench/losssim.py --port 8303`. `s0 eff=3 burst=6` for
  117 s, `s0 eff=10 burst=8` for 61 s, then `s0 loss=100` and
  `s1 loss=100` for 3 s (every body of both video layers dropped on both
  cards; the RCF uplink and telemetry stay up).
- The player record button was pressed with `kill -USR1`. The bench
  player records in `dvr.mode = "burned"`, target `both`.
- Pulling the USB radio needs a human. The 3 s full-loss burst and the
  drone power cut stand in for it.
- Judder seconds: `lat.log` seconds with e2e p99 ≥ p50 + 10 ms, the same
  measure as session 0133.

## Result

**Clean link (session 0136).** The window runs 326 s, from the player's
start to the end of the run.

| | 0136, demand refresh | 0133, GOP 10 s, no lock |
|---|---|---|
| `I` rows / `H` rows in the window | 0 / 0 | n/a |
| parameter-set AUs (IDRs) | 2, both `vcl` 19 | one refresh start per 10 s |
| `vcl` 1 (refresh start) rows | 0 | n/a |
| AUs / fps | 19558 / 60.0 | |
| judder seconds | 11 of 326, **3.4 %** | 5 % |
| e2e p50 (median second) | 42 ms | 41.8 ms |

The two IDRs belong to the ladder, not to the requester. One enhance FEC
episode (11 sources, all abandoned, `sid` 1) caused an `s3_residual`
demote 4 → 3 at 3644.6 s and a `promote_probed` back at 3646.3 s, and each
transition carried its bitrate-change IDR. Enhance damage armed nothing,
as designed.

Before the window, every link-up logged one request. The first base AUs
after link-up include one the stream force-advanced past, so maburgs
logs `I lost` 137 ms after the first AU, and the link-up IDR stream heals
it 99 ms later. The same thing happened in 0137 (heal 101 ms) and 0138
(twice, 83 and 149 ms). The player's start report then logs one
`I … start` and one `H` (86 ms). maburtop therefore shows
`refresh: idle req2 retry0 heal2 last86ms | hole0 salv0 lost1 player1`,
not `req1`.

**Standing gates.** ausniff:

| run | AUs | incomplete | `frame_id_gaps` | resyncs | fps |
|---|---|---|---|---|---|
| 0136, clean | 1815 | 0 | 0 | 0 | 60.5 |
| 0138, after the loss-sim binary was swapped back, pass 1 | 1815 | 0 | 0 | 0 | 60.5 |
| 0138, pass 2 | 1816 | 0 | 0 | 0 | 60.5 |

aucadence (`--gate-ms 4.0`, the regression criterion; the recorded
baseline is −1.1 to −3.0 ms at an MCS 5 park):

| run | n base / enh | IDR excluded | offset |
|---|---|---|---|
| 0136 | 750 / 750 | 0 | +0.02 ms (pass) |
| 0138 | 750 / 749 | 0 | −0.24 ms (pass) |

**Injected base loss (session 0137).**

| | 3 % eff, burst 6, 117 s | 10 % eff, burst 8, 61 s |
|---|---|---|
| bodies dropped (s0) | 16580 | 29501 |
| NACK requests / filled / wasted / deadline-dropped (`link.nack` deltas) | 260 / 1827 / 315 / 0 | 279 / 1913 / 251 / 96 |
| base AUs incomplete / slice-salvaged | 0 / 0 | 6 / 22 |
| requests (runs) by reason | 1 `lost` | 27 `lost`, 21 `hole`, 0 `salvaged` |
| requests with a retry | 0 | 1 |
| heal_ms p50 / p90 / max | 71 | 97 / 144 / 423 |
| most `I` rows in one second | 1 | 5 (two seconds had more than 3) |

At 3 %, FEC and the NACK repaired every base AU. The only request was
one `lost` during the demote cascade (4 → 0 in 3.1 s on `residual`),
after which the ladder bounced between rungs 0 and 1. An ausniff pass in
the middle of it read 1815 AUs, all complete, 0 gaps, 60.5 fps.

At 10 %, each of the 28 damaged base AUs had an `I` row in the same
second. The seconds with 4 or 5 `I` rows are separate requests, each
healed in about 100 ms before the next damage, not retries. For example,
in second 3988 five requests healed in 106, 109, 87, 97 and 83 ms. That
misses the brief's expectation of never more than 3 `I` rows per second:
two seconds had 5 and 4. Each was a separate request, not a retry.

Two things in the rows do not match the design:

- **Salvaged base AUs arm as `hole` (pre-fix build; fixed since in
  4af5a88c).** The 22 base AUs that au.log flags `0x40`
  (slice-salvaged) each logged `I … hole`. The sideport's
  `by_reason.salvaged` stayed 0 for the whole session while
  `slice_salvaged` counted 22. They still armed, so the heal was not
  affected, only the label.
- **A `lost` arms right after a heal (open).** 11 of the 32 `lost` requests in
  0137 start within 30 ms of the `H` before them (for example
  `H 3988517 87 0` then `I 3988517 lost 25`, healed 97 ms later). One
  pair is logged out of time order (`H 4030948` then `I 4030940 lost`).
  The force-advanced AU that arms it is older than the IRAP that just
  cleared, so these look like one extra IDR each.

`flightreport.py`'s REFRESH section on 0137 reads requests=50 (hole 21,
lost 28, start 1), retries 17, heals 54, heal_ms p50/p90/max 97/144/3739,
6 within 2 s of a demote, 48 within 2 s of an FEC episode. Read in file
order, the rows give 54 requests and 13 retries. When an `H` and the next
`I` share a millisecond, the report's sort puts the `I` first and counts a
new request as a retry. This miscount is open.

**Player triggers (session 0136).**

| trigger | rows | heal |
|---|---|---|
| `S97maburplay restart` | one `I … start` (epoch 3), one `H` | 158 ms, 0 retries; IRAP `vcl` 19 |
| record press (start), burned mode | none | no IRAP |
| record press (stop) | none | |

In burned mode the player sends no `refresh dvr`: only the raw DVR
(`dvr.mode = "raw"`) needs a key frame to open on. The burned recording
(564 frames) started at the next decoded frame. The raw-mode press was
not tested because it needs a GS `/config` edit.

**Outages.**

| | 3 s full loss, both layers (0137) | drone power cut, 8 s (0138 → 0139) |
|---|---|---|
| first `I` | `lost`, 0.40 s into the burst | none during the outage |
| retries | 12, every ~300 ms through the burst | none |
| `H` | 3739 ms (includes the dead time), as the loss stops | 36 ms after the new producer's first AUs |
| after the heal | one more `lost`, healed 85 ms | one more `lost`, healed 133 ms |
| demote | `starved` 4 → 0, 0.67 s after the first `I` | `starved` 4 → 0, 0.85 s after the cut |
| NACK | 0 requests (nothing arrives to NACK against) | |
| FEC rows | base: 4190 sources missing, 4189 abandoned; enhance: 2397 missing, all abandoned | |
| recovery | back at rung 4 7.5 s after the burst | climb 0 → 4 in 5.3 s once the drone relinked |

During the power cut nothing armed. No base AU was finished incomplete
while the drone was silent, so the request armed only when the restarted
producer's AUs force-advanced the stream past the dead one. That AU was
already covered by the link-up IDR. On this pre-fix build the 3 s burst
retried through the whole outage. Since 4af5a88c, retries accrue only
while maburgs is in a session, and the "Across an outage" text was
rewritten to match. The burst kept the session up (the RCF uplink and
telemetry were not dropped), so its retry rows would likely look the same
at HEAD; that was not re-run.

## Answer

On a clean bench link, demand refresh asks for nothing. In 326 s there
were 0 requests after the player's `start`, the only IDRs were the
ladder's own two, and there were no refresh-start pictures. Judder
seconds were 3.4 % against 5 % at the 10 s GOP, at the same 42 ms e2e
p50. ausniff stayed at 0 gaps and 60.5 fps, and aucadence at
+0.02 / −0.24 ms, inside the 4 ms gate. Each link-up still logs one
`lost` request, which the link-up IDR heals within 150 ms.

Under base loss, the requester fires only once FEC and the NACK have
given up: none at 3 %, and one per damaged base AU at 10 %, heal p50
97 ms and p90 144 ms, with a single retry in 48 requests. A 3 s blackout
retries every 300 ms and heals as the link returns, with the `starved`
demote inside a second.

Two things still need a code look before the flight:

- about a third of `lost` requests arm within 30 ms after a heal, likely
  one redundant IDR each, and one I/H pair is logged out of time order;
- flightreport's REFRESH section counts a new request in the same
  millisecond as a heal as a retry, because it sorts `I` before `H` on
  equal `t_ms`.

The `hole` label on salvaged AUs seen here is fixed in code since
(4af5a88c), but the fix is not benched.

The raw-DVR `dvr` trigger and a real radio pull remain untested. The
flight is pending.

## Loss-sim rerun on the fixed build (session 0139)

**Build.** The GS ran maburgs built from ec9aba5a (GS code as of
974ffc36, so the whole fix wave is in): the musl `MABUR_LOSS_SIM` build
with `--loss-sim 8303` for the loss runs, then the normal glibc build for
the gates. `maburplay` built from ec9aba5a is byte-identical to the one
already on the GS (the fix wave does not touch the player). The drone ran
the same maburd as before, `gop_s` 0 (MI proc node `CBR65535`). The
session directory 0139 was rejoined, so the rerun's rows follow the 8 s
power-cut rows of the earlier run; everything below is windowed by time
from the loss-sim commands' `mono=` stamps.

**Profile.** Same dials as 0137 (`s0 eff=3 burst=6` for 117 s,
`s0 eff=10 burst=8` for 61 s, `s0 loss=100` + `s1 loss=100` for 3 s).
0137 cleared the loss and let the ladder climb back to rung 4 before each
phase. The first rerun pass ran the three phases back to back, so its
10 % phase and its burst started at rung 0. A second pass repeated the
10 % phase and the burst, each from a settled rung 4, to match 0137. The
3 % column below is from the first pass, the 10 % and burst columns from
the second; the back-to-back 10 % phase is the third column. Both
sessions were scored with the same script (requests = `I` runs closed by
an `H`, read in file order; rows whose first `I` falls in the phase or
500 ms after it). It reproduces 0137's table except heal p90 143 against
144.

| | 0137, 3 % | 0139, 3 % | 0137, 10 % | 0139, 10 % from rung 4 | 0139, 10 % from rung 0 |
|---|---|---|---|---|---|
| bodies dropped (s0, loss-sim counter) | 16571 | 16119 | 12833 | 13880 | 12392 |
| NACK requests / filled / wasted / deadline-dropped | 260 / 1827 / 315 / 0 | 277 / 2151 / 132 / 0 | 279 / 1913 / 251 / 96 | 308 / 2396 / 287 / 104 | 296 / 1913 / 419 / 55 |
| base AUs incomplete / slice-salvaged | 0 / 0 | 0 / 1 | 6 / 22 | 10 / 32 | 3 / 20 |
| requests (runs) by reason | 1 `lost` | 2 `lost`, 1 `salvaged` | 27 `lost`, 21 `hole`, 0 `salvaged` | 23 `lost`, 4 `hole`, 21 `salvaged` | 25 `lost`, 2 `hole`, 15 `salvaged` |
| requests with a retry | 0 | 0 | 1 | 7 | 1 |
| heal_ms p50 / p90 / max | 71 | 130 / 161 / 161 | 97 / 143 / 423 | 115 / 416 / 454 | 98 / 157 / 693 |
| most `I` rows in one second | 1 | 1 | 5 (two seconds > 3) | 4 (one second > 3) | 3 |

Bodies dropped is the delta of maburgs' 1 Hz `LOSS-SIM[s0…]` counter
over the phase. The 10 % figure in 0137's table above (29501) is that
counter's cumulative value at the end of the phase, 3 % phase included;
over the 10 % phase alone it read 12833.

**Retries at 10 %.** All 7 retried requests in the rung-4 pass were the
same case. The IDR the drone served came in damaged: 5 of them ended
incomplete (no slice salvage on an IRAP), and in the other 2 the base
AUs around it were lost. So the first heal did not clear, and the 300 ms
retry's IDR did (heal 398–454 ms, which sets the p90). In that window,
6 of 56 IRAP-led base AUs were not complete. 0137 had 0 of 51, and the
back-to-back pass 0 of 42. The fix wave does not touch FEC, the NACK or
the drone's IDR, so this reads as the loss pattern hitting the IDR, not
a change in behaviour. A 61 s run is too short to say more.

**3 s full loss, both layers.**

| | 0137 (from rung 4) | 0139 second pass (from rung 4) | 0139 first pass (from rung 0) |
|---|---|---|---|
| first `I` | `lost`, 0.40 s into the burst | `hole`, 49 ms into the burst (the in-flight base AU) | none during the burst: no AU arrived and nothing armed |
| retries | 12, every ~300 ms | 10, every ~300 ms | — |
| `H` | 3739 ms | 3053 ms, 98 ms after the loss stopped | — |
| after the heal | one `lost`, healed 85 ms | one `lost` 19 ms after the `H`, healed 86 ms | one `lost` 20 ms after the loss stopped, healed 135 ms |
| demote | `starved` 4 → 0, 0.67 s after the first `I` | `starved` 4 → 0, 0.33 s after the first `I` (0.38 s into the burst) | none (already at rung 0) |
| back at rung 4 | 7.5 s after the burst | 7.6 s after the burst | |

The session stayed up through the burst (the RCF uplink and telemetry
were not dropped), so the in-session retry rule changes nothing here, as
the earlier section expected. From rung 0 nothing armed until the first
AU after the loss: the stream force-advanced past the lost ones. That is
the same shape as the drone power cut.

**Checks.**

- **(a) Salvaged base AUs book as `salvaged`: yes.** Each `I … salvaged`
  that opens a request sits on a base AU that au.log flags `0x40`. Each
  `I … hole` sits on a `0x00` AU. No salvaged AU logged `hole`.
  `link.refresh.by_reason.salvaged` matches au.log's slice-salvaged base
  count in every window: 1 = 1 at 3 %, 32 = 32 and 20 = 20 at 10 %. 0137
  had 0 against 22. `by_reason` counts salvaged AUs that land while a
  request is already armed, so it is higher than the number of
  `salvaged` runs (21 and 15).
- **(b) A `lost` arming within 30 ms of the preceding `H`: still open.**
  5 of 23 `lost` requests in the 10 % rung-4 pass, 5 of 25 in the
  back-to-back pass, 0 of 2 at 3 %, and the one `lost` after each burst
  heal. That is 12 of 52 across the loss windows. On 0137 the same windows
  give 10 of 29 (the earlier section's 11 of 32 counts the whole session).
  Two of the rerun's pairs share a millisecond (`H 5671093` / `I 5671093`,
  `H 5913590` / `I 5913590`). So flightreport's REFRESH section still
  counts them as retries: on 0139 it reads requests=99, retries=22, where
  the file order gives 101 and 20.
- **(c) I/H rows out of time order: none** in the rerun's fec.log (0137
  had one). Nothing in the fix wave touches the ordering, so this is a
  clean sample, not a fix.

**Standing gate.** After the swap back to the HEAD glibc maburgs and a
player restart, at rung 4, ausniff passes twice. Pass 1: 1816 AUs, all
complete, 0 `frame_id_gaps`, 0 resyncs, 60.5 fps. Pass 2: 1815 AUs, all
complete, 0 gaps, 0 resyncs, 60.5 fps.

**Answer.** On the fixed build, every slice-salvaged base AU arms as
`salvaged`, and the sideport's count matches au.log. Under 3 % and 10 %
loss the requester behaves as in 0137: none or one request per damaged
base AU, heal p50 98–130 ms, and a 3 s blackout retries every 300 ms
and heals within about 160 ms of the loss ending.
Still open: about a fifth of `lost` requests arm within 30 ms of a heal,
and flightreport miscounts same-millisecond H/I pairs. In this sample
more of the requested IDRs arrived damaged (6 of 56 at 10 %), and those
requests needed one retry.
