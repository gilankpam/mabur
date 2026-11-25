# 2026-11-25 — IGI floor watchdog: bench gates

**Status:** bench record for the IGI floor watchdog as built; the behaviour is
described in [`docs/deploy.md`](../deploy.md) ("wide-TX IGI floor").

## Question

Does `maburd`'s IGI floor watchdog leave a healthy 40 MHz link alone, recover a
unit whose floor is too low, and stay down on a busy channel? The motivation is
[2026-11-25-igi-floor-spike](2026-11-25-igi-floor-spike.md).

## Method

The bench 8812EU drone and the bench GS, nothing installed: the init services
stopped, the test `maburd` run from `/tmp`, `maburgs` from a copy of the live
config that differs only in the channel set and pin, the static MCS or the
ladder, and `[debug_log]`. Channel 144 is DFS, 161 is not (same pair side).
"Base 0x1e" is `MABUR_IGI_FLOOR_BASE=0x1e`, a floor below this unit's edge on
161. Gate 3's interferer is a second 8812EU on the bench host airing
1400-byte MCS0/20 frames on 144 with no gap (`txdemo`).

Video is read from the GS `au.log`: frames per second, and arrival = frame
complete minus capture stamp. The session-wide fastest frame is not a stable
reference: it drifts by about 10 ms over a few minutes on links with no loss
and no drone queueing. Each 30 s slice is therefore judged against its own
fastest frame, and its spread (99th percentile minus median) is reported
beside it. Healthy slices spread 5-8 ms; stalled ones about 100-230 ms.

## Results

Three builds were benched in turn. Each failure below changed the design
before the next build.

**Build 1 — longest radio send only (slow ≥ 30 ms).** Gates 1, 4 and 5
passed (no floor change; longest send ≤ 6 ms). Gates 2 and 6 failed: at
IGI 0x1e on 161 the video lost frames and ran 60-160 ms late at the 99th
percentile while every USB send returned within a few ms. The frames back up
inside the chip, and a send blocks only when the chip's queue is full, which
happened too rarely. One run never triggered in 200 s, one triggered after
about 150 s of bad video, and one kept 0x20 too early (two clean seconds by
chance) before a second episode. Gate 3 could not be judged: the jammer did
not slow the sends.

**Build 2 — plus the chip's PUB TX queue backlog (slow ≥ 40 %, then 60 %).**
The spike's per-second queue samples separated the cases (healthy linked
seconds: 1 % reach 40 % backlog; IGI 0x1e: 90 %). Gates 2 and 6 passed at
40 %, keeping 0x24 within 5-9 s of the first raise. Gate 1 on 161 regressed
at 40 %: two raises at 40-42 % backlog on a link that had been clean at 0x22.
At 60 % gates 1, 2, 4, 5 and 6 passed (the step from 0x20 fired at 60-61 %).
Gate 3 failed: the jammer filled the queue to 97-99 %, and the watchdog kept
0x24, 0x26 and 0x34 on the way up and finally 0x35. With the watchdog off
(cap = base) the same jammer gave the same video (33-36 fps, 99th
percentile ~235 ms), so the climb did not hurt the downlink; the defect was
a kept 0x35, which costs uplink sensitivity for the session.

**Busy-airtime spike (watchdog off, one 240 ms CLM window a second, 768/768
readings valid).**

| Run | Busy airtime p10/p50/p90 | Deepest backlog per second p10/p50/p90 | Longest send p50/p90 |
|---|---|---|---|
| IGI 0x1e, 161 | 0 / 0 / 0 % | 45 / 84 / 99 % | 5 / 73 ms |
| 0x22, 161 | 0 / 0 / 0 % | 6 / 13 / 21 % | 1 / 2 ms |
| 0x22, 144 | 0 / 0 / 0 % | 6 / 16 / 22 % | 1 / 2 ms |
| 0x22, 144, jammer | 48 / 56 / 61 % | 9 / 26 / 99 % | 10 / 18 ms |
| 0x35, 144, jammer | 49 / 57 / 61 % | 9 / 35 / 99 % | 3 / 18 ms |

An IGI stall is a quiet-channel fault, and the drone's own heavy TX does not
hide a busy channel from the reading.

**Build 3 — plus a busy-channel gate (≥ 20 % = window not judged) and a
stricter keep (three clean windows in a row; clean = backlog < 30 % and no
slow send; 30-60 % neither).** 170 s runs; gate 3 330 s.

| Gate | Setup | Watchdog | Video |
|---|---|---|---|
| 1 | MCS2/40, 161, base 0x22 | no change; longest send ≤ 6 ms | 60 fps; spread 17-21 ms for the first 60 s, then 6 ms |
| 2 | MCS2/40, 161, base 0x1e | 0x20 at 2 s (backlog 60 %), 0x24 at 6 s (99 %), kept at 9 s; no change after | 60 fps; spread 13-19 ms to ~100 s, then 5 ms |
| 3 | MCS2/40, 144, jammer | no change for 5½ min | 32-35 fps, 99th percentile 215-260 ms (the same as with the watchdog off) |
| 5 | adaptive 5-rung 40 MHz ladder, 161 | no change | 60 fps; spread ≤ 6.6 ms |
| 6 | adaptive 40 MHz ladder, 161, base 0x1e | 0x20 at 2 s (99 %), 0x24 at 4 s (72 %), kept at 7 s | 60 fps; clean after the first slice |

Gates 1 on 144 and 4 (the shipped all-20 MHz ladder) were not re-run on
build 3; they passed on builds 1 and 2 (no floor change), and build 3 adds
only a reason not to raise and reads busy airtime only while video is at
40 MHz. The rough first minute on 161 also appears in gate 1, where the
floor never moved.

## Not shown

- The 8812CU with `maburd` (no CU VTX on the bench).
- Range: the uplink cost of a kept raise, and the busy gate at weak signal.
- A second EU unit, and channels other than 144 and 161.
- An IGI stall on a channel with 20 % or more foreign airtime: the gate does
  not judge those windows, so such a stall is not recovered.
- Interference weak enough to read under 20 % busy.
- Runs longer than 5½ minutes.
