# Why did the 20 MHz ladder hop on a clean channel? — 2026-11-04

**Status:** fixed the same day — `docs/inflight-channel-hop.md` §2 owns the own-airtime reconstruction (delivery-scaled).

## Question

DVR session 0344 (2026-11-04, the six-rung 20 MHz ladder, channel set
64/112/144, jammer off) hopped six times in a flight with nobody else on
the air. The operator's read: "on the 20 MHz-only ladder the hop happened
on loss". Were the hops real, and if not, which route fired and why?

## Setup

- maburgs on the `ladder-derived-width` branch head of the day (hop
  detect+confirm route in, NHM-only detection), two USB cards on the GS,
  `hop.verdict` at bundle defaults (`detect_pct` 8, `detect_windows` 5,
  `blocked_pct` 30, `occupied_windows` 5, weak gate −78 dBm / 12 dB).
- Session directory `log/0344` on the DVR: `scan.log` (`scanlog 9`, 229
  `V` rows, 18 `H` rows), `ctl.log` (`ctllog 13`), `flight.jsonl`.
- Compared against session 0340 (same day, the five-rung 40 MHz ladder,
  jammer ON for part of it).

## Result

**Every hop was the detect route, none had foreign energy behind it.**
scan.log holds 4 `order` and 2 `escape` events; each followed two
`interfered 05` windows (impaired + detected). No window in the session
set `blocked` (0x20) or `occupied` (0x80). The best card sat at −62 to
−74 dBm at every order, above the weak wall, so `weak` never gated. Each
hop cost a `hop_restore` to rung 4 and a residual demote to rung 2–3.

**The foreign reading tracked the card's own decode failures.** The
verdict's foreign = NHM busy − own airtime, and own airtime was rebuilt
from CRC-good frames only. Per-card readings over all 229 `V` rows, by
that card's `crc_fail` count in the window:

| CRC-failed frames in the window | readings | median "foreign" % |
|---|---|---|
| 0 | 237 | 1.8 |
| 1–9 | 55 | 2.8 |
| 10–29 | 55 | 7.3 |
| 30–59 | 40 | 12.6 |
| ≥ 60 | 14 | 23.4 |

The first order's trigger window: card 0 decoded 163 frames (a healthy
window decodes ~480), 124 failed CRC, ~190 never reached the host; own
airtime 27 % against NHM busy 83 % → 56 % "foreign". Card 1: 408 decoded,
63 % own, 84 % busy → 20 %. Minimum across cards 20 %, over `detect_pct`.

**It was a fade, not traffic.** The `foreign` frame counter (CRC-good,
non-mabur sender) is 0 in all 401 per-card readings. NHM busy's median is
80.2 % in windows with no CRC failures and 78.1 % in windows with ≥ 30: the
air was equally full either way, only the decoded share changed. The
per-card seq loss at the trigger window was 61 % (card 0) and 38 % (card
1), with both cards at −73 to −75 dBm.

**Why 0344 and not 0340 (40 MHz).** The artifact is width-independent:
0340's ≥ 30 CRC bucket reads 15 % foreign too. Three things differed. Own
airtime: 74 % median at 20 MHz vs 61 % at 40, so the same miss fraction
reads ~20 % larger. Both cards faded together: 20 of 34 lossy windows in
0344 had both cards lossy (min-across-cards median 10 %, over the floor
65 % of the time) against 11 of 34 in 0340 (median 5 %, 38 %). Fade
length: 0344 had runs of 4, 5 and 7 consecutive windows over the floor;
0340's longest was 4 and most were single windows, under the five-window
ring median. 0340 also flew deeper (p10 RSSI −82.6 dBm), where the weak
gate takes over, and its four hops carried real jammer energy through the
blocked route. A 40 MHz ladder is less exposed, not immune.

**What would not fix it.** Adding the CRC-failed frames' airtime back
leaves card 0 at ~35 % foreign (the ~190 frames that never reached the
host are the larger share). Raising `detect_pct` breaks the sub-threshold
jammer route (`2026-11-03-nhm-hop-threshold-bench.md`).

**The fix.** Scale decoded own airtime by the window's seq delivery
(`seq_expected / seq_received` from the drone's hardware-seq walk, which
counts every frame the drone sent whether or not this card heard it),
capped at 100 %. Applied on paper to the four `order` trigger windows,
min-across-cards foreign reads 0–6 %, under the floor in every one.
Built the same day in `ChannelCore`'s window fill (test
`own_air_scales_by_the_cards_seq_delivery`).

## Caveats

- The seq walk advances only on a decoded frame, so a gap at a window's
  end is booked into the next window: the first under-counts, the second
  over-counts. The five-window ring median absorbs it.
- After the fix the detect route is less sensitive on a 20 MHz ladder by
  construction: at 77 % own airtime only ~23 % of the air is left for an
  interferer to show in NHM. The jammer walls of
  `2026-11-03-nhm-hop-threshold-bench.md` should be re-measured against the
  scaled figure; not done here.
- Below `busy_dbm` (−83) our own frames leave NHM busy as well as the seq
  walk, so the scale overcounts own air and foreign reads zero. That is the
  range edge the weak gate owns.
- Not flown after the fix.
