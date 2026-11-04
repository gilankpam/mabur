# Where does the hop threshold actually sit, and what happens below it? — 2026-11-03

**Status:** superseded in part by the detect route — docs/inflight-channel-hop.md owns detection

## Question

The in-flight hop's only route to `interfered` at the time of this bench
was `occupied ∧ blocked`: foreign NHM airtime (busy − own) ≥
`radio.scan.blocked_pct` (30 %) held for `hop.verdict.occupied_windows`
(5) consecutive 150 ms windows. Two questions: where does that wall
actually fire in practice against a real interferer, and what does the
link do at airtime below it? And, separately: would a lower floor
confirmed by loss — sketched as "detect low, confirm with loss" — hop
sooner down there, without ever firing on a clean site?

## Setup

Interferer: the host's EU card, `linkbench-tx --no-cca --foreign-sa
--pwr-mode override --pwr 63`, MCS0, channel matching the link's current
primary. Per trial: wait for the GS to report `idle`/`healthy` and settle
12 s, start a sideport capture and an `ausniff` ring read concurrently,
5 s later launch the TX for the trial's duration, then capture 2 s past
TX exit. Sideport datagrams arrive on an approximately 200 ms cadence —
coarser than the verdict's own 150 ms window — so every window count and
timing figure below is an approximation of what the verdict code actually
sees, not an exact replay of it.

The TX takes measurable time to actually get on air after its process
launches (antenna/PHY bring-up): measured 5.0–5.45 s across 27 timed
trials (one 7.2 s outlier), so "on-air" is anchored on the TX's **exit**
timestamp minus its run duration minus a 0.3 s close margin, not its
launch mark.
All "+Ns" times below are relative to that anchor.

Four sweeps, same method, different ladder/jammer combinations:

1. **20 MHz, six-rung ladder (ch144), 20 MHz jammer on the primary**
   (`a025`…`a150`, app rate 0.25–1.5 Mb/s) — the baseline case.
2. **20 MHz, six-rung ladder (ch144), HT40 (40 MHz) jammer** (`w1000`…`w3000`,
   1–3 Mb/s) — foreign energy the link's own radio can only half-hear.
3. **40 MHz, five-rung ladder at the bundle's 24 Mb/s cap, both jammer
   widths** (`p0500`…`p1200` at 20 MHz, `w1000`…`w3000` at HT40) —
   the ladder the drone was actually flying at the time.
4. **Same 40 MHz / 24 Mb/s-cap ladder, low app-rate HT40 jammer**
   (`w0100`…`w1000`, 0.1–1 Mb/s) — probing the floor below the 15–16 %
   airtime band.

Plus a 60 s repeat of the HT40 1 Mb/s trial for a clean-vs-jam harm
comparison, a 60 s capture with no jammer running at all (the clean-air
baseline), and one pilot trial (ch112, 20 MHz jammer) that confirmed the
method before the main sweep. The jammer's own frame/bitrate log gives an
"airtime %" independent of the GS's NHM reading (frames/s × per-frame
overhead, plus the encoded air bitrate, against the channel's raw
capacity at that bandwidth) — the two numbers below disagree by a
consistent small amount, which is itself a result (see "Clean-air
foreign distribution"). Three of the planned top-power trials (`a150`,
both `w3000` runs, and `runs40low`'s `w0700`) produced no capture data
(a TX or capture-process failure) and are excluded.

**Reading the result tables:** for a trial that never hops, the "jam
window" is the trial's whole duration; for one that hops, it is cut off
at the hop order, so its loss/fps/drop/trunc columns cover a shorter span
than a no-hop trial's — a fast hop's "0 drops" is not directly comparable
to a slow one's hundreds.

## Results

### 1. 20 MHz ladder, 20 MHz jammer (`runs/`)

| trial | app rate | jammer airtime | NHM foreign med/max | blocked/occupied windows | existing-route result | ch end | pre-FEC loss med/max | fps min | AU drop | AU trunc | rung path |
|---|---|---|---|---|---|---|---|---|---|---|---|
| a025 | 0.25 Mb/s | 7.9 % | 10.1/20.6 % | 0/0 | no hop | 144 | 0.0/0.1 | 49.5 | 1 | 1 | 0–0 |
| a050 | 0.5 Mb/s | 15.9 % | 19.3/29.1 % | 0/0 | no hop | 144 | 0.1/0.2 | 24.9 | 64 | 32 | 0–0 |
| a075 | 0.75 Mb/s | 23.8 % | 28.1/32.1 % | 6/0 | no hop | 144 | 0.2/0.2 | 15.0 | 278 | 185 | 0–0 |
| a090 | 0.9 Mb/s | 28.5 % | 31.8/35.0 % | 5/1 | hop, ordered +1.0 s | 64 | 0.1/0.2 | 40.0 | 1 | 9 | 1–4 |
| a100 | 1.0 Mb/s | 31.7 % | 34.9/35.4 % | 4/1 | hop, ordered +0.8 s | 144 | 0.1/0.1 | 39.8 | 4 | 7 | 1–3 |
| a120 | 1.2 Mb/s | 38.0 % | 40.4/41.3 % | 3/2 | hop, ordered +0.5 s | 64 | 0.3/0.4 | 24.6 | 5 | 9 | 1–5 |

The wall sits at roughly 28–30 % raw airtime: below it (a025–a075) the
hop never fires; at and above it (a090–a120) it orders in 0.5–1.0 s. The
worst sub-threshold case, a075, is examined below.

### 2. 20 MHz ladder, HT40 jammer (`runs40/`)

| trial | app rate | jammer airtime | NHM foreign med/max | blocked/occupied windows | existing-route result | ch end | pre-FEC loss med/max | fps min | AU drop | AU trunc | rung path |
|---|---|---|---|---|---|---|---|---|---|---|---|
| w1000 | 1.0 Mb/s | 15.6 % | 23.7/32.5 % | 3/0 | no hop | 144 | 0.2/0.2 | 29.7 | 135 | 102 | 0–0 |
| w1500 | 1.5 Mb/s | 23.5 % | 39.1/40.6 % | 2/2 | hop, ordered +2.6 s | 64 | 0.3/0.4 | 54.2 | 10 | 11 | 0–4 |
| w2000 | 2.0 Mb/s | 31.3 % | 34.8/37.3 % | 5/2 | hop, ordered +0.9 s | 144 | 0.3/0.4 | 19.7 | 6 | 24 | 2–4 |
| w2500 | 2.5 Mb/s | 39.1 % | 42.0/43.9 % | 4/1 | hop, ordered +0.9 s | 144 | 0.2/0.4 | 40.0 | 5 | 27 | 1–4 |

Same shape as the 20 MHz jammer, with the GS's radio only half-hearing
the HT40 jammer's energy (it cannot decode its frames at all, same as
`docs/findings/2026-11-03-ht40-dwell-secondary.md` found for a dwell) —
yet NHM still reads it as foreign airtime close to the 20 MHz case at
the same app rate.

### 3. 40 MHz ladder at the 24 Mb/s cap, both jammer widths (`runs40l/`)

| trial | jammer width | app rate | jammer airtime | NHM foreign med/max | blocked/occupied windows | existing-route result | ch end | pre-FEC loss med/max | fps min | AU drop | AU trunc | rung path |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| p0500 | 20 MHz | 0.5 Mb/s | 15.9 % | 22.4/30.2 % | 0/0 | no hop | 64 | 0.2/0.2 | 30.0 | 119 | 267 | 0–0 |
| p0750 | 20 MHz | 0.75 Mb/s | 23.8 % | 32.7/32.7 % | 1/1 | hop, ordered +2.1 s | 144 | 0.3/0.3 | 33.3 | 0 | 0 | 3–3 |
| p0900 | 20 MHz | 0.9 Mb/s | 28.5 % | 34.8/41.8 % | 8/2 | hop, ordered +1.5 s | 64 | 0.2/0.3 | 44.3 | 3 | 35 | 0–3 |
| p1200 | 20 MHz | 1.2 Mb/s | 38.0 % | 47.8/52.9 % | 5/2 | hop, ordered +1.0 s | 144 | 0.2/0.4 | 42.9 | 10 | 30 | 0–1 |
| w1000 | 40 MHz | 1.0 Mb/s | 15.6 % | 17.7/22.6 % | 0/0 | no hop | 144 | 0.0/0.0 | 54.7 | 4 | 1 | 0–4 |
| w1500 | 40 MHz | 1.5 Mb/s | 23.5 % | 25.6/31.7 % | 1/0 | no hop | 144 | 0.0/0.0 | 49.8 | 4 | 5 | 0–4 |
| w2000 | 40 MHz | 2.0 Mb/s | 31.3 % | 33.5/34.4 % | 5/2 | hop, ordered +0.8 s | 64 | 0.0/0.0 | 53.7 | 2 | 0 | 3–4 |
| w2500 | 40 MHz | 2.5 Mb/s | 39.1 % | 40.5/41.0 % | 5/1 | hop, ordered +1.0 s | 64 | 0.0/0.0 | 59.7 | 0 | 0 | 4–4 |

Striking against table 1: the 40 MHz/24 Mb/s-cap ladder's HT40 w1500
(23.5 % airtime) **never hops and shows 0.0 % pre-FEC loss**, while the
20 MHz ladder's p0750 at the same airtime hops at +2.1 s with 0.3 % loss.
The wider ladder has enough headroom to absorb the jam by sinking bitrate
alone — it never needs to lose a packet, so neither evidence term the
existing route can see (NHM occupancy or loss) ever fires for it. This is
the "controller trades quality for continuity" effect the design doc
names.

### 4. 40 MHz ladder at the 24 Mb/s cap, low-rate HT40 jammer (`runs40low/`)

Jammer airtime recomputed from its own frame/bitrate log (the "measured"
column — at these low rates the nominal-vs-delivered gap matters more
than at the higher ones above).

| trial | app rate | jammer airtime (measured) | NHM foreign med/max | blocked/occupied windows | existing-route result | ch end | pre-FEC loss med/max | fps min | AU drop | AU trunc | rung path |
|---|---|---|---|---|---|---|---|---|---|---|---|
| w0100 | 0.1 Mb/s | 1.6 % | 3.4/19.2 % | 0/0 | no hop | 64 | 0.0/0.1 | 45.0 | 4 | 1 | 3–4 |
| w0200 | 0.2 Mb/s | 3.1 % | 6.3/12.2 % | 0/0 | no hop | 64 | 0.0/0.0 | 50.0 | 4 | 2 | 1–3 |
| w0300 | 0.3 Mb/s | 4.7 % | 7.3/18.9 % | 0/0 | no hop | 64 | 0.0/0.0 | 50.0 | 2 | 7 | 0–2 |
| w0500 | 0.5 Mb/s | 7.9 % | 10.2/43.7 % | 1/0 | no hop | 64 | 0.0/0.0 | 50.0 | 0 | 7 | 0–2 |
| w1000 | 1.0 Mb/s | 15.6 % | 18.3/29.1 % | 0/0 | no hop | 64 | 0.0/0.0 | 29.7 | 1 | 9 | 0–3 |

Every one of these — up to 15.6 % measured airtime — never hops under
the existing route, and (unlike table 1's a025/a050/a075 on the narrower
ladder) shows negligible loss and drop too: on this ladder, at this
jammer level, the controller absorbs it almost for free. The damaging
sub-threshold band is width- and headroom-dependent, not a fixed airtime
number.

## Clean-air foreign distribution

Recomputed (`t7/harm60.py`, `t7/replay.py`) over two pools: the 60 s
capture with no jammer running (`control/t-ctl.jsonl`, 300 readings), and
the pooled pre-jam quiet segment of every trial above (238 s, 1211
readings) — 1511 readings total, ≈4 minutes:

| pool | n | median | p90 | p99 | max | ≥ 10 % | ≥ `detect_pct` (8 %) |
|---|---|---|---|---|---|---|---|
| control only | 300 | 1.5 % | 3.6 % | 8.7 % | 16.5 % | 3/300 | — |
| pooled (control + all pre-jam) | 1511 | 1.0 % | 3.3 % | 9.7 % | 30.2 % | 15/1511 | 25/1511 |

Single-window spikes above the floor happen in clean air (p99 is already
close to 8 %, and one window hit 30.2 %), but no ring of 5 ever carried
both a sustained median there *and* a confirming loss window — see the
rule replay below.

Comparing NHM's foreign reading against the interferer's own reported
airtime, the gap depends on whether the jammer's width matches the
ladder's tuned width. **Same-width trials** — table 1 (20 MHz jammer, 20
MHz ladder) and the HT40 rows of tables 3 and 4 (HT40 jammer, 40 MHz
ladder) — hold to **+1.4 to +4.3 points** (narrowest at table 3's w2500,
39.1 % reading 40.5 %; widest at a075's 23.8 % reading 28.1 %), matching
clean air's own ~1 % median plus noise. **Cross-width trials** — table 2
(HT40 jammer, 20 MHz ladder) and table 3's `p*` rows (20 MHz jammer,
40 MHz ladder) — run wider and less predictably, **+2.9 to +15.6
points** (table 2's w1500: 23.5 % reading 39.1 %, is the outlier; table
3's `p*` rows alone run +6.3 to +9.8). A ladder reading a jammer whose
width doesn't match its own tuned width is a less reliable NHM-foreign
measurement; the `detect_pct` floor (8.0) has enough margin over either
range to still separate clean air from jam, but a "NHM tracks real
airtime within a couple of points" claim only holds for the same-width
case — the common one, an interferer on the same channel plan as the
link.

## 60 s control vs. 60 s HT40-1M jam — the harm table

The `runs_re/w1000` repeat (same trial as table 4's row, 24 Mb/s-cap
40 MHz ladder, HT40 1 Mb/s jammer, measured ≈15.6 % airtime) run for 60 s
instead of 25, recomputed against the 60 s control capture:

| | control (no jam) | jam (HT40 1 Mb/s, ≈15.6 % airtime) |
|---|---|---|
| NHM foreign median/p90/p99/max | 1.5/3.6/8.7/16.5 % | 18.5/20.6/27.9/29.8 % |
| windows with `kEvImpaired` | 4/300 (1 %) | 72/289 (25 %) |
| windows with pre-FEC loss ≥ 1 % | 3/300 (1 %) | 57/289 (20 %) |
| fraction of windows at rung ≤ 1 | 0 % | 86 % |
| video bitrate, median | 26.6 Mb/s | 10.8 Mb/s |
| drone `txq_wait_ms` median/max | 4/5 ms | 1/51 ms |

The jammer's reported 15.6 % airtime (HT40, matching the ladder's own
40 MHz tuning — a same-width trial) reads 18.5 % median NHM foreign, a
+2.9 point gap, inside the +1.4–4.3 same-width range above. The ladder
spends 86 % of the minute at rung ≤ 1 and the video bitrate halves
(26.6 → 10.8 Mb/s), but drops stay near-zero and fps holds, because the
controller sinks quality instead of losing packets — the same effect as
table 3's w1500. The one `txq_wait` spike (to 51 ms, vs. a control max
of 5 ms) is not a sustained climb: it is a 1.8 s episode (t+32.7–34.5 s)
that coincides with a `promote_probed` re-climb (rung 2→3, confirmed at
t+32.9 s in the capture) cut short by the jam — carrier sense defers,
loss ticks to 2.2 %, an `s3_residual` demote lands at t+33.7 s and the
ladder settles back to rung 1 by t+34.7 s, and `txq_wait` falls back to
1–2 ms. The harm shows up as repeated short promote/demote episodes, not
a steady-state number.

## Chain of harm — a075 (0.75 Mb/s, 23.8 % jammer airtime, never hops)

a075 sits right at the edge the occupied route needs: NHM foreign median
28.1 %, max 32.1 % — close enough to `blocked_pct` (30 %) to cross it
occasionally (6 windows did), never for 5 in a row, so `occupied` never
latches and the link never hops. Sampled through the 25 s jam:

| t (s after on-air) | NHM foreign | pre-FEC loss | rung | fps | AU drop (cum.) | AU trunc (cum.) |
|---|---|---|---|---|---|---|
| −0.1 | 0.5 % | 0.0 % | 5 | 59.7 | 327 | 157 |
| +3.0 | 29.7 % | 17.5 % | 0 | 39.4 | 350 | 186 |
| +6.0 | 28.2 % | 19.7 % | 0 | 65.0 | 395 | 209 |
| +10.0 | 29.4 % | 20.8 % | 0 | 65.0 | 431 | 241 |
| +15.0 | 28.9 % | 17.4 % | 0 | 35.0 | 476 | 280 |
| +20.0 | 27.2 % | 20.0 % | 0 | 60.0 | 541 | 323 |
| +24.0 | 28.2 % | 19.5 % | 0 | 59.4 | 607 | 358 |

Within 3 s the ladder bottoms at rung 0 and **stays there for the
remaining 22 s**, with pre-FEC loss sustained at 17–21 % — more than 5x
the 3.0 % `loss_pct` threshold the detect route's confirm step uses, and
never transient — while fps bounces 35–65 and AU drops/truncations climb
steadily (280 drops, 201 truncations over the window). The capture's own
`link.ctl.last_event` names the cycle: a `promote_probed` climb to rung 5
just before on-air, then five `residual` demotes in 1.2 s (5→4→3→2→1→0,
one per onset window) — the onset cascade demotes under `residual`, not
`s3_residual` (that reason shows up later, once the controller has
re-climbed and settles into a steadier loss regime — the `runs_re`
episode above is an example of that path). None of this is visible to
the occupied route, because foreign airtime stays just under the blocked
wall. The finer-grained chain, from a090 (28.5 %, which does hop):
foreign jumps to 31–35 % within 0.15 s of the first interfering frame,
pre-FEC loss spikes to 10–18 % over the next 0.6 s as a `residual`
cascade demotes the ladder 4→1, the blocked streak reaches 5 at +0.95 s
and the hop orders immediately — landing costs only 1–3 AU drops,
confirming that *once it fires*, the existing route is cheap; the
problem is entirely how long it takes to fire, or whether it fires at
all, below the wall.

## Rule replay: a low floor confirmed by loss

Replayed datagram-by-datagram (`t7/replay.py`) over every raw capture
with the current defaults — a ring of the last `detect_windows` (5)
readings, firing when the ring is full, the current window has a
reading, the median of the ring's foreign readings is ≥ `detect_pct`
(8.0), and at least `detect_impaired` (1) of the ring's windows had
`pre_fec_loss × 100 > loss_pct` (3.0):

| trial | jammer airtime | first fire (s after on-air) | note |
|---|---|---|---|
| runs/a025 | 7.9 % | 1.0 | below the existing route's wall, never hops today |
| runs/a050 | 15.9 % | 0.5 | below the existing route's wall, never hops today |
| runs/a075 | 23.8 % | 0.5 | below the existing route's wall, never hops today |
| runs/a090 | 28.5 % | 0.5 | existing route also hops, +1.0 s |
| runs/a100 | 31.7 % | 0.4 | existing route also hops, +0.8 s |
| runs/a120 | 38.0 % | 0.3 | existing route also hops, +0.5 s |
| runs40/w1000 | 15.6 % | 0.6 | below the existing route's wall, never hops today |
| runs40/w1500 | 23.5 % | 0.5 | existing route also hops, but slower (+2.6 s) |
| runs40/w2000 | 31.3 % | 0.5 | existing route also hops, +0.9 s |
| runs40/w2500 | 39.1 % | 0.7 | existing route also hops, +0.9 s |
| runs40l/p0500 | 15.9 % | 0.7 | below the existing route's wall, never hops today |
| runs40l/p0750 | 23.8 % | 0.5 | existing route also hops, +2.1 s |
| runs40l/p0900 | 28.5 % | 0.5 | existing route also hops, +1.5 s |
| runs40l/p1200 | 38.0 % | 0.6 | existing route also hops, +1.0 s |
| runs40l/w1000 | 15.6 % | none in 25 s | 24 Mb/s-cap ladder, controller absorbs it with ~0 loss |
| runs40l/w1500 | 23.5 % | 9.0 | slow — the confirming loss window took most of the window |
| runs40l/w2000 | 31.3 % | 0.6 | existing route also hops, +0.8 s |
| runs40l/w2500 | 39.1 % | none in 25 s | existing route hops first (+1.0 s); channel changes before the ring fills |
| runs40low/w0100 | 1.6 % | none in 25 s | below `detect_pct`, as designed |
| runs40low/w0200 | 3.1 % | none in 25 s | below `detect_pct`, as designed |
| runs40low/w0300 | 4.7 % | none in 25 s | below `detect_pct`, as designed |
| runs40low/w0500 | 7.9 % | 0.7 | right at the floor — ring median crossed 8 % on noise |
| runs40low/w1000 | 15.6 % | none in 25 s | same case as `runs40l/w1000`, 24 Mb/s-cap ladder |
| runs_re/w1000 (60 s) | 15.6 % | 1.0 | 60 s repeat of the two `w1000` "none in 25 s" rows above |
| runs-pilot/a100 | 31.7 % | 0.8 | pilot trial, same rig, existing route also hops |
| control (60 s, no jammer) | 0 % | none (0/300) | — |

This is an approximation: the sideport's ~200 ms cadence is coarser than
the verdict's 150 ms window, so ring-fill and first-fire times are
offset from what the shipped code would see on the same jam.

- **Clean air: 0 fires.** Across the pooled 238 s of pre-jam quiet plus
  the 60 s control capture (≈5 minutes, 1511+ readings), the replayed
  rule never fired — comfortably clearing the design goal of "0 fires in
  two clean minutes."
- **At or above the floor, it mostly fires fast.** Every trial at
  ≥ 15.9 % measured/reported airtime fired within 0.3–1.0 s of on-air,
  except the three 24 Mb/s-cap-ladder exceptions below — including
  several below the existing route's wall that never hop today (a025 at
  7.9 %: 1.0 s; a050 at 15.9 %: 0.5 s; a075 at 23.8 %: 0.5 s; p0500 at
  15.9 %: 0.7 s).
- **Right at the floor, timing is inconsistent.** w0500 (≈7.9 %
  measured, just under `detect_pct`) fired once at 0.7 s — the ring's
  median can cross 8 % on noise even when the session average sits just
  under it. w0100–w0300 (1.6–4.7 %) never fired, as designed.
- **On the headroom-rich 40 MHz/24 Mb/s-cap ladder, confirmation can lag
  or miss within a short window.** `runs40l`'s w1000 (15.6 %) and w2500
  (39.1 %) and `runs40low`'s w1000 (also 15.6 %) never fired within
  their 25 s windows; `runs40l`'s w1500 (23.5 %) did fire, but slowly, at
  +9.0 s — more than 10x the floor-trial latency above. (w2500's own
  `occupied` route hops at +1.0 s — before its ring can fill — so the
  replay never gets a chance to fire there; immaterial, since the
  existing route already handles that case.) The 60 s repeat of the
  w1000 case (`runs_re/w1000`) did fire, at +1.0 s. Because this ladder
  can absorb jam by sinking bitrate alone (tables 3 and the harm
  comparison above), a confirming loss window is not guaranteed to land
  quickly — the detect route's latency here depends on when the
  controller's own quality-for-continuity trade happens to let a loss
  window through, and that varied run to run on the same nominal jam
  level.

## Conclusion

- The existing `occupied ∧ blocked` route's wall sits at **≈25–30 % raw
  interferer airtime** and, once crossed, orders a hop in 0.5–1.0 s at a
  cost of 1–10 AU drops.
- **The band below that wall is the worst place to be**, not a safe one:
  on a narrow ladder (table 1), sub-threshold jam drives continuous
  double-digit pre-FEC loss and hundreds of AU drops/truncations for as
  long as the jam runs, invisible to the existing route the whole time.
  On a wide ladder with bitrate headroom (tables 3–4, the harm
  comparison), the same or higher airtime instead collapses the bitrate
  and pins the rung low while hiding the harm from every loss-based
  signal — a different failure shape, equally invisible to `occupied`.
- NHM foreign airtime tracks the interferer's real airtime to within
  about +1.4 to +4.3 points, all the way down to ~1.6 % measured, when
  the jammer's width matches the ladder's tuned width (the common case);
  when the two widths differ the gap runs wider and less predictably,
  +2.9 to +15.6 points. Against a clean-air baseline of ~1 % median,
  ~3–4 % p90, and p99 under 10 %, the signal has enough headroom under a
  low floor (8 %) to be useful either way.
- A low-floor-plus-loss-confirmation rule, replayed against these same
  captures, would hop in well under 2 s for most of the damaging
  sub-threshold cases above while firing on none of ~5 clean minutes —
  but its timing is not uniform: it is fast when loss shows up promptly
  and can lag past a short observation window when the ladder's own
  quality trade defers the confirming loss window, which the ring design
  (several windows, not one) is built to tolerate rather than eliminate.
