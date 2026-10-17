# What GOP frame count does the SDK take for "no periodic refresh", and does a bitrate step still settle without one?

**Status:** bench record. `venc.gop_s = 0` (no periodic refresh, GOP =
`VENC_GOP_NONE` in `drone/venc/venc_cfg.h`) and the GOP caveat it tests
are described in `docs/airtime-model.md` ("Why the inputs are fixed",
"GOP is not a free knob"). The 10 s GOP run this compares with is
`docs/findings/2026-10-17-long-gop-spike.md`.

## Question

`venc.gop_s = 0` asks for no periodic refresh, but the star6e rate
attribute takes a GOP as a frame count, so "never" has to be the largest
count the SDK accepts. What is that count, and with no periodic refresh
at all does a bitrate step still take effect and settle, and does the
ladder still climb as it did at a 10 s GOP?

## Setup

Bench drone (IMX415 1080p60, H.265 CBR, 4 row slices, SVC-T) and bench
Radxa GS, rung 4, static daytime indoor scene, GS session 0135. maburd
built from branch `demand-refresh` (`gop_s = 0` maps to `VENC_GOP_NONE`;
the IDR pacer defers a refused chain-break IDR). The drone's
`/etc/mabur.toml` had only `venc.gop_s` 0.5 → 0 and
`venc.intra_refresh_frames` 1 → 0 changed (no GDR: with no GOP there is
no refresh period for the stripe to sweep).

- GOP ceiling: `VENC_GOP_NONE` set to 65535, then 216000 (one hour at
  60 fps), one rebuild and deploy each. Accepted means the channel starts
  (no `CreateChn`/`StartRecvPic` error in `/tmp/mabur.log`), video
  reaches the GS, and `/proc/mi_modules/mi_venc/mi_venc0` reports the
  count in the channel's `GOP` column.
- Ladder climb: `ctl.log` E rows after a maburd restart, twice at
  65535; the drone's power-on boot on the previous binary (`gop_s` 0.5)
  in the same session is the baseline.
- Bitrate step: the debug verb set the encoder to 7 Mbit/s and, 30 s
  later, to 22 Mbit/s, 34 s after the top rung; RcAgent re-asserts its
  own rate within 5 s, so each is a step away and a step back. `au.log`
  (`aulog 6`) gives per second the delivered rate (sum of `len`), the
  TxQueue wait `dq` maximum, and the parameter-set AUs (`nal0` 32), which
  is where an IDR shows.
- Verb paths that re-apply the GOP: the bitrate verb (`GetChnAttr` /
  `SetChnAttr` keeps the GOP the SDK holds) and the fps verb (30 then
  back to 60; `apply_fps` writes the GOP afresh).

## Result

**65535 is accepted; 216000 is refused, and not by `CreateChn`.** At
65535 the channel comes up and streams, and the MI proc node reads
`CBR65535` for channel 0. At 216000 `MI_VENC_CreateChn` returns 0 and the
refusal comes one call later: `MI_VENC_StartRecvPic failed -1610473469`,
with the SDK's own reason in the kernel log,
`[ven-e][wrap] Cbr invalid parameter, gop:216000, bit rate:1024000`.
maburd exits on it and the wrapper respawns it every 2 s, so the bad
count means no video at all, not a clamped GOP. No value between 65535
and 216000 was tried. Both the bitrate verb and the fps verb re-apply the
GOP without error at 65535 (`{"ok":true}`, `> FPS delivered 30, RC fpsNum
30, gop 65535 (bind 90:30)`, then 60 back, proc still `CBR65535`).

**No IDR arrives on a timer within the run.** From the top rung
(446.0 s) to the first fps-verb test (619.9 s), 173 s, the only
parameter-set AUs are the four at the two writes and their re-asserts.
The SDK still holds a GOP, though: 65535 frames is about 18 min at
60 fps, so a stream that goes that long with no bitrate change, chain
break or GS request still gets one GOP-start picture at the cap
interval. No run here lasted 18 min without another IDR restarting the
GOP, so that residual refresh was not observed. Every IDR is 7.3 to
7.6 kB on the base layer (`min_iqp` 34 holds it well below the ~23 kB
base P frames), with `dq` 0 or 1 ms.

**The step takes effect within the same second and settles.** Seconds
around the writes (session mono seconds; the 7 Mbit/s write landed at
480.3 to 480.5 s, the 22 Mbit/s write at 510.7 to 510.9 s):

| s | kbit/s | `dq` max ms | IDR AUs | largest AU B |
|---|---|---|---|---|
| 476 | 10297 | 1 | 0 | 22895 |
| 477 | 10452 | 1 | 0 | 23774 |
| 478 | 10435 | 1 | 0 | 23600 |
| 479 | 10298 | 1 | 0 | 22725 |
| 480 | 5656 | 1 | 1 | 22356 |
| 481 | 5865 | 1 | 1 | 18072 |
| 482 | 10686 | 1 | 0 | 30640 |
| 483 | 10480 | 1 | 0 | 23587 |
| 484 | 10378 | 1 | 0 | 23911 |
| 485 | 10380 | 1 | 0 | 23372 |
| … 486 to 509 | 10214 to 10590 | 1 | 0 | ≤ 25668 |
| 510 | 8582 | 1 | 1 | 22651 |
| 511 | 10763 | 1 | 0 | 29042 |
| 512 | 9049 | 1 | 1 | 29679 |
| 513 | 10536 | 1 | 0 | 23275 |
| 514 | 10509 | 1 | 0 | 23845 |
| … 515 to 602 | 10097 to 10626 | 1 | 0 | ≤ 25141 |

The 7 Mbit/s write cut the delivered rate from 10.3 to 5.7 Mbit/s in the
same second, RcAgent's re-assert (481.8 s, its own IDR) restored 10.7 in
the next, and the rate sat at 10.2 to 10.6 Mbit/s for the following
28 s. The 22 Mbit/s write did not raise the delivered rate (the scene
does not fill it, as in the 10 s GOP run); the write and its re-assert
each dipped one second (8.6, 9.0 Mbit/s). `dq` max was 1 ms in every
second from 455 to 602 s, the steps included. In the half second after
each upward step (the 481.8 s re-assert, the 22 Mbit/s write, its
re-assert) a few base frames reach 29 to 31 kB, the largest in the
window, against ≤ 25.7 kB otherwise.

**The climb is unchanged.** Rung 0 → 4 on four `promote_probed` steps
in every run, each promote carrying its bitrate-change IDR:

| run | `gop_s` | first → last promote (session mono ms) | 0 → 4 |
|---|---|---|---|
| drone power-on, previous binary | 0.5 | 121240 → 126482 | 5.24 s |
| maburd restart 1, GOP 65535 | 0 | 185508 → 190769 | 5.26 s |
| maburd restart 2, GOP 65535 | 0 | 440810 → 446038 | 5.23 s |

No demote after restart 2 in the 243 s to the end of the run (the
`starved` rows in the session are the two deliberate maburd stops).
Restart 1 was stopped 34 s after the top to try 216000.

## Answer

`VENC_GOP_NONE` is 65535 frames, 18 min at 60 fps: the largest count
tried that the star6e SDK takes. 216000 is refused, and the refusal is
at `MI_VENC_StartRecvPic`, not `CreateChn`, so a bad count kills the
channel rather than being clamped. The GOP the SDK takes is below an
hour (one hour, 216000, is refused), so "no periodic refresh" is really
one GOP-start picture per 65535 frames (about 18 min at 60 fps) whenever
nothing else restarts the GOP first; that residual refresh is left as it
is, not worked around, and was not observed in these runs. Short of that
interval the stream carries an IDR only when the encoder attributes
change: a bitrate step takes effect within the same second, settles in
the next, keeps `dq` at 1 ms or less, and the ladder climbs 0 → 4 in
5.2 s as it did at 0.5 s and 10 s GOPs. What this does not exercise: the
chain-break IDR path (clean bench, no losses), demote/promote under real
fades, and the 18 min residual GOP-start picture itself.
