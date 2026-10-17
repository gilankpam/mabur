# Can the refresh-start (GDR) picture be split into row slices?

**Status:** closed on both ends; the slice rule and the GS decoder's
slice-granular streaming are recorded in `docs/slices.md` ("Known
limits", both parts), the dead `intra_refresh_qp` in
`drone/venc/venc_cfg.h` and `docs/airtime-model.md`. The refresh-start
picture stays one slice at its full size, decoded whole, and
`intra_refresh_frames` stays 1.

## Question

`docs/findings/2026-10-15-regulator-chain-cuts.md` measured the
refresh-start picture as most of the ready-time tail: +5 ms encode, +2.7 ms
arrival, +2.1 ms decode — the last because it is one slice, so it neither
streams into the decoder nor salvages. The 2026-10-10 config sweep
(`docs/findings/2026-10-09-venc-slice.md`, "Can the refresh-start picture
be split?") saw it split only when the stripe was spread over several
frames, which did not heal a lost base frame. This spike asks what rule the
SDK applies, why the spread stripe does not heal, and whether any config
(GOP length, `ref_enhance`, GDR off) gets around it.

## Setup

Bench drone (maburd md5 `fdded764…`, `/etc/mabur.toml` `[venc]`:
1920x1080 @ 60, `gop_s 0.5`, `slices 4`, `ref_base 1`, `ref_enhance 1`,
`ref_pred true`, `intra_refresh_qp 40`, `min_iqp 34`, `max_ipprop 2`),
static scene, GS Radxa at rung mcs4/40. One 6 s capture (360 AUs) of the GS
AU ring per case with `tools/slices/gdrcap.py`; analysis on the host with
`tools/slices/gdrheal.py` (NAL/slice count per AU from the bitstream;
healing = drop one base P mid-GOP from the captured stream, decode both
streams with ffmpeg, and per later frame count the 32-px rows whose luma
differs from the full decode). Each case is a config edit and a maburd
restart; the pre-spike config was restored at the end.

## The refresh picture's slices follow the stripe

| `intra_refresh_frames` | stripe rows (of 34) | refresh-start picture | its size p50 | AU size on the frames after it (kB) | `slice_mismatch` |
|---|---|---|---|---|---|
| 1 (shipped) | 34 | 1 P slice | 79.5 kB | 28, 41, 25 … (P level ≈ 33 base / 29 enh) | 0 |
| 2 | 17 | 2 P slices | 52.1 | **58**, 33, 36 … | every refresh start |
| 3 | 12 | 3 P slices | 41.8 | **52, 49**, 38 … | every refresh start |
| 4 | 9 | 4 P slices | 40.0 | **48, 47, 44**, 38 … | 0 (count happens to match) |
| 5 | 7 | 5 P slices | 38.0 | **40, 46, 47, 41**, 37 … | every refresh start |
| 6 | 6 | 6 P slices | 35.6 | **36, 41, 43, 43, 43**, 37 … | every refresh start |
| 0 (GDR off) | — | IDR, 4 I slices | 5.4 | 0.9, 4.2, 6.3, 14, 17, then 39, 50, 51, 40 … | 0 |

- The refresh-start picture carries exactly F slices, one per stripe band,
  whatever the configured split (5 and 6 slices on a channel split 4 ways).
  The SDK cuts that picture at its stripe bands and does not apply
  `SetH265SliceSplit` to it at all. With F = 1 the band is the whole
  picture, so it is one slice by construction; no config value produces a
  sliced one-frame refresh.
- The stripe runs on frames 0 … F−1 after the refresh start, once per GOP
  (the bold sizes), not on every frame: outside that window every AU sits
  at P level.
- A refresh-start picture whose slice count differs from `slices` is
  stamped `slice_rows = 0` by maburd and counted `slice_mismatch`, so it
  passes through on the GS exactly like today's one-slice picture.
- GDR off: the IDR is sliced 4 ways by the normal split and heals in one
  frame, but the rate control answers it with a five-frame crater — a 5 kB
  IDR, then 0.9 / 4 / 6 / 14 / 17 kB P frames, then a 50 kB bounce — a
  visible quality hole every 0.5 s. `min_iqp` / `max_ipprop` are the knobs
  on that, not something this spike tuned; as shipped it is not a
  substitute.

## Why a spread stripe never heals under SVC-T

Rows wrong after one dropped base P (the 5th base after a refresh start),
from the next refresh start on; rows of 32 px, 34 per picture:

| case | at the refresh start | one frame later | steady state until the drop is forgotten |
|---|---|---|---|
| F = 1 | 0 | 0 | 0 |
| F = 2 | 17 (rows 17–33) | 3 (the enhance frame's own stripe) | 17 |
| F = 3 | 23 | 15 | 20 (rows 11–30) |
| F = 4 | 24 | 15 | 17 (rows 9–20, 29–33) |
| F = 5 | 26 | 21 | 20 |
| F = 2, GOP 31 frames | 17 | 3 | 17 |
| F = 2, `ref_enhance = 2` | 17 | 3 | 17 |
| F = 2, `ref_base = 2` / F = 3, `ref_base = 3` | 17 / 22 | 3 / 19 | 17 / 14 |
| F = 2, `ref_base = 2`, `ref_enhance = 2` | 17 | 3 | 17 |
| GDR off (IDR) | 0 | 0 | 0 |

- Stripe k lands on frame k after the refresh start. Frame 0 is always a
  true base; under `ref_enhance = 1` frame 1 is a non-referenced enhance
  frame, so its stripe is in the picture for one frame (the "one frame
  later" column) and the next base, which references frame 0, is wrong
  there again. With F = 2 the bottom half of the picture is therefore never
  refreshed in a referenced frame; with F = 3 and 5 the middle and bottom
  bands; with F = 4 bands 1 and 3. None of these ever converge.
- An odd GOP does not shift the layer parity: with `gop_s` 0.5167 (31
  frames) the refresh start is still a base frame every 31 frames — the
  SVC-T pattern restarts at every GOP start (`gop_s` cannot go below 0.5,
  so 29 was not available).
- `ref_enhance = 2` makes the pattern B E E where the first E is
  referenced only by the second (maburd stamps it sid 0, since only the
  last is `NOTFORREF`): dropping it corrupts one frame. Stripe 1 lands on
  that frame, which the next base does not reference either, so it is
  wasted the same way.
- `ref_base` (`u32Base`) 2 or 3 leaves the layer pattern exactly as 1 does
  (sid 0 1 0 1 …, the same heal maps): it does not make the frames after
  the refresh start base frames.
- The sweep only ever refreshes the top band in a referenced frame, so the
  whole-picture one-frame stripe (F = 1) is the only sweep that heals a
  lost base at all. That is what ships, and it is one slice.

## Verdict

The SDK will not slice the one-frame refresh picture, and every spread
stripe is structurally unable to heal under temporal layering. Keep
`intra_refresh_frames = 1`; the refresh-start picture stays one slice and
passes through on the GS.

## The SDK has no other verb, and call order does not matter

`libmi_venc.so` on the drone (md5 `6fc7fe48…`, identical to the copy the
venc port was built against) exports 77 `MI_VENC_*` symbols; the only
ones that touch slicing or refresh are `Set/GetH264SliceSplit`,
`Set/GetH265SliceSplit` (`{bSplitEnable, u32SliceRowCount}`) and
`Set/GetIntraRefresh` (`{bEnable, u32RefreshLineNum, u32ReqIQp}`). There
is no per-frame slice verb, no GDR verb and no slice-mode flag in
userspace; the library is a 34 kB ioctl shim and the combination logic
sits in `mi_venc.ko` and the encoder firmware (the module's only
slice-related strings are the ring-mode exclusion, "chn%d is slice mode,
can not set Ringmode", and the slice-done IRQ counter).

A hack maburd (not committed) then tried every ordering of the two calls
around `StartRecvPic`, F = 1, `slices = 4`:

| order | refresh-start picture | P pictures |
|---|---|---|
| shipped: split pre-Start, refresh post-Start | 1 slice, 79 kB | 4 |
| refresh pre-Start, then split pre-Start | 1 slice, 75 kB | 4 |
| shipped, then split re-applied after Start + refresh (SDK returns 0) | 1 slice, 78 kB | 4 |
| no pre-Start split; split applied only after Start + refresh (returns 0) | 1 slice, 75 kB | 4 |

The split is accepted at any point (post-Start too; that path also made
the startup vanish counter jump, 417 vanished / 56 `self_idr_refused` in
the 15 s run, so a live re-apply is not free) and never reaches the
refresh picture. Its slicing is the encoder's own rule for refresh
pictures, not a precedence between the two calls.

## Can the refresh picture be made smaller instead?

Same bench, F = 1, one capture per config edit from the restored config.
`enc_us` is the drone's own encode stamp in the AU meta (SDK pts → AU
out); PSNR is luma, each refresh picture against the P picture just before
it on the static scene (frame-to-frame floor 51.2 dB), over 10–11 refresh
starts.

| knob | refresh-start size p50 | enc_us refresh / P | frames 1, 2 after | PSNR f0 |
|---|---|---|---|---|
| `intra_refresh_qp` 36 / 40 / 44 / 48 / 51 | 78.1 / 77.9 / 78.0 / 77.8 / 75.3 kB | 8.5 / 7.5 ms (all) | 30, 37 kB (all) | 48.0 (all) |
| `min_iqp` 34 → 46 | 74.8 kB | 9.0 / 8.0 | 23, 43 | 48.0 |
| `max_ipprop` 2 → 1 | 77.9 kB | 8.5 / 7.5 | 30, 39 | — |
| `qp_delta` 4 → 12 | 77.8 kB | 8.5 / 7.5 | 30, 37 | — |
| `superframe_p_pct` 200 → 130 | **40.5 kB** | **12.8** / 7.5 | 37, **50** | 47.7 |

- **`intra_refresh_qp` (`u32ReqIQp`) is a no-op** on star6e CBR: 36 to 51
  change nothing — size, encode time or PSNR. The stripe is coded at the
  rate control's QP; the value is passed through as asked but does not
  reach the picture. The bundle's 36 and the bench's 40 are the same
  stream.
- The rate control treats the all-intra refresh picture as a **P**
  frame: the I-frame knobs (`min_iqp`, `max_ipprop`, `qp_delta` =
  `s32IPQPDelta`) do not catch it (the 4 % at `min_iqp` 46 is within the
  bitrate drift between captures: the ladder moved 14.1–24 Mbit/s across
  the runs, and `max_ipprop` 1 would have halved a real I frame).
- The only knob that catches it is the P-frame superframe cap
  (`MI_VENC_SetSuperFrameCfg` REENCODE): at 130 % of the per-frame budget
  the refresh picture comes out at 40 kB, 1.2× a P. The price: the
  re-encode adds 4.3 ms to that frame's encode (12.8 vs 8.5 ms enc_us) —
  about what the smaller AU saves on the air at this rung (the regulator
  record's +2.7 ms arrival span was for +20 kB) — and the deferred intra
  cost lands on frame 2, the next base, at 50 kB instead of 37. PSNR of
  the capped picture is 0.3 dB under the uncapped one on this
  low-light static scene; a real scene would pay more, unmeasured. Not a
  latency win: it moves the spike's bytes into encode time and into the
  frame after next.

The one encoder-side arm left untested is `ref_base = 0` (flat P chain):
every frame is referenced, so a spread stripe heals in F frames and the
refresh picture is F slices. The 2026-10-09 record already names this; it
drops the base/enhance layering that UEP and the enhance-drop policy are
built on. Otherwise the lever for the refresh picture's +9 ms is on the
GS side (whole-picture decode and no salvage are what it costs), not the
encoder.

## Can the GS decode the one-slice picture as it arrives?

The GS's own share of the refresh picture's +9 ms is its +2.1 ms decode:
one slice, so no streamed decode. rkvdec2's stream mode hands the decoder
a picture part by part — the hal pads every part with zeros and the
kernel feeds the next `(addr, len)` when the decoder pauses on
`buf_empty` — and the question was whether a *single slice* can be handed
over in byte parts, with the decoder resuming mid-CABAC.

Harness: a copy of `mpi_dec_stream_test` with a byte-chunk mode for
one-slice pictures and a hal built with three experiment flags (no
padding on a continuation part; "in place" = the bytes are already in
the stream buffer, hand them over without copying; "hold" = copy a part
contiguously but hand it over only once the next one is in place, a
one-part lag). Input: 150 pictures from the F = 1 capture, five one-slice
refresh pictures (74–75 kB) among 4-slice P pictures. Each run compared
per frame against the whole-picture decode; `dmesg` reset count per run;
latency = last part handed over → frame out, for the one-slice pictures.

| variant | parts | spread | one-slice latency | bit-exact | resets |
|---|---|---|---|---|---|
| whole | — | — | 12.3 ms | reference | 0 |
| streamed, one-slice pictures whole (as shipped) | — | 5 ms | 11.9 (4-slice pictures 6.2) | yes | 0 |
| raw byte parts, no padding | 4 / 8 / 16 | 5 ms | 6.3 / 5.9 / 5.3 | **no, and never the same twice** | 5 (one per picture) |
| in place: whole slice pre-written, `str_len` = part 0, parts handed over | 4 / 16 | 5 ms | 5.7 / 4.75 | yes | 0 |
| in place | 4 / 16 | **40 ms** | **−20 / −25 ms** | yes | 0 |
| 4-slice pictures streamed | 4 | 40 ms | +2.3 | yes | 0 |
| one-part lag (contiguous, part k given once k+1 is in place) | 4 / 16 / 64 | 5 ms | 5.8 / 4.9 / 1.5 | **no** | 5 |
| one-part lag | 16 | 40 ms | −25 | no | 5 |

- The in-place rows are the proof, and they cut against it: with the
  whole slice already in memory the frame came out 20–25 ms *before* the
  last part was handed over. The decoder ignores a `str_len` that ends
  inside a slice and reads on to the slice's end; "bit-exact" there only
  means it decoded the whole buffer. (A 4-slice picture on the same 40 ms
  spread finishes 2.3 ms after its last slice: slice-granular streaming
  is real.)
- With the bytes genuinely absent past a part's end (raw and lag rows)
  the decoder runs into whatever follows, corrupts the picture, errors
  and resets — every chunked picture, every variant, every spread. The
  read-ahead is not a prefetch depth that a one-part lag covers: the
  decoder does not pause mid-slice at all, only at a slice end.
- Verdict: rkvdec2's stream mode is slice-granular. A one-slice picture
  cannot be decoded as it arrives, so the refresh picture's decode stays
  whole-picture on this GS. The experiment hal and test were scratch
  (the staged MPP tree was restored and rebuilt clean).

## Harness

- `tools/slices/gdrcap.py` (on the GS): per-AU capture with the ring meta
  kept — `ausniff --dump-annexb` loses the AU boundaries, sid and slice
  count.
- `tools/slices/gdrheal.py summary|heal` (host): slice count per AU from
  the bitstream, the size-by-frames-since-refresh trace, and the drop-one-
  base row maps above. ffmpeg needs `-flags2 +showall -flags
  +output_corrupt -err_detect ignore_err` to decode from a VPS-led P
  picture (it emits nothing otherwise); the first picture's intra rows are
  exact even with synthesized references.
