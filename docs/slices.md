# H.265 row slices and slice salvage (as built)

Spec: `docs/superpowers/specs/2026-10-10-h265-slices-design.md` (gitignored).
Evidence: `docs/venc-slice-findings-2026-10-09.md`. Part 2 (streamed decode)
is not built yet.

## Drone

- `[venc] slices` (default 1 = off): row slices on the link channel, applied
  with `MI_VENC_SetH265SliceSplit` before StartRecvPic; only counts the SDK's
  whole-64-px-CTU-row geometry reproduces boot (1080p: 1,2,3,4,5,6,9,17).
- FrameHdr byte 3 = `slice_rows` (64-px CTU rows per slice of this AU, 0 =
  one slice). maburd stamps it only when the AU carries the expected slice
  count; a one-slice AU carrying VPS/SPS/PPS (refresh start, IDR — the SDK
  leaves those whole) sends 0; anything else sends 0 and counts
  `slice_mismatch` (5 s `frame_ring` line). That includes a bare one-slice
  AU without parameter sets: the SDK dropped the split (e.g. after a
  runtime `MI_VENC_SetChnAttr`), and salvage must not switch off silently.
  Wire flag day — byte 3 was an always-H.265 codec id before 2026-10-10.
- maburd stamps the geometry from the configured `[venc] size` height, the
  encoder splits the ENCODED height (clamped/auto-sized). Channel start
  fails (`[venc] ERROR: slices=…`) when the two give a different 64-px row
  count or rows per slice.
- TRAIL_N rewrite covers every slice of a split picture:
  `star6e_patch_pack_to_trail_n` patches every NAL inside each packetInfo
  entry (a by-frame split pack puts all slices in one entry), and an entry
  that begins directly at its NAL header, with no start code, still gets
  that first header patched (`h26x_util_hevc_patch_entry_trail_r_to_n`).

## GS core (maburgs and the web GS)

- `ParamTracker` keeps the SPS/PPS of complete parameter-set AUs.
- FrameStream runs a `SliceAssembler` per split AU: complete slices go out
  in order; when FrameStream gives up on a hole (`gap_ms` / lookahead,
  unchanged, so FEC and NACK get their full chance) each lost slice becomes
  a `make_skip_slice()` fill (template header with address set and SAO off,
  every CU skip/merge_idx 0). The picture is then legal and gap-free: no
  decoder error, no rkvdec reset.
- A slice is complete only if its start code and every byte up to the next
  start code (or the known AU end) lie in one run of consecutive fragments.
  A 3-byte start code that opens a run right after a hole is never trusted:
  it could be the tail of a 4-byte code whose leading zero was lost. That is
  spec-mandated conservatism, not a correctness need — the slice behind a
  "00 | 00 00 01" split is in fact intact — and it costs about 1/F of the
  slices next to a hole (F = fragment size), which are filled instead.
- Non-VCL NALs that follow the first slice (e.g. a suffix SEI) are dropped
  from salvaged output; only the run-0 prefix's leading non-VCL NALs are
  kept.
- Salvage runs only at a 64-px CTB (`slice_rows` counts 64-px CTU rows; at
  another CTB size a fill could overlap real slices): any other CTB size is
  `unsupported` passthrough.
- Passthrough (today's truncated prefix) when: unsplit AU, no parameter
  sets yet, unsupported stream (incl. a non-64-px CTB), I-slice picture,
  no complete slice to copy a header from, or slice addresses that don't
  match `slice_rows`. A
  contiguous-prefix fragment shorter than the first fragment's size, or
  losing fragment 0 itself (the fragment size is unknown), both fall under
  this geometry fallback. Each reason is a counter.

## Consumers

- AU ring flag `kRecFlagSliceSalvaged` (0x40). maburplay and the web GS
  decode salvaged base and enhance AUs; only a complete AU arms a decoder.
  maburplay's raw DVR records salvaged AUs too (`RawDvr::feed`'s
  `complete` argument is `au_decodable()` there, i.e. complete OR
  salvaged); maburplay's oneshot JSON gained `salvaged_base`/
  `salvaged_enhance` counters.

## Observability

- Sideport `link.video.slice_salvaged / slices_kept / slices_filled /
  slices_after_hole / slice_fallback{...}`; maburtop `salv`; flightreport
  SLICE SALVAGE; `au.log` aulog 5 (+ `slices kept filled after_hole`);
  ausniff `salvaged`.
- Web GS STATS gains `salvaged` (web stat `aus_salvaged` → JSON key
  `salvaged`): the count of AUs the core rebuilt by slice salvage. The
  JS Gate decodes a salvaged AU once armed but never arms on one.

## Known limits

- Refresh-start pictures stay one slice (SDK); damaged ones pass through.
- A damaged IDR passes through (`islice`).
- The web GS's local/replay recorder (`RawDvr` in `web/src/web_main.cpp`)
  still records complete AUs only — its `on_au` callback feeds `a.complete`
  (not the salvaged-inclusive flag maburplay's raw DVR feeds), unlike
  maburplay's raw DVR, which records salvaged AUs too.
- Fill bitstreams are checked by `tools/slices/slicefill_check.py`
  (ffmpeg oracle) against the bench drone's `cap4` capture
  (`tools/slices/make_slice_fixture.py`'s docstring:
  `sbc-groundstations-gilankpam/output/gs-p1-backup-2026-10-09/cap4.h265`),
  3 pictures × 4 fill modes = 12 cases; rerun it after touching
  `common/src/hevc_*`.

## Bench 2026-10-10

Bench drone (1080p60, 8812EU) + GS restored to the stock CI image (p1
`p1.orig.img`, kernel Sep 26; the rkvdec2 stream-mode spike kernel is off
it). Both ends at 6c27e74, live adaptive GS config.

- **No loss, slices = 4:** maburd logs `[venc] slices: chn=0 4 slices of
  5 CTU rows`; `slice_mismatch=0`, `enhance_disagree=0` through the ladder
  climb to mcs4. ausniff 30 s: 60.5 fps, every AU complete. aucadence
  offset +0.26 ms (gate ±4.0). A ring dump: 189 pictures of 4 slices, 7 of
  1 (refresh starts). au.log is `aulog 5` with `slices 4 kept 4`.
  slices = 1 baseline: 60.5 fps, `slice_mismatch=0`.
- **Loss-sim A/B, NACK on** (`MABUR_LOSS_SIM` maburgs on the live GS
  config, `[link.nack] enable = true`: ~500 requests / ~3 600 symbols filled
  per arm; `s0`+`s1 eff=1.5 burst=4`, 5 min per arm). Salvage acts on the
  truncations NACK could not repair in time:

  | | slices = 4 | slices = 1 |
  |---|---|---|
  | truncated | 258 | 275 |
  | slice_salvaged | 249 (96.5 %) | 0 |
  | fallback | no_template 9 | — |
  | slices kept / filled / after_hole | 491 / 505 / 216 | — |
  | ausniff 30 s window: incomplete / salvaged | 3 / 15 | 24 / 0 |
  | rkvdec `resetting` (dmesg) | 0 | 0 |
  | maburplay e2e p50/p99 ms | 48/66, 34/83 | 46/79, 47/81 |
  | drone cpu_pct p50 | 23.0 | 22.9 |

  The 9 `no_template` AUs lost every slice. `slice_mismatch` stayed 0.
- **Loss-sim A/B, NACK off** (same rig, a config copy with
  `[link.nack] enable = false`, sideport `link.nack` null):

  | | slices = 4 | slices = 1 |
  |---|---|---|
  | truncated | 200 | 200 |
  | slice_salvaged | 189 (94.5 %) | 0 |
  | fallback | no_template 4 | — |
  | ausniff 30 s window: incomplete / salvaged | 0 / 11 | 22 (14 base) / 0 |
  | dropped (never emitted) | 303 | 304 |
  | rkvdec `resetting` (dmesg) | 0 | 0 |

  The 7 truncated AUs neither salvaged nor counted as a fallback are
  unsplit refresh starts (`slice_rows` 0). Across the two A/Bs, NACK turns
  some would-be drops into truncations (dropped ~210 vs ~300) rather than
  cutting the total of damaged AUs much at this loss pattern; the runs were
  minutes apart on an adaptive ladder, so treat that as indicative.
- **Decoder legality of real salvaged AUs:** 60 s ring dump under the same
  loss (complete + 59 salvaged AUs, in ring order) through ffmpeg:
  3 584 frames, zero slice-data / CABAC errors; the only errors are a
  missing-ref chain after dropped AUs (fid gaps) and the dump's mid-GOP
  start. MPP on the GS decoded both arms with no decoder reset.
- **Visual check with motion** (operator waving a hand in front of the
  camera, 60 s, same loss; every complete + salvaged AU captured off the
  ring and decoded with libavcodec one packet per AU, 0 decode errors,
  39 salvaged: 36 enhance, 3 base):
  - The salvaged picture itself: each filled band is a slightly stale
    copy of the previous frame, seamless with the kept slices. No garbage,
    no blocks.
  - Enhance fills (non-reference, TRAIL_N) never propagate: the next
    frames are clean.
  - Base fills are references, so later pictures predict moving content
    from the stale band. Blocky drift appears inside that band only, and
    it clears at the next refresh-start picture (one refresh period,
    ≤ 0.5 s; 67 ms and 200 ms in the two decodable cases). On a static
    scene it is invisible.
  - The missing-middle-slice "slice-3 anomaly" of the findings doc did
    not appear (the decoder never sees a gap).
- **Not done yet:** the web GS with a USB card (Step 6) needs an
  operator; the bundle stays at `slices = 1` until it passes. The bench drone's own
  `/etc/mabur.toml` is at `slices = 4` (`.pre-slice` backups of binary
  and config on both devices).
