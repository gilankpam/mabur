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
  count; refresh-start pictures (one slice) send 0; anything else sends 0
  and counts `slice_mismatch` (5 s `frame_ring` line). Wire flag day — byte
  3 was an always-H.265 codec id before 2026-10-10.
- TRAIL_N rewrite covers every slice of a split picture (`star6e_patch_pack_to_trail_n`
  runs over every packet in the stream, not just the first).

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
  A 3-byte start code that opens a run right after a hole is never trusted
  (it could be the tail of a 4-byte code whose leading zero was lost) — that
  slice is treated as missing and filled, conservatively, rather than risking
  the wrong bytes.
- Non-VCL NALs that follow the first slice (e.g. a suffix SEI) are dropped
  from salvaged output; only the run-0 prefix's leading non-VCL NALs are
  kept.
- Passthrough (today's truncated prefix) when: unsplit AU, no parameter
  sets yet, unsupported stream, I-slice picture, no complete slice to copy
  a header from, or slice addresses that don't match `slice_rows`. A
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

## Bench

Bench: pending (plan Task 15).
