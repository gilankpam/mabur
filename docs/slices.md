# H.265 row slices and slice salvage (as built)

The drone encodes each picture as a few row slices. The GS rebuilds a picture
that lost slices into a legal one (slice salvage) and, on the stream-mode GS
image, starts decoding a picture slice by slice while it is still arriving
(streamed decode, "Part 2" below).

Spec: `docs/superpowers/specs/2026-10-10-h265-slices-design.md` (gitignored,
local only). Encoder evidence: `docs/findings/2026-10-09-venc-slice.md`.
Bench record: `docs/findings/2026-10-10-slice-salvage.md`.

## Drone

- `[venc] slices` (compiled default 1 = off; the bundle ships 4): row slices
  on the link channel, applied with `MI_VENC_SetH265SliceSplit` before
  StartRecvPic. Only the counts the SDK's whole-64-px-CTU-row geometry can
  produce pass config load (1080p: 1,2,3,4,5,6,9,17). How the count trades
  off: "Choosing the slice count" below.
- FrameHdr byte 3 = `slice_rows` (64-px CTU rows per slice of this AU, 0 =
  one slice). maburd stamps it only when the AU carries the expected slice
  count; a one-slice AU carrying VPS/SPS/PPS (refresh start, IDR — the SDK
  leaves those whole) sends 0; anything else sends 0 and counts
  `slice_mismatch` (5 s `frame_ring` line). That includes a bare one-slice
  AU without parameter sets: the SDK dropped the split (e.g. after a
  runtime `MI_VENC_SetChnAttr`), and salvage must not switch off silently.
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
  in order; when FrameStream gives up on a hole (`gap_ms` / lookahead —
  salvage never shortens them, so FEC and NACK get their full chance) each
  lost slice becomes a `make_skip_slice()` fill (template header with
  address set and SAO off, every CU skip/merge_idx 0). The picture is then
  legal and gap-free: no decoder error, no rkvdec reset.
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
- Passthrough (the truncated contiguous prefix, what an unsalvaged AU
  carries) when: unsplit AU, no parameter sets yet, unsupported stream
  (incl. a non-64-px CTB), I-slice picture, no complete slice to copy a
  header from, or slice addresses that don't match `slice_rows`. A
  contiguous-prefix fragment shorter than the first fragment's size, or
  losing fragment 0 itself (the fragment size is unknown), both fall under
  this geometry fallback. Each reason is a counter.

## Consumers

- AU ring flag `kRecFlagSliceSalvaged` (0x40). maburplay and the web GS
  decode salvaged base and enhance AUs; only a complete AU arms a decoder.
  maburplay's raw DVR records salvaged AUs too (`RawDvr::feed`'s
  `complete` argument is `au_decodable()` there, i.e. complete OR
  salvaged); maburplay's oneshot JSON carries `salvaged_base`/
  `salvaged_enhance` counters.

## Observability

- Sideport `link.video.slice_salvaged / slices_kept / slices_filled /
  slices_after_hole / slice_fallback{...}`; maburtop `salv`; flightreport
  SLICE SALVAGE; `au.log` columns `slices kept filled after_hole` (since aulog 5);
  ausniff `salvaged`.
- Web GS STATS carries `salvaged` (web stat `aus_salvaged` → JSON key
  `salvaged`): the count of AUs the core rebuilt by slice salvage. The
  JS Gate decodes a salvaged AU once armed but never arms on one.

## Known limits

- Refresh-start pictures stay one slice (SDK: it won't cut through the
  whole-picture refresh stripe); damaged ones pass through. Splitting them
  needs `intra_refresh_frames >= 4`, which breaks base-loss self-healing
  under SVC-T — measured, `docs/findings/2026-10-09-venc-slice.md`
  "Can the refresh-start picture be split?".
- A damaged IDR passes through (`islice`).
- The web GS's local/replay recorder (`RawDvr` in `web/src/web_main.cpp`)
  records complete AUs only — its `on_au` callback feeds `a.complete`, not
  the salvaged-inclusive flag maburplay's raw DVR feeds.
- The web GS's WebCodecs decoder dies on a missing reference (a dropped
  or unsalvageable base AU) and the page waits for a key frame; it
  freezes there where maburplay/MPP conceals. Salvage removes most base
  truncations from that path but not the drops.
- Fill bitstreams are checked by `tools/slices/slicefill_check.py`
  (ffmpeg oracle) against the bench drone's `cap4` capture
  (`tools/slices/make_slice_fixture.py`'s docstring:
  `sbc-groundstations-gilankpam/output/gs-p1-backup-2026-10-09/cap4.h265`),
  3 pictures × 4 fill modes = 12 cases; rerun it after touching
  `common/src/hevc_*`.

## Part 2 (as built): streamed decode

- **Platform.** GS image branch `slice-stream` (sbc-groundstations-gilankpam):
  the fpvOS rkvdec2 stream-mode kernel patch
  (`board/radxa/zero3/linux-patches/0102-…`; `rk_vcodec.rkvdec2_stream`,
  default on, turns rkvdec2 link mode off for every decode) and MPP pinned to
  rockchip-linux/mpp `14729dd5` (develop 2026-09-17, chosen for upstream's
  h265d PS/RPS first-task fix) — the commit `tools/build-arm64.sh` builds —
  with the fpvOS MPP patches ported to it. The canonical patches are
  `gs/player/mpp-patches/`; the image's `package/rockchip-mpp/` carries
  byte-identical copies; `build-arm64.sh` applies them and keys its MPP cache
  on `MPP_REF` + their sha256. `14729dd5` has no `libmpp_ext.so`. Flashing:
  `docs/deploy.md` "GS image: p1 squashfs swap".
- **AU ring v4.** maburgs claims a slot at the AU's first byte (`begin()`:
  odd seqlock for the whole AU, `state` open, RingHdr `open_rec`), copies
  every emitted piece into it and store-releases `valid_len`; `finish()`
  closes it. Writer contract: a split AU's `valid_len` always ends on a NAL
  boundary. Everything FrameStream emits for a split AU does (drained NAL
  runs, a complete remainder, salvage pieces) except the passthrough
  remainder at finish, which leaves through FrameStream's `frame_tail`
  callback (SliceAssembler `raw_out`) and is written with
  `append_unaligned()`: in the record's `len`, never in `valid_len`.
  `nslices` = the SliceAssembler's slice count (0/1 = whole AU). An AU that
  outgrows its slot is aborted on the spot (`state` aborted, never
  published, `dropped_oversize`). A reader follows the record at its cursor
  with `AuRingReader::peek_open()`: bytes below `valid_len` never change
  within one open instance, and the lock re-check catches a re-begun slot.
  maburgs rings the doorbell after every aligned append of a split AU.
- **maburplay StreamFeeder.** `RingClient` turns that into `kOpen`/`kGrow`/`kClose`
  events; a record that ends without closing (overflow, resync, writer
  restart, an enhance AU the policy drops) ends with an aborted `kClose`.
  `StreamFeeder` streams an AU when `[decoder] stream` is on (the default,
  and what the bundle ships), the decoder's probe passed, the backend is
  armed, `nslices >= 2`, and there is no discont or pending flush. Trusting
  the writer contract, it hands over every VCL NAL as soon as it is in the
  valid bytes: START = slice 0 and what precedes it
  (`MPP_PACKET_FLAG_STREAM_START | MPP_PACKET_STREAM_SLICES(n)`), each later
  slice by `MPP_DEC_SET_STREAM_APPEND` on the event that makes it valid —
  slice k goes in when slice k+1's start code reaches maburgs, the
  assembler's one inherent hold-back — never more than n−1 before the
  close, the last one at the close with `MPP_STREAM_APPEND_LAST`. A
  close that is not decodable (passthrough), or an aborted one, ends the
  picture with an empty LAST: `stream_aborted`, one rkvdec reset. A streamed
  AU is never also submitted whole. DVR, flush, arming, the decodable gate
  and `lat.on_submit` stay at the close, so `dec` keeps its meaning
  (t_complete → decoded). With streaming on, `MppBackend` sets
  `base:fast_parse = 0` before `mpp_init` (MPP defaults it to 1; the h265d
  hal only streams without it); with `[decoder] stream = false` MppBackend
  leaves `fast_parse` at MPP's default and every AU is submitted whole.
- **Decoder-input thread (FeedLoop).** The ring, the StreamFeeder and the
  AuRouter run on their own thread: `FeedLoop` blocks on the AU doorbell and
  owns `RingClient`, `StreamFeeder` and `AuRouter` — every MPP *input* call
  (a whole AU's `put_packet`, START, each append, LAST). Why: read from the
  busy render loop, the doorbell was picked up late (ring wake p99 5.8 ms
  vs 0.83 ms on its own thread), which ate most of the streamed-decode
  gain. The main thread keeps frame *output* (`get_frame`/release), the
  presenter, the regulator, OSD compose, the raw DVR and `lat`, plus the
  watchdog; it learns of submits, delivered bytes and figures through a
  bounded note queue (`FeedNote`, `kSubmit` before the MPP call so `lat`
  sees submit before decode, `kDelivery` after, with the AU's bytes moved
  in for the DVR — beyond 256 queued notes a note keeps its meta and drops
  its bytes, counted `notes_dropped`). A flush and the watchdog "park" the
  feed between ring events (outside every decoder call, the only place MPP
  reset/destroy/create is safe): main releases the frames the presenter,
  regulator and recorder hold and resets or recreates MPP while the feed is
  parked, then resumes it. A flush park is taken by the feed thread itself
  (`park_for_flush`); a watchdog park that does not happen within 2 s exits
  the player for respawn (the feed is wedged beyond this process).
- **Clean shutdown.** On SIGTERM no new picture starts (`StreamFeeder::stop_new()`):
  the feed keeps pumping the ring until the open streamed picture's own
  close reaches the feeder and sends LAST with its real last slice
  (`drain_stream`, budget ≤100 ms), then main waits up to 50 ms more for
  that picture's frame before anything tears MPP down. Only then does
  `mpp_destroy` run. Reason: ending the picture with an empty LAST/abort —
  which is what `mpi->reset()`/`mpp_destroy` do to a picture still open in
  the hal — makes the kernel pad it with 64 zero bytes
  (`rkvdec2_strm_cut`), the decode errors, and rkvdec resets; finishing it
  with its own last slice first avoids that. Logged: `maburplay: shutdown:
  <stream idle|stream finished|stream aborted> pts=… waited=… ms[, decoded]`
  and, always, `maburplay: feed: delivered=… submits=… parks=…
  notes_dropped=…`.
- **Observability.** maburplay stderr (`/tmp/maburplay.log`), a `stream:`
  line every 5 s, printed by the feed thread (it owns every counter the
  line reads): `on|off:<reason> streamed salvaged aborted whole opened
  open_aborted wake_us=p50/p99 n refused parks`. `refused` counts
  `MppBackend`'s refused `STREAM_APPEND`s (`stream_errors`); `parks`
  counts every park the feed took since it started (cumulative, never
  reset between lines) — a flush park (one
  per discont record, e.g. during maburd's 1 s sticky-discont window after
  a drone restart) and a watchdog park both land in the same counter.
  Oneshot JSON `streamed`, `stream_aborted`, `stream`. ausniff `nslices`.
- **Orange Pi Zero 2W (H618).** The same `StreamFeeder` drives
  `V4l2ReqBackend` through a patched libav (`AV_PKT_FLAG_PARTIAL`, one
  slice per packet) onto Cedrus, which only decodes slice by slice anyway;
  see `docs/opi-zero2w.md` "Streamed decode". The `stream:` line's
  `refused` stays 0 here -- it reads `MppBackend::stream_errors()`, which
  this board has none of -- so a refused slice shows instead in the
  `v4l2req:` line's `errors=`/`bitstream=`.

### Known limits (Part 2)

- The assembler's hold-back: slice k is only known whole when slice k+1's
  start code arrives, so of a 4-slice picture slices 0–2 decode while the
  air delivers the rest and slice 3 goes in at the close.
- Refresh-start pictures stay one slice: they decode whole, and pay the
  link-mode-off cost.
- Every stream abort costs one rkvdec reset (the kernel pads the picture
  and the decoder errors); salvaged AUs stream without one. A plain restart
  of maburplay does not: the clean-shutdown drain above finishes the open
  streamed picture with its own last slice before teardown. The exception
  is a picture whose close does not arrive within the 100 ms drain budget
  — that one gets an empty LAST/abort and its one reset.
- The web GS does not stream; `MPP_DEC_GET_STREAM_TOP` (early top display)
  is unused. Both are spec non-goals.

## Measured

Full record: `docs/findings/2026-10-10-slice-salvage.md`.

- **Salvage** (loss-sim, slices = 4): 94.5–96.5 % of truncated split AUs
  salvaged, 0 rkvdec resets, `slice_mismatch` 0; real salvaged AUs decode
  in ffmpeg with zero slice-data/CABAC errors. A filled band is a slightly
  stale copy of the previous frame. Enhance fills never propagate; a base
  fill is a reference, so moving content drifts blocky inside that band
  until the next refresh-start picture (≤ 0.5 s).
- **Streamed decode** (with FeedLoop): `dec` p50 −2.7 ms (means), `e2e`
  p50 −2.3 ms; ~96 % of AUs stream, the rest are the one-slice
  refresh starts, which decode whole. Turning rkvdec2 link mode off costs nothing
  measurable. Killing a stream-on maburplay mid-stream: 0 of 10 resets.

## Choosing the slice count

From the 2026-10-10 sweep of every 1080p count (findings record above):

- 4 is at the decode knee: from 3 slices on the `dec` gain sits at a floor
  of about −2.65 ms; 17 is worse (−2.15), because ring wake and per-append
  work grow with the doorbells per AU.
- 6 decodes as fast and shows ~8 points more of a truncated picture
  (head only 40 vs 32.5–35 %, with the after-hole slices 59 vs 52 %).
  The price is the findings' predicted +0.7 % bytes at flight frame sizes
  (more on the small bottom-rung frames, where the per-slice part
  weighs more), unmeasured (the encoder is CBR).
- 5 vs 4: +3 points of picture, no decode gain.
- 17 costs decode time and bytes for the last ~7 points.
- 2 is weak: a third of its truncations lost every slice (`no_template`).
- The bundle stays at 4 until a flight shows how often base truncations
  (untested on the bench: base AUs there were dropped whole) happen.

## Follow-ups

- **Pending: prove the web GS shows the salvaged picture, with motion.**
  A static replay of salvaged AUs through Chrome WebCodecs matched
  libavcodec, but on a static scene "decoded" and "repeated the previous
  frame" look the same (findings record, "Follow-up evidence"). Redo with
  an IDR-start ring capture under loss-sim while someone waves in front of
  the camera: on each salvaged AU the kept bands must change vs the
  previous output and match libavcodec's decode of the new AU, the filled
  bands must stay near the previous output; render Chrome's own
  before/salvaged frames. Harness: aucap (ausniff with framed AU dump) + a
  WebCodecs page that reads frames back via canvas — rebuild, it lived in a
  session scratchpad.
- **Cover dropped frames, not just truncated ones.** A base AU that never
  arrives (nothing received, or no complete slice to copy a header from)
  still leaves later pictures without their reference. maburplay/MPP
  conceals and keeps decoding; the web GS's WebCodecs decoder errors, the
  page drops it and freezes until a requested key frame.
  Idea: the GS core synthesizes a whole-picture fill for the missing base
  AU (every CTU skip, like a slice fill) so no consumer ever sees a missing
  reference. Harder than a slice fill: with no slice received, the header
  (POC, short-term RPS, slice QP, NAL type) has to be derived from the
  neighbouring AUs instead of copied, and it must stay consistent with what
  the encoder's next pictures reference. Needs a design of its own.
- **Futex: closed by measurement, don't re-open it.** The ring header's
  futex idea was never about a slow doorbell; the late wakes were the busy
  render loop reading its own wakeup late, which FeedLoop fixed outright.
  A futex on top would save only tens of µs on an already-~100 µs
  doorbell — not worth building.
- The decode watchdog's park/recreate path has not yet run on hardware: no
  decode watchdog fired in any Part 2 bench.
- An output thread blocking in `decode_get_frame` (design option E, not
  built) would cut main's ≤2 ms frame-pickup delay from `dec` in both the
  stream-on and stream-off arms — a further absolute −0.5 to −1 ms, not a
  change to Δdec.
- `parks=` on the `stream:` line mixes flush parks and watchdog parks (one
  counter, `FeedLoop::parks_`). Split it if watchdog parks ever need
  counting on their own.
- Stream-on arms ran about +0.7 ms mean `fec` over stream-off arms — not
  investigated; could be noise, or the feed thread's CPU competing with
  maburgs on the same core (findings record, "Follow-up evidence").
