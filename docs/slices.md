# H.265 row slices and slice salvage (as built)

Spec: `docs/superpowers/specs/2026-10-10-h265-slices-design.md` (gitignored).
Evidence: `docs/venc-slice-findings-2026-10-09.md`. Part 2 (streamed decode)
is not built yet.

## Drone

- `[venc] slices` (compiled default 1 = off; the bundle ships 4 since the
  2026-10-10 bench): row slices on the link channel, applied
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

- Refresh-start pictures stay one slice (SDK: it won't cut through the
  whole-picture refresh stripe); damaged ones pass through. Splitting them
  needs `intra_refresh_frames >= 4`, which breaks base-loss self-healing
  under SVC-T — measured, `docs/venc-slice-findings-2026-10-09.md`
  "Can the refresh-start picture be split?".
- A damaged IDR passes through (`islice`).
- The web GS's local/replay recorder (`RawDvr` in `web/src/web_main.cpp`)
  still records complete AUs only — its `on_au` callback feeds `a.complete`
  (not the salvaged-inclusive flag maburplay's raw DVR feeds), unlike
  maburplay's raw DVR, which records salvaged AUs too.
- The web GS's WebCodecs decoder dies on a missing reference (a dropped
  or unsalvageable base AU) and the page waits for a key frame; it
  freezes there where maburplay/MPP conceals. Salvage removes most base
  truncations from that path but not the drops (bench Step 6).
- Fill bitstreams are checked by `tools/slices/slicefill_check.py`
  (ffmpeg oracle) against the bench drone's `cap4` capture
  (`tools/slices/make_slice_fixture.py`'s docstring:
  `sbc-groundstations-gilankpam/output/gs-p1-backup-2026-10-09/cap4.h265`),
  3 pictures × 4 fill modes = 12 cases; rerun it after touching
  `common/src/hevc_*`.

## Follow-ups

- **Pending: prove the web GS shows the salvaged picture, with motion.**
  A replay of 31 salvaged AUs through Chrome WebCodecs (VAAPI, the page's
  own config) matches libavcodec at 45.3–46.9 dB (colour-conversion
  rounding only), fill bands ≥ 43.9 dB, one output per chunk, no error.
  But those captures were static (kept bands changed ~0.15 grey levels
  frame to frame), so "decoded" and "repeated the previous frame" are not
  told apart. Redo with an IDR-start ring capture under loss-sim while
  someone waves in front of the camera: on each salvaged AU the kept bands
  must change vs the previous output and match libavcodec's decode of the
  new AU, the filled bands must stay near the previous output; render
  Chrome's own before/salvaged frames. The operator saw no band edge on
  the phone (/edge, jam, 2026-10-10), which this would explain or refute.
  Harness: aucap (ausniff with framed AU dump) + a WebCodecs page that
  reads frames back via canvas — rebuild, it lived in a session scratchpad.
- **Cover dropped frames, not just truncated ones.** A base AU that never
  arrives (nothing received, or no complete slice to copy a header from)
  still leaves later pictures without their reference. maburplay/MPP
  conceals and keeps decoding; the web GS's WebCodecs decoder errors, the
  page drops it and freezes until a requested key frame (bench Step 6).
  Idea: the GS core synthesizes a whole-picture fill for the missing base
  AU (every CTU skip, like a slice fill) so no consumer ever sees a missing
  reference. Harder than a slice fill: with no slice received, the header
  (POC, short-term RPS, slice QP, NAL type) has to be derived from the
  neighbouring AUs instead of copied, and it must stay consistent with what
  the encoder's next pictures reference. Needs a design of its own.

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
- **Web GS** (this branch's page served locally, Chrome on the host's
  Intel iGPU / VAAPI, host a81a card, page NACK off; jammer = devourer
  `txdemo` on one GS card, ch 144, 6M 1000 B, ~243 fps on air, 5 min):
  `trunc` 43 (13 base), `salvaged` 37. The video froze at times and the
  console showed `[webgs] decoder error EncodingError`; it never showed the
  stale bands maburplay shows.
  - Attribution, by replaying two ring captures that start at an IDR
    through WebCodecs with the page's own config (`hvc1.1.6.L120.B0` +
    hvcC, `prefer-hardware`, `annexbToLengthPrefixed`): loss on the
    enhance stream only, 4 957 AUs incl. 18 salvaged → 0 errors. Loss on
    both streams, salvaged AUs included → decodes through 9 salvaged base
    + 4 salvaged enhance AUs; the first error is the enhance AU right
    after a base AU that never arrived (fid gap). Same capture with the
    salvaged AUs skipped (the page before slice salvage) → error at the
    first skipped base AU, 808 AUs earlier.
  - So WebCodecs decodes salvaged AUs fine; the errors are missing
    references from dropped or unsalvageable base AUs. On any decoder
    error the page drops its decoder and waits for a key frame (IDR
    request), so it freezes where MPP conceals and keeps decoding — the
    page shows a salvaged picture only between such resets. Not a slice
    salvage defect; a web GS limit (Known limits).
- **Turned on:** `bundle/mabur.default.toml` ships `slices = 4`; the bench
  drone runs it (`.pre-slice` backups of binary and config on both
  devices).

## Part 2 bench — platform (GS image `slice-stream`)

Image: kernel + fpvOS 0102, MPP rockchip-linux/mpp `14729dd5` + mabur's
`gs/player/mpp-patches` (no `libmpp_ext.so`). Link mode off at boot:
task-capacity 16 → 1. Bench 2026-10-10, default image
`rootfs.default.squashfs` md5 `f75f5f3f…`, `rootfs.stream0.squashfs`
`bb65f087…`; drone on bundle `slices = 4`, `intra_refresh_frames = 1`.

| check | result |
|---|---|
| `mpi_dec_stream_test` cap4, whole vs streamed (5 ms spread) | md5 equal (`94bdc50b…`), 600/600 frames out both, 0 resets; whole 4506 µs, streamed 1544 µs (repeat 4479 / 1550) |
| `lose_every 7` | 600 frames out, 83 resets (one per lost tail) |
| maburplay whole-AU on the new image | `dec` p50/p99 6/11 ms, `e2e` 36/46.5, 0 resets (second sample 6/10, 36/47) |
| link mode on (`rk_vcodec.rkvdec2_stream=0`) | `dec` p50/p99 7/12.5 ms → link-mode-off cost −1.0 ms p50 (none) |

- The `mpi_dec_stream_test` medians are with the YUV going to `/dev/null`.
  Written into a fifo for `md5sum` (the bit-exactness run) they read
  16 846 / 10 168 µs: the test stamps a frame before writing the one
  before it, so the consumer's speed is in the number. The spike's own logs
  through its python hasher read 28.7 / 22.7 ms for the same reason.
- `dec`/`e2e` are medians over 120 of maburplay's 1 Hz `lat:` lines
  (anchor=ok), integer ms. The means of the per-second p50s resolve
  further: `dec` 6.58 / 6.64 ms on the default image vs 7.14 ms link mode
  on, so turning link mode off costs nothing (≈ −0.5 ms). Stock CI image
  (stock MPP, link mode on), same maburplay binary, same drone: `dec`
  7/12, `e2e` 40/49 (means 7.38 / 40.04).

## Part 2 bench — streamed decode

GS on the `slice-stream` image (Part 2 platform bench above), ring v4
(maburgs md5 `113fce18…`). Bench drone `/etc/mabur.toml` `[venc]`:
`size = "1920x1080"`, `fps = 60`, `slices = 4`, `intra_refresh_frames = 1`,
`intra_refresh_qp = 36`. `dec`/`e2e` are medians of maburplay's 1 Hz `lat:`
lines (anchor=ok, integer ms), 2 × 120 s arms per side, each arm a fresh
maburplay process; Δ = mean(on1, on2) − mean(off1, off2).

Ring wake is measured on closed-record delivery (t_complete → delivery);
the per-slice wake rides the same doorbell path, so it stands in for the
per-append wake. The on arms also run the parser with
base:fast_parse = 0 (required for stream mode); Δdec includes that.

### First bench (Task 11, ring read on the render thread)

| | stream on | stream off |
|---|---|---|
| `dec` p50 / p99 ms | 6 / 9.5 | 7 / 11 |
| `e2e` p50 / p99 ms | 37.5 / 48.5 | 38.5 / 48.25 |
| rkvdec `resetting` during decode, on arms | 0 | — |
| `stream_aborted`, no loss | 0 | — |

- Δdec p50 −1.0 ms (means of lines −1.15), Δdec p99 −1.5 ms, Δe2e p50
  −1.0 ms.
- Ring wake p50 682 µs, p99 5797 µs (medians of the 5 s `stream:` lines).
- `whole` ≈ 7.0 /s with stream on (+836 and +798 over 115 s): ≈ 90 % of
  split AUs streamed, the rest closed before the player saw their first
  slice and went whole.
- One rkvdec reset in the A/B window, at the on2 → off2 restart (see the
  attribution below), none while a stream-on player was decoding.

### Reset attribution: killing a stream-on player (Task 11D)

- 26 restarts with uptime stamps: kills of a stream-on maburplay reset
  rkvdec 3 of 13 times (4 of 14 with Task 11's on2 kill); kills of a
  stream-off maburplay 0 of 13; a 600 s stream-off soak with no restarts
  0.
- Each reset lands 0.13–0.14 s after the kill, before the next maburplay
  is spawned (≈ 1.1 s after the kill): it belongs to the teardown of the
  killed stream-on process, whatever mode the next process runs.
- The picture the kill leaves open is ended with an empty LAST / abort,
  and that empty LAST / abort itself causes the reset (`resetting… /
  reset done`, 0.4 ms, no other kernel output).
- The dmesg (printk) clock on this GS runs 7.17 s ahead of
  CLOCK_MONOTONIC (`/proc/uptime`, `lat.log`, `au.log` stamps): convert
  with monotonic = dmesg − 7.17 s, and re-measure after a reboot
  (`echo mark > /dev/kmsg` bracketed by `/proc/uptime`; +7.16 s on the
  Task 11g boot).

### With the decoder-input thread (Task 11g)

maburplay `9271c81` (md5 `eed579d8…`): FeedLoop reads the ring and feeds
the decoder on its own thread, woken by the AU doorbell; on SIGTERM it
finishes an open streamed picture with its real last slice before MPP
teardown.

| | stream on | stream off |
|---|---|---|
| `dec` p50 / p99 ms | 3 / 8 | 6 / 9.5 |
| `e2e` p50 / p99 ms | 40 / 49.5 | 42 / 49.75 |
| rkvdec `resetting`, no loss (Steps 3–5, A/B incl. its 5 restarts and on3) | 0 | 0 |
| `stream_aborted`, no loss | 0 | — |

- Δdec p50 −3.0 ms (means of lines 3.83 vs 6.49 → −2.67), Δdec p99
  −1.5 ms (means −1.95), Δe2e p50 −2.0 ms (means 39.85 vs 42.13 → −2.28),
  Δe2e p99 −0.25 ms. Regulator: Δe2e p50 is more than half of Δdec p50,
  so the gain passes through.
- Absolute `e2e` is higher than in the first bench in both arms (off 42
  vs 38.5); the difference sits upstream of maburplay (`enc` p50 8 vs 7,
  `fec` p50 7–8 vs 5 on this drone boot), while `dec` off dropped from 7
  to 6. Compare Δ within a bench, not absolutes across the two.
- Ring wake p50 350.5 µs, p99 825.5 µs (medians of the on1 + on2 `stream:`
  lines; per-line p99 710–1011 µs).
- `whole` 2.10–2.23 /s with stream on (on1/on2/on3 +256/+242/+244 over
  115 s) vs 60.0 /s off; `streamed` ≈ 289 per 5 s line, 96.3–96.5 % of
  records; `parks` 0 in every on arm.
- Kills mid-stream: `kill_resets` 0/10 (10 restarts of a stream-on
  maburplay, 20 s apart): `stream finished` 2 (both `, decoded`, waited
  5 ms and 3 ms), `stream idle` 8, `stream aborted` 0. The A/B's 3
  stream-on kills and the loss-sim's restart were `stream idle`, 0 resets.
- Loss-sim (`s0` + `s1 eff=1.5 burst=4`, 5 min, stream on): streamed
  20181 (salvaged 167), `stream_aborted` 0, rkvdec resets 0, decode
  watchdog 0; ausniff (30 s) salvaged 11 (s1) + 1 (s0), incomplete 0,
  frame_id_gaps 3; `dec` 4/11, `e2e` 40/78.5 (300 lines).
- The 5 s stream: line's MppBackend read on the feed thread is
  hardware-only and not ThreadSanitizer-covered; it is safe because the
  watchdog recreates the backend only while the feed is parked.
