# VTX onboard SD recorder

`maburd` records a constant-bitrate fragmented-MP4 copy of the camera onto
the drone's own microSD card while the link stream stays dynamic. The GS
record button drives it. The onboard copy is the safety copy: it keeps
running through link loss and a battery pull loses at most about 2 s. A
drone with no slot, no card, an unmounted card or a full card never
crashes and never exits. It reports why it is not recording, and the
player OSD shows that reason.

Design: `docs/superpowers/specs/2026-09-26-vtx-recorder-design.md` (local
only, gitignored). Spike and the bind-order root cause:
`docs/findings/2026-09-26-sd-record.md`. Bench record of the as-built
recorder (CPU, memory, power cut, restarts, error states, `record.size`):
`docs/findings/2026-09-26-vtx-recorder-bench.md`.

## Targets — `dvr.target` in `/etc/maburplay.toml`

| `target` | a button press (GPIO or `kill -USR1 $(pidof maburplay)`) |
|---|---|
| `"gs"` (default) | GS DVR only; the VTX wish is always "off" |
| `"vtx"` | drone SD only; the GS DVR is never opened |
| `"both"` | GS DVR and drone SD together |

## Wire

- **GS → drone: `Rcf.rec`**, one byte in every RCF. Bit1 `kRecKnown` means
  the GS knows the operator's wish, and bit0 `kRecOn` carries that wish.
  `0` means unknown: `maburgs` has started but no player message has
  arrived yet. With `0`, the drone leaves the recorder as it is, so a
  `maburgs` restart never stops an onboard recording. The drone applies
  the wish only when it differs from the last wish it applied. It treats
  the wish as a level, not an event. Link loss changes nothing. After a
  re-link, the next valid RCF's wish applies.
- **Drone → GS: `Telem.rec_status`**, one byte. Bits 0–1 hold `RecState`
  (0 off, 1 recording, 2 error). Bits 2–7 hold `RecErr` (0 none,
  1 Disabled, 2 NoSlot, 3 NoCard, 4 NotMounted, 5 LowSpace, 6 WriteError).
- **Player → maburgs:** loopback UDP to port 8401
  (`maburgs::kRecControlPort`), with the text `vtx_rec on` or
  `vtx_rec off`. The player sends on every toggle and repeats its current
  wish every 1 s. `RecControl` in maburgs keeps the last command it
  received and never lets it decay. The same socket (`CoreControlClient`,
  `gs/player/src/core_control_client.h`) also carries one-shot
  `refresh <reason>` reports -- `start` on ring open, `resync` when the
  ring reader skips AUs, `decoder` from the decode watchdog's reset path,
  and `dvr` when the raw DVR starts -- asking maburgs for a fresh key
  frame; `docs/fec-nack.md` covers the demand-refresh mechanism itself.
- **Sideport:** `drone.rec = {state, err}` (`docs/observability.md`). The
  maburtop drone line shows `VREC` while recording, or `VREC!<why>` on an
  error.

## Config — `[record]` in `/etc/mabur.toml`

```toml
[record]
enable       = true             # create + bind ch1 at boot; false -> presses report Disabled
dir          = "/mnt/mmcblk0p1" # mdev automount point; must be a mountpoint at start
bitrate_kbps = 40000            # CBR, 2000..80000
fps          = 60               # 1..venc.fps
min_free_mb  = 512              # refuse to start / stop below this
size         = "1920x1080"      # default venc.size; see "Recording size" below
```

The following values are compiled in: fragment 1000 ms, GOP 1 s, a new
file at 3.9 GB (FAT32 limits a file to 4 GiB), and a writer queue of
64 AUs (about 1.07 s at 60 fps). Files are named `record-NNNN.mp4` in
`dir`, using the GS DVR's namer. `maburd` never deletes a file. This
channel's GOP is its own compiled-in 1 s, independent of `venc.gop_s` on
the link channel: the onboard recording keeps a periodic keyframe even
with the link's demand refresh running no periodic refresh of its own
(`docs/fec-nack.md` "Demand refresh").

## Recording size — `record.size`

The recorder records at `venc.size` unless `record.size` names another
size. The SigmaStar encoder cannot scale, so the sizes decide the port:

- **`record.size` = `venc.size`** (the bundle's 1920x1080 for both): ch1
  shares the link's VPE port 0.
- **Any other size** enables the same VPE channel's second scaler, port 1,
  at `record.size` and binds ch1 there (`venc_record_attach`; at the
  link's full rate the bind is src:src like the link's own, so the SDK's
  frame-rate limiter never drops a recorder frame — `docs/airtime-model.md`
  §3). Both ports
  scale the one capture window, so the loader requires `venc.size`'s aspect
  ratio, even dimensions, 320x180..3840x2160. `SetPortMode` + `EnablePort`
  succeed for sizes the scaler never drives, so the drain logs
  `[record] no frame 2000 ms after start` if none arrive. An idle port 1
  reads `Pixel MAX, OutputW 0` in `/proc/mi_modules/mi_vpe/mi_vpe0` until
  ch1 starts; that is not a failure.
- **Above 1920x1080 and above the selected sensor mode** (1920x1080 binned
  up to 90 fps, 1728x972 / 1728x816 binned above, 2944x1656 / 2304x1296
  native; `docs/camera-modes.md`): the pipeline switches to the
  3840x2160@33 mode (full view, not binned), and the loader requires
  `venc.fps <= 30`. The link then also runs at 30 fps from a downscaled 4K
  capture. A `record.size` above 1920x1080 that still fits the selected
  mode (2560x1440 under the native 60 fps tier) fits the selected mode, so
  port 1 scales it down from the mode's window, at the link's own rate.

4K is not usable next to the link: the SigmaStar H.265 engine runs one
encode at a time, and a 4K encode (~30 ms) fills the 31 ms frame period, so
the link queues behind it and the recorder drops frames. A 720p link with a
1080p port-1 recording costs the link nothing. Measurements:
`docs/findings/2026-09-26-vtx-recorder-bench.md`.

Above 90 fps the link runs a crop mode — binned (1728x972 to 100 fps,
1728x816 above) or, under `venc.binning = false`, the native 2304x1296
crop (no mode above 100 fps; `docs/camera-modes.md`) — and the recorder's
rules stay as they are, which carries three risks: a `record.size` larger
than the mode (the bundle's 1920x1080 against 1728x972) is an upscale on
port 1; a `record.fps` that does not divide `venc.fps` (60 from 100)
records uneven frame spacing, since the bind's frame-rate limiter passes
60 of every 100; and the live crop-mode encode alone is past the engine's
rated rate, so recording adds encode time to the link.

Switching `venc.size` needs no other key change: the rolling-refresh key
is `venc.intra_refresh_frames` (sweep length), whose rows per P the
pipeline derives from the encoded height.

## Bind-order invariant (why ch1 is bound before the link)

The SigmaStar H.265 engine has `MaxTaskCnt 1`. For each VPE frame it runs
one bound peer's whole encode before it starts the next peer's, and it
starts with the peer that was bound most recently. The spike bound the
recorder after the link, and the link then waited for one full recorder
encode: +8 ms `enc`. For that reason `venc_record_attach()`
(`drone/venc/venc_record.c`, called from `star6e_pipeline.c`) creates ch1
and binds it (to VPE port 0, or port 1 at its own size) **immediately
before** the link channel's bind,
and leaves it idle under `StopRecvPic`. The link channel is therefore the
newest peer. A live `venc_set_fps` rebind of the link keeps it the newest
peer. **Nothing may bind a new active peer to VPE port 0 after the
link.** The JPEG snapshot channel ch7 is bound after the link, but it is
idle and encodes on the JPEG device.

Boot log line that proves the order:
`[record] ch1 1920x1080 60 fps CBR 40000 kbps on VPE port0, bound before the link (idle)`.

## Live parameter sets — `REC_*`, `VENC_MODE`

A MAVLink set to any `[record]` key, or to `VENC_MODE`
(`docs/vtx-config-mavlink.md` "Applying a set"), rebuilds the encoder pipeline
in-process (`docs/link-adaptation.md` "Rebuilding the encoder pipeline
in-process") rather than restarting `maburd`. Before the rebuild,
`ConfigApplier` stops a recording in progress
(`VtxRecorder::request(false)`, waited out with `settled()`) so the
file closes on a whole fragment before the channel underneath it goes
away, then hands the recorder the new `[record]` section and link size
(`set_config` / `set_link_size`). The rebuild itself recreates ch1 in
the same bind-before-the-link order as a boot (above) — a rebuild is
not exempt from the invariant. If the GS's standing record wish (the
last `Rcf.rec` it sent) was "on", the next RCF after the rebuild
re-issues it (`RcAgent::note_pipeline_rebuilt`) and recording resumes,
into a new file.

A `VENC_MODE` set that changes the sensor mode's aspect, while the
recorder is on with an explicit `record.size`, rewrites `record.size` to
the new venc size in the same candidate (`mavparam::patch_row`,
`drone/src/mav_params.cpp`), so the aspect-ratio rule above is never
broken by the fps/size set alone; `record.fps` never follows.

## Power-loss contract

After every fragment cut, the writer calls `DvrMux::sync()` (`fflush` +
`fsync`). A file on a card whose power was cut is listed at the size of
its last synced fragment and plays up to that fragment. The loss bound is
the fragment being written plus at most one fragment whose fsync had not
returned: ≤ about 2 s (battery-pull bench: ≤ 2.0 s).

## Player OSD REC field

| situation | text |
|---|---|
| both targets recording | `● REC GS+VTX mm:ss` |
| single target recording | `● REC GS mm:ss` / `● REC VTX mm:ss` (target `gs` alone: the bare `● REC mm:ss`) |
| VTX selected, button on, no confirmation yet, or no drone report for > 3 s | `… VTX WAIT` |
| err NoSlot / NoCard / NotMounted | `… VTX NO CARD` |
| err LowSpace | `… VTX FULL` |
| err WriteError | `… VTX FAULT` |
| err Disabled | `… VTX OFF` |

The GS-only fault text is `● REC FAULT` (`● REC GS FAULT` while a VTX state is also shown). With `gs` among the
targets, the clock is the GS DVR's clock. With `vtx` alone, it counts from
the drone's first `Recording` report of this press.

## Link latency cost

The recorder's channel does not delay the link's frames in the H.265
engine (the bind-order invariant above; the MI per-stage delays read the
same recording and idle), but running it costs the link glass latency all
the same: while the second encode runs, every `maburd` thread — the TX
writer, the USB threads and the FEC worker on core 0, the frame producer
alone on core 1, the encoder thread — spends 30–45 % more CPU on unchanged
work, which is memory-system contention from the engine's second 1080p60
encode. The per-frame transit tails (`dq`, `air`, `fec`) widen, and the GS
regulator's capture-time allowance, the transit p99 − p1 over the last 600
frames (`docs/phase-lock.md`), holds every frame longer.

The cost scales with the pixels the record channel encodes per second and
not with the bytes it writes: recording to a sink that discards everything
costs the same as recording to the card, 5 Mb/s costs the same as
40 Mb/s, and the writer's placement, priority, pacing and the kernel's
writeback settings change nothing. Bench, 1080p60 link at rung 5, e2e p50:

- `size` 1920x1080, `fps` 60 (bundle): **+1 to +2 ms**; in flight (0239,
  `dvr.target = "both"`) **+4 ms**, the allowance pinned at `sched_cap_ms`
  most of the time, and after each demote the drone's queue wait reaches
  16–33 ms, which can trip the congestion shed (enhance layer dropped,
  30 fps on the glass for a couple of seconds);
- `fps` 30: about +1 ms;
- `size` 1280x720 at 60 fps (VPE port 1): +0.3 to +0.9 ms, the method's
  noise floor being about 0.5 ms.

Records: `docs/findings/2026-10-25-vtx-recorder-engine-cost.md` (the
mechanism and the levers tried) and
`docs/findings/2026-10-24-vtx-recorder-latency.md` (the flight numbers; its
writer-side reading is superseded).

## Known issues

- **Recording starts at 40 fps for about 0.9 s.** Every third frame is
  missing for the first ~0.9 s after `StartRecvPic`. ch1 is bound
  `src 90 → dst 60` (the sensor mode's 90 fps) and its frame-rate control
  seems to start out dropping 1 in 3; the link channel gets the cold-boot
  `SetFps(60)` re-kick, ch1 does not.
- **One missing frame every ~10.15 s**, in the recording and in the link's
  own `au.log` alike: a VPE/sensor frame-rate effect, not the recorder.
- **At `bitrate_kbps = 80000` the file carries less than at 40000** (about
  21–25 Mb/s against ~32 Mb/s). Not investigated; the bundle ships 40000.
- **Writer drops after a long-lived restart**: one `maburd` instance
  dropped AU bursts every recording until a power cycle. Cause unknown;
  each drop costs the frames up to the next requested IDR.
- Not tested on hardware: pulling the card mid-recording, unplugging the
  GS radio, and rotation at 3.9 GB.

## Deploy

The recorder is part of the RC wire (`Rcf.rec`, `Telem.rec_status`), so
both ends deploy together (`docs/deploy.md` "Flag-day checklist"). A new
`maburd` runs without `[record]` (the recorder reports Disabled); an old
binary refuses the key, so binary first, then the `[record]` block. The
same holds for `maburplay` and `dvr.target`.
