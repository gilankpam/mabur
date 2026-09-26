# VTX onboard SD recorder (as built, 2026-09-26)

`maburd` records a constant-bitrate fragmented-MP4 copy of the camera onto
the drone's own microSD card while the link stream stays dynamic. The GS
record button drives it. The onboard copy is the safety copy: it keeps
running through link loss and a battery pull loses at most about 2 s. A
drone with no slot, no card, an unmounted card or a full card never
crashes and never exits. It reports why it is not recording, and the
player OSD shows that reason.

Design: `docs/superpowers/specs/2026-09-26-vtx-recorder-design.md` (local
only). Spike and the bind-order root cause:
`docs/sd-record-findings-2026-09-26.md`.

## Targets — `dvr.target` in `/etc/maburplay.toml`

| `target` | a button press (GPIO or `kill -USR1 $(pidof maburplay)`) |
|---|---|
| `"gs"` (default) | GS DVR only, exactly as before; the VTX wish is always "off" |
| `"vtx"` | drone SD only; the GS DVR is never opened |
| `"both"` | GS DVR and drone SD together |

The bench GS runs `"both"`.

## Wire (RC_VERSION 11)

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
  received and never lets it decay.
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
```

The following values are compiled in: fragment 1000 ms, GOP 1 s, a new
file at 3.9 GB (FAT32 limits a file to 4 GiB), and a writer queue of
64 AUs (about 1.07 s at 60 fps). Files are named `record-NNNN.mp4` in
`dir`, using the GS DVR's namer. `maburd` never deletes a file.

## Bind-order invariant (why ch1 is bound before the link)

The SigmaStar H.265 engine has `MaxTaskCnt 1`. For each VPE frame it runs
one bound peer's whole encode before it starts the next peer's, and it
starts with the peer that was bound most recently. The spike bound the
recorder after the link, and the link then waited for one full recorder
encode: +8 ms `enc`. For that reason `venc_record_init()` creates ch1 and
binds it to VPE port 0 **immediately before** the link channel's bind,
and leaves it idle under `StopRecvPic`. The link channel is therefore the
newest peer. A live `venc_set_fps` rebind of the link keeps it the newest
peer. **Nothing may bind a new active peer to VPE port 0 after the
link.** The JPEG snapshot channel ch7 is bound after the link, but it is
idle and encodes on the JPEG device.

Boot log line that proves the order:
`[record] ch1 1920x1080 60 fps CBR 40000 kbps bound before the link (idle)`.

## Power-loss contract

After every fragment cut, the writer calls `DvrMux::sync()` (`fflush` +
`fsync`). A file on a card whose power was cut is listed at the size of
its last synced fragment and plays up to that fragment. The loss bound is
the fragment being written plus at most one fragment whose fsync had not
returned: ≤ about 2 s. Bench result, measured the same day: ≤ 2.0 s (see
below).

## Player OSD REC field

| situation | text |
|---|---|
| both targets recording | `● REC GS+VTX mm:ss` |
| single target recording | `● REC GS mm:ss` / `● REC VTX mm:ss` (target `gs` alone: the bare `● REC mm:ss` as before) |
| VTX selected, button on, no confirmation yet, or no drone report for > 3 s | `… VTX WAIT` |
| err NoSlot / NoCard / NotMounted | `… VTX NO CARD` |
| err LowSpace | `… VTX FULL` |
| err WriteError | `… VTX FAULT` |
| err Disabled | `… VTX OFF` |

The GS-only fault text (`● REC FAULT`) is unchanged. With `gs` among the
targets, the clock is the GS DVR's clock. With `vtx` alone, it counts from
the drone's first `Recording` report of this press.

## Bench results (2026-09-26, `.152` Runcam WiFiLink 2 + bench GS)

The binaries were built against devourer master 56eabe4. The link sat at
mcs4/40 MHz, 24 Mb/s, 1080p60, with `low_power.enable = false` for the
runs. MI numbers are averages over 60 s after `reset_counter`, in µs on
the MI origin. `lat.log` `enc` is the GS player's per-second p50/p99.

| run | VPE `FinDMADispatch` | ch0 enqueue | ch0 dequeue | ch0 GetStream | ch1 enqueue | ch1 GetStream | link `enc` p50/p99 (60 windows) |
|---|---|---|---|---|---|---|---|
| `[record] enable=false` | 16 744 | 16 832 (+0.09 ms) | 24 002 | 24 439 | — | — | 7/8 ×49, 8/8 ×12 |
| enable=true, idle | 16 747 | 16 832 (+0.09 ms) | 23 986 | 24 431 | — | — | 7/8 ×49, 8/8 ×11 |
| recording 40 Mb/s @ 60 | 16 723 | 16 807 (+0.08 ms) | 24 010 | 24 447 | 24 113 (+7.39 ms) | 31 941 | 7/8 ×55, 7/9 ×5 |

- **Bind order holds.** While recording, the link enqueues at VPE dispatch
  and ch1 enqueues one link encode later. The link's `enc` is unchanged.
  An idle ch1 costs the link nothing. With ch1 created, `MbRate %` reads
  147 even while ch1 is idle, so it counts configured channels, not work.
- **ausniff** during recording (30 s): 1816 AUs, 908/908 complete,
  `fid_gaps=0 resyncs=0`, 60.5 fps.
- **CPU** (`drone.sys.cpu_pct`, 60 s): 39.4 % off → 59.1 % recording,
  **+19.7 points**. This is higher than the spike's +14. Per thread:
  `mbr-rec` (writer) 12.4 % and `mbr-recdrain` 6.2 % of one core. The rest
  is kernel time for vfat/SD. A repeat after a fresh boot gave 39.9 → 56.7.
- **Memory** at 40 Mb/s: `maburd` VmRSS peaked at 18.8 MB and MemAvailable
  never fell below 58.8 MB. With a parallel `dd` of 300 MB `conv=fsync` to
  the card, the peaks were 22.5 MB RSS and 53.3 MB MemAvailable, with 18
  queue-full drop events and no OOM. At `bitrate_kbps = 80000` with the
  same `dd`: 21.1 MB RSS, 55.2 MB MemAvailable, 18 drops, no OOM.
- **Delivered bitrate.** At 40000 the file carries about 32 Mb/s
  (`ffprobe` 31.8 Mb/s). At 80000 the sizes on the card worked out to only
  about 21–25 Mb/s, *less* than at 40000. Not investigated. Keep 40000.
- **Content.** A stopped 61 s recording decodes cleanly: HEVC 1920x1080,
  61.32 s. ffmpeg's null muxer prints 5 "non monotonically increasing dts"
  warnings per file, with no decode errors.
- **Battery pull** (plug cut after about 65 s): after reboot the file is
  listed at 255 341 831 B, and `ffprobe` gives 63.32 s, decoding cleanly to
  the end. The time from the press to the last ping before the cut was
  64.7–65.3 s, so the loss is **≤ 2.0 s**. That is an upper bound, because
  the time from the press to the first IDR was not subtracted. A second,
  less precisely timed pull gave the same result.
- **`kill -9 maburd` while recording.** The wrapper respawns it. The new
  instance is not Disabled, and because the button was still on it
  resumed straight into a new file. The pre-kill file plays to 20.14 s, as
  large as it was listed just before the kill. dmesg shows the usual MI
  teardown noise (CMDQ `WAIT_TRIG_TIMEOUT`, `FlushInputPortTasks`, a
  `GetChnDevid CH 1 is not created` warning from the ch1 pre-init
  teardown) and no MMU callback storm.
- **`S00mabur stop` while recording**: a clean `rec: STOP`. The file
  (57.27 s) plays to the end.
- **Re-link resumes.** After a power cut with the button still on, the
  drone was recording into a new file within about 7 s of kernel start.
- **Link outage** (`maburgs` stopped for about 10 s): the drone went to
  state 3 and did a rendezvous retune, and there was no STOP. The file
  (78.66 s) is one continuous file with no gap > 40 ms.
- **GS restarts.** `S96maburgs restart` while recording: no STOP.
  `S97maburplay stop` for 18 s: the drone kept recording. Starting the
  player again with the button off made the drone log `rec: STOP` about 3 s
  later, including player startup. `killall maburplay` does not leave the
  player down, because the S97 loop respawned it within about 2–2.5 s. The drone
  then stopped 1–2 s after the new player appeared (3 runs, 3.3–4.8 s from
  the kill). This is the accepted "a restarted player with its button off
  stops the drone" behaviour.
- **Error states** (read from `drone.rec`, not from the screen):
  `dir = "/tmp/nocard"` gave err 4 NotMounted (`VTX NO CARD`).
  `min_free_mb = 1000000` gave err 5 LowSpace (`VTX FULL`).
  `enable = false` gave err 1 Disabled (`VTX OFF`). Pressing off clears
  each of them back to state 0, err 0.

### Open

- **Recording starts at 40 fps for about 0.9 s.** In every file, every
  third frame is missing for the first ~0.9 s after `StartRecvPic`. ch1 is
  bound `src 90 → dst 60` (the sensor mode's 90 fps), and its frame-rate
  control seems to start out dropping 1 in 3. The link channel gets the
  cold-boot `SetFps(60)` re-kick, but ch1 does not.
- **One missing frame every ~10.15 s** in the recording. The link's own
  `au.log` shows the same cadence (sessions 0250, 0253 and 0259). This is
  a VPE/sensor frame-rate effect that predates the recorder.
- **Writer drops in one instance.** A fresh boot drops 0–1 AU burst per
  minute. One `maburd` instance, started with `S00mabur start` after the
  GS restart tests and a rendezvous retune, dropped 3–22 bursts per
  recording, with 3.4 s of 29 s missing, and its idle CPU was 45 % instead
  of 40 %. A power cycle cleared it, and a repeated `kill -9` did not
  bring it back. Cause unknown. Each drop costs the frames up to the next
  requested IDR (67–383 ms).
- Not tested on hardware: pulling the card mid-recording, unplugging the
  GS radio, the OSD pixels themselves (the states were read from the
  sideport), and rotation at 3.9 GB (the longest file was 1.13 GB).

## Deploy and rollback

This is a flag day (RC_VERSION 11). Install the **binary first, then the
config**. The new `maburd` runs without `[record]` (recorder disabled),
and an old binary refuses the key. The same applies to `maburplay` and
`dvr.target`. `docs/deploy.md` has the details. The rollbacks on the bench
are `/usr/bin/maburd.pre-vtxrec` + `/etc/mabur.toml.pre-vtxrec` (drone),
and `/usr/local/bin/maburgs.pre-vtxrec`,
`/usr/local/bin/maburplay.pre-vtxrec` + `/etc/maburplay.toml.pre-vtxrec`
(GS). Roll back both ends together, each binary with its config.
