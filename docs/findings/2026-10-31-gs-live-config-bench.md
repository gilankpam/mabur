# GS live config bench gate: the Orange Pi and the Radxa with the bench drone linked

**Status:** bench gate for the GS live-config controls; the reference page
is `docs/gs-live-config.md`. Both GS boards passed, the Orange Pi Zero 2W
first and then the Radxa ZERO 3, on the same day. The Radxa run used the
two fixes made after the Orange Pi run (the `gsparam.py` ladder count and
the submit-time pin check).

The gate is the same on both boards: every live key away and back on both
daemons, the `flushes=` count per presenter rebuild, a recording across
the player sets, ausniff and aucadence across the maburgs sets, the
channel pin and `"auto"` with the link up, a pin sent during a relocate,
and a power cycle with values left set. The Orange Pi records a raw
recording (it has no MPP encoder). The Radxa records a burned one, with
the restarts the rebuilds cause.

## Orange Pi

### Setup

- Build: branch `gs-live-config` at `e8743cc9`, `tools/build-arm64.sh`
  stage 7 (`maburgs-opi`, `maburplay-opi`), installed as
  `/usr/local/bin/maburgs` and `/usr/local/bin/maburplay`. The image's own
  binaries were kept as `*.pre-livebench`.
- The GS image puts both TOMLs on vfat `/config`, behind symlinks:
  `/etc/maburgs.toml` and `/etc/maburplay.toml`. `/etc/maburgs.toml`
  matched the bundle byte for byte. `maburplay.toml` differed from the
  bundle only in the Orange Pi lines (`backend = "v4l2req"`,
  `dvr.mode = "raw"`, no `[input.rec]`). This image has `/media/dvr`.
- The image had no `/etc/mabur.key`, so `maburgs` booted on the default key
  and could not pair. The bench pair's key was copied in by hand and
  synced. It was still there after the power cycle below.
- The bench drone ran at 60 fps. The link formed on ch 144 at mcs3–4/40.
  The display was HDMI at 1920x1080@60. The connector offered 2560x1440,
  which `--alt-mode 2560x1440@60` used.
- Evidence comes from logs, the regulator line, the sideport, ausniff and
  aucadence, and DVR files. Nobody looked at the screen.

### Results

| Gate line | Result |
|---|---|
| `get` on both daemons | **PASS.** 9 lines each. The values match both TOMLs, which match the bundle plus the Orange Pi lines. |
| `--all player --opi --alt-mode 2560x1440@60` | **PASS, `ALL OK`.** All 18 steps match the expected replies. The `dvr.mode = "burned"` restore was refused with `err dvr.mode "burned" needs the MPP encoder; this build has none -- set dvr.mode = "raw"`. Each set wrote one `cfg set` log line. The file ended byte-identical to the pre-bench copy, and `.pre-live` was written once. |
| Presenter recreates | `screen_mode` → 2560x1440@60 in 329 ms and back in 246 ms. `colortrans.enable` false in 276 ms and true in 285 ms. Each recreate printed a fresh `DrmPresenter` mode/topology block, and the CTM was reinstalled on `true`. The phase lock re-acquired after each one. `v4l2req errors=0` throughout. |
| `flushes=` | **PASS.** The count went 43 → 47, exactly +1 on each of the four presenter-rule sets (`screen_mode` ×2, `colortrans.enable` ×2). It did not move on the other 14 sets. The `osd.enable` and `osd.gs.enable` toggles switched layers in place (`msp osd off/on`, `gs osd off/on`, 30–34 ms), since the other overlay kept the plane wanted. |
| Raw recording across the player sets | **PASS.** One raw recording (SIGUSR1 start/stop) ran across the whole `--all player`. `recstarts=` stayed at 1, giving one file: 2609 samples, 43.5 s, HEVC 1920x1080. On the host, `ffprobe -count_frames` read 2609 frames and `ffmpeg -f null` decoded it with no errors. |
| `--all gs` (daemon) | **PASS.** All 18 sets answered `ok`, and every `get` read back. Set times were 33–66 ms. The file ended byte-identical to the pre-bench copy. |
| `--all gs` (tool) | **FAIL, harness defect.** Both `link.ladder` steps reported `MISMATCH file check`, so the script exited 1. The file was correct; see "Defect" below. |
| ausniff across a second `--all gs` (60 s) | **Radio loss, not maburgs.** 3613 AUs, 60.2 fps, **2 `frame_id_gaps`, 0 resyncs**, 1 incomplete, 3 salvaged. `--all gs` begins by pinning ch 64, and the link was on ch 64 for this run. On ch 64 the link fails every rung-2 probe (`s3_residual`/`probation` demotes about every 10 s). In the same window, maburgs' own counters show `drop` 46 → 48 and `trunc` +3. A 60 s idle control on ch 64 with no sets showed 0 gaps, **1 resync**, 1 salvaged and `trunc` +1. |
| ausniff, ch 144 pinned, all 8 non-channel keys away and back | **PASS.** One pass (16 sets): 3616 AUs, **0 gaps, 0 resyncs**, 60.3 fps. Sets repeated across a full 60 s window (96 sets): 3616 AUs, **0 gaps, 0 resyncs**, 60.3 fps. |
| aucadence (25 s) after the ladder was back at the bundle's | **PASS.** Run on ch 144 at mcs4/40: offset **+0.209 ms**, n 750/750, 0 IDR excluded, 0 resyncs. `--gate-ms 4.0` passed. This is within the recorded envelope (−1.1 to −3.0 ms at the mcs5 park, +1.1 ms at mcs2). |
| Channel pin in flight | **PASS.** From ch 64, `set radio.channel = 144` answered ok in 44 ms. The log shows `relocate 64 -> 144 placed (pin)`, then `one_card_retune` at +254 ms, then `verify_pass`. The sideport then read `ch=144`, `scan.state = "off"`, `scan.pinned = true`, and the link climbed back to mcs4 within about 8 s and stayed there. The first `--all gs` step (`= 64` from ch 144) moved the link the same way (66 ms). |
| `set radio.channel = "auto"` | **PASS.** Answered ok in 33 ms with `maburgs channel: auto, link on 144`. No hop. The sideport read `scan.state = "frozen"`, `pinned = false`. |
| A pin during a hop | **Refused, but not with the expected text.** The second pin was sent 1 ms and 132 ms after the first pin's `ok`, while the relocate was still in flight. Both answered **`err relocate in flight`** (70–84 ms), not `err hop or calibration in flight`. Each refused set left the file holding the first pin's value. One sent 268 ms after, once the hop had finished, answered ok and moved the link. See "Hop refusal" below. |
| Power cycle | **PASS.** The last sets left `radio.channel = 144`, `dvr.burned.every = 2` and `dvr.target = "both"` set away from their defaults. After `sync`, the GS was off for 20 s. Back up: both TOML md5s were unchanged, both `/etc` symlinks were still symlinks, `/config` was mounted rw, and `/etc/mabur.key` was intact. `maburgs` booted with `mode pinned start 144` and the bench key, and its `get` read `144`. `maburplay`'s `get` read `2` and `"both"`. The link came up on ch 144 at mcs4, pinned, with `present=60`. All three values were then set back. Both TOMLs ended byte-identical to the pre-bench copies. |

### Defect: `gsparam.py --all gs` miscounts the ladder

`file_matches()` checks `link.ladder` with
`text.count("[[link.ladder]]") == value.count("{")`. That count also
matches the bundle's `[radio]` `width` comment (`... link speeds in
[[link.ladder]] below.`), so every ladder step reads one block too many.
On a bundle-shaped config, `--all gs` therefore fails both ladder steps
and exits 1. The daemon side was checked by hand with a line-anchored
count (`grep -c '^\[\[link.ladder\]\]'`): 4 blocks after the 4-rung set,
5 after the restore. The block layout was right, the `[link.probe]`
comment lead-in survived, and the restored file was byte-identical to the
pre-bench copy.

### Hop refusal

`hop_or_cal_busy` (the submit-time check) is
`chan.hopping() || chan.sweep_pending() || cal_session.running()`.

A live pin moves the link through the boot-pick relocate. That relocate is
`relocating()`, not `hopping()`, so the submit check lets the second pin
through. `ChannelCore::set_pin` then refuses it after the file has been
published (`relocate in flight`), and the applier re-publishes the
previous text. On this one-card GS the hop phase itself
(`one_card_retune` → `verify_pass`) lasts about 20 ms of a ~270 ms
relocate, so the submit-time text could not be hit by timing.

The other ways to provoke it are an interference hop (needs the jammer,
not allowed here) and a calibration run (writes the drone's config). So
`err hop or calibration in flight` was not observed on hardware.

The refusal itself is safe: nothing moved, and the file ended on the
previous value. For the ~40 ms between the publish and the restore,
though, the file holds the refused value.

### Other observations

- `maburplay` logs `warning: no MSP snapshot ... after 10 s` at boot as
  well as after the `osd.enable` re-enable. The bench drone has no flight
  controller, so this predates the live-config work.
- After the power cycle, `maburplay` logged one decode-watchdog reset
  (`attempt 1`) as video first arrived, then ran clean.
- Every `debug_log.enable = true` opened a new session directory under
  `/media/dvr/log/`, nine in all. They and the test recording were deleted
  afterwards.

## Radxa

### Setup

- Build: branch `gs-live-config` at `ff1df984`, `tools/build-arm64.sh`
  stage 6 (`out/arm64/maburgs`, `out/arm64/maburplay`, the Buildroot
  glibc builds with MPP), installed as `/usr/local/bin/maburgs` and
  `/usr/local/bin/maburplay`. The previous builds were kept as
  `*.pre-livebench`.
- On this image `/etc/maburgs.toml` is a regular file on the overlay.
  `/etc/maburplay.toml` is the symlink onto vfat `/config`.
- Neither TOML is bundle-shaped. `maburgs.toml` has the channel set
  `[64, 112, 144]` (no 40) and a six-rung ladder that starts with two
  20 MHz rungs (mcs0/20, mcs1/20, then mcs1–4/40). It also has
  `predict = false`, `debug_log.enable = true` and older comment text.
  `maburplay.toml` has `screen_mode = "2560x1440@99.95"`,
  `dvr.target = "both"` and `dvr.burned.bitrate_kbps = 20000`. `--all`
  restores bundle values, so afterwards those keys differed from the
  pre-bench copies until the final restore sets.
- `/etc/mabur.key` was already there (fingerprint 2263), and `maburgs`
  booted paired.
- Two GS cards: relocates run as `hop_lead` then `hop_follow`, not
  `one_card_retune`.
- The bench drone ran at 60 fps. The link formed on ch 144 and climbed to
  mcs4/40.
- The display was HDMI at 2560x1440@99.95. The connector also offers
  2560x1440 at 75.03 and 60 Hz, and 1920x1080 at 60, 59.94 and 50 Hz.
  With `--alt-mode 2560x1440@60`, `screen_mode` went 2560x1440@99.95 →
  2560x1440@60 (away) → 1920x1080@60 (the bundle value `--all` restores).
  Both are real mode changes.
- `--all player` was driven by a small wrapper. It calls `gsparam.py`'s
  own `step()` for each entry of `expected_pairs("player", alt)`, so the
  checks are the same. Between steps it reads `flushes=`/`recstarts=` off
  the regulator line.
- Evidence comes from logs, the regulator line, the sideport, ausniff and
  aucadence, DVR files decoded on the host, and file md5/mtime/inode.
  Nobody looked at the screen.

### Results

| Gate line | Result |
|---|---|
| `get` on both daemons | **PASS.** 9 lines each. The values match both TOMLs, including the six-rung ladder. |
| `--all gs` | **PASS, `ALL OK`, exit 0.** The ladder file check now passes, reading 4 blocks after the 4-rung set and 5 after the restore. All 18 sets answered `ok` in 30–108 ms (`msp.enable = true` took the 108 ms). `debug_log.enable = true` was a same-value set (0 ms, no write), because this GS already logs. The `= 64` pin relocated 144 → 64 (`verify_pass` +13 ms). |
| `--all player --alt-mode 2560x1440@60` | **PASS, `ALL OK`.** All 18 steps answered `ok` and read back. `dvr.mode` → `"raw"` took 27 ms and back to `"burned"` was **accepted** in 24 ms. `dvr.target = "both"` was a same-value set (0 ms). |
| Presenter recreates (no recording) | `screen_mode` → 2560x1440@60 in 528 ms, → 1920x1080@60 in 530 ms. `colortrans.enable` false in 239 ms, true in 366 ms. Each recreate printed a fresh `DrmPresenter` block, and `true` brought back `colortrans: display LUT on`. The phase lock re-acquired each time, and leaving 100 Hz logged `rate mismatch cleared: source 59.99 Hz, panel 60.006 Hz`. Over these four recreates the kernel logged two `mpp_rkvdec2 … resetting` / `reset done` pairs, and each `screen_mode` rebuild logged one `h265d: refs: … missing ref`. Both are known from the recreate spike, and video recovered every time. |
| `flushes=` | **PASS.** The count went 80 → 84, exactly +1 on each presenter-rule set (`screen_mode` ×2, `colortrans.enable` ×2) and +0 on the other 14. The `osd.enable`/`osd.gs.enable` toggles switched layers in place (`msp osd off/on`, `gs osd off/on`, 23–32 ms). |
| Burned recording across the sets | **PASS.** The bundle `dvr.*` values were live (burned, target `"gs"`, 8000 kbps, every 1, decoded size). The sequence was SIGUSR1 start, `screen_mode` away (561 ms) and back (654 ms), `osd.enable = false` (29 ms, in place, no restart), `osd.enable = true` (130 ms, `rec: burned recorder restarting -- an overlay was switched on and its palette is built at start`), `colortrans.enable` false (555 ms) and true (579 ms), then SIGUSR1 stop. `recstarts=` went 0 → 6 and `flushes=` 84 → 88. Six files, one per start, each HEVC 1920x1080 (the decoded size, including the one recorded while the panel ran 2560x1440@60), 6.7–14.5 s and 386–861 frames. On the host, `ffprobe -count_frames` and `ffmpeg -v error -f null -` read every file with exit 0 and no error lines. The file started under `colortrans.enable = false` logged `ct=off`. All six were deleted from the GS afterwards. |
| ausniff across a second `--all gs` (60 s) | **Radio loss on ch 64, not maburgs.** 3601 AUs, 60.0 fps, **10 `frame_id_gaps`, 0 resyncs**, 0 incomplete, 6 salvaged. Between the two runs the auto link had hopped back to ch 144 (an in-flight hop, `order epoch 2 target 144`). So this run's `= 64` pin relocated 144 → 64 and restarted at rung 0. On ch 64 the link then demoted repeatedly (`timeout`, `s3_residual`, `probation`, `residual`). In the same window maburgs' own counters show `drop` 23 → 47 and `trunc` 12 → 18, the same ch 64 picture as on the Orange Pi. |
| ausniff, ch 144 pinned, the 8 non-channel keys away and back for 60 s | **PASS.** Five passes, 80 sets: 3610 AUs, 60.2 fps, **0 gaps, 0 resyncs**, 0 salvaged. maburgs' `drop`/`trunc` did not move. |
| aucadence (25 s) on the bundle ladder | **PASS.** Run on ch 144 at mcs4/40: offset **−0.441 ms**, n 750/750, 0 IDR excluded, 0 resyncs, and `--gate-ms 4.0` passed. This is within the recorded envelope (−1.1 to −3.0 ms at the mcs5 park, +1.1 ms at mcs2) and near the Orange Pi's +0.209 ms. |
| Channel pin in flight | **PASS, with one relocate that lost the link.** Five pins went between 144 and 112. Each answered `ok` in 40–64 ms. The sideport then read `link.channel` = the pin, `scan.state = "off"` and `scan.pinned = true`, and the link stayed there. Four were clean: `lead_confirm` came at +96–155 ms and `verify_pass` at +3–12 ms, one regulator second read `present=48–50`, and mcs4/40 was back within about 10 s. The first one (144 → 112) dropped the link for about 11 s before it landed on 112; see "A pin relocate that lost the link" below. |
| `set radio.channel = "auto"` | **PASS.** Answered ok in 38 ms with `maburgs channel: auto, link on 112`. No hop. The sideport read `scan.state = "frozen"`, `scan.pinned = false`. |
| A pin during a relocate | **PASS, refused before any file work.** One python on the GS sent the second pin right after the first pin's `ok`. Sent 1.0 ms after, it answered `err relocate in flight` in 1 ms. Sent 81 ms after, it answered the same in 4 ms. The daemon logged `cfg set radio.channel = 64 -> err relocate in flight (0 ms)` both times, during `hop_lead`, before `lead_confirm`. Between the first pin's `ok` and the refusal, `/etc/maburgs.toml` kept the same md5, mtime and inode. The `.pre-live` mtime did not change and no `.new` scratch file was left. This is the pre-check that replaced the Orange Pi's post-publish refusal. |
| Power cycle | **PASS.** The last sets left `radio.channel = 112`, `dvr.burned.every = 2` and `dvr.target = "both"` away from their bundle values. `screen_mode = "1920x1080@60"`, the bundle ladder and `debug_log.enable = false` were also still away from this GS's own values. After `sync`, the GS was off for 20 s. Back up: both TOML md5s were unchanged and `/etc/maburplay.toml` was still a symlink. `/config` was mounted rw; the kernel said `Volume was not properly unmounted` for both vfat partitions, as it does after any power cut. `/etc/mabur.key` was intact. `maburgs` booted with `mode pinned start 112` and key 2263, and its `get` read `112`. `maburplay` came up at 1920x1080@60, and its `get` read `every = 2` and `"both"`. The link came up on ch 112 at mcs4/40, pinned, with `present=60`, and this time there was no decode-watchdog reset. Live sets then restored `"auto"`, the GS's own six-rung ladder, `debug_log.enable = true`, `screen_mode = "2560x1440@99.95"`, `bitrate_kbps = 20000` and `every = 1`. Both TOMLs ended byte-identical to the pre-bench copies, so the six-rung ladder was written back in the original layout. |

### A pin relocate that lost the link

The first pin of the channel line (112, with the link on 144 at mcs4/40)
answered `ok` in 44 ms. The GS then logged `relocate 144 -> 112 placed
(pin)`, `hop_lead card 0`, `lead_confirm` at **+9 ms**, `hop_follow` and
`verify_pass` at +6 ms. Every relocate that landed in this run (ten of
them, pins and the one in-flight hop) confirmed its lead at +69 to
+189 ms.

The drone did not move. Its log has no `retune 144 -> 112` for this
relocate, and it went to state 3 on 144. The GS lost the session
(`video tail -> off (no session)`) and logged one `REFUSING video: peer
session did not advertise CAP_FRAME_WIRE (chip_caps=0x000f)`. Its search
then found the drone still on 144 (`drone found on 144 (op 112)`). It
relocated again, this time as a plain `relocate 144 -> 112 placed` with no
pin tag, and again confirmed its lead fast (+7 ms, `verify_pass` +3 ms).
The drone retuned this time (`retune 144 -> 112 (hop)`, its only retune
to 112), but the GS logged `ack_override card -1 112 -> 144` and lost the
session a second time. It then found the drone on 112, and the link
formed there and climbed to mcs4/40.

Video was out for about 11 s: ten regulator seconds read `present=` 0–6.
Throughout, `get` and the file read 112, which is what the reference page
says an `ok` pin means while its relocate has not yet landed. The four
pins after it were clean.

This is the relocate and hop-confirm path (`ChannelCore` and the hop
machinery), not the live-config file path. Both relocates that lost the
link confirmed their lead in under 10 ms. That is too fast to be the
drone answering from the new channel, so the lead may have confirmed on
something stale. That was not chased further here.

### Other observations

- `maburplay` logged `warning: no MSP snapshot ... after 10 s` after the
  `osd.gs.enable` re-enable, as on the Orange Pi. The bench drone has no
  flight controller.
- Seven `debug_log.enable = true` sets changed the value: one in the
  second `--all gs`, five in the ch 144 loop, and the final restore. Each
  opened a new session directory under `/media/dvr/log/`. The six bench
  ones were deleted, and the one the restore opened is the running
  session and was kept.
- The applier's `.pre-live` files did not exist before the bench. Each
  daemon run wrote one at its first publish, and after the power cycle
  they held the bench's pinned state, so both were deleted.

## Not covered

- Anything that needs eyes on the screen: whether colortrans keeps the
  picture flat/corrected across the toggle, the console-mode flash during
  a recreate, whether the OSD planes visibly come back, and what the
  burned recordings show (OSD drawn in, colour) beyond decoding clean.
- HDMI unplug/replug.
- The pre-check's other refusals on hardware: `hop in flight` (needs an
  interference hop, so the jammer) and `calibration running` (a maburcal
  run writes the drone's config). Only `relocate in flight` was hit.

## Device state after the run

- GS (Orange Pi, swapped out after its run): runs the branch binaries.
  - Rollbacks: `/usr/local/bin/maburgs.pre-livebench` and
    `maburplay.pre-livebench` (the image's own builds).
  - Configs: byte-identical to their pre-bench copies, kept as
    `/config/*.pre-livebench`. The applier's `.pre-live` files are
    alongside.
  - The bench link key is installed.
- GS (Radxa): runs the branch binaries built at `ff1df984` (stage 6).
  - Rollbacks: `/usr/local/bin/maburgs.pre-livebench` and
    `maburplay.pre-livebench` (the builds it ran before).
  - Configs: byte-identical to their pre-bench copies, kept as
    `/etc/maburgs.toml.pre-livebench` and
    `/config/maburplay.toml.pre-livebench`. No `.pre-live` files.
  - The bench link key is installed, as it was before the run.
- Drone: powered off.
