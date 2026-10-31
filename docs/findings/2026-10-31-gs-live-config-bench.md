# GS live config bench gate: the Orange Pi with the bench drone linked

**Status:** bench gate for the GS live-config controls; the reference page
is `docs/gs-live-config.md`. Only the Orange Pi Zero 2W was up for this
run. The Radxa half of the gate (its `--all` runs, the burned-recording
restarts, the `flushes=` check on a burned recorder) is still to do.

## Setup

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

## Results

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

## Defect: `gsparam.py --all gs` miscounts the ladder

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

## Hop refusal

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

## Other observations

- `maburplay` logs `warning: no MSP snapshot ... after 10 s` at boot as
  well as after the `osd.enable` re-enable. The bench drone has no flight
  controller, so this predates the live-config work.
- After the power cycle, `maburplay` logged one decode-watchdog reset
  (`attempt 1`) as video first arrived, then ran clean.
- Every `debug_log.enable = true` opened a new session directory under
  `/media/dvr/log/`, nine in all. They and the test recording were deleted
  afterwards.

## Not covered

- Anything that needs eyes on the screen: whether colortrans keeps the
  picture flat/corrected across the toggle, the console-mode flash during
  a recreate, and whether the OSD planes visibly come back.
- HDMI unplug/replug.
- The Radxa: its `--all` runs with the MPP build, `dvr.mode = "burned"`
  accepted, and a burned recording restarting into a new file across a
  `screen_mode` set and an overlay switch-on.

## Device state after the run

- GS (Orange Pi): runs the branch binaries.
  - Rollbacks: `/usr/local/bin/maburgs.pre-livebench` and
    `maburplay.pre-livebench` (the image's own builds).
  - Configs: byte-identical to their pre-bench copies, kept as
    `/config/*.pre-livebench`. The applier's `.pre-live` files are
    alongside.
  - The bench link key is installed.
- Drone: powered off.
