# CLAUDE.md

Guidance for Claude Code when working in this repository.

mabur is an RTP-free FPV video link: `maburd` (drone, OpenIPC/SigmaStar,
armv7) encodes and injects; `maburgs` (ground station, aarch64) receives,
FEC-decodes, and publishes whole access units to a shm AU ring; `maburplay`
(gs/player/, same GS binary family) consumes the ring — MPP hardware decode
straight to DRM/KMS, plus the fMP4 DVR on /media/dvr. `common/` holds the
shared wire formats and FEC; `third_party/devourer` (plus the sibling
checkout `../devourer`) is the userspace radio driver. PixelPilot and the
RTP output were deleted in PR C.

## Build & test

NixOS host: wrap every cmake/ctest invocation in
`nix-shell -p pkg-config libusb1 --run "..."`. Configure with
`-DDEVOURER_DIR=$PWD/../devourer` when the default `../devourer` doesn't
resolve. Host suite: `ctest --test-dir build -R 'test_|host_e2e'`.
Cross-builds: `tools/build-arm64.sh` (maburgs), `tools/build-arm.sh`
(maburd). Devices: drone `root@192.168.10.152`, GS `root@10.18.0.1`.
Never load the 8812eu kernel module on the GS cards.

## Debugging & observability — which tool for what

The **stats sideport** is the primary debug surface. maburgs pushes a JSON
datagram (schema v1) to UDP `127.0.0.1:8300` every 500 ms: per-card and
per-RF-class signal stats, link op point, per-stream FEC health, video/AU-ring
health, TX/injection rates, airtime estimate, and drone telemetry (state,
applied op, encoder, queues, uplink RF, temps — via the 1 Hz T_TELEM
control frame); `link.ctl` carries the ladder controller's rung, util,
probation/counters, and last-transition reason, and `tools/flightreport.py`
is the post-flight analyzer over a recorded jsonl. Since 2026-08-11 the GS
also requests IDRs itself: reference-layer (sid ≤ 2) truncated/dropped
frames latch `RCF_F_IDR_REQ` onto the RCF uplink until an IDR-flagged frame
arrives; the drone grants at most one per `waybeam.idr_cooldown_ms` (default
1000). Kill switch `link.idr_req` (GS, default true); observability
`link.video.idr_req` {pending,episodes,wait_ms} + `drone.enc.idr_grants` on
the sideport, `idrq` cell in maburtop, `idr-req: set/cleared` lines in
/tmp/maburgs.log. Spec
`docs/superpowers/specs/2026-08-11-idr-request-design.md`. Promotes now probe the
candidate MCS on the s3 enhancement stream first when the drone advertises
`CAP_S3_PROBE`, and s3 residual/util loss demotes in steady state (kill-switch
`link.s3_demote`); spec
`docs/superpowers/specs/2026-08-05-s3-probe-promote-design.md`. Consume it
with:

- `maburtop` on the GS (`tools/maburtop.py`) — full-screen console,
  grouped by link; color thresholds carry the judgment.
- Ad-hoc capture: any UDP listener on :8300, or passively via AF_PACKET on
  `lo` when a consumer already holds the port.
- Flight recorder: `socat -u udp-recv:8300 - | jq -c . >> flight.jsonl` on
  the GS logs every metric at 2 Hz for post-flight analysis. The `jq -c` is
  REQUIRED: sideport datagrams carry no trailing newline, so bare socat
  appends concatenated JSON, not JSONL (recover such a file with
  `jq -c . < file`). This ad-hoc capture is now the ONLY way to get FULL
  sideport data on disk: `statsrec.py`/`/etc/init.d/S97statsrec` (the boot
  recorder that auto-wrote `/media/dvr/flight-NNNN_<date>.jsonl` and fanned
  out to :8301) were REMOVED from the GS on 2026-08-05 (device-only files,
  never in this repo). The adaptive-link record survives separately and
  automatically: maburgs writes its own compact ctl log whenever
  `link.ctl_log` is set in `/etc/maburgs.json` (shipped default `false`,
  like `stats.enable` — the bench GS turns it on), one DVR-style indexed
  file per boot at `/media/dvr/ctl-NNNN_<date>.log` (`ctl_log_dir`,
  default `/media/dvr`), a `ctllog 1` header followed by compact S/E/P/N
  lines (rung/state, ctl events, probe events, penalties). `flightreport.py`
  auto-detects this format alongside the jsonl format, so no separate
  invocation is needed. Tuning invariant: the controller's s3 loss/residual
  windows are 500 ms wide, while the post-transition blanking
  (`s3_settle_ms`, default 300) and probe settle (`probe_settle_ms`, 150)
  are shorter — so up to ~200 ms of pre-transition symbols remain in view
  after blanking expires. Shipped defaults are safe (stale weight decays
  fast against the 250/500 ms confirm windows), but do NOT lower
  `s3_settle_ms`/`s3_residual_confirm_ms` toward their floors together: a
  rung transition's FEC re-key artifacts could then satisfy the s3-residual
  confirm and self-demote on every promote.

⚠ **"Wire clean" does not mean "no frame loss"** (2026-08-12): encoded
frames can vanish inside the drone's venc frame ring BEFORE maburd assigns
a frame_id — the wire sequence closes seamlessly, and a vanished base
frame silently corrupts the decoder until an IDR. Root cause is CPU
famine, not ring depth: above ~12 Mbps at the mcs5 bench op point the
2×A7 SoC runs out (measured: hot thread unscheduled for 100s of ms, ring
pinned full 15% of the time at a 16 Mbps cap vs never past 2/8 at 12
Mbps), so the honest knob is `waybeam.bitrate_max_kbps` — the bench runs
10000 since 2026-08-12 and master (`6086880`) on BOTH ends. The pts-jump
detection that counts this class (`drone.enc.{vanished_base,vanished_enh}`
+ the 5 s `frame_ring:` stderr line, commit `65c94fd`) lives on the
PARKED `idr-request` branch — the deployed master is blind to it again; a
smeared picture over an all-green sideport is this class, healed only by
rally's natural 2 s GOP. If the branch redeploys, know that its self-IDR
path amplified an overload into an IDR storm (rolling smear + stutter,
bitrate inflated by I-frame density, `air_pct` LOW throughout — the
bottleneck is drone CPU, not RF) and needs its queued redesign first;
`waybeam.idr_cooldown_ms: 5000` was the stopgap (key REJECTED by master's
config parser — strip it before any master deploy). Evidence:
`docs/venc-ring-vanish-findings-2026-08-12.md`. Sibling same-day bug with
opposite signature (video <1 Mbps, everything green, encoder stuck at the
1400 boot floor): `docs/waybeam-bitrate-wedge-2026-08-12.md` — cure is
re-applying `video0.bitrate`; when reading ausniff output, ALWAYS check
the bytes-derived bitrate, not just fps/gaps.

**Rule of thumb: if you want to KNOW something about the running link,
read the sideport. Reach for other tools only in these cases:**

- **Verifying maburgs itself → `tools/bench/ausniff.py`** (on the GS:
  `python3 ausniff.py --ring /dev/shm/mabur-au --seconds 30 --json`). It
  attaches to the AU ring read-only from OUTSIDE the daemon. This is the
  standing regression gate for any change that touches maburgs: the
  sideport is maburgs self-reporting, so gating a maburgs change on the
  sideport is circular — a bug that mangles frames or lies in its own
  counters sails through. Never replace the ausniff gate with sideport
  numbers. (Oneshot/post-hoc reads of a quiescent ring are exact; live
  mode is best-effort — Python cannot fence.) Host-side, the same
  invariant is `ctest -R 'gs_e2e|gs_au_e2e|player_e2e'` (byte-exact
  fixture-to-ring/AU comparisons via `verify_aus.py`/`--out-aus`).
- **Packet-level forensics → capture tools** (`tools/bench/seqdump.py`,
  `decode_bodies.py`, `live_decode.py`/`live_play.py`). The sideport is
  aggregates; when a summary number looks wrong, these record raw bodies
  for offline dissection. (The RTP-era FU tools — `fu_probe.py`,
  `fu_chain_analyze.py` — survive for reading OLD recordings only.)
- **Post-mortem when no consumer was listening → the 1 Hz stderr stats
  line** in `/tmp/maburgs.log` (maburd's in `/tmp/mabur.log`, maburplay's
  fps-log + respawn history in `/tmp/maburplay.log`). It is
  numerically redundant with the sideport but persists on disk; the UDP
  feed is ephemeral. The MSP OSD is now rendered by maburplay itself, from
  the UDP snapshot feed maburgs emits (maburgs no longer draws pixels); the
  OSD startup line and blanking notices land in `/tmp/maburplay.log`
  alongside the fps-log. Since 2026-08-11 maburplay also drives the display
  at startup rather than at the first decoded frame: it modesets immediately
  with a splash image (`/usr/local/share/mabur/splash.bin`, raw XRGB8888,
  regenerate with `tools/gen_splash.py`) so the sink locks a mode before video
  exists, and it retries display acquisition once a second while none is
  connected — a display plugged in or powered on after the player started is
  picked up without a restart, which it never was before. That only holds if
  a display was NEVER acquired: one that disconnects after a successful
  acquire is still unrecoverable, deliberately (a stated non-goal — KMS
  retains CRTC state and a replug normally re-lights it on its own). Neither
  has a config key. The splash shows from process start, or from a late
  display acquire only when no frame has been decoded yet, until the first
  decoded frame — deliberately, because the image is an aerial photo that
  would read as a live feed if it ever appeared mid-flight, so a mid-flight
  replug comes up on video rather than on the photo. Two log lines cover a
  no-display episode (`no display at startup -- retrying`, `display acquired
  after N.N s`); the per-attempt DRM failures are silenced on purpose, since
  /tmp is tmpfs.
- **The GS link-status OSD on the screen is a sideport consumer, not a
  separate instrument.** maburplay draws it from the SAME datagram
  `maburtop` reads: `stats.out` in `/etc/maburgs.json` is a list, and the
  bench GS fans out to `:8300` (maburtop / ad-hoc capture) and `:8302`
  (maburplay's `osd.gs.port`). So the screen and the recorder cannot
  disagree — if the OSD shows something surprising, the answer is in that
  jsonl, and `flightreport.py` will say the same thing with more
  precision. Config is
  `osd.gs` in `/etc/maburplay.json` (`enable` default false, `stale_ms`
  dims every link-derived field after silence; fps/jitter/bitrate/REC are
  player-measured and never dim). The glyph atlas is
  `/usr/local/share/mabur/gs_osd.gfont`, committed and staged by
  `tools/build-arm64.sh` — if it is missing, maburplay logs the reason to
  `/tmp/maburplay.log` and runs with the MSP overlay only. Host-side you can
  see the actual pixels without hardware: `maburplay --gs-render` dumps a
  rendered frame, `tests/test_gs_asset.cpp` gates the real asset's layout at
  720p/1080p/1440p/2160p, and `tools/bench/gs_overlay_bench.cpp` measures
  draw+quantize per update at 1080p and 2160p. Read that bench before
  changing anything on this path: it runs on the 2 ms pump loop, and the
  rule of thumb is that anything projecting past ~1.5 ms on the A55 is a
  defect. **The shipped code already reports 3.7 ms at 1080p and 9.9 ms at
  2160p for a full repaint, and that is accepted, not overlooked** — a full
  repaint is a startup/re-layout event, the quantize half of it is
  burned-DVR-only (raw or no-DVR mode pays the draw column alone: 1.3 ms and
  3.6 ms), and `ring.pump(2)` is a `poll()` ceiling over a slotted shm ring,
  so the cost lands as one-vsync-late presentation rather than a lost AU.
  What is NOT accepted is anything that puts a full repaint on a *cadence*:
  the burn restate after an MSP collision is scoped to the fields the
  collision actually hit for exactly that reason (`osd_compose.cpp`), and it
  used to cost 179,392 px per changed cell instead of ~13,000. Steady state
  is 0.13 ms.
- **Record button (GPIO).** `maburplay` can toggle the DVR from a button on
  the GS header: `input.rec` in `/etc/maburplay.json` (`pin` is the header
  pin number, resolved at startup by matching the kernel's line names —
  the Radxa ZERO 3 names its header `PIN_7`…`PIN_40` across gpiochip1/3/4;
  `active_low`/`bias` default to a button between the pin and GND with the
  internal pull-up). Short press, 50 ms debounce, edge-triggered: each
  press-pair produces one file, in whichever `dvr.mode` is configured — raw
  mode waits for the next sync point (up to ~2 s) before the new file
  opens, so the OSD REC indicator visibly lags the press, while burned mode
  resumes at the next decoded frame; two presses inside the same
  wall-clock second get a `-1`, `-2`… filename suffix (one-second
  timestamp resolution). No `input` block means no button. `dvr.autostart`
  (renamed from `dvr.enabled` on 2026-08-11 — the old key now FAILS boot,
  by design) picks whether the player boots recording or armed. There is
  no config kill switch and no `--no-dvr` flag: `autostart: false` with no
  button is a player that never records. That also means a `--decode-only`
  measurement run has no CLI way to opt out of the DVR any more, so give it
  a config with `"autostart": false` — otherwise the hardware decode gate
  writes a raw file and charges SD-card I/O to the fps number. Startup logs the resolved mapping
  and every toggle logs START/STOP to `/tmp/maburplay.log`; the OSD REC
  field is the live indicator. GPIO failures (pin not found, line held by
  another consumer) are non-fatal and logged once — the player runs
  without the button.
- **Radio/PHY bring-up below mabur → devourer's own tools** (`rxdemo` with
  `DEVOURER_RX_ALLPATHS=1`, `doctor`, etc. — see
  `third_party/devourer/CLAUDE.md`). Use these when the question is about
  the chip/driver rather than the mabur link.
- **Bench harnesses** (`bench/linkbench`, `bench/txagcbench`) drive
  special TX/RX modes for characterization; they are not monitoring tools.
  Note: the drone radio RX can wedge after a linkbench run — restart
  maburd.

**Carrier sense is OFF on both daemons since 2026-08-05.** `maburd` and
`maburgs` both set `dev_cfg.tuning.disable_cca = true` at bring-up, so the
radios inject without the MAC CCA/EDCCA gate — no deferral to co-channel
802.11, and no politeness toward it either. Rationale is in
`docs/superpowers/specs/2026-08-05-cca-disable-design.md` (gitignored,
hence this note) — that spec is a protocol with an expectations table, not
results: the 41-45% injection-deferral figure it cites is devourer's own
measurement, taken with its `txdemo` injector on an 8822EU (same chip
family), not with maburd, and the mabur-side bench A/B has not run yet.
Expected practical consequence for anyone reading old data: in a congested
environment a pre-2026-08-05 recording and a post- one would not be the
same experiment, since the drone's injection rate under interference is
expected to change. On a clean channel the gate never trips and the two
stay comparable. Bench harnesses (`linkbench`, `txagcbench`) are
deliberately left building a plain `DeviceConfig` — carrier sense stays ON
there, so don't assume they share the daemons' MAC config. Nothing in the
sideport reports the CCA state — date the recording against this line. To
confirm it on a running device, grep the daemon log for `carrier sense`:
both daemons print a one-line bring-up record (`maburd radio:` / `maburgs
radio card N:`, once per card bring-up — a recovered front-end reprints
it, so more than one line per card on a multi-card GS is expected).
devourer's own carrier-sense line is info-level and so is compiled out of
the cross-builds' `DEVOURER_LOG_MAX_LEVEL=WARN` — its absence means
nothing. Both lines record the state mabur *requested*, not a register
readback.

Schema/design references (local, gitignored):
`docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md` and
`docs/superpowers/specs/2026-07-26-drone-telemetry-design.md`. The schema
is additive-only under `v: 1`; consumers must ignore unknown keys. The
sideport config lives in `/etc/maburgs.json` under `stats`
(default-off in the shipped bundle; enabled on the bench GS).

**Scale break, 2026-08-04 — `classes.*.snr` is now dB, was half-dB.** The
sideport had been exporting devourer's raw half-dB SNR under a key
documented as dB, so every `classes.*.snr` (and the `snr_min`/`snr_max`
derived from it) in any recording made BEFORE that date reads exactly
2× the real figure. Recordings that span the change are not numerically
comparable and must not be pooled — a "9 dB improvement" across it is an
artifact. `flightreport.py` warns on the old scale — but that warning is a
BACKSTOP, not a detector: it fires only at `max(snr) > 60`, where the old
scale exceeds anything a real link produces. A normal 10–25 dB link reads
20–50 on the old scale and never trips it, and no threshold can do better,
because a pre-fix 48 (24 dB) and a post-fix 48 (an ordinary strong bench
link) are the same number with nothing in the schema to tell them apart.
**Silence from that warning means "not obviously old", never "confirmed
dB".** Date the recording instead — anything before 2026-08-04 is half-dB.
This is recorded here because it is the only committed, discoverable place:
the schema doc lives under gitignored `docs/superpowers/`.

`drone.uplink.snr_a`/`snr_b` had the SAME bug and were fixed the same day —
the drone's own receiver reads the uplink through the same devourer
`RxAtrib.snr`, and `telemetry.cpp` forwards it raw. Both are corrected at
the exporter, not at the source: the uplink's wire field is an `int8_t` the
drone `lround()`s, so halving before that rounding would quantize to whole
dB and lose half the resolution. Everything above about pre-2026-08-04
recordings applies to `drone.uplink.snr_*` too, and `flightreport.py` warns
on both with the same backstop threshold and the same caveat.

`classes.*.evm[_a|_b]` (added 2026-08-10) never had the bug: it is dB from
day one (devourer raw is the same half-dB family, halved at the exporter).
⚠ Raw EVM is op-point-dependent — the same clean bench link legitimately
reads −16 dB at mcs0 and −30 dB at mcs7, because per-MCS TX power moves the
PA between compression and linear regimes. Never compare EVM across rungs
or threshold it globally; use deviation from the same rung's baseline. The
sweep that established this (and the interpretation: walls stay
delivery-defined; EVM's job is per-rung baselines + live PA-compression
watchdog) is `docs/evm-sweep-findings-2026-08-10.md`.
