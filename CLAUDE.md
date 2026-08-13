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
is the post-flight analyzer over a recorded jsonl. Promotes now probe the
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
  default `/media/dvr`), a `ctllog 1` header followed by compact S/E/P/N/R
  lines (rung/state, ctl events, probe events, penalties, per-rung EWMA store snapshots). `flightreport.py`
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

**TX power is constant since 2026-08-12.** There is no runtime power
control anywhere in mabur: no GS-commanded offset, no thermal derate, no
per-profile offset. `maburd` programs the wall-equalized per-rate diff
table and zeroes the global offset once at bring-up — both steps live
inside the `radio.power_mode: "offset"` branch — and never calls a power
API again: for the life of the process each rate `r` sits at effective
TXAGC index `rate_walls_idx[r] - round(wall_margin_db * 4)`. Note the
`* 4`: the chip's index step is 0.25 dB, so `wall_margin_db` is a dB
figure converted to index steps, and a 1 dB margin is 4 steps
(`make_power_plan()` in `drone/src/power_plan.h`). The config keys
`radio.thermal_max_delta`,
`radio.min_offset_qdb`, `radio.power_offset_qdb` and
`link.static_offset_qdb` were REMOVED and now FAIL BOOT, as does
`radio.power_mode: "override"` (it had become identical to `"none"`).
Sideport keys `link.op.offset_qdb`, `drone.applied.offset_qdb` and
`drone.applied.derate_qdb` are gone; `thermal_delta` REMAINS and is the
only surviving signal that a PA is running hot — nothing acts on it, so
acting on it is a human decision (most likely an airframe cooling fix,
not a power one). `bench/txagcbench` still drives `SetTxPowerOffsetQdb`
directly and is still how the walls are measured; it was deliberately
left alone. Date any recording against this line, the same way the
2026-08-04 SNR scale break is dated.

This change bumped `RC_VERSION` 1 -> 2, the first bump the protocol has
ever had, and mabur's RC wire goldens are now owned by
`tests/test_rc.cpp` rather than mirrored from devourer's frozen
`tools/precoder/rc_proto.py` (which is pinned at version 1 and cannot
follow). **A drone and GS at different versions reject each other's
frames in BOTH directions**, and because DISC_ACK is what carries
`CAP_FRAME_WIRE`, the symptom is NO VIDEO AT ALL — visually identical to
the stale-caps restart deadlock, which will send you to `restart
maburd`. That will not help. Recovery is to finish the deploy. Deploy
order is config-before-binary on both devices: a stale or removed key
makes `maburd`/`maburgs` fail to start (a config-load error exits **1** in
`maburd` and **2** in `maburgs` — both in the `load_config` try/catch in
their respective `main()`; the two daemons do NOT agree, so do not key a
script off either number), and unlike `maburplay`'s
`S97maburplay` — which stops respawning on exit
2 or 143 — neither daemon's wrapper (`S96mabur`, `S96maburgs`,
`maburgs.service` under systemd) checks the exit code at all, so it
crash-loops the daemon forever at the unconditional 2 s respawn delay.
The visible symptom is a repeating `unknown key` line in `/tmp/mabur.log`
/ `/tmp/maburgs.log` every 2 seconds, not a daemon that exits and stays
down. The clean sequence is still: stop both daemons, edit both configs,
swap both binaries (`df` and prune first — the drone rootfs fits max 2
maburd), start both. Rollback is PAIRED: an old binary needs its old
config restored alongside it. **The repo's install scripts will not do
this for you and will produce exactly the crash-loop above if you let
them:** `bundle/install.sh` and `gs/bundle/install.sh` scp the binary
(`:32` in both), then copy the shipped default config ONLY if the device
has none (`bundle/install.sh:33`, `gs/bundle/install.sh:38-39` — "never
clobber a tuned one"), then start the service immediately
(`bundle/install.sh:36-40`, `gs/bundle/install.sh:44-48`). Neither
migrates an existing config, so on a device that has ever been tuned the
four removed keys must be deleted from `/etc/mabur.json` and
`/etc/maburgs.json` BY HAND before the new binaries start.

Schema/design references (local, gitignored):
`docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md` and
`docs/superpowers/specs/2026-07-26-drone-telemetry-design.md`. The schema
is additive-only under `v: 1`; consumers must ignore unknown keys — with
one recorded exception, 2026-08-12, when `link.op.offset_qdb`,
`drone.applied.offset_qdb` and `drone.applied.derate_qdb` were REMOVED
rather than deprecated (constant-TX-power note above) without bumping
`v`. Those three keys are simply absent now — not null — so a consumer
that required them sees a missing key; `tools/maburtop.py` was updated in
the same wave. Additive-only is still the rule for everything else, and
the next removal would need the same kind of dated note here. The
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

**"Wire clean" does NOT mean "no frame loss" — venc-ring vanish class,
detected since 2026-08-13.** Frames can vanish INSIDE the drone (between
waybeam's encoder and maburd's ring read) and never get a `frame_id`: the
wire sequence closes seamlessly over the hole, every FEC/loss counter reads
zero, and a vanished BASE frame silently smears the decoder until an IDR
(rally's natural 2 s GOP is the only healer on this build). Root cause is
CPU famine, not ring depth: above ~12 Mbps at the mcs5 bench op point the
2×A7 SoC starves maburd's hot thread (ring pinned full for 100s of ms),
so the honest knob is `waybeam.bitrate_max_kbps` — the bench runs 10000.
Full findings: `docs/venc-ring-vanish-findings-2026-08-12.md` (committed
with the detection port). The detection (pts-jump, EMA-period,
shed-immune) ships in maburd and exports as
`drone.enc.{vanished_base,vanished_enh,self_idr_refused}` on the sideport
(Telem wire grew 61→67 — a version-mismatched pair just drops T_TELEM on
CRC, so telemetry reads absent until both ends run the same build; video
is unaffected) plus a 5 s `frame_ring:` stderr line in `/tmp/mabur.log`.
`self_idr_refused` counts base vanishes suppressed by the IDR-adjacency
guard — the self-IDR CONSUMER is deliberately not wired: on the parked
`idr-request` branch it amplified CPU overload into an IDR storm (rolling
smear, I-frame-inflated bitrate, `air_pct` low throughout) and needs its
queued redesign (kill switch, GOP-aware suppression, rate-based guard)
before it returns. `tools/bench/ringwatch.c` (branch `bench/loss-sim-v2`)
samples the ring live when attribution is needed.
