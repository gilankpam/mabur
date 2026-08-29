# Observability — which tool for what

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
- Flight recorder: since 2026-08-13 the GS boot-starts
  `/etc/init.d/S97flightrec` (device-only file, like the ctl log's
  siblings), which appends every sideport datagram to a per-boot indexed
  `/media/dvr/flight-NNNN.jsonl` via `/root/rec8300.py` (a python UDP
  binder; keeps the newest 30 files, ~1 MB/min). It exists because three
  incidents in a row (ctl-0030 crash, the 2026-08-13 lag crash, the bench
  false alarm) had no recording — reinstated as a minimal successor to the
  `S97statsrec` that was removed 2026-08-05. ⚠ UDP unicast means ONE
  consumer per port: `/etc/init.d/S97flightrec stop` before running
  maburtop against :8300 on the GS, then `start` after. The old ad-hoc
  recipe (`socat -u udp-recv:8300 - | jq -c . >> flight.jsonl`) still
  works ATTENDED, but dies on ssh detach even under nohup (jq buffering +
  SIGHUP — measured 2026-08-13); don't use it for anything that must
  survive the session. The adaptive-link record survives separately and
  automatically: maburgs writes its own compact ctl log whenever
  `link.ctl_log` is set in `/etc/maburgs.json` (shipped default `false`,
  like `stats.enable` — the bench GS turns it on), one DVR-style indexed
  file per boot at `/media/dvr/ctl-NNNN_<date>.log` (`ctl_log_dir`,
  default `/media/dvr`), a `ctllog 4` header (v1 before 2026-08-14, v2 for that day's
first wave, v3 for that day's second wave, v4 since 2026-08-15 — see the
  pooled-RF note in `docs/link-adaptation.md`) followed by compact S/E/P/N/R
  lines (rung/state, ctl events, probe events, penalties, per-rung EWMA store snapshots). `flightreport.py`
  auto-detects this format alongside the jsonl format, so no separate
  invocation is needed. The controller-side tuning invariants that govern
  what those lines mean live in `docs/link-adaptation.md`.

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
  resumes at the next decoded frame. Files are `record-NNNN.mp4` under
  `dvr.dir`, indexed one past the highest `record-NNNN` already on the card
  — no timestamp, since the GS RTC is wrong at boot (same reasoning as
  `ctl-NNNN` and `flight-NNNN`). The index therefore climbs across boots and
  never overwrites an earlier flight; date-stamped `record_<date>.mp4` files
  from before 2026-08-26 do not match the pattern, so they are ignored by
  the scan and keep their names. No `input` block means no button.
  `dvr.autostart`
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
- **The drone's encoder, up close → the localhost debug endpoint.** Since the
  2026-08-29 venc fold-in `maburd` serves three routes on
  `127.0.0.1:<venc.debug_port>` (shipped 8301), localhost-only, always on, no
  enable flag — a bind failure logs and disables itself rather than being
  fatal. It is reachable only from the drone, so `ssh root@<drone> 'wget -qO-
  http://127.0.0.1:8301/venc'`:
  - `GET /venc` → `{"req_bitrate_kbps", "ring_fill_pct", "full_drops",
    "frames"}`. `req_bitrate_kbps` is the REQUESTED rate — what RcAgent last
    successfully commanded — not a readback of what the encoder programmed;
    the key name says `req_` for that reason. `frames` advancing is the
    cheapest possible "is the camera alive" check, and the one that
    distinguishes a stalled encoder (fps flat, ring empty) from a starved link.
  - `GET /snapshot.jpg` → a JPEG straight off the encoder's snapshot channel
    at `venc.snapshot_quality`. Answers 503 on a build without the venc core,
    500 on a capture failure — the two are deliberately distinguishable.
  - `POST /venc/set?k=v`, whitelist `bitrate` / `qp_delta` / `roi_qp`. **An
    override is not self-clearing.** RcAgent pushes a bitrate only when its
    computed value changes, so on a parked link whatever you set here holds
    until the next rung change or failsafe entry (measured: 20 s+ with no
    sign of reverting). That is what makes it a usable bench knob, and it is
    also how you wedge a flight if you forget to put it back — see
    `docs/link-adaptation.md`.

  It is a debug surface, not an instrument: nothing records it, and the same
  encoder numbers reach the GS sideport as `drone.enc.*` once per second.
  Reach for it when the GS cannot see the drone at all (`drone` is `null`, a
  half-finished deploy, no video) and you need to know whether the encoder is
  running.
- **Radio/PHY bring-up below mabur → devourer's own tools** (`rxdemo` with
  `DEVOURER_RX_ALLPATHS=1`, `doctor`, etc. — see
  `third_party/devourer/CLAUDE.md`). Use these when the question is about
  the chip/driver rather than the mabur link.
- **Bench harnesses** (`bench/linkbench`, `bench/txagcbench`) drive
  special TX/RX modes for characterization; they are not monitoring tools.
  Note: the drone radio RX can wedge after a linkbench run — restart
  maburd.

**"Wire clean" does NOT mean "no frame loss" — venc-ring vanish class,
detected since 2026-08-13.** Frames can vanish INSIDE the drone (between
the encoder and maburd's ring read — a separate waybeam process when this was
found, the in-process venc ring since the 2026-08-29 fold-in) and never get a
`frame_id`: the
wire sequence closes seamlessly over the hole, every FEC/loss counter reads
zero, and a vanished BASE frame silently smears the decoder until an IDR
(rally's natural 2 s GOP is the only healer on this build). Root cause is
CPU famine, not ring depth: above ~12 Mbps at the mcs5 bench op point the
2×A7 SoC starves maburd's hot thread (ring pinned full for 100s of ms),
so the honest knob is `encoder.bitrate_max_kbps` (`waybeam.bitrate_max_kbps`
before the fold-in renamed the section) — the bench runs 10000.
Full findings: `docs/venc-ring-vanish-findings-2026-08-12.md` (committed
with the detection port). The detection (pts-jump, EMA-period,
shed-immune) ships in maburd and exports as
`drone.enc.{vanished_base,vanished_enh,self_idr_refused}` on the sideport
(Telem wire grew 61→67, then 67→70 for the venc ring stats below — a
version-mismatched pair just drops T_TELEM on CRC, so telemetry reads
absent until both ends run the same build; video is unaffected) plus a 5 s
`frame_ring:` stderr line in `/tmp/mabur.log`.

Since the venc fold-in (spec 2026-08-28) the drone also reports the
PRODUCER side of that ring, straight from `venc_get_stats()`:
`drone.enc.venc_ring_fill_pct` (0–100 occupancy at the telemetry tick) and
`drone.enc.venc_full_drops` (lifetime access units the encoder discarded
because maburd had not drained the ring). maburtop shows them as
`vring NN% drop N` on the encoder row. Read them against
`drone.enc.ring_drops`, which is the CONSUMER side of the same ring: fill
climbing with `venc_full_drops` rising means the encoder is outrunning
maburd, while `ring_drops` rising means maburd rejected slots it did read.
A *stalled* encoder shows as neither — `drone.enc.fps`/`enc_frames` simply
stop advancing.
`self_idr_refused` counts base vanishes suppressed by the IDR-adjacency
guard — the self-IDR CONSUMER is deliberately not wired: on the parked
`idr-request` branch it amplified CPU overload into an IDR storm (rolling
smear, I-frame-inflated bitrate, `air_pct` low throughout) and needs its
queued redesign (kill switch, GOP-aware suppression, rate-based guard)
before it returns. `tools/bench/ringwatch.c` (branch `bench/loss-sim-v2`)
samples the ring live when attribution is needed.
