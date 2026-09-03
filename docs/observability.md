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
`docs/superpowers/specs/2026-08-05-s3-probe-promote-design.md`. Since
2026-08-31 (latency-accounting) `link.video.lat` carries the end-to-end
head-segment latency breakdown (`n`, and p50/p99 pairs for `enc`/`dq`/`air`/
`fec`) once the AU ring's SlotHdr v2 stamps are flowing — omitted, not
null, until then — and `maburplay`'s OSD grows a matching latency block
above the fps line, reading `--` while its own e2e-latency tracker is
cold or discontinuous.

Since 2026-09-01 that block is **two rows, `P50` and `P99`** (the label
was `LAT` when there was only the tail row — renamed because with two
rows present it has to say which statistic it is). Each row is ONE REAL
FRAME from the last 1 s window, ranked by e2e, showing that frame's own
seven segments — never independently-ranked per-segment percentiles,
so each row's segments sum to its own headline. Read them together:
`P99` alone is a tail statistic sitting among averages (fps/jit/mbps)
and gets misread as typical — an operator reporting "fec is 10–30 ms"
while the median was ~10 ms is what prompted the second row
(`docs/dq-spike-findings-2026-08-31.md` §20).

Since 2026-09-02 (link-rtt) `link.rtt` carries a clock-sync-free
**control-path RTT** and the pts-clock offset behind the absolute LAT
floor: the drone echoes which RCF `rcf_age_ms` ages against
(`Telem.rcf_seq_echo`, validity = flags bit3) plus its MI-domain clock at
telem build (`pts_at_build`), and maburgs matches the echo against its
recorded send times — `rtt = (telem_rx − rcf_send) − rcf_age`, every
1 Hz telem a sample. Keys: `ms` (EWMA), `min_ms` (session min — the
floor bound), `n`, `pts_off_us` (min-RTT-filtered `pts − GS-mono`; null
until the drone ships a usable pts clock), `floor_ms` (maburgs' anchor +
offset = absolute network floor). The whole block is null until the
first matched sample. Read it as CONTROL-path time: the telem reply
queues behind video on the drone's half-duplex TX, so `ms` inflates
under saturation (honest congestion signal, not PHY RTT), and the
offset's residual error is the up/down asymmetry of the best samples,
±1–2 ms class. On the player OSD the offset (combined with the player's
OWN anchor) folds the floor into the `P50`/`P99` rows — `air+` and the
headline both grow by it, so the segments still sum — and the headline
carries a `~` prefix while the e2e is still relative (estimator cold, or
sync lost at range: the floor freezes at last-good and drifts ~1 ms/min
of outage). An `RTT <n> ms` field sits one row above `P50`; it is
adjacent for one-glance reading but is two-way control time and never
part of the e2e sum. What the absolute rows still exclude: sensor
exposure/readout before the pts stamp (~10–16 ms est.) and everything
past scanout start (HDMI + display processing) — LED/camera-lump
territory, see `docs/latency-budget-findings-2026-08-31.md`.

`link.rcf_slot` (2026-09-03) counts how the RCF slotter released each
control-frame send: `au` (at an AU completion with idle ahead, incl. the
in-grace immediate sends), `timeout` (hold reached
`link.rcf_slot_hold_ms`), `passthru` (slotter off, or no AU in the last
100 ms). Cumulative; diff across records. Healthy video = almost all `au`,
`timeout` well under 1 %. See `docs/link-adaptation.md` "RCF slotting".

Consume the same numbers programmatically with:

- `maburtop` on the GS (`tools/maburtop.py`) — full-screen console,
  grouped by link; color thresholds carry the judgment.
- Ad-hoc capture: any UDP listener on :8300, or passively via AF_PACKET on
  `lo` when a consumer already holds the port.
- Flight instrument: since 2026-08-30 the GS boot-starts
  `/etc/init.d/S95flightrec` running `/root/flightrec.py` (both checked in
  under `tools/gs/`, scp'd as-is), one always-on daemon writing a session
  pair into `/media/dvr/log/` per boot (shared index NNNN = max+1, no
  auto-pruning — recordings outlive the code, prune by hand):
  - `flight-NNNN.jsonl` — every sideport datagram off UDP :8300 (absorbs
    the old `/root/rec8300.py`, whose device-only `S97flightrec` wrapper
    from 2026-08-13 silently vanished — probably clobbered by a later
    S97* deploy — which is how the 2026-08-2x flights went unrecorded, the
    recorder gap's 4th bite; the recorder is in-repo now for that reason).
  - `au-NNNN.log` — per-AU meta rows, SlotHdr v2 since 2026-08-31 behind a
    `# aulog 2` marker line written once at session start (`t_us pts sid
    fid len flags nal0 t_first t_complete enc dq`; a log without the
    marker is the pre-2026-08-31 7-column v1 format — `t_us pts sid fid
    len flags nal0` only), read from the `/dev/shm/mabur-au` ring exactly
    like `ausniff.py` (seqlock copy, epoch resync ⇒ `# resync` marker;
    attaches at the write head so pre-attach history can't be stamped with
    attach time), plus `# sync <t_us> <t_ms>` clock anchors every 10 s
    pairing wall time with the datagram's **top-level** `t_ms` (nested
    `last_event.t_ms` is frozen — a first-match regex records a dead
    clock; parse the JSON). `t_first`/`t_complete` are the AU's SlotHdr v2
    mono-µs latency stamps (first body / finish()); `enc`/`dq` are the
    drone's SBI-latched `enc_us`/`drone_q_ms` (venc encode time, TX queue
    wait) carried through on the AU's first fragment. Since 2026-08-31
    (streaming push) `q_ms` is stamped at the body's actual TxQueue push
    and is the TRUE queue wait, ~0 when healthy — before that it also
    swallowed the venc-ring wait and the FEC/SBI CPU (~6 ms standing; see
    the scale-break note in `docs/data-provenance.md`).
  `tools/flightjitter.py` is the analyzer: reproduces the player's jitter
  EMA from the AU rows — using each row's `t_complete` as the arrival
  basis when present (the writer-stamped ring completion time, sharper
  than `t_us`'s reader-poll stamp), falling back to `t_us` for v1 rows —
  splits it into size-explained vs residual (airtime-model §4
  decomposition), and classifies each stutter event — `gap` /
  `rung-change` / `size` / `fec-wait` / `loss-recovery` / `transport` —
  using the jsonl for ladder and loss context (clock offset from the sync
  anchors). `fec-wait` is direct per-frame evidence (the stutter AU's own
  `t_complete − t_first > 20 ms`), checked before the jsonl-inferred
  `loss-recovery` class so it wins when both apply. Tests:
  `ctest -R 'test_flightrec|test_flightjitter'`.
  ⚠ UDP unicast means ONE consumer per port:
  `/etc/init.d/S95flightrec stop` before running maburtop against :8300
  on the GS, then `start` after. The old ad-hoc recipe
  (`socat -u udp-recv:8300 - | jq -c . >> flight.jsonl`) still works
  ATTENDED, but dies on ssh detach even under nohup (jq buffering +
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
- **Verifying AU-completion cadence → `tools/bench/aucadence.py`** (on the
  GS: `python3 aucadence.py --ring /dev/shm/mabur-au --seconds 25 --json`).
  Same outside-the-daemon ring posture as ausniff; reports the base−enh
  completion offset (p50 of `arrival_mono − pts` per class, IDR-excluded —
  metric provenance in `docs/airtime-balance-spike-findings-2026-08-29.md`).
  This is the **standing acceptance number for any change touching the
  AirBalancer, the venc pipeline, UEP overhead, or the bitrate policy** —
  the sideport `jitter_ms` EMA is the symptom of this offset and is noisier
  (±1.4 ms vs ±0.5 ms), so judge on the offset. Baselines recorded at the
  2026-08-30 v4 flag-day acceptance, mcs5 park: **−1.1 to −3.0 ms
  enh-late depending on scene/frame size** (21–33 KB frames, ~9.5 Mbps;
  balancer verified at its equal-air optimum 0.71/1.29 throughout, so the
  residual is encoder-side per-class completion latency — SVC-T
  alternation, scene-dependent) and **+1.1 ms at a pinned-mcs2** run
  (partly the 0.5×cmd rail). Regression criterion at the mcs5 park:
  **`--gate-ms 4.0`** — inside the known encoder envelope anything passes;
  a transport regression (lost balance, one-sided overhead, rate misroute)
  shows as the offset leaving that envelope. The design target ≤0.5 ms
  becomes the gate once the open venc-class-latency work lands. A capture whose
  per-class count is under `--min-samples` (default 100) is refused, not
  scored — enh silence (shed, loss-sim kill) is not a cadence sample.
- **Packet-level forensics → capture tools** (`tools/bench/seqdump.py`,
  `decode_bodies.py`, `live_decode.py`/`live_play.py`). The sideport is
  aggregates; when a summary number looks wrong, these record raw bodies
  for offline dissection. (The RTP-era FU tools — `fu_probe.py`,
  `fu_chain_analyze.py` — survive for reading OLD recordings only.)
- **Player tail-latency persistence → `lat-NNNN.log`** (since 2026-08-31,
  vsync-locked-regulator). The player's 1 Hz `lat:` stderr line used to
  die in tmpfs with the power-off (`/tmp/maburplay.log`, gone on
  reboot — this is how the flight-0035 tail segments were lost, see
  `docs/latency-budget-findings-2026-08-31.md`). It is now additionally
  appended to `<display.lat_log_dir>/lat-NNNN.log` (default
  `/media/dvr/log`, `""` disables). Format: `# latlog 1` then
  `# sync <mono_us> <wall_us>` (written once, at the first successful
  open — the clock bridge that lets a lat log's rows line up against the
  flight jsonl's `t_ms`), then one `<mono_us> <lat-payload>` line per
  second, line-buffered. Index is next-free `lat-NNNN` in the directory
  (same scan idiom as `ctl_log.cpp`) — **deliberately no date suffix**,
  unlike the old `ctl-NNNN_<date>.log` naming: the GS RTC is bogus at
  boot, and `ctl`'s date suffix plus its index-reuse history has already
  caused session-identification confusion once (see the
  `data-provenance.md` DVR-filename note); sessions are matched by the
  `# sync` bridge against the flight jsonl's `t_ms`, not by filename. A
  failed open (DVR not mounted yet) is retried every 30 s and never
  blocks or spams; it never blocks the stderr line either, which keeps
  going regardless. `tools/bench/latab.py latA.log latB.log` reads a pair
  of these logs and prints the vsync A/B verdict (four log-derived gates:
  `e2e` p50 B≤A−8, `dsp` p50 B≤6 (level), `dsp` p99 B≤A−8, `dsp` p50
  4 s-bucket sweep ≤3 (flatness, separate from the p50 level gate) —
  `anchor=warm` windows are excluded from all four,
  with a printed count); see `docs/bench-protocols-latency-2026-08-31.md`
  protocol 1 for the full arm procedure and the manual gates it doesn't
  cover.
- **Regulator line → vsync servo state** (since 2026-08-31). The 1 Hz
  `regulator:` stderr line in `/tmp/maburplay.log` gained
  `vsync=locked|fallback skips=<n> fallback=<n> pend=<n>`: `vsync=` is
  whether the vblank estimator is currently locked (servo release) or
  has fallen back to the old `anchor_floor(pts) + regulate_ms` rule;
  `skips=` counts deep-burst slot claims in servo mode — a frame whose
  natural vblank slot AND the next slot are both occupied claims the
  later one and the older occupant is dropped (bench steady state
  ~1–1.4/s at the mcs5 park, from fec-batch 4-frame bursts; ordinary
  servo drops surface as `replaced=` evictions instead). The beat wrap
  itself never drops: the 59.939 Hz sensor is slower than the 60.000 Hz
  panel, so the ~16.4 s wrap produces one panel repeat, visible in
  `--fps-log`, not here; `fallback=` counts frames
  released via the fallback rule while `display.vsync_lock` is on (climbs
  during a cold start or a stale estimator; cold start needs 8 exact flips
  to first warm, but validity is recency-based, so after a stall the
  counter stops within one fresh exact flip of flips resuming — the warm
  count saturates at 8 and never resets, it's only `last_exact_us_` that
  goes stale); `pend=` is the presenter's present()-while-flip-in-flight
  mailbox engagement count — a superset of the displaced-frame subset,
  0 with no presenter (decode-only or init failed). Since the bench
  session later the same day the line also carries `heals=` (chain-break
  slips: pending releases pushed one slot after two mailbox engagements
  within 100 ms — a backstop that fires ~only at startup now) and
  `pdrop=` (paced-mode mailbox drops: with the servo locked, a parked
  frame is a missed latch and is dropped at flip completion instead of
  resubmitted a period late — the mechanism that keeps a single miss
  from becoming a one-vsync-late chain; steady state ~0.2/s at
  `vsync_lead_ms` 6). A `pend` increment in servo mode therefore costs
  one dropped frame in the common case (two when the mailbox was already
  occupied — the replaced parked frame counts in `--fps-log`'s `repl=`,
  not on this line), never a chain. `pend`, `pdrop`, the regulator
  line's `replaced=`, and `--fps-log`'s `repl=` together are the full
  drop accounting. Since 2026-09-02 the line also carries the
  sequential-slot chain accounting — `chained=` (cumulative frames that
  took natural+1 vblank because a predecessor held their natural slot),
  `chain=` (current consecutive run), `chain_max=` (longest run),
  `chains=` (runs started — chains/s × mean length = chained/s) and
  `cuts=` (runs cut by `display.chain_budget`: the frame that would extend
  a run past the budget takes its natural slot and the held occupant is
  displaced — one dropped frame, counted in `replaced=` too).
  These are the OTHER kind of one-vsync-late chain, the one the servo
  itself creates: one collision (two decodes inside one vblank window)
  shifts every following frame a period late until an arrival gap wider
  than a period frees a slot. The extra period lands in the `lat:` line's
  `reg` segment, so read `chained/held` per second next to `reg` p50.
  Bench 2026-09-02 (mcs5 park, agg6+fb6, lead 6, 196 s): 38.9 % of held
  frames chained, per-second fraction 7–71 %, longest run 52 frames,
  `hold_ema` 13.1 ms with no chain running vs 18.1 ms with one — about
  6.5 ms of mean regulator hold (0.39 × 16.7) that the vblank lock does
  not need. Collisions come from arrival jitter (AU completion gaps 8–27
  ms, ~10 % under 10 ms), not from pair bunching — the AU ring shows
  frames completing one per period, not in pairs.
  **`display.chain_budget` A/B, same session, 150 s arms, budget 0 →
  6 → 0 → 3, config-only restarts** (`lat:` p50 means over 1 Hz windows,
  bootstrap 95 % CI vs arm A; A′ reproduced A within noise, e2e −0.2
  [−1.2, +0.8]):

  | budget | chained | chains/s | cuts/s | hold_ema | reg p50 mean | e2e p50 mean | reg p99 | lat n |
  |---|---|---|---|---|---|---|---|---|
  | 0 (A) | 39.7 % | 6.9 | 0 | 15.1 | 15.0 | 45.5 | 25.1 | 59.4 |
  | 6 (B) | 27.1 % | 6.4 | 0.56 | 13.1 | 12.3 (−2.7 [−3.5, −1.8]) | 43.0 (−2.4 [−3.3, −1.6]) | 24.1 | 58.8 |
  | 0 (A′) | 40.6 % | 6.9 | 0 | 15.4 | 15.1 | 45.3 | 25.0 | 59.4 |
  | 3 (C) | 19.6 % | 6.1 | 1.44 | 11.9 | 10.7 (−4.3 [−5.1, −3.5]) | 41.6 (−3.9 [−4.8, −3.0]) | 24.3 | 57.9 |

  Chains start ~7×/s with a mean length of 3.4 frames, so a budget
  only trims the long tail of runs: 6 buys 2.4 ms of median e2e for
  0.56 drops/s, 3 buys 3.9 ms for 1.44 drops/s, and budget 1 would be
  freshest-wins at ~7 drops/s. reg p99 is untouched by any budget (a
  chained frame plus a full phase margin is the p99 whatever the run
  length). present_jitter EMA rose 0.56 → 0.90 → 1.37 ms with the
  drops. Operator picked **3 as the shipped default** (2026-09-02); 0
  restores the unbounded behavior. The counters make either auditable.
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
    There is deliberately no `qp`: this SDK has no encoder-QP readback
    (`h265Info.startQual` is 0 on every frame, no `GetChnStat` QP), and the
    key that briefly carried it on 2026-09-03 was removed the same night.
  - `GET /snapshot.jpg` → a JPEG straight off the encoder's snapshot channel
    at `venc.snapshot_quality`. Answers 503 on a build without the venc core,
    500 on a capture failure — the two are deliberately distinguishable.
  - `POST /venc/set?k=v`, whitelist `bitrate` / `qp_delta` / `roi_qp` /
    `max_ipprop` / `superframe_p_pct` (the last two are volatile encoder
    pokes, `docs/airtime-model.md` §3; `min_qp` was deleted 2026-09-03). **An
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
(Telem wire grew 61→67, then 67→70 for the venc ring stats below, 70→83
for link-rtt, 83→84 for `roi_qp` and back to 83 the same night when the
never-filled encoder `qp` byte was dropped — a
version-mismatched pair just drops T_TELEM on CRC, so telemetry reads
absent until both ends run the same build; video is unaffected) plus a 5 s
`frame_ring:` stderr line in `/tmp/mabur.log`.

**ROI QP, no encoder QP, and the congestion-shed bit (2026-09-03).**
`drone.enc.roi_qp` is RcAgent's ROI QP *override* as commanded (signed
delta, `encoder.roi_qp_low/normal`). Until this date the same value was
exported as `drone.enc.qp`, read 0 for entire flights, and the flight-0011
analysis mistook it for "rate control never moved"
(`docs/handover-venc-overshoot-2026-09-03.md`). For a few hours that day
`drone.enc.qp` carried the encoder's `startQual` instead — which this
firmware never fills — so the key was deleted rather than shipped as a
permanent 0: **there is no encoder-QP readback on this SDK.** maburtop's
encoder row shows `roi -NN`. Alongside,
`drone.congestion_shed` (Telem flags bit4) is true while
`RcAgent::run_congestion_guard` holds any shed level — the drone-local
TxQueue-pressure / USB-failure shed (`docs/link-adaptation.md`, "Drone
congestion shed") — distinct from `failsafe_shed` (rung 0 / lost link).
A shed enh layer is silence to the GS ladder, so this bit is the only way
to attribute an enh gap to congestion rather than RF, and the only way a
bench can count sheds at all. maburtop's system row renders the pair as
`shed FS|CONG|off`. The drone `stats:` stderr line carries `enc_pk100=`,
the peak 100 ms encoder byte rate (kbit/s, decimal) inside that stats
second — the burst the 1 Hz `drone.enc.mbps` average hides.

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
