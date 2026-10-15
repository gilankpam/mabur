# Observability — which tool for what

**Rule of thumb: if you want to KNOW something about the running link,
read the stats sideport.** Reach for the other tools on this page only in
the cases listed under "Tools: which one when". The one hard exception:
a change to maburgs itself is gated on `tools/bench/ausniff.py`, never on
the sideport (the sideport is maburgs self-reporting).

Removed sideport keys, old log-format versions and every scale break a
recording can carry are in `docs/data-provenance.md`; this page describes
only what the code emits now.

## The stats sideport

maburgs pushes one JSON datagram (schema `v: 1`) per `[stats] interval_ms`
(code default 500, shipped bundle 200) to every `[[stats.out]]` entry. The
bundle fans out to `127.0.0.1:8300` (maburtop, ad-hoc capture) and
`127.0.0.1:8302` (maburplay's `osd.gs.port`). `[stats] enable` gates the UDP
sinks only: the same snapshot is written to `flight.jsonl` whenever the
debug-log session is on (below), so turning debug logging on never blanks
the OSD and vice versa. Producer: `gs/src/stats_exporter.cpp`.

Top level: `v`, `session` (random nonzero u32 per maburgs process — a
consumer detects restarts by it changing), `seq`, `t_ms`, then the blocks
below. Cumulative counters are per maburgs process; diff across records.

Consumers in-repo: `tools/maburtop.py` (binds :8300 directly), the
player's GS OSD (:8302), `tools/flightreport.py` (over `flight.jsonl`).

### `link` — op point, ladder, loss

- `channel`, `state` (`session` | `beaconing` | `key_mismatch`), `key_fp`,
  `tx_card` — pairing is `docs/link-pairing.md`.
- `op = {mcs, bw, sgi, vht, overhead_base, overhead_enh, snr_req}` — the
  GS-commanded op point.
- `ctl` — the ladder controller's rung, util, probation/counters and last
  transition reason; `null` in static-pin mode (`link.static_mcs >= 0`),
  which never ticks the controller. `rungs[]` is the per-rung EWMA store.
  Semantics: `docs/link-adaptation.md`.
- `residual_loss`, `pre_fec_loss`, `attrib.{residual_cur, close_ms}`,
  `layer_delivery_pct`.
- `scan = {state, rounds, pick}` (top level, not under `link`) — the boot
  channel scan, `docs/channel-select.md`.

#### Pre-FEC loss is late, not lost

`link.pre_fec_loss` — the OSD's LOSS row — is the ArrivalTracker's
"booked missing at the settle line" fraction with BOTH video layers pooled
in one window; `link.ctl.pre_fec_loss` is the base-only sample the ladder
acted on, and the player reads it only as the fallback for a null pooled
window (`gs/player/src/gs_snapshot.cpp`). Booked missing means: a seq
counts missing once a later seq arrives `link.arrival_guard_syms` ahead of
it, and a symbol heard after that counts `streams[*].arr_late` and is never
un-booked. The drone's parallel USB TX pool reorders bodies on air, so a
short guard reads reorder as loss; the shipped guard is 192
(`gs/bundle/maburgs.default.toml`; guard sweep:
`docs/findings/2026-09-23-cca-on.md`). To read real air loss use
`streams[*].recovered` + `abandoned` per second, or `fec.log`.

### `link.streams[]` — per-layer FEC, arrival accounting, salvage

One entry per video layer (`stream` 0 base, 1 enh) once that layer has
been seen: the commanded `ov`, FEC `recovered`/`abandoned` (and their
`_stale`/`_s` variants), rates, and:

- `arr_expected / arr_arrived / arr_expected_stale / arr_arrived_stale /
  arr_late` — cumulative arrival-time pre-FEC accounting, the ladder's util
  input. `expected` is seq advance past the settle line, `arrived` what was
  heard, `_stale` the pre-transition share the watermark attributes to the
  old rung, `late` symbols heard after their seq was already booked missing
  (healthy = 0; a rising `late` means reorder deeper than the guard).
  Current-only loss = `1 − Δ(arrived−arrived_stale)/Δ(expected−expected_stale)`.
- `corrupt`, `salvaged`, `sub_fail`, `salvage_only` — corrupt-frame
  salvage. The GS asks devourer to keep FCS-failed frames
  (`rx.keep_corrupted`, set unconditionally in `gs/src/radio_frontend.cpp`),
  so a corrupt frame reaches the aggregator, bumps that card's `crc_fail`,
  stays out of the seq walk, and hands its body to the UEP decoder, which
  keeps the SBI sub-blocks whose own CRC16 still passes. `corrupt` =
  FCS-corrupt bodies delivered, `salvaged` = CRC16-clean sub-blocks taken
  out of them, `sub_fail` = sub-blocks that failed (from any body),
  `salvage_only` = seqs whose ONLY arrival inside the arrival guard was a
  salvaged sub-block. `salvaged` is the upper bound and `salvage_only` the
  value: the other card's clean copy of the same body shadows most salvaged
  sub-blocks, and a salvaged repair symbol never counts because it carries
  no seq of its own (flights: `docs/findings/2026-09-09-sbi-salvage-flights.md`).
  `flightreport.py` prints these as its SALVAGE section.

### `link.video` — delivery, latency, slices

Frame rate, bitrate, jitter EMA, AU ring publish counters (`ring`), and:

- `lat` — the head-segment latency breakdown: `n` plus `[p50, p99]` pairs
  for `enc`/`dq`/`air`/`fec`. The whole key is OMITTED (not null) until the
  AU ring's latency stamps are flowing, so its presence means "real numbers
  here".
- Slice salvage (`docs/slices.md`): `slice_salvaged` (AUs rebuilt this
  window), `slices_kept` / `slices_filled` (slices that arrived complete
  vs. were skip-slice fills, summed over those AUs), `slices_after_hole`
  (kept slices past the first missing one), and `slice_fallback{no_params,
  unsupported, islice, no_template, geometry}` (why a damaged split AU fell
  back to the truncated-prefix passthrough instead of salvaging). maburtop
  shows `salv`; `flightreport.py` has a SLICE SALVAGE section.

### `link.rtt` — control-path RTT and the absolute latency floor

The drone echoes which RCF `rcf_age_ms` ages against (`Telem.rcf_seq_echo`)
plus its MI-domain clock at telem build (`pts_at_build`), and maburgs
matches the echo against its recorded send times —
`rtt = (telem_rx − rcf_send) − rcf_age`, every 1 Hz telem a sample. Keys:
`ms` (EWMA), `min_ms` (session min — the floor bound), `n`, `pts_off_us`
(min-RTT-filtered `pts − GS-mono`; null until the drone ships a usable pts
clock), `floor_ms` (maburgs' anchor + offset = absolute network floor). The
whole block is null until the first matched sample.

Read it as CONTROL-path time: the telem reply queues behind video on the
drone's half-duplex TX, so `ms` inflates under saturation (an honest
congestion signal, not PHY RTT), and the offset's residual error is the
up/down asymmetry of the best samples, ±1–2 ms class. On the player OSD the
offset (combined with the player's OWN anchor) folds the floor into the
`lat:` figures, which carry no marker while their e2e is still relative
(estimator cold, or sync lost at range: the floor freezes at last-good and
drifts ~1 ms/min of outage) — the cell has no room for a qualifier. What
the absolute figure still excludes: sensor exposure/readout before the pts stamp and
everything past scanout start (HDMI + display processing) — see
`docs/findings/2026-08-31-latency-budget.md`.

### `link.rcf_slot` — RCF slotter releases

Counts how the RCF slotter released each control-frame send: `au` (at an AU
completion with idle ahead, incl. the in-grace immediate sends), `probe` (at
the probe body's arrival), `timeout` (hold reached `link.rcf_slot_hold_ms`),
`passthru` (slotter off, or no AU in the last 100 ms). Cumulative.
`tail_ub_ms` is the slotter's learned completion→probe deadline (a lost
probe's fallback release; ceil(decaying max) + 1). Healthy video = almost
all `au`+`probe`; at a high duty cycle `timeout` climbs and phase-locks to
the burst (`docs/findings/2026-09-05-probe-blanking-fix.md`). Mechanism:
`docs/link-adaptation.md` "RCF slotting".

### `link.probe` and `classes.probe`

The probe stream's live gate state, exported unconditionally (also in
static-pin mode, where there is no controller): `link.probe = {on, rung,
mcs, state, u, loss, streak_bodies, n, exp, rx, off_profile, cards:[{loss,
rx}]}` — `u`/`loss` are `null` when the gate has no usable sample yet;
`streak_bodies` is the clean streak in probe bodies, the unit
`link.probe.clean_bodies` is set in. Per card, `classes` has `s0`, `s1`,
`probe`, `msp`, `ctrl` (`RfClass`, `gs/src/aggregator.h`); `probe` carries
the probe stream's own RSSI/SNR/EVM. maburtop's LADDER panel shows a live
gate line, `probe: r3 mcs5 clean 54b u0.12 n48 | c0 0.00 c1 0.05` (`54b` =
the clean streak in bodies). Mechanism: `docs/link-adaptation.md` "Probe
stream".

### `link.nack` — software NACK

`link.nack{requests, repeats, syms_requested, tail_requests, filled,
late_fill, wasted, dropped_deadline, suppressed, lead_skipped, fill_pps,
fill_ms{p50,p90,max}, settle_ms, late_ms_max}`: the GS NackTracker,
counters cumulative, `fill_*`/`late_ms_max` per export window; present only
while `[link.nack] enable`. `lead_skipped`: seqs whose request was withheld
by `link.nack.min_lead_ms` because the answer could not land before the
deadline. The drone side is `drone.nack` (below). maburtop draws a `nack:`
line in the LADDER panel. Design: `docs/fec-nack.md`.

### `cards[]` — per card

- `id`, `up`, `kind` (`"usb"` | `"relay"`; relays follow the USB cards),
  `frames`, `crc_fail`, rates, and `classes` (per-RF-class RSSI/SNR/EVM).
- `crc_fail` counts FCS-failed frames (see salvage above). Foreign traffic
  that fails FCS is counted too (the SA filter cannot trust a corrupt
  address), so a busy channel shows a slow creep with the drone off;
  `MABUR_GAPLOG=1` prints one `crcfail card=… seq=…` stderr line per event.
- `energy = {cca, fa, own, foreign, igi, busy_pct, own_air_pct}` — refilled
  from the verdict engine's ~150 ms window reads. `busy_pct` is NHM busy %
  (`null` when that window's reading was invalid or none has landed),
  `own_air_pct` the GS's own reconstructed airtime on that card (`null` only
  before the first window). maburtop's `busy` column tracks `energy`, and
  its `fbusy` column is `max(0, busy_pct − own_air_pct)` — the same
  foreign-busy-airtime figure the verdict's `blocked` bit and both rankers
  use (`docs/findings/2026-09-25-nhm-airtime-spike.md`).
- `dwell = {visits, score, cost_us}` (`null` until that card's first
  completed scout dwell) — `visits` counts every dwell, success or failure;
  `score`/`cost_us` are the last **successful** dwell's.
- `relay` (relay cards only; `docs/cpe510-relay.md`) = `{state, owned, ch,
  sec, frames, gaps, your_drops, tx, tx_fail, tx_refused, reconnects,
  tx_scan_drop, sweeps}`: `state`/`ch`/`sec` are the last `STATUS` (state 0
  tuned, 1 retuning, 2 failed, 3 refused; `sec` 0 HT20, 1 HT40+, 2 HT40-),
  `owned` is owned-and-tuned on our target, `frames` the `FRAME`s seen,
  `gaps` the relay→GS `seq` gaps — Ethernet/UDP loss between the CPE and the
  GS, **not** air loss — `your_drops` the relay's own count of frames it
  could not send us, `tx`/`tx_fail`/`tx_refused` the relay's injection
  counters (`tx_refused` = `TX` messages it would not inject: not from the
  owner, `mcs` > 7, a reserved flag bit, a bad length), `reconnects` the
  client restarts (refused-restarts and reopens after a lost relay),
  `tx_scan_drop` owner `TX` frames dropped while a sweep had the relay off
  channel, `sweeps` the `SCAN`s this card has sent. `RelayStatsIn` also
  carries `you_own` and the transport's `rx_drops`/`tx_drops`, but maburgs
  does not export them — only the web GS's stats JSON does. On a relay card
  the per-class `snr*`/`evm*` are always `null` (the relay's "SNR" is RSSI
  above a calibrated floor — `CardCaps::snr_ok = false`); RSSI is real;
  `energy.cca`/`fa` read 0 and `busy_pct` stays `null` (no NHM);
  `own`/`foreign`/`own_air_pct` are real.

maburtop labels relay rows `r<id>` (USB `c<id>`) and adds one strip row per
relay under the cards grid (`relay st= own= gaps= drops= txref= reconn= sw=
txsd=`, warn when not owned or `gaps` grew); the player OSD draws an `R` id
and a dashed SNR cell and counts a relay card as heard on RSSI alone.
maburgs' 1 Hz stderr `stats:` line appends `relay[own= gaps= drops=
reconn=]` per relay card.

### `hop` — in-flight channel hop

Top level, unconditional (idle defaults while pinned): `hop = {verdict,
evidence, ref_rung, epoch, state (idle|ordered|verifying|hold), target,
hops, holds, last_ms, sweep_timeouts}` — `ref_rung` and `target` are
`null` while unfrozen / before the first-ever order, `last_ms` is `null`
until any hop event has fired. `sweep_timeouts` counts relay `SCAN`s that
got no `SCAN_RESULT` before their timeout
(`max(1000, passes·n·(observe_ms+40)+300)` ms); 0 on a GS that never sweeps
a relay. maburtop's header shows `hop <state>/<verdict>`. Full key
semantics, the OSD `(h)` mark and the `flightreport.py` HOP section:
`docs/inflight-channel-hop.md`.

### `drone` — telemetry from T_TELEM

`null` until the first Telem. Values come from the drone's ~1 Hz T_TELEM and
are repeated on every record until the next one, so sample per-period
fields once per `drone.tlm_seq` (`flightreport.py` does).

- `state` (`boot`|`rendezvous`|`linked`|`failsafe`), `tlm_seq`,
  `tlm_age_ms`.
- `failsafe_shed` (rung 0 / lost link) and `congestion_shed` (Telem flags
  bit4: `RcAgent::run_congestion_guard` holds a drone-local
  TxQueue-pressure / USB-failure shed, `docs/link-adaptation.md` "Drone
  congestion shed"). A shed enh layer is silence to the GS ladder, so this
  bit is the only way to attribute an enh gap to congestion rather than RF.
  maburtop renders the pair as `shed FS|CONG|off`; `flightreport.py`'s
  DRONE TX PATH section counts the periods each held.
- `low_power` (flags bit7) — the low-power (disarmed) operating point
  (`docs/link-adaptation.md`, "Low-power (disarmed) mode"). maburtop shows
  `LP` on the summary SYS row and on the DRONE panel's SoC line; the compact
  OSD tints its fps cell caution while set and the sideport is fresh.
- `auth_reject` (flags bit1) — control-frame auth rejections this period,
  `docs/link-pairing.md`.
- `rec = {state, err}` — the VTX onboard SD recorder (`docs/vtx-recorder.md`).
  `state`: 0 off, 1 recording, 2 error. `err`: 0 none, 1 Disabled
  (`[record] enable = false` or ch1 failed to bind), 2 NoSlot (no
  `/sys/class/mmc_host/mmc*`), 3 NoCard (no `/dev/mmcblk0p1`), 4 NotMounted
  (`record.dir` is not a mountpoint), 5 LowSpace (below
  `record.min_free_mb`), 6 WriteError. An error stays until the GS wish goes
  off (status back to `{0, 0}`); the next on retries, the drone never
  retries by itself. maburtop shows `VREC` /
  `VREC!<OFF|NOSLOT|NOCARD|NOMNT|FULL|WRERR>`; the player turns it into the
  REC field's VTX leg.
- `nack = {rx, retx_syms, retx_refused}` — the drone's T_NACK answers, per
  Telem period.
- `rcf.{age_ms, rx_pps}`, `enc.cmd_kbps`, `txq_wait_ms` (per-period max
  TxQueue wait), `txq.{drops, drop_pps}`, `radio.usb_fail`.
- `radio.rx = {own, foreign, crcfail}` — the drone's own RX-side view of
  the channel for the last telemetry period: CRC-clean RC from this GS
  (`own`), CRC-clean not-ours (`foreign`, other 802.11) and CRC-failed
  (`crcfail`, preamble heard, payload undecodable). With carrier sense on,
  `foreign + crcfail` is what the drone's transmitter deferred to. Software
  counts, not the chip's CCA/FA registers — that read stalls the drone's TX
  pool (`docs/findings/2026-09-23-cca-on.md`). Per period, NOT cumulative.
  `flightreport.py` has a DRONE RX section; maburtop shows the split on the
  drone `radio` row and derives its `DEAF` cell from own + foreign +
  crcfail = 0.
- `uplink = {rssi_a, rssi_b, snr_a, snr_b}` — per drone antenna (the
  dead-antenna check); all `null` while the drone has heard no RC frame.
- `sys.{soc_temp_c, cpu_pct}` (`cpu_pct` null on the first tick after a
  maburd start).

## Debug-log session directory

maburgs writes a per-session directory when `debug_log.enable` is set
(shipped default `false` — the bench GS turns it on): `<debug_log.dir>/NNNN/`
holding `flight.jsonl`, `ctl.log`, `probe.log`, `au.log`, `fec.log` and
`scan.log` (`au.log` only while `[au_ring] enable`); maburplay writes
`lat.log` and `frame.log` into the same directory by following the
`/tmp/mabur-session` marker and holds no logging config of its own
(`gs/src/debug_session.h`). Nothing is written until the knob is set, and
`enable = false` removes the marker, which stops maburplay too.

- **Session identity.** NNNN is one past the highest existing index — never
  a timestamp, since the GS RTC is wrong at boot. The marker lives in tmpfs,
  so a reboot starts a new session while a 2 s wrapper respawn rejoins the
  current one and appends (which is why a format-marker line can appear more
  than once in a file).
- **A drone restart is a new flight.** When T_TELEM's `tlm_seq` steps
  backwards by more than 100 (maburd restarts it from 0; a fade or
  re-rendezvous keeps it climbing; `gs/src/drone_restart.h`) maburgs rotates
  in place — next NNNN, marker rewritten, all six files reopened under it
  with their format marker at the top — with no process restart and no video
  blink; maburplay's `lat.log` and `frame.log` follow the marker within 5 s. Cumulative
  sideport counters do NOT reset at a rotation (same maburgs process), so the
  new `flight.jsonl` starts at nonzero values; `flightreport.py` differences
  within the file. A maburd crash-restart mid-flight therefore splits that
  flight in two, by design.
- **One clock.** Every file stamps CLOCK_MONOTONIC, so any two rows join
  directly.
- **One writer thread.** All six maburgs files go through one writer thread
  (`gs/src/log_writer.h`) whose per-stream ring can fill under load; when it
  does, the writer counts the dropped lines and at its next 1 Hz flush
  appends `# dropped N` into the affected file, so a gap is visible in the
  data itself. `flight.jsonl` opts out (`mark_drops=false`: NDJSON can't
  carry a comment line, and a gap is already visible from the datagram's own
  `seq`). Every logger is non-fatal.

Older recordings (`ctl-NNNN_<date>.log`, `au-NNNN.log`, `lat-NNNN.log`,
`# sync` anchors, wall-clock `t_us`) and every previous format version are
described in `docs/data-provenance.md`; the analyzers still read them.

### `flight.jsonl`

One sideport snapshot per `stats.interval_ms`, written straight from
`StatsExporter` (no UDP round-trip). No format marker; the datagram's `v`
and `seq` are the self-description.

### `ctl.log` (`ctllog 12`)

The adaptive-link record, written also in static-pin mode. Header
`ctllog 12 <header_info>` (the ladder with per-rung width, thresholds), then
compact lines — formats locked in `gs/src/ctl_log.h`:

- `S` — dwell sample every `debug_log.ctl_period_ms`: rung, util `u`/`u3`
  (arrival-booked pre-FEC loss), residuals, RF labels, fade deltas, and three
  trailing probe columns `probe_rung probe_u probe_n` (`-1 nan 0` when
  nothing is commanded).
- `E` — rung transition with reason.
- `P` — probe gate STATE EDGE (`clean|lossy|noinfo`) — a gate edge, not a
  probe outcome.
- `N` — penalty booked. `R` — per-rung EWMA store snapshot, every
  `debug_log.rung_period_s`.

In static-pin mode (`link.static_mcs >= 0`) the `S` line's controller
columns are frozen (`u`/`util` read 0, `E`/`P`/`N`/`R` never fire); only
its probe columns move, `<probe_rung> nan <probe_n>` — `probe_rung` is
`min(probe.rung_offset, top)` off a frozen rung 0, `u` is `nan` because the
gate never leaves `Off`, and `probe_n` is nonzero whenever
`link.probe.pin_mcs >= 0`. In pin mode only the probe rows, and
`probe.log`, carry information.

`flightreport.py`'s `PROBE GATE` report scores the edges as a demote
predictor: per demote episode, `lead` is the time from the FIRST lossy edge
since the held rung was entered (the last `E` line before the episode,
either direction) — edges before that transition were measured under a
different hold and never count — and a lossy edge is a false alarm unless
the next `E` line is a demote within 10 s. The episode report also prints
an "s3-settle-refire canary" (demotes landing 300–360 ms after an
`s3_residual` demote, the debris double-step `transition_edge.h` removed;
~0 expected). Controller invariants behind these lines:
`docs/link-adaptation.md`.

### `probe.log` (`probelog 3`)

One row per finalized received probe body — roughly one per AU, far denser
than the ctl log's `S` lines (`gs/src/probe_log.h`). Header
`probelog 3 bpb=<bpb>`, then
`<t_ms> <seq> <mcs> <bw> <enh_fid> <blocks_ok> <card_mask> <snr_c0> <snr_c1> <evm_c0> <evm_c1> <first_ms>`.
`t_ms` is the finalize tick (~10 ms coarse), `first_ms` the radio's arrival
stamp of the body's first sight on any card (mono ms to 3 decimals, same
clock as `au.log`'s `t_complete`), `blocks_ok` the union of surviving
blocks, `card_mask` the cards that delivered any block, snr/evm per card in
dB (`nan` when that card heard nothing). A row is written for EVERY
finalized body, on- or off-profile: `mcs`/`bw` are that body's OWN profile,
so an RCF-lag body (arrives just after a profile switch) logs at its stale
mcs instead of vanishing. A wholly-lost body is the only case with no row;
`flightreport.py` derives losses from `seq` gaps and joins rows to `au.log`
on `enh_fid` (nearest completion in time — the id wraps every ~36 min) to
print the completion→probe offset percentiles. `tools/bench/probesend.py`
adds the GS send stamps from a `MABUR_GAPLOG=1` run to place every send
against the probe (`docs/findings/2026-09-05-probe-blanking-fix.md`).

### `au.log` (`# aulog 6`)

Per-AU meta rows, fed in-process from `AuRingWriter::last_record()`
(`gs/src/main.cpp`'s FrameStream finish callback; `gs/src/au_log.h`) —
maburgs is the ring WRITER, so the log cannot miss an AU to a ring overrun.
Nineteen columns:

    t_us pts sid fid len flags nal0 t_first t_complete enc dq air slices kept filled after_hole t_last t_drain t_push

`t_us` is CLOCK_MONOTONIC µs. `t_first`/`t_complete` are the AU's mono-µs
latency stamps (first body / finish()). `enc`/`dq` are the drone's
SBI-latched `enc_us`/`drone_q_ms` (venc encode time, true TxQueue wait —
~0 when healthy) carried on the AU's first fragment. `air` is the drone's
modelled air backlog at that frame (`AirClock`, `docs/link-adaptation.md`
"Drone air clock"). `slices kept filled after_hole` are slice salvage
(`docs/slices.md`), all 0 on an AU that was neither split nor salvaged.
The last three are GS stamps (mono µs, 0 = unknown) that split the fec
segment per frame. `t_last` is the arrival of the AU's LAST body:
`t_last − t_first` is the arrival span (drone hand-off plus air),
`t_complete − t_last` the GS publish tail. `t_drain` and `t_push` belong to
the AU's last-pushed fragment and split that tail further: `t_drain −
t_last` is the wait in the RX queue before the core thread drained the
body's batch, `t_push − t_drain` the work on earlier bodies of that batch
plus FEC decode, `t_complete − t_push` FrameStream emit and the ring write.

### `fec.log` (`feclog 3`)

Per-episode FEC loss record (`gs/src/fec_log.h`), the measurement behind
"is the rung table's overhead pair oversized". A *loss episode* is a run of
source symbols on one video layer that the channel never delivered directly
(repair-recovered-then-heard symbols do NOT count: the repair merely won an
arrival race), merged with any other such run within one repair window of
it, since those compete for the same repairs. `SwDecoder` books it at
horizon eviction — the only point where "never delivered" is final — so a
row lands roughly a horizon after the loss. Header `feclog 3`, then
`<t_ms> <sid> <mcs> <bw> <ov> <first_seq> <span> <m> <rec> <rtx> <aband> <stale> <r> <w>`:
drain tick (mono ms, ~10 ms coarse), video layer (0 base / 1 enh), the op
MCS and width and that sid's commanded overhead at drain time (so a row
scores against its own rung with no ctl.log join), wire seq of the first
missing source, seqs spanned, missing = recovered + rtx + abandoned (`rtx`:
filled by a software-NACK retransmit, `docs/fec-nack.md`), of those how many
fell below the transition watermark (`stale`, the debris class the ladder
excludes), distinct covering repairs received (`r`, a two-card copy counts
once) and the repair window as flown (`w`).

`flightreport.py`'s FEC EPISODES section groups rows per (sid, mcs, bw, ov)
and prints, for the non-stale ones, the overhead each episode would have
needed, `ov_req = (sqrt(1+4c)−1)/2` with `c = m·ov·(1+ov)/r` — at overhead
x the same lost air carries `m(1+ov)/(1+x)` sources against `r·x/ov`
covering repairs — plus how many episodes would have failed at
0.25/0.35/0.50/0.75/1.00. Read it as the input to a *static* retune of
`[[link.ladder]]`'s `overhead_base`/`overhead_enh`. Nothing consumes the
file live; `fec.log` alone is also a valid `flightreport.py` argument. The
first ~18 s after a maburgs restart log a block of rung-0 boot warm-up
episodes — drop them by time. Bench record:
`docs/findings/2026-09-16-fec-log-overhead-bench.md`.

### `scan.log` (`scanlog 6`)

The channel-selection record (`gs/src/scan_log.h`, formats locked there).
Six record letters, one line each:

- `C` — a card's adapter-caps identity plus sensor validity flags, once at
  bring-up.
- `D` — one scout dwell (the channel-ranker's raw input), boot-time or
  in-session (trailing `sess` flag and step-timing columns), with the
  dwell's width, NHM busy %, and a relay sweep rx % (`-` for a USB dwell,
  `docs/cpe510-relay.md`).
- `K` — the pick at freeze, with the full ranking.
- `M` — a GS retune that changes where the link lives (`commit`,
  `ack_override`, `link_found`, and the hop reasons
  `hop_lead`/`hop_follow`/`hop_withdraw`/`hop_one_card`).
- `V` — one verdict-engine window, on every verdict change and every
  non-healthy window.
- `H` — one hop-controller event (`order`, `lead_confirm`,
  `one_card_retune`, `verify_pass`, `verify_fail`, `withdraw`, `hold_cap`,
  `hold_exhausted`, `relocate`, …).

Full formats, the config, and the `cca − own` ranking assumption for the
boot-time records are in `docs/channel-select.md`; the in-flight hop's rule
table, evidence bits and hop sequence in `docs/inflight-channel-hop.md`.

### `lat.log` (`# latlog 2`)

maburplay's 1 Hz `lat:` stderr line, also appended to `lat.log` in the
current session directory: `# latlog 2`, then one `<mono_us> <lat-payload>`
line per second, line-buffered (`gs/player/src/lat_log.h`). The marker is
re-read every 5 s; a missing session or failed open (DVR not mounted yet)
never blocks and never spams, and the stderr line keeps going regardless.
`tools/bench/latab.py latA.log latB.log` reads a pair of these logs and
prints the vsync A/B verdict (four log-derived gates: `e2e` p50 B≤A−8,
`dsp` p50 B≤6 (level), `dsp` p99 B≤A−8, `dsp` p50 4 s-bucket sweep ≤3
(flatness) — `anchor=warm` windows are excluded from all four, with a
printed count).

### `frame.log` (`# framelog 2`)

The per-frame rows behind the `lat:` line, from the same `LatTracker`
window (`gs/player/src/lat_tracker.h` `FrameRow`): one row per frame that
completed the whole chain and joined the window — a displaced frame or one
submitted on a cold anchor gets no row, exactly as it is absent from the
`lat:` percentiles. Written by the same follow-the-marker writer as
`lat.log`, in the same idle-gated 1 Hz burst (never on the flip or decode
path), buffered so each burst reaches the file in one write. Twenty columns:

    t_flip pts fid sid len flags slices t_submit t_complete t_decoded t_release enc dq air fec dec reg dsp e2e exact

`t_flip` is the frame's scanout-start stamp (mono µs); `t_submit` is when
the player handed the AU (or its first slice) to the decoder, `t_complete`
the ring's finish stamp (as `au.log`), `t_decoded` when the decoded picture
came back, `t_release` when the regulator released it. `enc dq air fec dec
reg dsp e2e` are the frame's own segments (µs, `dec = t_decoded −
t_complete`, `e2e` the sum); `exact` is 1 when `t_flip` came from a real
vblank event. `pts` joins the row to `au.log` (same value), which carries
`nal0`, `t_first` and the GS stamps; `fid`, `sid`, `len`, `flags` and
`slices` are copied from the AU record so a row reads on its own, `flags`
in hex as `au.log` prints it (`framelog 1` printed it in decimal). A
streamed frame has `t_submit < t_complete`, so `t_decoded − t_submit` is
not decode time for it; split `dec` with `t_complete` instead.

### `cal.log` (`callog 3`)

The TX-power calibration record (`gs/src/cal_log.h`). It writes regardless
of `debug_log.enable` — a calibration run is a bounded trace on a deliberate
operator action, not continuous flight logging — into the session directory,
or `<debug_log.dir>/cal` with logging off. Location, record format and the
`maburcal report` reader: `docs/calibration.md` "Offline analysis:
`maburcal report`".

### Analyzers

All take a session directory (resolved by `tools/session.py`), a single
legacy file classified by its format marker, or no argument (the
highest-numbered session under `/media/dvr/log` — highest index, never
newest mtime).

- `tools/flightreport.py` — the post-flight report over the whole session
  (sections named above, plus BW40, NACK, wall/rung-store, episode reports).
- `tools/flightjitter.py` — reproduces the player's jitter EMA from the AU
  rows (arrival basis `t_complete`), splits it into size-explained vs
  residual (`docs/airtime-model.md` §4), and classifies each stutter event —
  `gap` / `rung-change` / `size` / `fec-wait` / `loss-recovery` /
  `transport` — using the jsonl for ladder and loss context. `fec-wait` is
  direct per-frame evidence (the stutter AU's own `t_complete − t_first >
  20 ms`), checked before the jsonl-inferred `loss-recovery` class so it
  wins when both apply. Tests: `ctest -R test_flightjitter`.
- `tools/bench/airdrain.py` — see "Tools: which one when".
- `tools/bench/probesend.py` — see `probe.log` above.

## Player (maburplay) stderr lines and OSD

maburplay logs to `/tmp/maburplay.log` (tmpfs). maburgs' 1 Hz `stats:`
line is in `/tmp/maburgs.log`, maburd's in `/tmp/mabur.log`. These are
numerically redundant with the sideport but persist on disk until reboot;
the UDP feed is ephemeral — they are the post-mortem when no consumer was
listening.

### `stream:` line (streamed decode)

Every 5 s, printed by the decoder-input thread (FeedLoop;
`docs/slices.md` "Part 2 (as built)"): `on|off:<reason>`, `streamed` (of
them `salvaged`), `aborted` (`stream_aborted`, one rkvdec reset each),
`whole`, `opened`/`open_aborted` (ring records seen open / ended without
closing), `wake_us=p50/p99` and `n` (closed-record delivery − maburgs'
`t_complete`, µs), `refused` (STREAM_APPENDs the decoder refused,
`MppBackend::stream_errors`) and `parks`. `maburplay --oneshot` JSON carries
`streamed`, `stream_aborted` and `stream`. On shutdown maburplay logs
`maburplay: shutdown: <stream idle|stream finished|stream aborted> pts=…
waited=… ms…` (how the picture that was streaming when SIGTERM arrived was
finished before decoder teardown) and `maburplay: feed: delivered=…
submits=… parks=… notes_dropped=…` (the feed thread's tally for the run).

### Regulator line

The 1 Hz `regulator:` line: `held= late= replaced= disconts= hold_ema=
present_jitter= vsync= skips= fallback= pend= heals= pdrop= chained= chain=
chain_max= chains= cuts= alw= lslot=`.

- `vsync=locked|fallback` — whether the vblank estimator is locked (servo
  release) or has fallen back to the `anchor_floor(pts) + regulate_ms`
  rule.
- `skips=` — deep-burst slot claims in servo mode: a frame whose natural
  vblank slot AND the next slot are both occupied claims the later one and
  the older occupant is dropped. Ordinary servo drops surface as
  `replaced=` evictions instead. The sensor/panel beat (the drone's fps vs
  the panel's 60 Hz) produces periodic panel repeats, visible in
  `--fps-log`, not here.
- `fallback=` — frames released via the fallback rule while
  `display.vsync_lock` is on (climbs during a cold start or a stale
  estimator; cold start needs 8 exact flips to first warm, validity is
  recency-based, so after a stall the counter stops within one fresh exact
  flip).
- `pend=` — the presenter's present()-while-flip-in-flight mailbox
  engagements, 0 with no presenter.
- `heals=` — chain-break slips: pending releases pushed one slot after two
  mailbox engagements within 100 ms (a backstop that fires ~only at
  startup).
- `pdrop=` — paced-mode mailbox drops: with the servo locked, a parked
  frame is a missed latch and is dropped at flip completion instead of
  resubmitted a period late, which keeps a single miss from becoming a
  one-vsync-late chain. A `pend` increment in servo mode therefore costs one
  dropped frame in the common case (two when the mailbox was already
  occupied — the replaced parked frame counts in `--fps-log`'s `repl=`).
  `pend`, `pdrop`, `replaced=` and `--fps-log`'s `repl=` together are the
  full drop accounting.
- `chained=`/`chain=`/`chain_max=`/`chains=`/`cuts=` — sequential-slot
  chains, the one-vsync-late chain the servo itself creates: one collision
  (two decodes inside one vblank window) shifts every following frame a
  period late until an arrival gap wider than a period frees a slot.
  `chained` = cumulative frames that took natural+1 vblank, `chain` the
  current run, `chain_max` the longest, `chains` runs started (chains/s ×
  mean length = chained/s), `cuts` runs cut by `display.chain_budget` (the
  frame that would extend a run past the budget takes its natural slot and
  the held occupant is displaced — one dropped frame, counted in
  `replaced=` too). The extra period lands in the `lat:` line's `reg`
  segment, so read `chained/held` per second next to `reg` p50. Code and
  bundle default to budget 0 (never cut): a chain then ends only on an arrival gap, so
  through the part of the sensor/panel beat where arrivals sit just past
  the release point the whole stream runs one period late (the `lat:` p50
  sawtooth) but nothing is dropped, and the refresh-start frame's ~10 ms
  of extra encode, air and decode fits inside that slack. A budget N
  instead keeps the lower latency and pays one dropped frame per cut; on a
  clean link the refresh-start frame misses its release in about half of
  each beat cycle and every miss then costs a repeat plus a cut (bench:
  budget 3 = 59–73 % of seconds with a visible event and 1.6–2.5 drops/s,
  budget 0 = 14 % and 0.16 drops/s, at +12.5 ms median). Records:
  `docs/findings/2026-09-02-regulator-chain-budget.md` (budget 3 chosen),
  `docs/findings/2026-10-15-regulator-chain-cuts.md` (budget 0 chosen).
- `alw=`/`lslot=` — capture-time scheduling (`display.sched_cap_ms`, 0 = off
  in code and bundle; bench result and cost:
  `docs/findings/2026-10-15-capture-sched.md`). In servo mode a frame's vblank follows its
  capture time, not its arrival: the first vblank whose release is at or
  after capture + delivery floor + `alw`. The window holds each frame's
  transit, arrival − capture pts on the raw clocks, measured before the
  regulator's own hold (enc+dq+air+fec+dec, not `reg`/`dsp`); raw rather
  than against the pts anchor, whose floor snaps down on one fast frame
  and would shift every later sample. The floor is its p1 over the last
  600 frames (~10 s), `alw` is p99 − p1 + 1 ms capped at `sched_cap_ms`,
  and salvaged AUs stay out of the window. A late-tail frame within `alw`
  keeps its slot, so in the phases where frames arrive near the latch the
  repeat/skip judder (OSD LAT p99 a refresh above p50) becomes one extra
  refresh on every frame instead (p50 meets p99). `lslot` counts frames
  that missed their capture slot (loss, or a delivery shift beyond the
  cap); they are placed by the arrival rule above, and the next on-time
  frame that claims the same vblank wins it, so a stall costs a local
  freeze and catch-up with no trailing chain.

### fps-log, display bring-up, MSP OSD

- `--fps-log` prints a once-per-second `fps-log:` line; with a burned DVR
  it carries the recorder fields including `burn_ctfb=`/`burn_ct_ms=`/
  `burn_enc_ms=` (`docs/colortrans.md`).
- The MSP OSD is rendered by maburplay from the UDP snapshot feed maburgs
  emits; its startup line and blanking notices land in
  `/tmp/maburplay.log`.
- maburplay drives the display from process start: it modesets immediately
  with a splash image (`/usr/local/share/mabur/splash.bin`, raw XRGB8888,
  regenerate with `tools/gen_splash.py`) so the sink locks a mode before
  video exists, and retries display acquisition once a second while none is
  connected — a display plugged in after the player started is picked up
  without a restart. That holds only if a display was NEVER acquired: one
  that disconnects after a successful acquire is deliberately not recovered
  (KMS retains CRTC state and a replug normally re-lights it on its own).
  Neither has a config key. The splash shows only until the first decoded
  frame — it is an aerial photo that would read as a live feed mid-flight,
  so a mid-flight replug comes up on video. Log lines:
  `no display at startup -- retrying …`, `display acquired after N.N s`;
  per-attempt DRM failures are silenced on purpose (/tmp is tmpfs).

### GS link-status OSD

maburplay draws it from the SAME datagram `maburtop` reads (`osd.gs.port`
= one of maburgs' `[[stats.out]]` ports), so the screen and the recorder
cannot disagree — if the OSD shows something surprising, the answer is in
`flight.jsonl`. Config is `[osd.gs]` in `/etc/maburplay.toml` (`enable`,
code default false, bundle true; `stale_ms` dims every link-derived field
after silence; fps/jitter/bitrate/REC are player-measured and never dim).
There is one layout, the compact bar — two rows along the bottom edge,
radio above and picture below, each item an icon, a space and its figures,
with the REC indicator alone in the top-right corner
(`gs/player/src/gs_compact.cpp`). `[name]` below stands for the icon:

```
                                              ● REC 12:47   <- top right
...
[wifi] 149 [tune-vertical] 5/40 [chart-donut] 62% [signal] -70/-72 [waveform] 22/20 [thermometer] 41
[speedometer] 8.1 [aspect-ratio] 720p [filmstrip] 60 [pulse] 5.2 [timer-outline] 45/78 [link-variant-off] 0.3/0.0
```

| icon | item | source |
|---|---|---|
| wifi | channel the RECEIVER is tuned to, `(a)` auto-selected / `(h)` hopped suffix; `KEY?` replaces the item on a link-key mismatch | `link.channel` |
| tune-vertical | MCS / channel width (MHz) | `link.ctl.rung` (else `link.op`) |
| chart-donut | airtime % | `link.air_pct` |
| signal | RSSI per GS card, dBm | `cards[]` |
| waveform | SNR per GS card, dB | `cards[]` |
| thermometer | drone SoC temperature, °C | `drone.sys.soc_temp_c` |
| speedometer | video bitrate, Mb/s | player-measured |
| aspect-ratio | decoded picture height + `p` (1440×1080 reads `1080p`) | player-measured |
| filmstrip | fps on screen | player-measured |
| pulse | frame-interval jitter, ms | player-measured |
| timer-outline | latency p50/p99, ms | player-measured |
| link-variant-off | loss before/after FEC, % | `link.pre_fec_loss` / `link.residual_loss` |

The icons are Material Design icons baked from JetBrains Mono Nerd Font
Mono (`gs_icon` in `gs_compact.h`, `BAR_ICONS` in
`tools/msp/gen_gsfont.py`); each is one cell wide. Only the icons come
from that font — the figures stay upstream JetBrains Mono, whose hinting
the Nerd Fonts patcher changes. A missing value reads
`--`, never zero. `lat` reads `--/--` while the latency anchor is cold —
each figure is ONE REAL FRAME from the last 1 s window, ranked by e2e, and
the pair is read together because p99 alone is a tail statistic that gets
misread as typical (`docs/findings/2026-08-31-dq-spike.md` §20); the
per-segment breakdown is not on the glass (the `lat:` line, `lat.log` and
`flightreport.py` carry it). No meters or bars. Two colour exceptions,
both single-threshold: the fps item turns caution while `drone.low_power`
is set, and temperature turns caution from 70 °C; both tints drop while
the sideport is stale. An icon takes its item's colour. The REC indicator
is a red dot, `REC` and an mm:ss clock (`REC FAULT` in amber), with a VTX
leg sized by `dvr.target` (`docs/vtx-recorder.md`); armed draws nothing.
It sits in the corner because in a row it would eat into the bar's width
and shift that row whenever the recorder was merely armed.

The two rows share columns: each item starts at the same x as the item
above or below it (ch/bitrate, mcs/res, air/fps, rssi/jit, snr/lat,
temp/loss), each column as wide as the wider of its two items' worst
cases, and the grid as a whole is centred. Every item sits in a box sized
for its WORST-CASE string (four cards), so short values leave trailing
space — that is what stops a row reflowing every time a figure gains a
digit. Only a card-count change moves the columns. The
type size is one size for both rows, and it is pinned: the largest baked
size at which a 76-character, 6-item reference row fits between the insets
— 25/37/48/75 px at 720p/1080p/1440p/2160p with the shipped asset
(`tests/test_gs_asset.cpp` pins them). That reference is the widest row
from when the items carried text labels; the icons made the real rows
narrower, and the freed width stays slack instead of growing the bar into
the picture. The worst-case grid (72 cells plus 5 gaps) must fit too, so a
wider item can never overflow. Two rows because all twelve items on one line would cap the type
too small to read on the GS screen.

The glyph atlas is `/usr/local/share/mabur/gs_osd.gfont`, committed and
staged by `tools/build-arm64.sh` — if it is missing, maburplay logs the
reason and runs with the MSP overlay only. Host-side you can see the actual
pixels without hardware: `maburplay --gs-render` dumps a rendered frame
(`--res WxH` and `--lat P50/P99` set the bar's player-measured fields; the
render is pixel-gated in `tests/integration/run_player_e2e.sh` PART D),
`tests/test_gs_asset.cpp` gates the real asset's layout at
720p/1080p/1440p/2160p, and `tools/bench/gs_osd_bench.cpp` measures
draw+quantize per update at 1080p and 2160p. Read that bench before
changing anything on this path: it runs on the 2 ms pump loop, and the rule
of thumb is that anything projecting past ~1.5 ms on the A55 is a defect. A
full repaint costs more than that and is accepted, because it is a
startup/re-layout event (the quantize half is burned-DVR-only, and
`ring.pump(2)` is a `poll()` ceiling over a slotted shm ring, so the cost
lands as one-vsync-late presentation rather than a lost AU). What is NOT
accepted is anything that puts a full repaint on a *cadence*: the burn
restate after an MSP collision is scoped to the fields the collision
actually hit (`osd_compose.cpp`).

### Record button (GPIO)

`maburplay` toggles the DVR from a button on the GS header: `[input.rec]`
in `/etc/maburplay.toml` (`pin` is the header pin number, required,
resolved at startup by matching the kernel's line names — the Radxa ZERO 3
names its header `PIN_7`…`PIN_40` across gpiochip1/3/4;
`active_low`/`bias` default to a button between the pin and GND with the
internal pull-up). No `[input.rec]` table means no button. Short press,
50 ms debounce, edge-triggered; `kill -USR1 $(pidof maburplay)` is the same
press for benches. `dvr.target` (`gs` | `vtx` | `both`) picks what a press
records (`docs/vtx-recorder.md`); on the GS, each start produces one file
in whichever `dvr.mode` is configured:

- **raw** waits for the next sync point before the file opens, so the OSD
  REC indicator visibly lags the press. The sync point is decided by
  `mabur::RawDvr` (`common/include/mabur/raw_dvr.h`, shared with the web
  GS's local recording): a complete IRAP (`au_is_irap()`) once VPS+SPS+PPS
  are known. Older raw files that open on a non-IRAP need repair — see
  `docs/findings/2026-09-29-web-gs-raw-dvr-irap-sync.md`.
- **burned** resumes at the next decoded frame (`docs/colortrans.md`).

Files are `record-NNNN.mp4` under `dvr.dir`, indexed one past the highest
`record-NNNN` already on the card (`common/include/mabur/dvr_name.h`) — no
timestamp, since the GS RTC is wrong at boot — so the index climbs across
boots and never overwrites an earlier flight; files not matching the
pattern are ignored. `dvr.autostart` picks whether the player boots
recording or armed. There is no config kill switch and no `--no-dvr` flag:
`autostart = false` with no button is a player that never records, so give
a `--decode-only` measurement run a config with `autostart = false` —
otherwise the decode gate writes a raw file and charges SD-card I/O to the
fps number. Startup logs the resolved mapping and every toggle logs
START/STOP; the OSD REC field is the live indicator. GPIO failures (pin not
found, line held by another consumer) are non-fatal and logged once.

## Tools: which one when

- **Verifying maburgs itself → `tools/bench/ausniff.py`** (on the GS:
  `python3 ausniff.py --ring /dev/shm/mabur-au --seconds 30 --json`). It
  attaches to the AU ring read-only from OUTSIDE the daemon. This is the
  standing regression gate for any change that touches maburgs: the
  sideport is maburgs self-reporting, so gating a maburgs change on the
  sideport is circular — a bug that mangles frames or lies in its own
  counters sails through. Never replace the ausniff gate with sideport
  numbers. (Oneshot/post-hoc reads of a quiescent ring are exact; live mode
  is best-effort — Python cannot fence.) Host-side, the same invariant is
  `ctest -R 'gs_e2e|gs_au_e2e|player_e2e'` (byte-exact fixture-to-ring/AU
  comparisons via `tests/integration/verify_aus.py`/`--out-aus`). With ring
  v4 (`docs/slices.md` "Part 2 (as built)") maburgs publishes a slot while
  it fills it; ausniff counts closed records only (it waits on a slot still
  open and skips one whose AU overflowed, as a resync) and reports
  `nslices` — records per slice count, `1` = whole AU.
- **Verifying AU-completion cadence → `tools/bench/aucadence.py`** (on the
  GS: `python3 aucadence.py --ring /dev/shm/mabur-au --seconds 25 --json`).
  Same outside-the-daemon ring posture as ausniff; reports the base−enh
  completion offset (p50 of `arrival_mono − pts` per class, IDR-excluded —
  metric provenance in `docs/findings/2026-08-29-airtime-balance-spike.md`).
  This is the **standing acceptance number for any change touching the
  AirBalancer, the venc pipeline, UEP overhead, or the bitrate policy** —
  the sideport `jitter_ms` EMA is the symptom of this offset and is noisier
  (±1.4 ms vs ±0.5 ms), so judge on the offset. Recorded baselines (v4
  flag-day acceptance, 2026-08-30, mcs5 park): **−1.1 to −3.0 ms enh-late
  depending on scene/frame size** (21–33 KB frames, ~9.5 Mbps; the residual
  is encoder-side per-class completion latency) and **+1.1 ms at a
  pinned-mcs2** run. Regression criterion at the mcs5 park: **`--gate-ms
  4.0`** — inside the known encoder envelope anything passes; a transport
  regression (lost balance, one-sided overhead, rate misroute) shows as the
  offset leaving that envelope. The design target ≤0.5 ms becomes the gate
  once the encoder-side per-class latency is fixed. A capture whose
  per-class count is under `--min-samples` (default 100) is refused, not
  scored — enh silence (shed, loss-sim kill) is not a cadence sample.
- **Per-frame `air` excess around rung transitions →
  `tools/bench/airdrain.py`** (`python3 tools/bench/airdrain.py
  [<session-dir>] [--profiles] [--model]`, host-side, no lat log needed).
  Replays the player's `air` arithmetic over the AU log — `t_first − enc −
  q − pts` minus a `PtsAnchor`-style leaky min floor with the 2 s
  pts-discontinuity reset — and joins the ctl log's E lines: per cascade the
  250 ms-binned excess around the first demote with IDRs marked, peak /
  time-to-peak / settle, the 1 s pre-demote excess, and the first-500 ms
  on-air bytes against the new rung's nominal PHY rate; single-demote and
  promote peaks; steady-state excess per rung; standalone spike seconds;
  and a **transition IDRs** table — every non-starved `E` line with the
  first base IDR completing within 600 ms of it, kB p50/max by target rung
  split demotes | promotes and the ratio to the post-transition base P
  (`--profiles` lists every row). That table is the `venc.min_iqp` tuning
  readout (`docs/airtime-model.md`): read the rung-0 demote column after a
  flight, then move the floor. The default window runs from the first E
  line to the last S line; a starve opens no episode. `--model` compares the
  drone's per-frame `air` column against the measured excess
  (`docs/findings/2026-09-06-air-clock-flight.md`). Origin:
  `docs/findings/2026-09-05-probe-stream-flight.md` §9;
  `tests/test_airdrain.py` pins it on a synthetic cascade.
- **Packet-level forensics → capture tools** (`tools/bench/seqdump.py`,
  `decode_bodies.py`, `live_decode.py`/`live_play.py`). The sideport is
  aggregates; when a summary number looks wrong, these record raw bodies
  for offline dissection. (`fu_probe.py`, `fu_chain_analyze.py` read
  RTP-era recordings only.)
- **Loss injection → the loss-sim rig.** `MABUR_LOSS_SIM` builds only
  (`gs/src/loss_sim.h`, `docs/airtime-model.md` §4): `LossSim::kStreams` is
  `s0..s5`, `5` = probe — `tools/bench/losssim.py s5 eff=<pct>` injects
  loss on the probe stream for the gate's refusal test (parked clean,
  inject, expect `Lossy`/`probe_holds`, no promote). Host build: `cmake -S .
  -B build-losssim -DDEVOURER_DIR=$PWD/../devourer -DMABUR_LOSS_SIM=ON`
  (gitignored dir, separate from the normal `build/`).
- **Radio/PHY bring-up below mabur → devourer's own tools** (`rxdemo` with
  `DEVOURER_RX_ALLPATHS=1`, `doctor`, etc. — see
  `third_party/devourer/CLAUDE.md`). Use these when the question is about
  the chip/driver rather than the mabur link.
- **Bench harnesses** (`bench/linkbench`) drive special TX/RX modes for
  characterization; they are not monitoring tools. The drone radio RX can
  wedge after a linkbench run — restart maburd.
- **TX-power wall calibration → `maburcal`** (`gs/bundle/maburcal`,
  `docs/calibration.md`). `maburcal start` sweeps a unit's per-rate PA
  compression walls and applies them; `maburcal report <cal.log>`
  re-renders a saved run offline.

## maburd debug endpoint

`maburd` serves three routes on `127.0.0.1:<venc.debug_port>` (shipped
8301; `drone/src/debug_http.cpp`), localhost-only, always on, no enable
flag — a bind failure logs and disables itself rather than being fatal. It
is reachable only from the drone, so `ssh root@<drone> 'wget -qO-
http://127.0.0.1:8301/venc'`:

- `GET /venc` → `{"req_bitrate_kbps", "ring_fill_pct", "full_drops",
  "frames", "sensor_mhz"}`. `req_bitrate_kbps` is the REQUESTED rate — what
  RcAgent last successfully commanded — not a readback of what the encoder
  programmed. `sensor_mhz` is the sensor's measured frame rate in milli-Hz
  from the boot-time trim (`docs/airtime-model.md` §3), 0 until its first
  1 s window closes; healthy is a few tens of mHz under `venc.fps`.
  `frames` advancing is the cheapest "is the camera alive" check, and the
  one that distinguishes a stalled encoder (fps flat, ring empty) from a
  starved link. There is no `qp`: this SDK has no encoder-QP readback
  (`h265Info.startQual` is 0 on every frame, no `GetChnStat` QP).
- `GET /snapshot.jpg` → a JPEG straight off the encoder's snapshot channel
  at `venc.snapshot_quality`. Answers 503 on a build without the venc core,
  500 on a capture failure — deliberately distinguishable.
- `POST /venc/set?k=v`, whitelist `bitrate` / `qp_delta` / `roi_qp` /
  `max_ipprop` / `superframe_p_pct` / `min_iqp` / `max_iqp` / `fps` /
  `ov_base_pct` / `ov_enh_pct` (`max_ipprop`/`superframe_p_pct` are
  volatile encoder pokes, `docs/airtime-model.md` §3; `ov_*_pct` is a
  per-layer FEC overhead override, `-1` clears, 0..300 %, and works without
  the venc core). Nothing is persisted. **A bitrate/ROI-QP override is
  bounded:** RcAgent re-asserts its computed values every
  `RcAgent::kReassertMs` (5000 ms) on top of the pushes at every rung
  transition, failsafe/LINKED entry and ROI threshold crossing, so an
  override lasts at most ~5 s on a parked link — a bench procedure must
  re-POST inside that window (`docs/link-adaptation.md`).

It is a debug surface, not an instrument: nothing records it. Reach for it
when the GS cannot see the drone at all (`drone` is `null`, a half-finished
deploy, no video) and you need to know whether the encoder is running.

maburd's own stderr (`/tmp/mabur.log`): the 1 Hz `stats:` line (incl.
`enc_pk100=`, the peak 100 ms encoder byte rate in kbit/s inside that
second, `lp=`/`armed=` for low-power mode, and `snr_mhz=`, the trimmed
sensor rate in milli-Hz (same value as `GET /venc`'s `sensor_mhz`);
`rc: low_power ENTER/EXIT` lines mark transitions, `[sensor] cal:` lines
the trim's rounds and lock) and the 5 s `frame_ring:` line (below).

## Frame loss inside the drone ("wire clean" ≠ no frame loss)

Frames can vanish INSIDE the drone — between the encoder and maburd's ring
read — and never get a `frame_id`: the wire sequence closes seamlessly over
the hole, every FEC/loss counter reads zero, and a vanished BASE frame
silently smears the decoder until the next refresh. Root cause is CPU
famine, not ring depth, so the honest knob is `encoder.bitrate_max_kbps`.
maburd detects it from the pts step between consecutive ring reads
(`drone/src/frame_pipeline.h`, shed-immune: shedding happens downstream of
the ring read) and prints the counts on the 5 s `frame_ring:` stderr line:
`vanished=<base>/<enh> self_idr_refused=<n>`. They are not on the sideport.
`self_idr_refused` counts base vanishes inside the IDR-adjacency guard; the
self-IDR consumer is deliberately not wired (an earlier attempt amplified
CPU overload into an IDR storm). Findings:
`docs/findings/2026-08-12-venc-ring-vanish.md`.
