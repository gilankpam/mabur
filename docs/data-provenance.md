# Data provenance — date every recording

Several link-wide behaviours and metric scales changed on known dates.
Recordings that span one of these are not the same experiment and must
not be pooled. Nothing in the sideport reports most of them, so the only
reliable method is to date the recording against this page.

Quick index: carrier sense off 2026-08-05 · TX power constant 2026-08-12 ·
sideport key removals 2026-08-12, 2026-08-15, 2026-08-29, 2026-08-30 and
2026-09-04 ·
SNR half-dB scale break 2026-08-04 · EVM op-point dependence 2026-08-10 ·
RF labels pooled and fade deltas unsuppressed 2026-08-15 (see
`docs/link-adaptation.md`) · DVR filenames un-dated 2026-08-26 · UEP
overhead flatten 2026-08-29 · overhead literal + 4→2 stream collapse
2026-08-29 (airtime-balance-uep) · overhead splits into base/enh pairs +
ctllog 8 2026-08-30 (same-rate-fixed-pairs) · SlotHdr v2 (`# aulog 2`) +
aucadence completion clock switched to ring `t_complete_us` 2026-08-31
(latency-accounting) · `reg`/`dsp` split redistributed by the vsync
servo 2026-08-31 (sum still comparable, split is not) · **post-FEC
residual re-based from packet-seq to symbol abandonment + ctllog 9
2026-09-02 (residual-phantom-demotes)** · **discrete s3 probe replaced by
the always-on probe stream + ctllog 10, `probe_u`/`probe_n` become a
continuous EWMA 2026-09-04 (probe-stream)** · `probelog 2` adds the
per-body `first_ms` arrival stamp 2026-09-05 (probe-blanking fix; a
`probelog 1` file's `t_ms` is the finalize tick and cannot be joined to
`au-NNNN.log` for timing).

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

**DVR filenames carry no date since 2026-08-26.** `maburplay` writes
`record-NNNN.mp4` (index one past the highest already on the card), where it
used to write `record_%Y-%m-%d_%H-%M-%S.mp4` with a `-1`/`-2` suffix for
collisions inside the same second. The date came from the GS RTC, which is
wrong at boot, so it never dated anything reliably — but it did sort, and
files written before this date still sort chronologically among themselves
while the new ones sort by index. A card holds both: the two families never
collide (the scan ignores `record_`-prefixed names) and old files keep their
names, so pair an old recording with its jsonl/ctl neighbours by mtime, not
by the stem. Same reasoning as `ctl-NNNN` and `flight-NNNN`, which were
index-first all along.

Schema/design references (local, gitignored):
`docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md` and
`docs/superpowers/specs/2026-07-26-drone-telemetry-design.md`. The schema
is NOT additive-only — per the compatibility policy in CLAUDE.md, keys get
removed and re-typed under `v: 1` without bumping `v`, and the rule is to
update `tools/maburtop.py` and `tools/flightreport.py` in the same commit.
Removals so far: 2026-08-12 `link.op.offset_qdb`,
`drone.applied.offset_qdb`, `drone.applied.derate_qdb` (constant-TX-power
note above); 2026-08-15 `link.attrib.on` (pooled-RF note in
`docs/link-adaptation.md`); 2026-08-29 `drone.applied.overhead` (singular)
→ `drone.applied.overhead_base`/`drone.applied.overhead_enh`, and
`link.streams` shrank from 4 entries to 2 (overhead scale break note
below); 2026-08-30 `link.op.overhead` → `link.op.overhead_base`/
`link.op.overhead_enh`, `link.ctl.rung.ov` → `link.ctl.rung.ov_base`/
`link.ctl.rung.ov_enh`, `link.ctl.ladder[].ov` →
`link.ctl.ladder[].ov_base`/`link.ctl.ladder[].ov_enh`, `link.rungs[].ov`
→ `link.rungs[].ov_base`/`link.rungs[].ov_enh` (same-rate-fixed-pairs
scale-break note below); 2026-08-30 `link.deadline_ms` (the
`fec.decode_deadline_ms` row-expiry knob it mirrored was deleted after
the wall-2 bench measurement,
`docs/wall2-deadline-findings-2026-08-30.md` addendum — the video
decoders now rely on the seq horizon alone; MSP's SwDecoder keeps its
own 2 s expiry); 2026-09-02 `link.attrib.suppressed` (deleted with the
packet-level delivery window it was defined against — it counted windows
where the total and attributed packet views disagreed, a question the
symbol-based measure cannot ask; residual-phantom-demotes note below);
2026-09-04 `link.ctl.last_probe` (the discrete probe-attempt snapshot —
there is no discrete attempt any more, its live state is `link.probe`
instead), `counters.probes_started`/`probes_ok`/`probe_fails`/
`probe_aborts` (replaced by `counters.promotes_probed`/`probe_holds`),
`classes.s2`/`classes.s3` (removed, were always empty since the
2026-08-29 UEP flatten; `classes.probe` added) — probe-stream note below.
Removed keys are absent, not null. Keep appending to that list — not to protect
consumers, but because a recording made before a removal still carries the
key and `flightreport.py` still reads old recordings. The
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

**Scale break, 2026-08-29 — UEP overhead flatten.** `kUepRefOverhead`
changed {1.00, 0.75, 0.50, 0.25} → {1.00, 0.50, 0.50, 0.50}. Sideport
`streams[].ov` values and the ladder's utilization/`u` readings are on a
new scale from this date; per-stream ov in recordings before/after the
flag day is not comparable. Ladder thresholds (down_util etc.) were NOT
retuned. Same day: drone venc.resilience ltr:1 → rally (frame sizes
equalize; base/enh size split in older recordings is a preset artifact,
not a regression). Later the same day, s0 joined the flat ladder too
({0.50, 0.50, 0.50, 0.50}) — sideport s0 `ov` values change scale again.
See `docs/superpowers/specs/2026-08-29-uep-flatten-rally-design.md`.

**Scale break, 2026-08-29 — overhead goes literal, streams collapse 4→2
(airtime-balance-uep, RC_VERSION 4).** LATER THE SAME DAY as the flatten
above, the wire's `fec_overhead` field changed meaning from a per-layer
`uep_layer_overhead`-scaled fraction to the LITERAL FEC command overhead
(`repair/data`) — old cmd × 2 = new actual. Every overhead-shaped value
in a recording from before this date is cmd-scale: **HALF the actual air
overhead a post-break recording of the same nominal number would carry.**
That covers `link.op.overhead`, `link.ctl.rung.ov`,
`link.ctl.ladder[].ov`, `link.streams[].ov`, `link.rungs[].ov` and the old
singular `drone.applied.overhead` (see the removal note above). The video
link collapsed from 4 UEP streams to 2 in the same change (BASE sid0 +
ENH sid1, `docs/link-adaptation.md`), so **a 4-entry `link.streams` array
is the shape signature of a pre-break recording** — `flightreport.py`
detects it (`streams` length) and LABELS the overhead values "cmd-scale
(x0.5 air)" rather than converting them, per the provenance policy in
CLAUDE.md: recordings outlive the code that wrote them, so a historical
number is read as what it was, never silently rescaled to match today's
meaning. A 2-entry array is post-break and needs no label.

⚠ The ×2 rescale is not the only wrinkle: the old ladder's config
overhead values were ALSO coarser than they looked. mcs6/mcs7 used to
read 0.15/0.10 — two visibly different numbers — but both collapsed to
the *same* repair-symbol count on the FEC's quantization grid, so their
actual on-air overhead was identically ~0.125 in every pre-break
recording (**0.10 ≡ 0.15 ≡ 0.125 on air**). The 2026-08-29 config bump
that doubled the nominal values (0.15→0.3, 0.10→0.2 — see
`gs/bundle/maburgs.default.json`) was not "just x2": it also resolved
that collision, so mcs6 and mcs7 now carry genuinely different overhead.
Do not back-compute a pre-break "real" overhead by simply halving a
post-break number for mcs6/mcs7 specifically — the halved figure lands
on the wrong side of the quantization collision. Every other rung is a
clean ×0.5.

**Scale break, 2026-08-30 — overhead splits into base/enh pairs
(same-rate-fixed-pairs); ctllog 8.** Every overhead-shaped sideport key
that used to carry ONE value per rung/op now carries a base/enh PAIR,
mirroring the RCF wire and per-rung config split landed the same day
(RC_VERSION 5, `gs/src/op_point.h`, `Rung::overhead_base/overhead_enh`):
`link.op.overhead` → `link.op.overhead_base`/`link.op.overhead_enh`;
`link.ctl.rung.ov` → `link.ctl.rung.ov_base`/`link.ctl.rung.ov_enh`;
`link.ctl.ladder[].ov` → `link.ctl.ladder[].ov_base`/
`link.ctl.ladder[].ov_enh`; `link.rungs[].ov` → `link.rungs[].ov_base`/
`link.rungs[].ov_enh`. A recording from before this date has ONE `ov`/
`overhead` number per rung where a post-break one has two — that is the
shape signature, the same detection idiom as the 2026-08-29 4→2 streams
collapse above. `link.streams[].ov` keeps its key (not renamed) — sid0
already read `overhead_base`-equivalent and sid1 `overhead_enh`-
equivalent before this date whenever telemetry was live, so only its
pre-telemetry fallback value changed (sid1 now falls back to
`overhead_enh` instead of `overhead_base`; a divergent enh rung was
silently masked as the base value until telemetry arrived, on any
recording from before this date with a gap at session start).

The maburgs ctl log's header line bumped `ctllog 7` → `ctllog 8` the
same day (`gs/src/ctl_log.h`): a HEADER-ONLY change — no S/E/P/N/R
per-tick line carries an overhead field at all, so nothing about a
recording's per-tick rows moved. Only the header's `ladder=` token
changed shape, from `<mcs>/<ov>,...` (one value per rung, ×100) to
`<mcs>/<ovb>:<ove>,...` (a base/enh pair, both ×100). `flightreport.py`
reads ctllog v1–v8: a pre-v8 log's single per-rung value is read as both
base and enh (that rung had no split to lose), and the tool prints a
NOTE on any log older than v8 saying so.

**2026-08-31: aucadence completion clock changed read-time → ring
`t_complete_us`; offsets recorded before this date fold in reader poll
latency (~0–0.5 ms one-sided).** `tools/bench/aucadence.py`'s per-AU
completion delay used to be `arrival_mono_us − pts`, where `arrival_mono_us`
was stamped by the ~0.5 ms poll loop that reads the ring's write index —
not the writer's own completion time, which SlotHdr v2 (2026-08-30) now
carries as `t_complete_us`. The tool switched to the ring-stamped clock the
same day the latency-accounting tools landed; the JSON output's `"clock"`
key records which basis produced a given capture (`"t_complete"` post-
switch, and a slot whose `t_complete_us` is 0 — pre-epoch writer, or a
caller that never passed `AuLatMeta` — still falls back to the old poll-time
basis, counted in `fallback_rows`). `tools/flightjitter.py`'s jitter-EMA
reproduction made the same switch: it now uses a row's `t_complete` (when
nonzero) as the inter-AU arrival basis instead of `au-NNNN.log`'s `t_us`
column, which is flightrec's own read-time stamp, same imprecision as the
old aucadence poll. Pre-2026-08-31 `--gate-ms` baselines and `flightjitter`
`jitter_ema_ms`/`residual_jitter_ms` numbers are not directly comparable to
post-switch ones for this reason — expect the poll-time noise floor (~0.5
ms) to shrink, not the underlying transport behavior to have changed.

**2026-08-31 (same day, later): the `air` segment's basis changed —
enc-excess fix.** The first deployed build computed `air` by clamping
`enc`/`dq` against the span above a capture-anchored floor, which
double-counted encode time (the floor already contained the floor frame's
encode) and pinned `air` at 0 in every clean capture from that build. The
fix anchors on the encode/queue-corrected arrival: `enc`/`dq` now report
their full wire values (previously clamped to the span) and `air` is real
transit excess. `e2e` (and the anchored floor) therefore shifted lower by
roughly the floor frame's `enc` (~2–4 ms) between the two same-day builds;
`link.video.lat` and player `lat:` numbers from the few hours in between
under-report `enc`/`dq`/`air` accordingly and show `air` ≈ 0 by
construction. Corrupt-body protection also landed with the fix: wire
`enc`/`dq` from FCS-failed bodies read as 0 = unknown from this point on.

au-NNNN.log rows grew 4 columns (`t_first t_complete enc dq`, SlotHdr v2's
latency fields) behind a `# aulog 2` marker line, written once at session
start by `flightrec.py`. A log without the marker is the old 7-column v1
format; `flightjitter.load_au_log` keys its row parsing on the marker's
presence, and a v1 row is padded with `t_first=t_complete=enc_us=dq_ms=0`
so the fec-wait class and the t_complete arrival-basis fallback both treat
"absent" the same as "present but zero".

**2026-09-06 (air clock): `# aulog 3`, SBI ver 2, SlotHdr v3.** AU rows
gain a 12th column, `air_ms` — the drone's modelled air backlog at the
AU's arrival (`AirClock`, `docs/link-adaptation.md`), carried in the SBI
body header (ver 2, 13-byte header; ver-1 bodies are rejected) and the AU
ring slot at offset 52 (`kAuRingVersion` 3). A row without the column
predates the model; `airdrain.py --model` says so instead of comparing.
`flightjitter.load_au_log` and `flightreport.load_aulog` ignore the extra
column. Recordings from the observe-only period (`air_clock.shed_ms` 0)
carry a live `air_ms` and zero `drone.air_shed_drops`; a non-zero drop
count means the gate was armed for that flight.

**2026-08-31 (vsync servo): `reg`/`dsp` semantics change when
`display.vsync_lock` is on.** `reg` becomes hold-to-vblank (grows to
~½ panel period mean, up from the old D=12 dejitter hold); `dsp`
collapses to ≈ lead (`display.vsync_lead_ms`, default 6 ms) + flip
completion, down from the old 10–25 ms beat sweep. Their SUM is
comparable across the change — total release→latched time moves the way
the vsync-locked-regulator spec predicts — but the SPLIT between the two
segments is not: a `reg`/`dsp` pair from before this date and one from
after cannot be compared segment-by-segment, only as a sum, and only when
both recordings agree on `vsync_lock`. `lat-NNNN.log` (`# latlog 1`,
under `display.lat_log_dir`, default `/media/dvr/log`) starts existing at
this date — see `docs/observability.md` for its format. `lat:` lines from
before this date lived only in tmpfs (`/tmp/maburplay.log`) and are gone;
there is no way to recover the player tail segments for any flight
recorded before 2026-08-31.

**2026-08-31 (later, streaming push): `dq` changed meaning and scale —
~6 ms → ~0.** Before this date the drone stamped every body's
`enqueued_ms` at the TOP of the hot-loop iteration, so the wire `q_ms`
(→ player/flightrec `dq`, sideport `drone.txq_wait_ms`) spanned venc-ring
wait (~2.6 ms, fictitious — the frame did not exist yet) + FEC/SBI-pack
CPU (~3.4 ms) + true queue wait (~40 µs) — see
`docs/dq-spike-findings-2026-08-31.md`. After it, bodies stream to the
TxQueue as each seals, stamped at the actual push: `dq` is the true
TxQueue wait and reads ~0 in a healthy link; the pre-push CPU overlaps
the radio drain instead of preceding it, which also moved real latency —
same-config A/B on the bench: `e2e` p50 55.9 → 45.3 ms, `air` 2–3 → 0–1,
`reg` mean −2.6 (shallower vsync hold). A `dq` from before this date
cannot be compared to one after it at any scale; a healthy pre-change
recording shows `dq`≈6–7 where a healthy post-change one shows 0.
`dq` > a few ms now genuinely means TxQueue backlog.


**Scale break, 2026-09-02 — post-FEC residual is symbol abandonment, was
packet-seq gaps.** Everything that reports "residual"/"post-FEC loss"
changed what it counts: `link.residual_loss` and `link.attrib.residual_cur`
on the sideport, `resid`/`resid_cur` on the ctl log's S lines, `resid` on
its R lines, `link.layer_delivery_pct`, and the player OSD's post-loss row.

Before: `1 - delivered/expected` over a per-RCF-period window, from FRAG-seq
continuity of completed packets. That measure inferred loss from sequence
gaps, and a gap is indistinguishable from a unit that has not completed
*yet*. Sliding-window FEC completes a repaired unit *after* a later clean
one, so the next forward gap re-booked an already-delivered unit as a fresh
expectation. In a 50 ms window holding 3–6 units that reads as exactly
0.2500 or 0.3333.

After: `abandoned/expected` from the FEC decoder's own counters
(`SwDecoder::syms_abandoned`), which is seq arithmetic over a span and is
immune to arrival order. One formula for every consumer, in
`gs/src/ladder_residual.cpp`.

**Why recordings must be dated against this.** The old number was not a
noisy version of the new one — on a clean bench it was pure artifact. A
57-minute run on 2026-09-02 logged **200 `reason=residual` demotes with
`syms_abandoned` frozen at 139 across 6.2 M packets**, i.e. zero real
post-FEC loss the entire time, and `syms_recovered_arrived` (late originals)
= 218, tracking the demotes ~1:1. Consequences when reading pre-2026-09-02
data:

- A pre-break `resid > 0` does **not** mean video was lost. Do not pool
  per-rung `resid` across the boundary.
- Pre-break per-rung residual tables (including `flightreport.py`'s
  inversion callout) are contaminated by *reorder rate*, which scales with
  packet rate and therefore with rung — so they systematically overstate
  the high rungs. The "mcs5 is lossier" shape in older rung stores may be
  this artifact rather than PHY.
- Pre-break ladder behaviour is not comparable: the link could not hold a
  rung, so dwell-time-per-rung, promote/demote counts and time-at-max are
  all measuring the artifact.
- The ladder's demote input additionally narrowed from pooled base+enh to
  **BASE only** at the same date (the pooling was a 4-stream-era leftover);
  the S line still logs the pooled view.
- Post-break loss is booked ~80 ms later — a symbol counts abandoned only
  once the sliding-window horizon passes it (`seq_horizon` 512 at ~6.4 k
  sym/s). Sub-100 ms timing comparisons against pre-break recordings are
  not valid.

The ctl log self-identifies: `ctllog 9` and later are symbol-based, v1–v8
are packet-based, and `flightreport.py` prints a pre-v9 warning.

**Scale break, 2026-09-02 (link-rtt) — the OSD LAT headlines became
ABSOLUTE.** Before this date the burned-in `P50`/`P99` e2e (and `air+`)
were relative to the luckiest observed frame's transit — the anchor ate
the absolute network floor. After it, when `link.rtt.pts_off_us` is live
the player folds the floor (own anchor + telem-derived pts offset) into
`air+` AND the headline, so both read a few ms HIGHER than a pre-change
recording of the same link; the segments still sum to the headline. The
rows self-identify: a `~` prefix on the headline means the number is
still the old relative kind (offset estimator cold, or sync lost — the
floor freezes at last-good and drifts ~1 ms/min of outage). Sideport
`link.video.lat` is untouched — the fold-in is display-side only. A new
`RTT <n> ms` row above `P50` is control-path RTT (telem queues behind
video on the drone TX: reads high under saturation by design), never a
segment. Recordings' jsonl gains `link.rtt` the same date (additive).
Even absolute, the headlines still exclude sensor pre-pts (~10–16 ms
est.) and post-scanout display latency — only an LED/camera measurement
sees those.

## 2026-09-03: per-card `loss_pct` phantom before the MSP seq fix

Recordings made before the MSP seq-walk fix (branch `msp-seq-fix`)
carry an inflated `cards[].loss_pct`: the GS excluded MSP bodies and
the drone's RC frames (DISC_ACK, T_TELEM) from the per-card 802.11 seq
walk on the (wrong) assumption that they carried their own counters. The
chip stamps ONE hardware seq on every frame it injects (EN_HWSEQ), so
each such frame booked one phantom lost frame. At the bench's ~5–10 MSP
+ ~2 RC frames/s over ~2300 bodies/s that is ~0.3–0.5 percentage points
of phantom on top of the real figure (0.67 % reported vs ~0.35 % real,
2026-09-02). `pre_fec_loss` and every ladder input are unaffected. When
comparing `loss_pct` across the fix, subtract the non-video rate
(`cards[].classes.msp.pps + ctrl.pps`, over `inj_pps`) from the old
values, or compare `pre_fec_loss` instead.


## 2026-09-03: `drone.enc.qp` changes meaning; `roi_qp` and `congestion_shed` added

Before this date the sideport key `drone.enc.qp` (Telem byte `qp`) was
RcAgent's **ROI QP override** (`actuator.last_roi_qp`, 0 = normal,
`encoder.roi_qp_low` when the bitrate sat under `roi_threshold_kbps`).
It read 0 for whole flights and says nothing about the encoder's rate
control — flight-0011's analysis fell for exactly that
(`docs/handover-venc-overshoot-2026-09-03.md`). From this date
`drone.enc.qp` is the **encoder's QP** for the last frame it published
(venc `startQual`, 0 = unavailable — host build or no frame yet) and the
override lives in the new `drone.enc.roi_qp` (signed). Do not compare a
`qp` column across the date: an old recording's 0 is "ROI normal", a new
recording's 30-ish is a real QP. `drone.congestion_shed` (Telem flags
bit4) is additive the same day: true while the drone-local
TxQueue-pressure / USB-failure shed holds the enh layer, absent (not
false) in older recordings — an enh gap in a pre-date recording cannot be
attributed to congestion after the fact.

**Same night: `drone.enc.qp` deleted.** The encoder `startQual` it was
re-pointed at reads 0 on every frame this firmware emits (no
`GetChnStat` QP exists either), so the key and the Telem byte behind it
were removed rather than shipped as a permanent 0 (Telem 84 → 83). The
only recordings with a non-zero `drone.enc.qp` would be from the few
hours between the two changes, and would still be 0. `roi_qp` and
`congestion_shed` stay. `vencprobe` captures from that window carry a 3rd
`s,` column (always 0 or −1); both analyzers tolerate its absence.


## 2026-09-04: `link.pre_fec_loss` added (a gauge, not the ladder's number)

Additive: recordings before this date have no `link.pre_fec_loss` and
only the in-ctl `link.ctl.pre_fec_loss`. The two are NOT interchangeable
and neither replaces the other.

- `link.ctl.pre_fec_loss` is the last s1 sample the **ladder controller
  acted on**. `LadderController::update()` returns early on a starved or
  `!sample_valid` window (`gs/src/ladder_controller.cpp`), so through
  those windows this key **holds its previous value**. It is null in
  static-pin mode, where the controller is never ticked at all.
- `link.pre_fec_loss` is that poll's **raw measured** s1 window loss,
  exported unconditionally alongside the post-FEC `link.residual_loss`.
  It is null when the window produced no valid sample, and — the reason
  it exists — it is present in static-pin mode.

Expect them to differ by a fraction of a poll's worth of drift while the
ladder runs (bench 2026-09-04: 0.01029 vs 0.01001 on the same datagram),
and to diverge properly across a starved run. Prefer the ctl figure when
reconstructing a ladder decision; prefer the gauge when asking what the
link was actually doing, and use it exclusively for pinned flights.

Before this date the player's OSD read the rung and the pre-FEC half of
its LOSS row **only** from `link.ctl`, so both rendered as em-dashes for
the entire duration of any `link.static_mcs >= 0` flight. A pinned
recording from before 2026-09-04 therefore shows a blank MCS/FEC/LOSS-pre
OSD in its burned DVR video while the jsonl beside it has the real values
in `link.op` — the display was wrong, the data was not.

## 2026-09-04: `venc.resilience` deleted; the encoder structure is six keys

The drone config key `venc.resilience` — a string naming a row in a preset
table (`rally`, `fpv`, `ltr:<N>`, …) that expanded into intra-refresh and
SVC-T settings — no longer exists. A config still carrying it fails boot on
the unknown-key path. Its components are now config keys of their own, each
one an MI struct field verbatim:

| was, inside the preset | is now |
|---|---|
| intra mode name → target ms → CTU rows/P | `venc.intra_refresh_rows` (rows/P directly, 0 = off) |
| per-mode stripe QP (36/32/28) | `venc.intra_refresh_qp` |
| `ref_base` / `ref_enhance` / `ref_pred` | `venc.ref_base` / `venc.ref_enhance` / `venc.ref_pred` |
| preset-pinned GOP | `venc.gop_s`, which is now always authoritative |

The shipped values (`rows 4, qp 36, base 1, enhance 1, pred true, gop_s
2.0`) are what `rally` expanded to at 1080p60, so **recordings either side
of this date are directly comparable** — the encoder structure did not move,
only the way it is spelled. Two behaviour changes worth knowing when reading
older material:

- The preset path **clamped** an over-wide stripe to the picture height and
  warned on stderr; config now refuses to boot instead. No flown config ever
  hit the clamp.
- Auto-GOP (one IDR per full stripe sweep, when no explicit GOP was set) is
  gone. It was unreachable in mabur for its whole life — every preset pinned
  a GOP — so no recording was ever produced under it.
- **`venc.gop_s` was previously MASKED and is now authoritative.** Under the
  preset path `gop_sec` was `preset.gop_overridden ? preset.gop_s :
  cfg->gop_s`, and every named preset set `gop_overridden` — so whatever
  `venc.gop_s` a config carried alongside `resilience: "rally"` was ignored
  and 2.0 s was used. It takes effect from this date. The bundle already
  says 2.0, but a deployed drone config is known to diverge from the bundle
  (see `handover-venc-overshoot-2026-09-03.md`: `max_ipprop` 2 deployed vs 0
  in the bundle), so **read `venc.gop_s` out of the drone's live
  `/etc/mabur.json` before swapping** — anything but 2.0 changes the IDR
  interval on this deploy, with nothing in the boot log calling it out.
- The `rescue` preset's 0.25 s GOP is no longer expressible. It wrote
  `gop_sec` directly and so bypassed the `venc.gop_s` range check, whose
  floor is 0.5 s. The IDR-spam escape hatch is gone; nothing flew on it.

Mentions of `venc.resilience ltr:1 → rally` and of the "rally preset" in
the notes above and in `venc-ring-vanish-findings-2026-08-12.md` describe
configs that produced data on the DVR. They stay accurate about that data;
they just name a key that is no longer settable.

## 2026-09-04: probe stream replaces the discrete s3 probe — RC_VERSION 6, ctllog 10

The s3 probe-before-promote design (2026-08-05) is gone: the drone no
longer moves the whole ENH stream to a candidate MCS for 2 s per promote
attempt. Instead it always sends a dedicated probe-stream canary body
(SBI stream id 5) behind every ENH access unit, at the MCS of rung
`current + link.probe.rung_offset`, and the ladder's promote trigger
reads a continuous verdict instead of starting a discrete probe. Detail
in `docs/link-adaptation.md` "Probe stream". RCF gains a fixed
`probe_profile` byte (`RCF_F_PROBE_ENH`/`CAP_ENH_PROBE`/`CAP_S3_PROBE`
deleted); RC_VERSION 5 → 6 — a drone/GS pair must be swapped together
(`docs/deploy.md`).

Sideport (schema `v: 1`, unchanged): REMOVED `link.ctl.last_probe`,
`counters.probes_started/probes_ok/probe_fails/probe_aborts`,
`classes.s2`, `classes.s3` (the last two were always empty since the
2026-08-29 UEP flatten — no data lost). ADDED `counters.promotes_probed`,
`counters.probe_holds`, `classes.probe`, and the `link.probe` block
(gate state, rung/mcs, streak, union + per-card loss and counts — see
`docs/observability.md`).

**Scale break: `link.rungs[].probe_u`/`probe_n` change from a discrete
per-attempt sample to a continuous EWMA.** Before this date a rung's
`probe_u` was the last COMPLETED 2 s discrete probe's utilization at that
rung — one sample roughly every few minutes, only while a promote to
that rung was actually being attempted. From this date it is the same
`RungStore` field fed by the always-on gate, updating at ~20 samples/s
continuously while that rung is the commanded probe candidate. **Do not
pool `probe_u`/`probe_n` populations across this date**: a pre-date
`probe_n` of a few tens means a handful of discrete attempts: a post-date
one of the same magnitude means well under a second of continuous
sampling. The `age_s` column next to it in the `R` ctl-log line is
unaffected — `age_s` was always wall-clock time since the last sample.

**`ctllog 10`'s `P` line is REPURPOSED, not just reformatted.** Through
v9 a `P` row was one discrete probe ATTEMPT outcome (`pass|fail|abort`)
written once per 2 s probe. From v10 it is a GATE-STATE EDGE
(`clean|lossy|noinfo`) written every time
`LadderController::probe_gate()`'s state changes — a continuous-state
transition, not an attempt outcome. `flightreport.py`'s wall-fit maps
`clean→pass, lossy→fail, noinfo→abort` for the per-rung wall report, but
a v9-vs-v10 P-row COUNT is not the same kind of number (attempts vs
edges) and must not be pooled. The `E` line's reason vocabulary gains
`promote_probed` (a probe-gated promote); existing reasons are
unchanged. `R`'s `probe_u`/`probe_n` line position is unchanged but
carries the new continuous meaning above from this date.

## 2026-09-05: `u`, `u3`, `link.pre_fec_loss`, `link.ctl.util` are arrival-booked — ctllog 11

Scale break, not additive. Before this deploy the ladder's pre-FEC
util number was `1 − (delivered + recovered_arrived) / (delivered +
recovered + abandoned_cur)` from the FEC decoder's completion counters:
a lost symbol entered it only when repaired or given up on (a repair
window to ~80 ms late), and right after a rung change the first bucket
had almost no deliveries, so a handful of repairs read as 50–100 %
(flight-0023's two `probation` demotes at +151/+160 ms; bench ctl-0299
u = 1.125). Since ctllog 11 the number is `1 − arrived/expected` from
`SwDecoder`'s `ArrivalTracker`: `expected` is sequence advance past a
32-seq settle line, `arrived` is what was heard, both booked at arrival,
current-only via the transition watermark. Consequences for readers:

- `u`/`u3` on S lines, `u` on util/probation E lines, `link.pre_fec_loss`,
  `link.ctl.util`/`util3` and the RungStore `u` columns read LOWER and
  SMOOTHER across transitions and react sooner to a fade. Do not pool
  them across the ctllog 10/11 boundary; `flightreport.py` prints which
  definition a log used.
- New `link.streams[].arr_*` keys (cumulative). The old
  `recovered`/`recovered_arrived`/`abandoned` keys stay and still describe
  decoder completion; they are no longer what the ladder acts on.
- The 150 ms util settle blank is gone; the residual blank stays.
