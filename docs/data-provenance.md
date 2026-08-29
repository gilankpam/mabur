# Data provenance — date every recording

Several link-wide behaviours and metric scales changed on known dates.
Recordings that span one of these are not the same experiment and must
not be pooled. Nothing in the sideport reports most of them, so the only
reliable method is to date the recording against this page.

Quick index: carrier sense off 2026-08-05 · TX power constant 2026-08-12 ·
sideport key removals 2026-08-12, 2026-08-15 and 2026-08-29 · SNR half-dB
scale break 2026-08-04 · EVM op-point dependence 2026-08-10 · RF labels
pooled and fade deltas unsuppressed 2026-08-15 (see
`docs/link-adaptation.md`) · DVR filenames un-dated 2026-08-26 · UEP
overhead flatten 2026-08-29 · overhead literal + 4→2 stream collapse
2026-08-29 (airtime-balance-uep).

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
below). Removed keys are absent, not null. Keep appending to that list — not to protect
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

