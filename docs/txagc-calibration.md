# TXAGC Calibration Findings — RTL8812EU (Jaguar3)

Bench measurement of the real TXAGC-index → radiated-power transfer on the
deployed drone hardware, 2026-07-16. Written to be reproducible from a fresh
session. Paths relative to the mabur repo root; tool source under
`bench/txagcbench/`.

**TL;DR:** the adaptive controller's original power model (`kTxagcGainDb`,
since deleted from `gs/src/gen/gen_tables.h`) bore no resemblance to the
hardware. The real curve is a flat floor below idx ~29, a ~0.3 dB/step ramp,
and a **per-MCS compression wall** — max clean power falls ~7 dB from MCS0 to
MCS7. No layer of the current stack (efuse → devourer → maburd) enforced
per-rate limits at the time of this sweep. The measured wall table below is
the authoritative clamp data for the fix, now consumed as config (see
"Consuming the walls"). **Deployed-baseline safety finding (2026-07-16/17):**
the original "MCS6/7 already overdriven" claim was a flat-override artifact —
under the actually-shipped `power_mode: "none"`, devourer's default per-rate
diffs keep every MCS 3–6 dB below its wall at 98–100% delivery; the deployed
baseline was never in the danger zone this doc originally implied.

## The wall table (the headline result)

Last TXAGC index with ≥90% frame delivery, per MCS. Walls are TX-side PA
physics (compression vs the modulation's peak-to-average ratio), so they are
**distance-independent** and transfer from bench to field:

| MCS | modulation  | compression wall (idx) | peak clean power vs MCS0 |
|-----|-------------|------------------------|--------------------------|
| 0–2 | BPSK–QPSK   | none (clean to 127)    | 0 dB (reference)         |
| 3   | 16-QAM 1/2  | ~95                    | 0 dB                     |
| 4   | 16-QAM 3/4  | ~73                    | −2 dB                    |
| 5   | 64-QAM 2/3  | ~56                    | −5 dB                    |
| 6   | 64-QAM 3/4  | ~51                    | −6 dB                    |
| 7   | 64-QAM 5/6  | ~49                    | −7 dB                    |

Suggested clamp values (wall − ~1 dB margin): `{0-2: 127, 3: 91, 4: 69,
5: 52, 6: 47, 7: 45}`.

Delivery vs index, measured (100 frames per cell, every 8th index shown):

```
MCS|   0   8  16  24  32  40  48  56  64  72  80  88  96 104 112 120
---+-----------------------------------------------------------------
 0 | 100 100 100 100 100 100 100 100 100 100 100 100 100  99 100 100
 1 | 100 100 100 100 100 100 100  99  99 100 100 100 100 100  99 100
 2 | 100 100 100 100 100 100 100 100  99 100 100 100 100  99 100  99
 3 | 100 100 100 100 100  99 100 100 100 100 100 100  84  50  60   8
 4 |  99 100 100  98 100 100 100 100  99 100   4  30   0   0   0   0
 5 |  33  61  69  65  98  96  99  93   1   0   0   0   0   0   0   0
 6 |   4  21  26  29  81  94  99  99  32   0   0   0   0   0   0   0
 7 |   0   0   1   0  34  85  93   2   0   0   0   0   0   0   0   0
```

(The low-index dropoff on MCS5–7 is the *sensitivity* floor — RX-side,
geometry-dependent, measured here at 3 m / ~−77 dBm. The high-index cliff is
the compression wall — TX-side, geometry-independent. The MCS0–2 rows are the
control proving the TX emits at every index.)

## The transfer curve (gain shape)

Measured at MCS0 across the full 7-bit range (two independent runs, identical
shape, up/down-pass drift ≤ 1 dB):

```
dBm  -62 ┤                              ╭────────────────  ← PA saturation
     -66 ┤                    ╭─────────╯                    (MCS0 ceiling)
     -70 ┤             ╭──────╯    ~0.3 dB/idx
     -74 ┤       ╭─────╯
     -77 ┼───────╯
         └──┬────────┬─────────┬─────────┬─────────┬──
   idx      0       29        63        91        127
            └─ floor ─┘                  └── dead ──┘
```

- **Floor** idx 0..~28: output constant at the PA/TSSI minimum. Commanding
  power changes here does nothing.
- **Ramp** idx ~29..91: ~0.3 dB/index (nominal chip scale is 0.25 dB/step;
  1 dB RSSI quantization limits precision).
- **Ceiling** idx ~91+ (MCS0): PA fully saturated. Higher-MCS walls (table
  above) occur at compression *onset*, well before this.
- Total real authority: **~15 dB** (MCS0), shrinking to ~10 dB usable at MCS5.

Contrast with the committed `kTxagcGainDb`: a smooth 0→25 dB saturating curve
with its steepest steps at the *bottom* — where the hardware does nothing at
all. Shape error vs measurement: RMS 15 dB, max 19 dB. The table is a modeled
curve from the devourer Python prototype ("energy is modeled, not metered"),
never derived from hardware.

## Where per-rate power dies in the stack

| Layer | Per-rate power capability | Status on this rig |
|---|---|---|
| eFuse (silicon) | base index + rate-group calibration | present; base = **53** for ch149 (read back, `rb:1`) |
| devourer read | reads base + PA-bias trim | works — the 53 is applied |
| devourer apply | 8822E per-rate diff registers | zeroed **only under flat-override** — `set_tx_power_ref(idx, zero_diffs=true)`: "every rate emits at idx" (`RadioManagementJaguar3.h:85-91`); the 8822E **default path applies the phy_reg_pg per-rate diffs**, and `SetTxPowerOffsetQdb` preserves them (light ref-only writes) — see 2026-07-16/17 correction below |
| per-packet TXAGC | TX-descriptor power field | inert on Jaguar3 (devourer docs) — per-frame power is impossible |
| maburd deployed | `radio.power_mode` | `"none"` → fixed at 53, phy_reg_pg diffs live; `"offset"` → wall-equalized custom diffs, adaptive offsets (see "Consuming the walls" below); commanded `pwr_idx` under `"none"` is ignored |

Consequences at the factory-default 53, **under flat-override** (the mode
active during the original sweep — devourer's per-rate diffs zeroed, every
rate forced to emit at raw idx 53):

- MCS5 (wall 56) has **1 dB of margin — by luck**, which is why the clean
  bench runs worked.
- MCS6 (wall 51) and MCS7 (wall 49) are **already overdriven**: any UEP ladder
  rung flying them transmits into compression and dies at TX before the link
  or FEC get a vote.

> **Correction (2026-07-16/17):** the above is a flat-override artifact, not
> the deployed behavior. Under the actually-deployed `power_mode: "none"`,
> devourer takes the 8822E **default path** and applies the efuse
> `phy_reg_pg` per-rate diffs instead of zeroing them — each MCS is *not*
> pinned to raw idx 53. Re-measured per-MCS on-air power under `"none"`:
>
> | MCS | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
> |---|---|---|---|---|---|---|---|---|
> | on-air power | −66 dBm | −65 dBm | −66 dBm | −67 dBm | −70 dBm | −72 dBm | −71 dBm | −72 dBm |
>
> 98–100% delivery at every MCS, **3–6 dB of margin** below the compression
> walls above — no rate is overdriven. The "MCS6/7 already overdriven"
> finding holds only for the flat-override configuration (raw idx 53, diffs
> zeroed), which is not how the shipped baseline runs.

## Cross-validation against the vendor driver (`../rtl88x2eu-20230815`)

- 7-bit index, 0.25 dB/step confirmed: `txgi_max = 127; txgi_pdbm = 4`
  (`hal/rtl8822e/rtl8822e_halinit.c:60`).
- TSSI closed-loop mode exists, efuse-gated (`tpt_mode` nibble,
  `rtl8822e_phy.c:254`); trims are flat per-channel-group scalars — the
  vendor does **not** model the transfer non-linearity either.
- No software low-index clamp anywhere — the floor and walls are analog.
- The vendor's own TSSI anchor is index **60**, normal operation ~50–90
  (`halrf.c:5004`) — squarely inside our measured ramp, ending at our
  measured wall region. Realtek never intended the bottom quarter to be used.

## Implications for the adaptive controller

1. `gain_db()` and `min_txagc_for_gain()` operate on fiction: path-loss
   normalization can be off by up to ~19 dB, and "add 10 dB" commands can buy
   0 dB. Replace the table with the measured ramp.
2. **Power and MCS are not independent knobs** — they buy from the same PA
   linearity budget. A candidate op's available power gain must be capped by
   `wall(fastest MCS in its ladder)`. This belongs in the op-table search so
   the energy optimizer trades power headroom against airtime honestly.
3. Per-frame per-rung power is impossible on this chip; the clamp must be a
   single index bounded by the worst active rung, enforced in the drone
   actuator as the safety net.
4. Extending the command range past idx 63 is not worth it (+4 dB at MCS0,
   nothing at MCS4+); remapping onto idx 28..91 is actively dangerous for
   MCS5+ ladders. The wall-aware clamp is the correct shape of the fix.
5. **Model validity range**: `gs/src/energy.h`'s `gain_db()` linear 0.25
   dB/qdB model is bench-valid only over the PA ramp region (idx ~29..91).
   Below effective index ~29 output floors and the model overstates
   back-off. For this unit's wall-equalized diffs, that floor lands at
   offset_qdb ~= -16 on the fastest rungs (MCS7 sits at effective idx 45 at
   offset 0 with the default 1 dB margin, and 45-16=29). The pre-flight
   bench campaign for any unit must either raise `min_offset_qdb` to this
   floor-aware value or re-measure the ramp/floor boundary for that unit's
   card; an MCS7 offset sweep down to the floor belongs in the acceptance
   list before flight.

## Consuming the walls

The measured wall table above is now live config, not just findings. It's
read at `radio.rate_walls_idx` in `bundle/mabur.default.json`, set to this
unit's measured ceilings (with `legacy_wall_idx` for the OFDM control rate),
alongside `wall_margin_db` and `base_ref_idx`. `radio.power_mode: "offset"`
switches the drone from the deployed `"none"` baseline to the adaptive path:
`drone/src/power_plan.h` derives per-rate qdB diffs from the walls so that,
at offset 0, every rate's effective TXAGC index sits at `wall − margin`
(`wall_margin_db`, in qdB steps) — every rate parked at wall-minus-margin,
level-continuous with the `"none"` baseline measured above. `max_offset_qdb`
is always 0 under this formulation: offset 0 already equalizes every rate to
its wall, so the controller only ever backs off (`offset ≤ 0`), scaling
every rate down uniformly by the commanded amount. A different unit's card
gets its own walls by re-running the sweep (`bench/txagcbench/`) and
updating `rate_walls_idx` — the walls do not transfer unit-to-unit (see
Caveats below).

## How the measurement works (methodology)

Tooling: `bench/txagcbench/` (branch `txagcbench`; spec under gitignored
`docs/superpowers/specs/2026-07-16-txagcbench-design.md`).

- **Instrument**: the GS-side mabur radio's per-frame RSSI (chain B — chain A
  is off-scale on the 8822E, `common/include/mabur/node.h:23`). No absolute
  calibration needed: only the curve *shape* matters, because a constant
  offset cancels between the controller's `update()` normalization and its
  `resolve()` inversion.
- **Sweep** (`txagcbench-tx`, drone): one radio bring-up, then per index:
  `SetTxPowerIndexOverride(idx)` → 100 ms settle → 50 raw frames (no FEC) at
  ~2 ms spacing, each payload stamped with `{idx, pass}` so RX attribution
  needs no clock sync. Full range ascending (pass 1) then descending
  (pass 2) — up/down disagreement per index exposes thermal drift. A
  gap-length drain before each index change stops FIFO carryover
  (a frame transmitting at power N+1 while stamped N).
- **Record** (`txagcbench-rx`, GS): one JSONL line per CRC-clean bench frame;
  corrupt frames are counted but never attributed (their payload can't be
  trusted to name an index). Bodies arrive FCS-suffixed per devourer's RX
  contract — the parser accepts 64 and 68 bytes, validating only the first 64.
- **Analyze** (`analyze_sweep.py`): median RSSI per index (impulse-robust),
  both curves re-anchored to the lowest measured index, PASS iff RMS ≤ 1 dB
  and max ≤ 2 dB vs the committed table. Health guards: median > −45 dBm →
  RX-saturation warning; < −85 dBm → sensitivity-floor warning; per-index
  sample minimums; monotonicity check. `--emit-table` prints a drop-in
  replacement `kTxagcGainDb[]`. `--selftest` runs hardware-free (positive and
  negative control).
- **Walls**: delivery fraction per index from the same JSONL (received/100),
  per MCS; the MCS0 run is the control separating "TX didn't emit" from
  "PA compression killed decode".

### Reproduction

```sh
tools/build-arm.sh && tools/build-arm64.sh      # cross-build tx (drone) / rx (GS)

# gain-shape run + verdict vs committed table (stops if daemons are running):
bench/txagcbench/run_sweep.sh --emit-table

# per-MCS wall matrix (~12 min, daemons stopped around the whole loop):
for m in 0 1 2 3 4 5 6 7; do
  TX_ARGS="--mcs $m --hi 127" OUT=/tmp/txagc_mcs$m.jsonl bench/txagcbench/run_sweep.sh || true
done
```

Rig for the 2026-07-16 runs: drone (`root@192.168.10.152`, TX) and GS
(`root@10.18.0.1`, RX chain B) 3 m apart on the bench, channel 149, 20 MHz.
Run quality: 0 CRC errors across all ten sweeps, drift ≤ 1 dB, no RX
saturation (max −62 dBm), MCS0 delivery ~100% at every index.

### Caveats

- RSSI quantization is 1 dB, so the 0.25 dB fine structure is coarsely
  sampled; the ramp slope (~0.3 dB/idx) is an average.
- Sensitivity floors (low-index dropoff at MCS5–7) are geometry-dependent —
  re-derive at range if needed. The walls are not.
- Single unit characterized. PA compression points vary unit-to-unit; a
  second drone card should get its own sweep before trusting these exact
  numbers (the tool makes that a 12-minute job).
- Walls measured at 20 MHz / 5.8 GHz ch149. Different bandwidth or band =
  re-measure.
- The per-MCS delivery-vs-RSSI edges captured in these JSONLs are also a
  free cross-check dataset for the `LinkTable` SNR→delivery model (the
  controller's other uncalibrated table) — unanalyzed so far.

## Data

Raw sweep JSONLs (one line per received frame: `idx, pass, seq, rssi_a,
rssi_b, snr_a, snr_b`): `txagc_sweep.jsonl` (MCS0 0..63),
`txagc_sweep_ext.jsonl` (MCS0 0..127), `txagc_mcs0..7.jsonl` (the matrix) —
session scratchpad copies; regenerate with the commands above.
