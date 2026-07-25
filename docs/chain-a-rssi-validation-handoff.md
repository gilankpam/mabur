# Handoff: validate the "chain-A RSSI is off-scale on the 8822E" claim

Status: RESOLVED 2026-07-25 — **claim REFUTED on hardware** (devourer HEAD
`13998ec`, both GS cards, both DPDT modes, ~42k frames: chain A never read
≥128; it tracks chain B within ~1.5 dB). `rssi_a` is unblocked for the GS
stats sideport. Hypothesis 1 (stale-era artifact) confirmed as the verdict;
see "Validation results" below for the evidence. mabur fixes applied:
CardTrack now has `rssi_a_ema`, comments corrected, per-card stderr stats
print rssiA. The rebuild of deployed maburgs against synced ../devourer is
still pending (tracked separately).

## Why this matters

The GS stats sideport design (paused, see task list / future spec at
`docs/superpowers/specs/`) publishes per-card link stats for debug tooling and
the PixelPilot OSD. It omits chain-A RSSI entirely because the codebase claims
the value is garbage. If the claim is stale, we are throwing away half the
per-antenna RSSI picture on every GS card; if it is real, devourer's parser is
passing through a value it should be flagging invalid. Either outcome produces
a concrete fix.

## The claim, verbatim

Two comments in mabur, both introduced in commit `88b44cb` (2026-07-12,
"feat(gs): aggregator — RxBody routing + per-card link tracking"):

- `gs/src/aggregator.h:12-15`:
  > rssi_b_ema tracks chain B: chain A reads off-scale (128-131) on the 8822E
  > (devourer phystatus bug); snr_ema tracks the best chain.
- `common/include/mabur/node.h:23`:
  > `uint8_t rssi[2] = {0, 0};  // per-chain raw (chain A is off-scale on 8822E)`

Consequences in code today:

- `CardTrack` has `rssi_b_ema` but no `rssi_a_ema` (`gs/src/aggregator.h`).
- The adaptive-link score window feeds `max(rssi[0], rssi[1])`
  (`gs/src/main.cpp`, the `vrx.on_video(...)` call in the drain loop) — so if
  chain A really is pinned at 128-131, **the score window's RSSI input is
  dominated by garbage** (128 raw beats any real signal), and only the
  snr-weighted scoring (rssi_weight 0.3 vs snr_weight 0.7,
  `gs/src/score.h`) has been masking it. This makes validation more than a
  cosmetic question.

Provenance: the comment is the only record. There is no captured log or bench
note with the raw 128-131 observation. It dates to the 2026-07-12 per-chain
perf-gap triage era.

## Data path (where the byte comes from)

1. Chip emits a 32-byte "jgr3" PHY-status report before each PSDU
   (APP_PHYSTS). Parsed in devourer:
   `../devourer/src/jaguar3/FrameParserJaguar3.h`, `parse_phy_sts_jgr3()`
   (~line 262 at HEAD `13998ec`). For OFDM pages, per-path pwdb bytes sit at
   physts bytes 1..4 → `a.rssi[i] = physts[1 + i]` — **raw pass-through, no
   validity check**. Convention: dBm = raw − 110.
2. GS `RadioFrontend::on_packet` copies `pkt.RxAtrib.rssi[0..1]` and
   `snr[0..1]` into the `RxBody` wire struct (`gs/src/radio_frontend.cpp:138`).
3. `Aggregator::on_rx_body` (CRC-clean frames only) updates the EMAs and drops
   chain-A RSSI on the floor.

Note the raw values: 128-131 − 110 = **+18..+21 "dBm"**, physically impossible
for RX power. Also note 128-131 ≈ `0x80-0x83`: a set high bit is a classic
vendor-phydm "invalid/not reported" sentinel pattern. Chain-A **SNR**
(physts bytes 24..27, same report) is trusted and used everywhere, which is
itself evidence the report as a whole is being parsed at the right offsets.

## Do NOT conflate with the chain-B DPDT saga

devourer has a *documented, fixed* per-chain problem on the same silicon that
is easy to confuse with this one — `../devourer/docs/8822e-quirks.md`
("eFEM pin-mux" section):

- The old static DPDT route parked the antenna transfer switch TX-favoring:
  **RX path B lost its antenna on every 8812EU** — chain-B pwdb pinned at the
  *noise floor* (~10 raw / −99 dBm). Fixed by the full eFEM GPIO pin-mux
  (`RtlJaguar3Device::efem_pinmux_8822e()`), bench-verified 2026-07-14
  (RX A/B −70/−77 dBm concurrently). Selectable for A/B via
  `DEVOURER_DPDT_MODE=efem|legacy|bit24|skip`.

Two different chains, two different signatures (B pinned *low* ~10 raw vs A
pinned *high* 128-131), two different eras. Crucially, the mabur claim
(2026-07-12) **predates** the eFEM pin-mux fix (2026-07-14), so the chain-A
observation was made on a build with the RF front-end misrouted — the deployed
maburgs is still built against pre-fix devourer as of this writing. The claim
may simply be stale.

## Hypotheses, ranked

1. **Stale-era artifact**: chain-A pwdb misreported under the old static-DPDT
   front-end route; sane on devourer HEAD with eFEM pin-mux active.
2. **Vendor sentinel passed through**: raw ≥ 0x80 means "path not
   reporting" in the vendor's phydm and devourer's parser forwards it
   unfiltered. (Would mean the bug is "no validity gating", not "chain A is
   garbage" — and it might now appear on *no* chain, or on whichever chain is
   idle.)
3. **Parser layout bug for pwdb_a specifically** (unlikely: chain-A SNR from
   the same report parses sane, and jgr3 offsets match jgr2 which is
   independently validated on 8821C).
4. **Board-specific front-end behavior** on these 8812EU modules leaving
   path-A pwdb undefined in monitor RX.

## Validation plan

Environment: nix-shell with `pkg-config libusb1` for devourer builds. Devices:
drone `root@192.168.10.152`, GS `root@10.18.0.1`. **Never load the 8812eu
kernel module on the GS cards** (known to wedge them); any kernel-driver
cross-check happens on a bench adapter on a dev host only.

1. **Reproduce/refute on devourer HEAD** (sibling checkout
   `../devourer`, HEAD `13998ec`, includes the eFEM fix):
   build `rxdemo`, run against a GS-type 8812EU on the bench with
   `DEVOURER_RX_ALLPATHS=1` (emits per-chain `rx.path` JSONL events with
   RSSI/SNR/EVM per path) while a second adapter transmits. Record the raw
   `rssi[0]` vs `rssi[1]` distributions.
   - Sane, tracking chain B within a few dB → hypothesis 1; claim stale.
   - Pinned 128-131 → continue.
2. **DPDT A/B**: repeat with `DEVOURER_DPDT_MODE=legacy` and `bit24`. If
   off-scale chain A appears only in non-`efem` modes, that's confirmation of
   hypothesis 1 with a mechanism.
3. **Antenna swap**: swap A/B pigtails, then remove antenna A entirely. Does
   the 128-131 signature follow the RF port (hardware) or stay with the chain
   index (parser/sentinel)? An idle/disconnected path reading 0x80-ish
   supports hypothesis 2.
4. **Vendor cross-check** (hypothesis 2/3): `git submodule update --init
   reference/rtl88x2eu` in `../devourer`, then read
   `hal/phydm/phydm_phystatus.{h,c}` — `phy_sts_rpt_jgr3_ofdm_cmn` byte layout
   and, specifically, whatever validity gating / conversion the vendor applies
   to per-path pwdb before it becomes RSSI (look for 0x80 sentinels, path-count
   masks from `rf_path` config, or `>> 1` half-dB scaling). Compare against
   `parse_phy_sts_jgr3()`.
5. **Attenuation sweep** (only if chain A reads sane): verify rssi_a actually
   *tracks* — step an attenuator or distance and check monotonic response on
   both chains, plus `GetActiveRxPaths()` agreement.

## Exit criteria and follow-ups

- **Claim refuted on current devourer** →
  - fix the comments in `gs/src/aggregator.h` and `common/include/mabur/node.h`;
  - add `rssi_a_ema` to `CardTrack` (same EMA treatment as B);
  - feed the score window an honest per-frame max again;
  - add `rssi_a` to the sideport schema (additive, v1 field) and a `rssiA`
    column to the planned `tools/maburtop.py` layout;
  - note that this rides the already-pending "rebuild maburd/maburgs against
    synced ../devourer" work — do the RTP regression check afterwards
    (`tools/bench/rtpsniff.py` on GS :5600).
- **Claim confirmed** →
  - file/fix in devourer: `parse_phy_sts_jgr3()` should mark invalid paths
    (e.g. clamp/sentinel raw ≥ 0x80 → 0) rather than forwarding garbage;
  - keep `rssi_a` out of the sideport; update the aggregator comment with a
    pointer to this doc and the devourer issue;
  - either way, stop feeding raw `max(rssi[0], rssi[1])` into the score
    window until the value is trustworthy.
- Update this doc's Status line and the memory entry that points here when
  resolved.

## Validation results (2026-07-25)

Bench: drone (maburd, ch149/20MHz, stock config) transmitting; rxdemo built
from devourer HEAD `13998ec` via mabur's arm64 cross toolchain, run on the GS
with `DEVOURER_RX_ALLPATHS=1`; maburgs stopped during capture, restarted and
RTP-verified clean afterwards (59.4 fps, 0 gaps, 0 bad frames). Raw per-chain
distributions, first 200 frames (AGC settle) excluded:

| Cell | n | rssi_a mean (raw) | rssi_b mean (raw) | A or B ≥128 |
|---|---|---|---|---|
| card1, efem (HEAD default) | 9977 | 58.8 | 60.0 | 0 |
| card1, DPDT legacy (pre-fix route) | 10009 | 59.0 | **10.9** | 0 |
| card1, TX −3 dB (wall_margin 4.0) | 5231 | 57.2 | 59.0 | 0 |
| card1, TX −5 dB (wall_margin 6.0) | 5225 | 58.5 | 58.0 | 0 |
| card2 (port 1.4), efem | 5274 | 59.0 | 60.5 | 0 |

Findings:

- **Chain A is sane and tracks chain B within ~1.5 dB in every cell.** Raw
  58–60 ≈ −51 dBm, plausible for the bench geometry. Zero samples ≥128 in
  ~42k frames across two cards.
- The `legacy` DPDT cell reproduces the *documented* chain-B-at-noise-floor
  signature (raw ~11 ≈ −99 dBm) from `../devourer/docs/8822e-quirks.md` —
  and even under that pre-fix front-end route, chain A does **not** read
  128–131. The original observation was made on a pre-eFEM build; whatever
  produced 128–131 does not reproduce on current devourer in either mode.
- devourer git history shows no jgr3 pwdb parsing change since the claim era
  — only the eFEM pin-mux (#289) landed in between, supporting hypothesis 1.
- Dynamic range confirmed: chain A reads noise floor (~11–12 raw) during
  init/AGC settle and normal levels after; both chains move together when TX
  power changes. (TX-side response to the margin sweep is compressed because
  the walls sit at PA-compression ceilings — equally visible on both chains,
  so not an rssi_a validity issue.)
- Vendor phydm cross-check (plan step 4) skipped: it gates hypotheses 2/3,
  which have no observable support on this hardware.

Follow-ups applied in mabur (refuted path): `CardTrack.rssi_a_ema` added with
the same EMA treatment as B (`gs/src/aggregator.{h,cpp}`), stale comments
fixed (`gs/src/aggregator.h`, `common/include/mabur/node.h`), per-card stderr
stats now print `rssiA`, `tests/test_aggregator.cpp` updated (58/58 host
tests pass). The score window's `max(rssi[0], rssi[1])` input is now known to
be honest — no change needed. When the GS stats sideport spec is written,
include `rssi_a` (additive v1 field) and a `rssiA` column in
`tools/maburtop.py`.
