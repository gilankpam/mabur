# Handoff: validate the "chain-A RSSI is off-scale on the 8822E" claim

Status: OPEN — claim unverified, blocks `rssi_a` from the planned GS stats
sideport schema. Written 2026-07-25. This doc is self-contained; start here,
not from the conversation that produced it.

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
