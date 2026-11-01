# Drone self-calibration spike — can the 8812EU measure its own TX-power walls?

**Status:** negative result; the walls stay a receiver-side measurement
(`docs/calibration.md`). Nothing built. The throwaway register hook lives
on a local branch of the sibling devourer checkout (`spike-dpk-gl`), never
to merge.

Question: `maburcal` finds each MCS's wall by having the GS count delivered
frames while the drone sweeps its TXAGC index — it needs a linked, silent GS
for ~80 s. Could the drone find the walls on its own, with no receiver,
from something the chip can observe about its own transmitter?

## What the chip can observe

The RTL8822E family has three self-measurement paths in silicon, all
driven through the IQK engine (`0x1b00` commands) over an internal TX→RX
loopback inside the transceiver:

- **DPK gain-loss** (`halrf_dpk_8822e.c`): transmits a pattern at a chosen
  RF gain code (`RF 0x00[4:0]`, 0..31) and reports the PA's gain loss in
  dB (`0x1bfc[7:4]` with `0x1bd4[21:16] = 6`) plus a 64-entry "PA scan" —
  output amplitude at 64 input amplitudes, the AM-AM curve, normalised to
  8192 at the top bin. The vendor's own `_dpk_pas_agc` walks this to find
  the code where gain loss reaches its target, i.e. it is literally a
  compression search.
- **TSSI detector** (`halrf_tssi_8822e.c`): a one-shot power-detector read
  (`0x28a4[8:0]`).
- **TXGAPK**: per-gain-step offsets (a ±half-dB correction table, not a
  power readback).

Two vendor facts narrow it at once. `do_dpk_8822e` **force-bypasses DPK
on RFE type 21/22** — the external-FEM front end every 8812EU module here
reports (`rfe_type=0x15`); devourer logs the same bypass. And the 8822E
runs **open-loop** in the vendor driver: `power_track_type` is hard-coded
0 (thermal tracking), and neither `halrf_do_tssi` nor
`halrf_tssi_get_de` dispatches to the E at all — the TSSI code for it is
dead. So the DPK loopback was the one path worth running, bypass or not.

## Method

Host EU card (0bda:a81a, RFE 21), channel 144, 20 MHz.

**Chip side.** A throwaway env-gated hook in devourer's `Halrf8822e`
ports the DPK measurement core (tx pause → MAC/BB/AFE/KIP presets → RF
loopback preset `RF0 = 0x50016`, RX DCK → per code: SYNC_DC, DAGC with
the vendor's PGA adjust, one LBK RXIQK, MDPK_DC, GAIN_LOSS → read GL,
DGain, DC/corr and the 64-entry PA scan), run after bring-up as a sweep
over RF gain code 0..31 instead of the vendor's search. Restores the
vendor's register lists afterwards.

**Air side, same card.** `txdemo` bursts (200 frames, 5 ms apart, QoS-Data,
LDPC+STBC) at a flat TXAGC index (`DEVOURER_TX_PWR`, per-rate diffs
zeroed), stepping the index 40..127, at MCS7 and MCS0; the GS's second
card (maburgs stopped) ran `rxdemo` with per-frame dumps (CRC, RSSI,
EVM), the burst's index encoded in its payload length. The two axes meet
through the vendor's TXAGC table init: **RF gain code = index >> 2**
(`phydm_txagc_tab_buff_init_8822e`), 1 dB per code.

## Results

Chip-side sweep (one run; a second run was not needed — the engine's
in-range region is unambiguous):

| RF code | DGain | GL | PA-scan small/large-signal ratio |
|---|---|---|---|
| 0–25 | 2896 (DAGC pinned at max, PGA at its 0xf cap) | 0–15 (noise) | 0.2–32 (noise) |
| 26 | 2604 | 0 | 1.08 (0.3 dB) |
| 27 | 2327 | 0 | 1.06 (0.25 dB) |
| 28 | 2084 | 1 | 1.05 (0.2 dB) |
| 29 | 1863 | 0 | 1.26 (1.0 dB) |
| 30 | 1666 | 0 | 1.35 (1.3 dB) |
| 31 | 1490 | 0 | 1.44 (1.6 dB) |

The loopback signal on this module is ~15 dB weaker than the vendor
sequence expects (the vendor starts at code 22 / PGA 0xc and expects
DGain 768–1535; here PGA 0xf still pins DGain at 2896 up to code 25), so
the engine only measures from code 26 up. There it sees a **soft knee at
code 29**: 1.0–1.6 dB of AM-AM gain loss over the last three codes, GL
reading 0 throughout.

Air side, the same card into the GS:

| index | RF code | MCS7 delivered | RSSI | EVM dB | MCS0 delivered | RSSI | EVM dB |
|---|---|---|---|---|---|---|---|
| 64 | 16 | 200/200 | 74.0 | −53 | 200/200 | 70.0 | −56 |
| 68 | 17 | 200/200 | 75.3 | −49 | | | |
| 72 | 18 | 200/200 | 76.0 | −45 | 200/200 | 74.2 | −47 |
| 76 | 19 | 106 then nothing | 76.8 | −42 | | | |
| 80 | 20 | 0 (two sweeps; one isolated burst did arrive) | | | 200/200 | 75.3 | −37 |
| 84–127 | 21–31 | 0, not even CRC-bad | | | | | |
| 96 | 24 | | | | 200/200 | 76.7 | −26.5 |
| 112 | 28 | | | | 199/200 | 76.2 | −21 |

MCS7 EVM degrades 4 dB per 1 dB of drive from code 16 and the frames stop
decoding at code 19–20; RSSI stops rising at code 20 and MCS0 EVM runs
down to −21 dB by code 28 while MCS0 still decodes. That is the known
shape of the wall: the host card's MCS7 wall at code ~19 sits where the
drone's calibrated `+21` (relative to an anchor of 46–51) lands too.

Put on one axis: **the air shows output power flat and EVM −26 dB at code
24–28, while the chip's loopback at the same codes reports 0.2–0.3 dB of
gain loss.** The loopback taps before the stage that compresses — the
external FEM behind the PAPE pin — exactly why the vendor bypasses DPK on
this RFE type. The chip only ever sees its own driver stage, whose knee
(code 29) is ~10 dB above the MCS7 wall.

## Where does the loopback tap? — the lever tests

The result above infers "the loopback taps before the FEM" from the
vendor's bypass plus the mismatch. To test it directly the hook grew
levers, each checked with the GS as a witness (an energy monitor,
`rx.energy`/`rx.nhm` at 200 ms, and frame RSSI):

- **Radiation.** A long train of one-shots at RF code 26–31 trips the GS's
  OFDM CCA at 70–89 counts per window (quiet: ≤11; the bring-up's own IQK:
  ≤8) and lands in the NHM's top bucket — but so do ordinary frames at
  index 0 (RSSI 66.7) and index 100 (RSSI 80): the top bucket saturates
  and cannot tell a chip-only burst from a FEM burst.
- **BB RFE pin selects** (`0x1840/0x1844/0x4140/0x4144`), all pins forced
  to constant 0 (sel 7), to sel 0 (the rfm PAPE source) and to constant 1
  (sel 7 + inversion `0x1850/0x4150`): loopback DGain identical in all
  three (≈2540 at code 26, ≈1470 at 31), radiated CCA counts identical.
- **GPIO4** (halmac: RFE_CTRL_8 = `WL_DPDT_SEL`, the antenna switch),
  muxed to a plain output and driven low/high: **path A's loopback moves
  3–4 dB** (DGain 2599→1882 at code 26, 1498→925 at 31; path B unchanged,
  already at the "high" level) while the GS sees **no change in frame
  RSSI** (76.0 in all four runs). So the loopback does feel what sits
  beyond the chip's RF pin (a load/switch change), but this pin is not
  the PA enable.
- **RFE pin 7** (the other sel-0 pin; halmac: RFE_CTRL_7 on the EECS pad)
  forced to constant 0 / 0xf / 8 during normal frames: RSSI 75 vs 76 —
  not the PA enable either. A select scan (sel 0..15 on pins 7 and 8
  inside the loopback at code 31): path A flat (≈2530), path B drops 4 dB
  on sel 2 only.

So nothing the BB or MAC GPIO block controls toggles the FEM's PA on this
module: PAPE is wired to the transceiver's own PAPE output, which follows
the RF mode word, and the DPK loopback runs in RF mode 5. Whether that
mode asserts PAPE is the one fact still unmeasured; it decides between
"the FEM is off during a one-shot" and "on, and the tap is before it".
Both are blind for software, but only the first could be fought (by a
mode that keeps PAPE up while looping back), and the 3–4 dB switch
sensitivity says the loopback does not traverse the antenna switch — a
post-FEM path would move 20 dB with it, not 3. The direct check is a
spectrum analyser on channel 144: a one-shot train at code 31
(`DEVOURER_DBG_DPK_GL=31:31:1 DEVOURER_DBG_DPK_REPEAT=200`) against a
normal MCS7 burst at index 124 — equal level means FEM on and tap
pre-FEM; 20+ dB lower means the FEM is not driven in loopback mode.

## Conclusion

On RFE-21 hardware the chip has no observable that reaches the FEM, so a
drone-only wall calibration is not possible in software: DPK gain-loss and
the PA scan are blind to the wall, TSSI is unwired/unused on the E, and
TXGAPK is a correction table. It would take a detector after the FEM — a
coupled PDET output from the FEM into the chip's TSSI input — which is a
module-design question, not a firmware one. The efficiency table is also a
receiver-side quantity (delivered bits over the receive span) and was not
attempted.

Bench caveats: RX overload was not excluded by attenuation (the GS card is
metres from the host), but the gradual EVM slope with RSSI still rising
linearly, and the match to the drone's independently calibrated wall,
point at the transmitter. The rxdemo bench was flaky — whole bursts
occasionally never appeared (an entire MCS0 sweep in one pass; the same
indices arrived a few minutes later), per-frame dumps capped near 200
frames per burst, and the GS's management Wi-Fi dropped ssh several times
— so the per-index counts above are the bursts that were logged, not a
delivery percentage; the EVM/RSSI trends are the evidence. Also found on
the way: the vendor's `_dpk_pas_read` LOSS_CHK computes its second power
from the first sample (a copy-paste bug), so its loss ratio is always 1.
