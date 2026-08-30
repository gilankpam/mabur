#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace mabur::rc {

// PHY-facing profile byte, adaptive-link ladder spec, and PHY rate tables.
// Byte-exact port of devourer's tools/precoder/rc_proto.py
// (encode_profile/decode_profile/DEFAULT_PROFILE_TABLE) and
// tools/precoder/adaptive_link.py (ladder_spec) and
// tools/precoder/energy_model.py (phy_rate_mbps rate tables). The per-seq
// bandwidth-probe schedule was removed 2026-07-27 (SDD ladder-controller
// Task 5): the ladder controller never varies bw independently of the
// commanded rung, so probing alternate widths per seq had no consumer left.

enum class PhyMode : uint8_t { HT, VHT };

struct LayerTxSpec {
  PhyMode mode = PhyMode::HT;
  uint8_t mcs = 0;
  uint8_t bw = 20;
  bool sgi = false, ldpc = false, stbc = false;
  int8_t power_offset_db = 0;  // carried, unused in v1
};

// (mode, mcs, bw) -> wire PROFILE byte. bits[3:0]=mcs, bits[5:4]=bw code
// (20->0, 40->1, 80->2), bit6=VHT.
uint8_t encode_profile(PhyMode mode, uint8_t mcs, uint8_t bw);

// Wire PROFILE byte -> (mode, mcs, bw). mcs is clamped to 7 (HT) / 8 (VHT).
void decode_profile(uint8_t p, PhyMode& mode, uint8_t& mcs, uint8_t& bw);

// Builds the 2-slot ladder (BASE, ENH) for a (mode, scored mcs, bw)
// operating point. Both slots ride the scored mcs — same-rate, UEP is
// per-layer FEC overhead only (2026-08-30 ruling, see the comment on the
// definition). Both slots carry LDPC+STBC unconditionally on (the config
// policy was removed 2026-07-26 — all-true was the only shape ever flown,
// and flags-off measured 2-3 dB weaker at the same MCS).
std::array<LayerTxSpec, 2> ladder_from(PhyMode mode, uint8_t mcs, uint8_t bw);

// DEVOURER_SVC_LADDER-style spec string, 2-slot since the 2026-08-29
// mcs-1 UEP-via-rate rule: "BASE={name}{m-1}/{bw};ENH={name}{m}/{bw}" with
// name = "VHT1SS_MCS" (VHT) or "MCS" (HT).
std::string ladder_spec_str(PhyMode mode, uint8_t mcs, uint8_t bw);

struct ProfileRow {
  uint8_t mcs;
  double ov_base;
  double ov_enh;
  uint8_t bw;
};

// DEFAULT_PROFILE_TABLE verbatim (energy-ranked, index 0 = max range/failsafe
// -> index 4 = max quality).
const std::array<ProfileRow, 5>& profile_table();
constexpr int MAX_RANGE_PROFILE = 0;

// Builds the 2-slot ladder for profile_table()[idx] — equivalent to
// ladder_from(mode, row.mcs, row.bw) (HT mode; the vendored table is
// HT-only).
std::array<LayerTxSpec, 2> ladder_for_row(int idx);

// On-air PHY data rate (Mbps) for a LayerTxSpec.
double phy_rate_mbps(const LayerTxSpec& s);

}  // namespace mabur::rc
