#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace mabur::rc {

// PHY-facing profile byte, adaptive-link ladder spec, bandwidth probe
// schedule, and PHY rate tables. Byte-exact port of devourer's
// tools/precoder/rc_proto.py (encode_profile/decode_profile/probe_bw/
// DEFAULT_PROFILE_TABLE) and tools/precoder/adaptive_link.py (ladder_spec)
// and tools/precoder/energy_model.py (phy_rate_mbps rate tables).

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

struct FlagPolicy {
  bool crit_ldpc = true;
  bool crit_stbc = true;
  bool t0_ldpc = true;
  bool t0_stbc = true;
};

// Builds the 4-rung ladder (CRIT, T0, T1, T2) for a (mode, base mcs, bw)
// operating point, applying fp's ldpc/stbc flags to CRIT and T0 only. All
// four rungs ride the same base mcs (hw 2026-07-26: per-rung +1/+2 MCS
// bumps on T1/T2 put SVC-T enhance traffic past the link wall); per-layer
// differentiation is exclusively FEC overhead plus the CRIT/T0 flag policy.
std::array<LayerTxSpec, 4> ladder_from(PhyMode mode, uint8_t mcs, uint8_t bw,
                                        const FlagPolicy& fp);

// DEVOURER_SVC_LADDER-style spec string, Python adaptive_link.ladder_spec
// identical: "CRIT={name}{m}/{bw};T0=...;T1=...;T2=..." with
// name = "VHT1SS_MCS" (VHT) or "MCS" (HT).
std::string ladder_spec_str(PhyMode mode, uint8_t mcs, uint8_t bw);

struct ProfileRow {
  const char* svc_ladder;
  // qdB power offset from the calibrated baseline (RCF wire semantics,
  // rc_proto bias-64). Bench-tunable; 0 = full legal power — every rate
  // parks at wall - margin under the wall-equalized diffs (max legal
  // offset is ZERO, docs/txagc-calibration.md).
  int8_t pwr_offset_qdb;
  double fec_overhead;
  uint8_t bw;
};

// DEFAULT_PROFILE_TABLE verbatim (energy-ranked, index 0 = max range/failsafe
// -> index 4 = max quality).
const std::array<ProfileRow, 5>& profile_table();
constexpr int MAX_RANGE_PROFILE = 0;

// Builds the 4-rung ladder for profile_table()[idx] by parsing its committed
// svc_ladder spec string (tokens "MCSn"/"VHT1SS_MCSn", "/20|/40|/80",
// optional "/LDPC" "/STBC" "/SGI" in any order). fp's ldpc/stbc flags are
// ONLY ADDED to CRIT/T0 (never removes a flag the ladder string itself sets)
// and never applied to T1/T2.
std::array<LayerTxSpec, 4> ladder_for_row(int idx, const FlagPolicy& fp);

// Bandwidth this video seq must fly at as a rung probe, else -1. rungs =
// sorted(bw_set); slots {0,8,16} of seq%32 -> rungs[i] if i < rungs.size().
int probe_bw(uint16_t seq, const std::vector<uint8_t>& bw_set);

// On-air PHY data rate (Mbps) for a LayerTxSpec.
double phy_rate_mbps(const LayerTxSpec& s);

}  // namespace mabur::rc
