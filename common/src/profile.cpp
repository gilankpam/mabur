#include "mabur/profile.h"

#include <algorithm>
#include <string>

namespace mabur::rc {

namespace {
uint8_t bw_to_code(uint8_t bw) {
  switch (bw) {
    case 20: return 0;
    case 40: return 1;
    case 80: return 2;
    default: return 0;
  }
}

uint8_t code_to_bw(uint8_t code) {
  switch (code) {
    case 0: return 20;
    case 1: return 40;
    case 2: return 80;
    default: return 20;
  }
}

const double kHt20Lgi[8] = {6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0};
const double kHt40Lgi[8] = {13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0};
const double kVht20Lgi[9] = {6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0, 78.0};
const double kVht40Lgi[9] = {13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0, 162.0};
const double kVht80Lgi[9] = {29.3, 58.5, 87.8, 117.0, 175.5, 234.0, 263.3, 292.5, 351.0};
constexpr double kSgiFactor = 10.0 / 9.0;

}  // namespace

uint8_t encode_profile(PhyMode mode, uint8_t mcs, uint8_t bw) {
  uint8_t p = static_cast<uint8_t>((mcs & 0x0F) | (bw_to_code(bw) << 4));
  if (mode == PhyMode::VHT) p |= 0x40;
  return p;
}

void decode_profile(uint8_t p, PhyMode& mode, uint8_t& mcs, uint8_t& bw) {
  mode = (p & 0x40) ? PhyMode::VHT : PhyMode::HT;
  bw = code_to_bw(static_cast<uint8_t>((p >> 4) & 0x3));
  uint8_t raw_mcs = static_cast<uint8_t>(p & 0x0F);
  uint8_t top = (mode == PhyMode::VHT) ? 8 : 7;
  mcs = std::min(raw_mcs, top);
}

std::string ladder_spec_str(PhyMode mode, uint8_t mcs, uint8_t bw) {
  int top = (mode == PhyMode::VHT) ? 8 : 7;
  const char* name = (mode == PhyMode::VHT) ? "VHT1SS_MCS" : "MCS";
  int m = std::clamp(static_cast<int>(mcs), 0, top);
  int base_m = std::max(m - 1, 0);
  std::string base_tok = std::string(name) + std::to_string(base_m) + "/" + std::to_string(bw);
  std::string enh_tok = std::string(name) + std::to_string(m) + "/" + std::to_string(bw);
  return "BASE=" + base_tok + ";ENH=" + enh_tok;
}

// hw 2026-07-26: the inherited devourer default (T1 = m+1, T2 = m+2) put
// SVC-T enhance traffic past this link's wall — 20-42% RF loss concentrated
// on the enhance frames, FEC (0.25x overhead) hopeless against it, lost
// frame_ids stalling the GS FrameStream. Ruling: mcs is scored for the base
// operating point, not for a faster per-rung rate; all four rungs (CRIT,
// T0, T1, T2) ride that same base mcs. Same-day follow-up: T1/T2 also carry
// T0's ldpc/stbc — flags-off T1/T2 measured 2-3 dB weaker on air at the
// same MCS (single-chain TX, no coding gain), with the weaker GS card
// losing 3-5x more of stream 3's frames. Per-layer differentiation is
// exclusively the FEC overhead ladder (1.00/0.50/0.50/0.50 x scale,
// flattened 2026-08-29 -- was 1.00/0.75/0.50/0.25).
//
// 2026-08-29: base rides mcs−1, enh rides the scored mcs — UEP via rate,
// always on (fixed rule mirrored by the GS, RC_VERSION 4). The 2026-07-26
// rule "nothing flies ABOVE the scored mcs" still holds; base below extends
// margin downward. Spec 2026-08-29-airtime-balance-uep.
//
// 2026-08-30 RULING (spec 2026-08-30-same-rate-fixed-pairs): both streams
// ride the scored mcs. UEP is per-rung FEC overhead pairs carried in the
// v5 RCF — no rate split, no runtime redistribution. Measurement basis:
// docs/same-rate-uep-findings-2026-08-30.md (static/motion/loss sweeps).
std::array<LayerTxSpec, 2> ladder_from(PhyMode mode, uint8_t mcs, uint8_t bw) {
  int top = (mode == PhyMode::VHT) ? 8 : 7;
  int m = std::clamp(static_cast<int>(mcs), 0, top);

  std::array<LayerTxSpec, 2> ladder;
  // BASE
  ladder[0].mode = mode;
  ladder[0].mcs = static_cast<uint8_t>(m);  // EXPERIMENT: same-rate (was max(m-1,0))
  ladder[0].bw = bw;
  ladder[0].ldpc = true;
  ladder[0].stbc = true;
  // ENH
  ladder[1].mode = mode;
  ladder[1].mcs = static_cast<uint8_t>(m);
  ladder[1].bw = bw;
  ladder[1].ldpc = true;
  ladder[1].stbc = true;
  return ladder;
}

const std::array<ProfileRow, 5>& profile_table() {
  static const std::array<ProfileRow, 5> table = {{
      {0, 2.00, 2.00, 20},
      {1, 1.50, 1.50, 20},
      {2, 1.00, 1.00, 20},
      {4, 0.50, 0.50, 20},
      {5, 0.20, 0.20, 20},
  }};
  return table;
}

std::array<LayerTxSpec, 2> ladder_for_row(int idx) {
  const auto& row = profile_table().at(static_cast<size_t>(idx));
  return ladder_from(PhyMode::HT, row.mcs, row.bw);
}

double phy_rate_mbps(const LayerTxSpec& s) {
  double r = 0.0;
  if (s.mode == PhyMode::HT) {
    const double* base = (s.bw == 40) ? kHt40Lgi : kHt20Lgi;
    int mcs = std::clamp(static_cast<int>(s.mcs), 0, 7);
    r = base[mcs];
  } else {
    const double* base = kVht20Lgi;
    if (s.bw == 40) base = kVht40Lgi;
    else if (s.bw == 80) base = kVht80Lgi;
    int mcs = std::clamp(static_cast<int>(s.mcs), 0, 8);
    r = base[mcs];
  }
  return r * (s.sgi ? kSgiFactor : 1.0);
}

}  // namespace mabur::rc
