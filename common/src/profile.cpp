#include "mabur/profile.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace mabur::rc {

namespace {
// Parses one rung token, e.g. "MCS0/20/LDPC" or "VHT1SS_MCS7/40/SGI", into a
// LayerTxSpec. Mirrors devourer's parse_tx_mode_str minimally, for the
// LayerTxSpec fields this port needs (mode/mcs/bw/ldpc/stbc/sgi).
LayerTxSpec parse_rung_token(const std::string& tok) {
  LayerTxSpec spec;
  std::vector<std::string> parts;
  std::stringstream ss(tok);
  std::string part;
  while (std::getline(ss, part, '/')) parts.push_back(part);

  const std::string& name = parts[0];
  constexpr const char* kVhtPrefix = "VHT1SS_MCS";
  if (name.rfind(kVhtPrefix, 0) == 0) {
    spec.mode = PhyMode::VHT;
    spec.mcs = static_cast<uint8_t>(std::stoi(name.substr(std::string(kVhtPrefix).size())));
  } else {
    spec.mode = PhyMode::HT;
    constexpr const char* kMcsPrefix = "MCS";
    spec.mcs = static_cast<uint8_t>(std::stoi(name.substr(std::string(kMcsPrefix).size())));
  }
  spec.bw = static_cast<uint8_t>(std::stoi(parts[1]));
  for (size_t i = 2; i < parts.size(); ++i) {
    if (parts[i] == "LDPC") spec.ldpc = true;
    else if (parts[i] == "STBC") spec.stbc = true;
    else if (parts[i] == "SGI") spec.sgi = true;
  }
  return spec;
}

// Parses a full "CRIT=...;T0=...;T1=...;T2=..." ladder spec string into the
// 4-rung array, in CRIT/T0/T1/T2 order (the order the spec strings are
// always written in, per ladder_spec_str above).
std::array<LayerTxSpec, 4> parse_ladder_spec(const std::string& s) {
  std::array<LayerTxSpec, 4> out;
  std::stringstream ss(s);
  std::string clause;
  int i = 0;
  while (std::getline(ss, clause, ';')) {
    size_t eq = clause.find('=');
    out[static_cast<size_t>(i)] = parse_rung_token(clause.substr(eq + 1));
    ++i;
  }
  return out;
}

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
  // All four rungs ride the same (scored) base mcs — see ladder_from below
  // for why.
  std::string tok = std::string(name) + std::to_string(m) + "/" + std::to_string(bw);
  return "CRIT=" + tok + ";T0=" + tok + ";T1=" + tok + ";T2=" + tok;
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
std::array<LayerTxSpec, 4> ladder_from(PhyMode mode, uint8_t mcs, uint8_t bw) {
  int top = (mode == PhyMode::VHT) ? 8 : 7;
  int m = std::clamp(static_cast<int>(mcs), 0, top);

  std::array<LayerTxSpec, 4> ladder;
  // CRIT
  ladder[0].mode = mode;
  ladder[0].mcs = static_cast<uint8_t>(m);
  ladder[0].bw = bw;
  ladder[0].ldpc = true;
  ladder[0].stbc = true;
  // T0
  ladder[1] = ladder[0];
  // T1/T2: identical PHY to T0 (rate and flags).
  ladder[2] = ladder[1];
  ladder[3] = ladder[1];
  return ladder;
}

const std::array<ProfileRow, 5>& profile_table() {
  static const std::array<ProfileRow, 5> table = {{
      {"CRIT=MCS0/20/LDPC;T0=MCS0/20/LDPC;T1=MCS0/20;T2=MCS0/20", 1.00, 20},
      {"CRIT=MCS0/20/LDPC;T0=MCS1/20;T1=MCS2/20;T2=MCS2/20", 0.75, 20},
      {"CRIT=MCS1/20/LDPC;T0=MCS2/20;T1=MCS4/20;T2=MCS4/20", 0.50, 20},
      {"CRIT=MCS2/20;T0=MCS4/20;T1=MCS5/20;T2=MCS7/20/SGI", 0.25, 20},
      {"CRIT=MCS4/20;T0=MCS5/20;T1=MCS7/20;T2=MCS7/40/SGI", 0.10, 20},
  }};
  return table;
}

std::array<LayerTxSpec, 4> ladder_for_row(int idx) {
  const auto& row = profile_table().at(static_cast<size_t>(idx));
  auto ladder = parse_ladder_spec(row.svc_ladder);
  ladder[0].ldpc = true;
  ladder[0].stbc = true;
  ladder[1].ldpc = true;
  ladder[1].stbc = true;
  // hw 2026-07-26 ruling (+ same-day follow-up): the vendored rows keep
  // devourer's byte-exact spec strings (some still spell out a per-rung
  // T1/T2 spread), but all video streams above CRIT ride T0's ENTIRE
  // spec — rate fields and the unconditional ldpc/stbc alike. Redundancy,
  // not PHY, differentiates layers: flags-off T1/T2 was tried first and
  // measured 2-3 dB weaker on air at the same MCS. Enforce at this parse
  // choke point so the table stays a faithful port while the applied
  // ladder is uniform; matches ladder_from (the RCF path).
  ladder[2] = ladder[1];
  ladder[3] = ladder[1];
  return ladder;
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
