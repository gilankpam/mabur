#include "scan_log.h"

#include <algorithm>
#include <cstdio>

namespace maburgs {

ScanLog::ScanLog(LogWriter& w, const std::string& dir, const std::string& header_info)
    : w_(w), s_(w.open(dir, "scan.log", "scanlog 1 " + header_info)) {}

void ScanLog::put_(const char* b, int n) {
  if (s_ == LogWriter::kBadStream || n <= 0) return;
  w_.line(s_, b, static_cast<size_t>(n));
}

void ScanLog::caps(double t_ms, int card, const CardCaps& c) {
  char b[256];
  const int n = std::snprintf(b, sizeof(b), "C %.0f %d %s %s %dx%d %x %u-%u %d %d %d %d %d",
                              t_ms, card, c.chip.c_str(), c.gen.c_str(), c.tx_chains,
                              c.rx_chains, c.bw_mask, c.tune5g_lo, c.tune5g_hi,
                              c.fast_retune ? 1 : 0, c.fa_ok ? 1 : 0, c.igi_ok ? 1 : 0,
                              c.nhm_ok ? 1 : 0, c.floor_ok ? 1 : 0);
  put_(b, std::min(n, static_cast<int>(sizeof(b) - 1)));
}

void ScanLog::dwell(double t_ms, int card, const ScoutDwell& d) {
  const auto& s = d.survey;
  char igi[8], floor[8];
  if (s.valid_igi) std::snprintf(igi, sizeof(igi), "%d", static_cast<int>(s.igi));
  else std::snprintf(igi, sizeof(igi), "-");
  if (d.floor_valid) std::snprintf(floor, sizeof(floor), "%d", static_cast<int>(d.floor_dbm));
  else std::snprintf(floor, sizeof(floor), "nan");
  char b[256];
  const int n = std::snprintf(
      b, sizeof(b), "D %.0f %d %u %llu %lld %u %u %u %u %s %s %x", t_ms, card,
      static_cast<unsigned>(s.def.primary), static_cast<unsigned long long>(s.round),
      static_cast<long long>(s.observe_ms), s.cca_ofdm, s.fa_ofdm, s.dvr_frames,
      s.frames - s.dvr_frames, igi, floor, static_cast<unsigned>(s.flags));
  put_(b, std::min(n, static_cast<int>(sizeof(b) - 1)));
}

void ScanLog::pick(double t_ms, std::optional<uint8_t> picked, uint64_t rounds,
                   const std::vector<RankEntry>& all, int min_rounds) {
  if (s_ == LogWriter::kBadStream) return;
  char tb[32];
  std::snprintf(tb, sizeof(tb), "%.0f", t_ms);
  std::string line = "K " + std::string(tb);
  line += picked ? " " + std::to_string(static_cast<unsigned>(*picked)) : " none";
  line += " " + std::to_string(static_cast<unsigned long long>(rounds));
  if (picked) {
    for (const RankEntry& e : all) {
      if (e.visits < static_cast<uint32_t>(min_rounds)) continue;
      line += " " + std::to_string(static_cast<unsigned>(e.ch)) + ":" + std::to_string(e.worst_busy);
      if (e.floor_valid) line += ":" + std::to_string(static_cast<int>(e.floor_dbm));
    }
  }
  put_(line.c_str(), static_cast<int>(std::min(line.size(), LogWriter::kMaxLine - 1)));
}

void ScanLog::move(const MoveEvent& e) {
  char card[8];
  if (e.card < 0) std::snprintf(card, sizeof(card), "all");
  else std::snprintf(card, sizeof(card), "%d", e.card);
  char b[128];
  const int n = std::snprintf(b, sizeof(b), "M %.0f %s %u %u %s", e.t_ms, card,
                              static_cast<unsigned>(e.from), static_cast<unsigned>(e.to),
                              to_string(e.reason));
  put_(b, std::min(n, static_cast<int>(sizeof(b) - 1)));
}

void ScanLog::energy(double t_ms, int card, uint8_t ch, const ScoutEnergy& e,
                     uint64_t own, uint64_t foreign) {
  char igi[8];
  if (e.igi_valid) std::snprintf(igi, sizeof(igi), "%d", static_cast<int>(e.igi));
  else std::snprintf(igi, sizeof(igi), "-");
  char b[160];
  const int n = std::snprintf(b, sizeof(b), "A %.0f %d %u %u %u %llu %llu %s", t_ms, card,
                              static_cast<unsigned>(ch), e.cca_ofdm, e.fa_ofdm,
                              static_cast<unsigned long long>(own),
                              static_cast<unsigned long long>(foreign), igi);
  put_(b, std::min(n, static_cast<int>(sizeof(b) - 1)));
}

}  // namespace maburgs
