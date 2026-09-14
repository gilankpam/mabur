#include "scan_log.h"

#include <algorithm>
#include <cstdio>

namespace maburgs {

ScanLog::ScanLog(LogWriter& w, const std::string& dir, const std::string& header_info)
    : w_(w), s_(w.open(dir, "scan.log", "scanlog 2 " + header_info)) {}

void ScanLog::put_(const char* b, int n) {
  if (s_ == LogWriter::kBadStream || n <= 0) return;
  w_.line(s_, b, static_cast<size_t>(n));
}

void ScanLog::caps(double t_ms, int card, const CardCaps& c) {
  // A C record is positional and space-separated, so no field may ever be
  // empty: an empty %s would silently shift every column after it and the
  // record would still parse. A card whose GetAdapterCaps came back
  // unsupported carries no chip/gen at all -- print "?" for both (the
  // numeric fields are already zeroed by CardCaps' defaults, which is a
  // truthful "nothing was read").
  const char* chip = (c.valid && !c.chip.empty()) ? c.chip.c_str() : "?";
  const char* gen = (c.valid && !c.gen.empty()) ? c.gen.c_str() : "?";
  char b[256];
  const int n = std::snprintf(b, sizeof(b), "C %.0f %d %s %s %dx%d %x %u-%u %d %d %d %d %d",
                              t_ms, card, chip, gen, c.tx_chains,
                              c.rx_chains, static_cast<unsigned>(c.bw_mask),
                              static_cast<unsigned>(c.tune5g_lo),
                              static_cast<unsigned>(c.tune5g_hi),
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
      b, sizeof(b), "D %.0f %d %u %llu %lld %u %u %u %u %s %s %x %d %lld %lld %lld", t_ms, card,
      static_cast<unsigned>(s.def.primary), static_cast<unsigned long long>(s.round),
      static_cast<long long>(s.observe_ms), s.cca_ofdm, s.fa_ofdm, s.dvr_frames,
      s.frames - s.dvr_frames, igi, floor, static_cast<unsigned>(s.flags), d.in_session ? 1 : 0,
      static_cast<long long>(d.to_us), static_cast<long long>(d.read_us),
      static_cast<long long>(d.back_us));
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

void ScanLog::verdict(double t_ms, const VerdictOut& o, const std::vector<VerdictCardIn>& cards,
                      const VerdictLinkIn& link) {
  if (s_ == LogWriter::kBadStream) return;
  char rung[8];
  if (o.ref_rung < 0) std::snprintf(rung, sizeof(rung), "-");
  else std::snprintf(rung, sizeof(rung), "%d", o.ref_rung);
  char head[128];
  const int hn = std::snprintf(head, sizeof(head), "V %.1f %s %02x %s %.1f %u", t_ms,
                               to_string(o.v), static_cast<unsigned>(o.evidence), rung,
                               link.pre_fec_loss * 100.0,
                               static_cast<unsigned>(link.recovered));
  std::string line(head, static_cast<size_t>(std::min(hn, static_cast<int>(sizeof(head) - 1))));
  for (size_t i = 0; i < cards.size(); ++i) {
    const VerdictCardIn& c = cards[i];
    char cb[160];
    const int cn = std::snprintf(cb, sizeof(cb), " %u %u %u %u %u %.1f %.1f %.1f",
                                 static_cast<unsigned>(i), c.foreign, c.fa, c.cca, c.crc_fail,
                                 c.rssi_dbm, c.snr_db, o.d_rssi_db);
    line.append(cb, static_cast<size_t>(std::min(cn, static_cast<int>(sizeof(cb) - 1))));
  }
  put_(line.c_str(), static_cast<int>(std::min(line.size(), LogWriter::kMaxLine - 1)));
}

void ScanLog::hop(const HopEvent& e) {
  char b[128];
  const int n = std::snprintf(b, sizeof(b), "H %.1f %s %u %u %u %.1f", e.t_ms, e.kind.c_str(),
                              static_cast<unsigned>(e.epoch), static_cast<unsigned>(e.target),
                              static_cast<unsigned>(e.score), e.elapsed_ms);
  put_(b, std::min(n, static_cast<int>(sizeof(b) - 1)));
}

}  // namespace maburgs
