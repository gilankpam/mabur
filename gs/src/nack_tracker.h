#pragma once
// SPIKE 2026-10-05 (fec-nack): software selective-repeat on top of the
// sliding-window FEC. Watches each video layer's erasure set (SwDecoder::
// missing_sources), and once a seq has been missing for settle_ms asks the
// drone to re-send it (T_NACK), repeating every repeat_ms up to max_tries.
// Books what happened to every requested seq: filled by a direct source
// copy (the retransmit, or a late original), wasted (a repair recovered it
// first), or abandoned (fell off the decoder's floor unknown).
//
// Core-thread-only, like the decoder it reads. Throwaway: measurement rig,
// not a design.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "mabur/rc_proto.h"
#include "mabur/sw_decoder.h"

namespace maburgs {

struct NackCfg {
  bool enable = false;
  int settle_ms = 0;     // missing this long before the first request
  int repeat_ms = 16;    // re-request cadence while still missing
  int max_tries = 2;     // requests per seq, total
  int lookback = 256;    // erasure window behind newest, symbols (< horizon)
  bool slotted = false;  // true: ride the RcfSlotter (burst-end); false: send now
};

struct NackStats {
  uint64_t sent = 0, repeats = 0, syms_requested = 0;
  uint64_t filled = 0, wasted = 0, abandoned = 0;
  // fill latency, ms from the FIRST request to the direct copy arriving
  uint64_t fill_sum_ms = 0, fill_max_ms = 0;
  std::vector<uint32_t> fill_ms;  // every sample (bench run: small)
  // Seqs that showed up by themselves (direct copy, never requested): how
  // long they had been "missing" -- the natural reorder/second-card window
  // settle_ms must exceed.
  uint64_t late_n = 0;
  std::vector<uint32_t> late_ms;
};

class NackTracker {
 public:
  using MissingFn = std::function<std::vector<uint32_t>(int sid)>;
  using StateFn = std::function<mabur::SwDecoder::SourceState(int sid, uint32_t seq)>;

  explicit NackTracker(NackCfg cfg) : cfg_(cfg) {}

  // One poll: refresh the erasure view, resolve finished entries, and
  // build at most one T_NACK worth of due requests. Returns the frame body
  // fields (unpacked) or nullopt.
  std::optional<mabur::rc::Nack> poll(uint64_t now_ms, const MissingFn& missing,
                                      const StateFn& state) {
    if (!cfg_.enable) return std::nullopt;
    // 1. resolve: entries no longer missing
    for (auto it = entries_.begin(); it != entries_.end();) {
      const int sid = it->first.first;
      const uint32_t seq = it->first.second;
      const auto st = state(sid, seq);
      bool done = true;
      using S = mabur::SwDecoder::SourceState;
      if (st == S::kUnknown) {
        done = false;
      } else if (st == S::kDirect) {
        if (it->second.tries > 0) {
          ++stats_.filled;
          const uint64_t lat = now_ms - it->second.first_sent_ms;
          stats_.fill_sum_ms += lat;
          if (lat > stats_.fill_max_ms) stats_.fill_max_ms = lat;
          stats_.fill_ms.push_back(static_cast<uint32_t>(lat));
        } else {
          ++stats_.late_n;
          stats_.late_ms.push_back(static_cast<uint32_t>(now_ms - it->second.first_missing_ms));
        }
      } else if (st == S::kRecovered) {
        if (it->second.tries > 0) ++stats_.wasted;
      } else {  // below floor
        if (it->second.tries > 0) ++stats_.abandoned;
      }
      if (done) it = entries_.erase(it); else ++it;
    }
    // 2. admit new erasures
    for (int sid = 0; sid < 2; ++sid)
      for (uint32_t seq : missing(sid)) {
        auto key = std::make_pair(sid, seq);
        if (!entries_.count(key)) entries_[key] = Entry{now_ms, 0, 0, 0};
      }
    // 3. collect due seqs
    std::vector<std::pair<int, uint32_t>> due;
    for (auto& [key, e] : entries_) {
      if (e.tries >= cfg_.max_tries) continue;
      const bool first = e.tries == 0;
      const uint64_t since = first ? e.first_missing_ms : e.last_sent_ms;
      const uint64_t wait = first ? static_cast<uint64_t>(cfg_.settle_ms)
                                  : static_cast<uint64_t>(cfg_.repeat_ms);
      if (now_ms >= since + wait) due.push_back(key);
    }
    if (due.empty()) return std::nullopt;
    std::sort(due.begin(), due.end());
    // 4. pack runs of 32 into entries (max kMaxNackEntries per frame)
    //
    // Task 1 (fec-nack): the final wire moved `sid` from NackEntry to the
    // Nack frame as a whole (one sid per T_NACK, base layer only --
    // global-constraints.md), so this spike tracker -- which still admits
    // both sids and used to tag each entry -- now groups by sid locally
    // and stamps the FIRST group's sid onto the frame; a mixed-sid `due`
    // set loses the second sid's entries from this frame (they stay
    // outstanding and go out next poll). Deleted + replaced by
    // common/include/mabur/nack_tracker.h in Task 6.
    mabur::rc::Nack n;
    n.counter = ++seq32_;
    size_t i = 0;
    bool any_repeat = false;
    bool sid_set = false;
    while (i < due.size() && n.n < mabur::rc::kMaxNackEntries) {
      const int cur_sid = due[i].first;
      if (!sid_set) { n.sid = static_cast<uint8_t>(cur_sid); sid_set = true; }
      if (cur_sid != n.sid) { ++i; continue; }  // defer to a later frame
      mabur::rc::NackEntry en;
      en.first_seq = due[i].second;
      en.bitmap = 0;
      while (i < due.size() && due[i].first == cur_sid &&
             due[i].second - en.first_seq < 32) {
        en.bitmap |= 1u << (due[i].second - en.first_seq);
        Entry& e = entries_[due[i]];
        if (e.tries == 0) e.first_sent_ms = now_ms; else any_repeat = true;
        ++e.tries;
        e.last_sent_ms = now_ms;
        ++stats_.syms_requested;
        ++i;
      }
      n.e[n.n++] = en;
    }
    if (any_repeat) n.flags |= mabur::rc::kNackFlagRepeat;
    ++stats_.sent;
    if (any_repeat) ++stats_.repeats;
    return n;
  }

  const NackStats& stats() const { return stats_; }
  size_t outstanding() const { return entries_.size(); }

  // 5 s stderr line, like the other core-thread gauges.
  void report(uint64_t now_ms, FILE* f) {
    if (last_report_ms_ == 0) last_report_ms_ = now_ms;
    if (now_ms - last_report_ms_ < 5000) return;
    last_report_ms_ = now_ms;
    const auto& s = stats_;
    std::vector<uint32_t> v(s.fill_ms);
    std::sort(v.begin(), v.end());
    auto pct = [&](double p) -> uint32_t {
      if (v.empty()) return 0;
      return v[std::min(v.size() - 1, static_cast<size_t>(p * v.size()))];
    };
    std::vector<uint32_t> l(s.late_ms);
    std::sort(l.begin(), l.end());
    auto lpct = [&](double p) -> uint32_t {
      if (l.empty()) return 0;
      return l[std::min(l.size() - 1, static_cast<size_t>(p * l.size()))];
    };
    std::fprintf(f,
                 "maburgs nack: sent=%llu repeats=%llu syms=%llu filled=%llu wasted=%llu "
                 "abandoned=%llu outstanding=%zu fill_ms p50=%u p90=%u max=%llu "
                 "late_n=%llu late_ms p50=%u p90=%u p99=%u max=%u\n",
                 (unsigned long long)s.sent, (unsigned long long)s.repeats,
                 (unsigned long long)s.syms_requested, (unsigned long long)s.filled,
                 (unsigned long long)s.wasted, (unsigned long long)s.abandoned,
                 entries_.size(), pct(0.5), pct(0.9), (unsigned long long)s.fill_max_ms,
                 (unsigned long long)s.late_n, lpct(0.5), lpct(0.9), lpct(0.99),
                 l.empty() ? 0u : l.back());
  }

 private:
  struct Entry {
    uint64_t first_missing_ms = 0;
    uint64_t first_sent_ms = 0;
    uint64_t last_sent_ms = 0;
    int tries = 0;
  };
  NackCfg cfg_;
  std::map<std::pair<int, uint32_t>, Entry> entries_;
  NackStats stats_;
  uint32_t seq32_ = 0;
  uint64_t last_report_ms_ = 0;
};

}  // namespace maburgs
