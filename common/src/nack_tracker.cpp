#include "mabur/nack_tracker.h"

#include <algorithm>

namespace mabur {

using S = SwDecoder::SourceState;

NackTracker::NackTracker(NackCfg cfg) : cfg_(cfg), settle_ms_(cfg.settle_seed_ms) {}

// frame_wire edge: entries reset; the request counter is kept, because the
// drone's accept_nack_counter keeps the last counter for as long as the vtx
// nonce is unchanged -- a GS that drops to BEACONING after link_lost_ms and
// rejoins on the first video body (no DISC) is still under that nonce. Only
// a new vtx nonce restarts it (restart_counter(), gs/src/main.cpp). The
// settle estimate (a property of the link, not the session) is kept.
void NackTracker::clear() { entries_.clear(); }
const NackStats& NackTracker::stats() const { return stats_; }
int NackTracker::settle_ms() const { return settle_ms_; }

size_t NackTracker::outstanding() const {
  size_t n = 0;
  for (const auto& [s, e] : entries_)
    if (!e.dead && e.tries < cfg_.max_tries) ++n;
  return n;
}

NackWindow NackTracker::take_window() {
  NackWindow w = std::move(win_);
  win_ = NackWindow{};
  return w;
}

void NackTracker::note_late(uint64_t now_ms, uint32_t late_ms) {
  late_.emplace_back(now_ms, late_ms);
  while (!late_.empty() && now_ms - late_.front().first > static_cast<uint64_t>(cfg_.settle_window_ms))
    late_.pop_front();
  if (late_ms > win_.late_ms_max) win_.late_ms_max = late_ms;
  if (late_.size() < static_cast<size_t>(cfg_.settle_min_samples)) return;
  uint32_t mx = 0;
  for (const auto& [t, l] : late_) mx = std::max(mx, l);
  settle_ms_ = std::clamp(static_cast<int>(mx) + 2, cfg_.settle_min_ms, cfg_.settle_max_ms);
}

void NackTracker::resolve(uint64_t now_ms, const NackInputs& in) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    Entry& e = it->second;
    bool done = true;
    switch (in.state(it->first)) {
      case S::kUnknown:
        done = false;
        // Deadline: dead but kept until terminal, so missing() cannot
        // re-admit it.
        if (now_ms - e.first_missing_ms > in.gap_timeout_ms) {
          e.dead = true;
          if (e.tries > 0 && !e.deadline_counted) {
            ++stats_.dropped_deadline;
            e.deadline_counted = true;
          }
        }
        break;
      case S::kRetx:
        if (e.tries > 0) {
          ++stats_.filled;
          ++win_.filled;
          if (win_.fill_ms.size() < NackWindow::kMaxFillSamples)  // cap: stop appending
            win_.fill_ms.push_back(static_cast<uint32_t>(now_ms - e.first_sent_ms));
        }
        break;
      case S::kDirect:
        if (e.tries > 0)
          ++stats_.late_fill;
        else
          note_late(now_ms, static_cast<uint32_t>(now_ms - e.first_missing_ms));
        break;
      case S::kRecovered:
        if (e.tries > 0) ++stats_.wasted;
        break;
      case S::kBelowFloor:
        if (e.tries > 0 && !e.deadline_counted) ++stats_.dropped_deadline;
        break;
    }
    if (done)
      it = entries_.erase(it);
    else
      ++it;
  }
}

void NackTracker::admit(uint64_t now_ms, const NackInputs& in) {
  for (uint32_t s : in.missing())
    if (!entries_.count(s)) entries_[s] = Entry{now_ms, 0, 0, 0, false, false, false};
  if (auto tv = in.tail()) {
    if (now_ms >= tv->last_progress_ms + static_cast<uint64_t>(settle_ms_) &&
        tv->max_idx + 1 < tv->count) {
      const uint32_t n_tail = static_cast<uint32_t>(tv->count - tv->max_idx - 1);
      for (uint32_t k = 1; k <= n_tail; ++k) {
        const uint32_t s = tv->seq_at_max + k;
        // Only still-unknown seqs: a stale tail view (slot not advanced past
        // a seq that already resolved) must not re-admit it every poll.
        if (!entries_.count(s) && in.state(s) == S::kUnknown)
          entries_[s] = Entry{tv->last_progress_ms, 0, 0, 0, true, false, false};
      }
    }
  }
}

std::optional<rc::Nack> NackTracker::poll(uint64_t now_ms, const NackInputs& in) {
  if (!cfg_.enable) return std::nullopt;
  resolve(now_ms, in);
  admit(now_ms, in);  // admitted even while stopped: entries age and resolve normally
  std::vector<uint32_t> due;
  for (auto& [s, e] : entries_) {
    if (e.dead || e.tries >= cfg_.max_tries) continue;
    const uint64_t since = e.tries == 0 ? e.first_missing_ms : e.last_sent_ms;
    const uint64_t wait = e.tries == 0 ? static_cast<uint64_t>(settle_ms_)
                                       : static_cast<uint64_t>(cfg_.repeat_ms);
    if (now_ms < since + wait) continue;
    const uint64_t deadline = e.first_missing_ms + in.gap_timeout_ms;
    if (now_ms + static_cast<uint64_t>(cfg_.min_lead_ms) > deadline) {
      e.dead = true;  // the answer could not land in time: never asked (again)
      ++stats_.lead_skipped;
      continue;
    }
    due.push_back(s);
  }
  if (due.empty()) return std::nullopt;
  if (in.util() >= in.down_util) {
    // Stop rule: one suppressed per poll that would have sent; the due
    // entries are dead so there is no catch-up burst when util drops.
    ++stats_.suppressed;
    for (uint32_t s : due) entries_[s].dead = true;
    return std::nullopt;
  }
  rc::Nack n;
  n.counter = ++counter_;
  n.sid = 0;
  size_t i = 0;
  bool any_repeat = false;
  while (i < due.size() && n.n < rc::kMaxNackEntries) {
    rc::NackEntry en;
    en.first_seq = due[i];
    en.bitmap = 0;
    while (i < due.size() && due[i] - en.first_seq < 32) {
      en.bitmap |= 1u << (due[i] - en.first_seq);
      Entry& e = entries_[due[i]];
      if (e.tries == 0)
        e.first_sent_ms = now_ms;
      else
        any_repeat = true;
      ++e.tries;
      e.last_sent_ms = now_ms;
      ++stats_.syms_requested;
      if (e.from_tail && e.tries == 1) ++stats_.tail_requests;
      ++i;
    }
    n.e[n.n++] = en;
  }
  if (any_repeat) {
    n.flags |= rc::kNackFlagRepeat;
    ++stats_.repeats;
  }
  ++stats_.requests;
  return n;
}

}  // namespace mabur
