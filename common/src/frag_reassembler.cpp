#include "mabur/frag_reassembler.h"

namespace mabur {

FragReassembler::FragReassembler(size_t max_pending, uint64_t max_age_ms, bool wide)
    : max_pending_(max_pending), max_age_ms_(max_age_ms), wide_(wide) {}

void FragReassembler::sweep_expired(uint64_t now_ms) {
  if (max_age_ms_ == 0 || now_ms == 0) return;
  if (now_ms < last_sweep_ms_ + 50) return;  // throttle the O(pending) scan
  last_sweep_ms_ = now_ms;
  for (auto it = pending_.begin(); it != pending_.end();) {
    // now_ms > guard: multi-card body stamps interleave slightly out of
    // order; a younger-than-clock entry must read as age 0, not underflow.
    if (now_ms > it->second.t_ms && now_ms - it->second.t_ms > max_age_ms_) {
      it = pending_.erase(it);
      ++evicted_;
    } else {
      ++it;
    }
  }
}

std::vector<FragCompleted> FragReassembler::add(const uint8_t* pkt, size_t len,
                                                uint64_t now_ms) {
  sweep_expired(now_ms);
  const size_t hdr_len = wide_ ? 6 : 4;
  if (len < hdr_len) return {};
  const uint16_t seq = static_cast<uint16_t>(pkt[0] | (pkt[1] << 8));
  int idx, count;
  if (wide_) {
    idx = static_cast<int>(pkt[2] | (pkt[3] << 8));
    count = static_cast<int>(pkt[4] | (pkt[5] << 8));
  } else {
    idx = pkt[2];
    count = pkt[3];
  }
  if (count == 0) return {};

  auto it = pending_.find(seq);
  if (it == pending_.end()) {
    it = pending_.emplace(seq, Entry{}).first;
    it->second.order = order_counter_++;
    it->second.t_ms = now_ms;
  }
  Entry& e = it->second;
  e.count = count;
  e.chunks.emplace(idx, std::vector<uint8_t>(pkt + hdr_len, pkt + len));

  if (static_cast<int>(e.chunks.size()) < e.count) {
    if (pending_.size() > max_pending_) {  // evict oldest incomplete entry
      auto oldest = pending_.begin();
      for (auto j = pending_.begin(); j != pending_.end(); ++j)
        if (j->second.order < oldest->second.order) oldest = j;
      pending_.erase(oldest);
      ++evicted_;
    }
    return {};
  }
  // size == count: verify the indices are exactly 0..count-1 (corruption can
  // produce out-of-range idx); otherwise this entry can never complete.
  FragCompleted done;
  done.seq = seq;
  for (int i = 0; i < e.count; ++i) {
    auto c = e.chunks.find(i);
    if (c == e.chunks.end()) {
      pending_.erase(it);
      ++evicted_;
      return {};
    }
    done.pkt.insert(done.pkt.end(), c->second.begin(), c->second.end());
  }
  pending_.erase(it);
  ++completed_;
  std::vector<FragCompleted> out;
  out.push_back(std::move(done));
  return out;
}

}  // namespace mabur
