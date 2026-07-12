#include "mabur/frag_reassembler.h"

namespace mabur {

FragReassembler::FragReassembler(size_t max_pending) : max_pending_(max_pending) {}

std::vector<FragCompleted> FragReassembler::add(const uint8_t* pkt, size_t len) {
  if (len < 4) return {};
  const uint16_t seq = static_cast<uint16_t>(pkt[0] | (pkt[1] << 8));
  const int idx = pkt[2], count = pkt[3];
  if (count == 0) return {};

  auto it = pending_.find(seq);
  if (it == pending_.end()) {
    it = pending_.emplace(seq, Entry{}).first;
    it->second.order = order_counter_++;
  }
  Entry& e = it->second;
  e.count = count;
  e.chunks.emplace(idx, std::vector<uint8_t>(pkt + 4, pkt + len));

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
