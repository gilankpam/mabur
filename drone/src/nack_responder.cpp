#include "nack_responder.h"

#include <algorithm>
#include <utility>

#include "air_rate.h"

namespace mabur {

NackResponder::NackResponder(size_t env_len, int base_blocks_per_body)
    : env_len_(env_len),
      packer_(static_cast<int>(env_len), base_blocks_per_body, static_cast<uint8_t>(0 | kSbiRetxMark)),
      env_(env_len) {}

NackResponder::Tally NackResponder::answer(const rc::Nack& n, const AppliedOp* op, const AirClockCfg& air,
                                           const NackDroneCfg& cfg, uint64_t now_us, const RetxRing& ring,
                                           std::vector<std::vector<uint8_t>>* push_order) {
  Tally t;
  const double sym_per_s =
      op ? delivered_mbps(op->ladder[0], air) * 1e6 / 8.0 / static_cast<double>(env_len_) : 0.0;
  if (op) bucket_.set_depth(TokenBucket::depth_for(sym_per_s, cfg.burst_ms));
  bucket_.refill(now_us, sym_per_s * cfg.air_pct / 100.0);
  const bool twice = (n.flags & rc::kNackFlagRepeat) != 0;
  const double cost = twice ? 2.0 : 1.0;
  std::vector<std::vector<uint8_t>>& out = *push_order;
  out.clear();
  for (uint8_t i = 0; i < n.n; ++i) {
    for (int b = 0; b < 32; ++b) {
      if (!(n.e[i].bitmap & (1u << b))) continue;
      const uint32_t seq = n.e[i].first_seq + static_cast<uint32_t>(b);
      if (!ring.get(seq, env_.data())) { ++t.miss; continue; }
      if (!bucket_.take(cost)) { ++t.refused; continue; }
      ++t.syms;
      auto body = packer_.add_one(env_.data(), env_.size());
      if (!body.empty()) out.push_back(std::move(body));
    }
  }
  auto tail = packer_.flush_one();
  if (!tail.empty()) out.push_back(std::move(tail));
  std::reverse(out.begin(), out.end());
  if (twice) {
    // The second pass copies the first; reserve first so push_back never
    // reallocates under the element it copies.
    const size_t k = out.size();
    out.reserve(2 * k);
    for (size_t j = 0; j < k; ++j) out.push_back(out[j]);
  }
  return t;
}

}  // namespace mabur
