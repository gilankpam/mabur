#include "card_scan.h"

#include <algorithm>
#include <limits>
#include <tuple>

namespace maburgs {
namespace {

std::vector<ScannedCard> normalize(std::vector<ScannedCard> cards,
                                   int max_cards) {
  std::sort(cards.begin(), cards.end(), [](const ScannedCard& a,
                                           const ScannedCard& b) {
    return std::tie(a.bus, a.port_path) < std::tie(b.bus, b.port_path);
  });
  if (max_cards > 0 && cards.size() > static_cast<size_t>(max_cards))
    cards.resize(static_cast<size_t>(max_cards));
  return cards;
}

}  // namespace

std::vector<ScannedCard> scan_until_settled(const ScanPolicy& policy,
                                            const Enumerate& enumerate,
                                            const NowMs& now_ms,
                                            const SleepMs& sleep_ms) {
  const uint64_t started = now_ms();
  size_t last_size = std::numeric_limits<size_t>::max();
  uint64_t changed_at = started;
  for (;;) {
    std::vector<ScannedCard> cur = normalize(enumerate(), policy.max_cards);
    const uint64_t t = now_ms();
    if (cur.size() != last_size) {
      last_size = cur.size();
      changed_at = t;
    }
    if (!cur.empty() &&
        t - changed_at >= static_cast<uint64_t>(policy.settle_ms))
      return cur;
    if (t - started >= static_cast<uint64_t>(policy.timeout_ms)) return cur;
    sleep_ms(policy.poll_ms);
  }
}

}  // namespace maburgs
