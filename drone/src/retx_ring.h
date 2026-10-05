#pragma once
// SPIKE 2026-10-05 (fec-nack): per-layer ring of the last `cap` SOURCE
// envelopes the encoder sealed, keyed by wire seq, so a T_NACK can be
// answered from the RX thread while the hot thread keeps writing. One
// mutex; both sides touch it for microseconds.
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace mabur {

class RetxRing {
 public:
  explicit RetxRing(size_t cap_per_layer = 256) : cap_(cap_per_layer) {}

  void put(uint8_t sid, uint32_t seq, const uint8_t* env, size_t len) {
    if (sid >= 2) return;
    std::lock_guard<std::mutex> l(m_);
    Layer& L = layers_[sid];
    L.by_seq[seq].assign(env, env + len);
    L.order.push_back(seq);
    while (L.order.size() > cap_) {
      L.by_seq.erase(L.order.front());
      L.order.pop_front();
    }
  }

  std::optional<std::vector<uint8_t>> get(uint8_t sid, uint32_t seq) const {
    if (sid >= 2) return std::nullopt;
    std::lock_guard<std::mutex> l(m_);
    const Layer& L = layers_[sid];
    auto it = L.by_seq.find(seq);
    if (it == L.by_seq.end()) return std::nullopt;
    return it->second;
  }

 private:
  struct Layer {
    std::map<uint32_t, std::vector<uint8_t>> by_seq;
    std::deque<uint32_t> order;
  };
  size_t cap_;
  mutable std::mutex m_;
  Layer layers_[2];
};

}  // namespace mabur
