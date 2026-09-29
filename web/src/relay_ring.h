#pragma once
// SPSC byte ring between the WASM core and the page's relay Worker
// (web/ui/src/lib/relay_ring.js is the JS twin -- keep them byte-identical;
// both test files pin the same goldens). Header-only, no Emscripten APIs:
// waking/waiting is the caller's (relay_transport.cpp).
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

namespace webgs {

constexpr uint32_t kRingHdr = 32, kRxCap = 1u << 20, kTxCap = 1u << 16;
constexpr uint32_t kRingBytes = kRingHdr + kRxCap + kTxCap;
enum RingWord : int { kRxHead = 0, kRxTail, kTxHead, kTxTail, kState, kRxDrops, kTxDrops };
enum RingState : uint32_t { kConnecting = 0, kOpen = 1, kClosed = 2, kStopReq = 3 };
constexpr uint32_t kWrap = 0xFFFFFFFFu;

struct Ring {
  uint32_t* head;
  uint32_t* tail;
  uint8_t* data;
  uint32_t cap;   // power of two
  uint32_t* drops;
};

inline uint32_t* ring_word(uint8_t* base, int w) { return reinterpret_cast<uint32_t*>(base) + w; }
inline Ring rx_ring(uint8_t* base) {
  return {ring_word(base, kRxHead), ring_word(base, kRxTail), base + kRingHdr, kRxCap,
          ring_word(base, kRxDrops)};
}
inline Ring tx_ring(uint8_t* base) {
  return {ring_word(base, kTxHead), ring_word(base, kTxTail), base + kRingHdr + kRxCap, kTxCap,
          ring_word(base, kTxDrops)};
}

inline uint32_t ring_load(uint32_t* p) {
  return std::atomic_ref<uint32_t>(*p).load(std::memory_order_acquire);
}
inline void ring_store(uint32_t* p, uint32_t v) {
  std::atomic_ref<uint32_t>(*p).store(v, std::memory_order_release);
}

inline bool ring_write(const Ring& r, const uint8_t* p, uint32_t n) {
  const uint32_t need = 4 + ((n + 3) & ~3u);
  const uint32_t head = *r.head;   // producer-owned
  const uint32_t tail = ring_load(r.tail);
  const uint32_t pos = head & (r.cap - 1);
  const uint32_t room_end = r.cap - pos;
  const uint32_t total = room_end < need ? room_end + need : need;
  if (need > r.cap / 2 || total > r.cap - (head - tail)) {
    std::atomic_ref<uint32_t>(*r.drops).fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  uint32_t h = head, at = pos;
  if (room_end < need) {
    std::memcpy(r.data + pos, &kWrap, 4);
    h += room_end;
    at = 0;
  }
  std::memcpy(r.data + at, &n, 4);
  std::memcpy(r.data + at + 4, p, n);
  ring_store(r.head, h + need);
  return true;
}

inline bool ring_read(const Ring& r, std::vector<uint8_t>& out) {
  uint32_t tail = *r.tail;   // consumer-owned
  for (;;) {
    const uint32_t head = ring_load(r.head);
    if (tail == head) return false;
    const uint32_t pos = tail & (r.cap - 1);
    uint32_t len;
    std::memcpy(&len, r.data + pos, 4);
    if (len == kWrap) {
      tail += r.cap - pos;
      ring_store(r.tail, tail);
      continue;
    }
    out.assign(r.data + pos + 4, r.data + pos + 4 + len);
    ring_store(r.tail, tail + 4 + ((len + 3) & ~3u));
    return true;
  }
}

}  // namespace webgs
