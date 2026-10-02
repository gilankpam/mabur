#include "relay_ring_transport.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/em_asm.h>
#include <emscripten/threading.h>

#include <climits>
#include <cstdlib>
#include <cstring>

#include "relay_ring.h"

namespace webgs {
namespace {
class RingTransport final : public maburgs::RelayTransport {
 public:
  RingTransport() {
    base_ = static_cast<uint8_t*>(std::aligned_alloc(8, kRingBytes));
    std::memset(base_, 0, kRingBytes);
    rx_ = rx_ring(base_);
    tx_ = tx_ring(base_);
    // The page hands (the shared heap, ptr) to its relay Worker.
    MAIN_THREAD_ASYNC_EM_ASM({ if (Module['onRelayRing']) Module['onRelayRing'](HEAPU8.buffer, $0); }, base_);
  }
  ~RingTransport() override { close(); }   // base_ intentionally leaked: the Worker may still touch it
  bool send(const uint8_t* p, size_t n) override {
    if (!ring_write(tx_, p, static_cast<uint32_t>(n))) return false;
    emscripten_futex_wake(ring_word(base_, kTxHead), INT_MAX);
    return true;
  }
  int recv(uint8_t* buf, size_t cap, int timeout_ms) override {
    for (int pass = 0; pass < 2; ++pass) {
      // Load the futex's expected value BEFORE ring_read, not after: a
      // Worker write+notify landing between an empty ring_read and the old
      // post-read load was a lost wakeup, stalling recv() for the full
      // timeout (review round 1, defect 2).
      const uint32_t seen = ring_load(ring_word(base_, kRxHead));
      if (ring_read(rx_, scratch_)) {
        const size_t n = scratch_.size() < cap ? scratch_.size() : cap;
        std::memcpy(buf, scratch_.data(), n);
        return static_cast<int>(n);
      }
      const uint32_t st = ring_load(ring_word(base_, kState));
      if (st == kClosed || st == kStopReq) return -1;
      if (pass == 0) emscripten_futex_wait(ring_word(base_, kRxHead), seen, static_cast<double>(timeout_ms));
    }
    return 0;
  }
  void close() override {
    ring_store(ring_word(base_, kState), kStopReq);
    emscripten_futex_wake(ring_word(base_, kTxHead), INT_MAX);   // the Worker's waitAsync
    emscripten_futex_wake(ring_word(base_, kRxHead), INT_MAX);   // our own recv()
    emscripten_futex_wake(ring_word(base_, kState), INT_MAX);    // a Worker still waiting in CONNECTING
  }
  uint64_t rx_drops() const override { return ring_load(ring_word(base_, kRxDrops)); }
  uint64_t tx_drops() const override { return ring_load(ring_word(base_, kTxDrops)); }

 private:
  uint8_t* base_;
  Ring rx_{}, tx_{};
  std::vector<uint8_t> scratch_;
};
}  // namespace

std::unique_ptr<maburgs::RelayTransport> open_ring_transport() { return std::make_unique<RingTransport>(); }
}  // namespace webgs
#endif
