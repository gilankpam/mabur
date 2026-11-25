#pragma once
#include <cstdint>

namespace mabur {

// Time a body spent in the TX queue, in ms, for the wire's q_ms field and
// the window-max gauge. enqueued_ms == 0 means the producer never stamped it
// (dry-run/test paths may push bare bodies): report 0 rather than a bogus
// wait. A pop not after the push also reads 0. Clamped to the u16 the wire
// carries.
inline uint16_t q_wait_ms(uint32_t enqueued_ms, uint32_t pop_ms) {
  const uint32_t w = (enqueued_ms && pop_ms > enqueued_ms) ? pop_ms - enqueued_ms : 0;
  return static_cast<uint16_t>(w > 65535 ? 65535 : w);
}

}  // namespace mabur
