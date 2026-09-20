#include "mabur/msp_status.h"

namespace mabur {

std::optional<bool> msp_status_armed(const MspMessage& m) {
  if (m.cmd != MSP_CMD_STATUS || m.payload.size() < 11) return std::nullopt;
  const uint32_t flags = static_cast<uint32_t>(m.payload[6]) |
                         (static_cast<uint32_t>(m.payload[7]) << 8) |
                         (static_cast<uint32_t>(m.payload[8]) << 16) |
                         (static_cast<uint32_t>(m.payload[9]) << 24);
  return (flags & 1u) != 0;
}

void msp_append_status_request(std::vector<uint8_t>& out) {
  msp_append_message(out, MSP_CMD_STATUS, nullptr, 0);
}

}  // namespace mabur
