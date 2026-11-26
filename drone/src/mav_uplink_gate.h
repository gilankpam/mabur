#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

#include "mav_component.h"

namespace mabur {

// The MAV thread's send decisions. Heartbeat: 1 Hz, only on a published
// session and never while a calibration sweep airs. Uplink: needs a
// published session or an update-only pair (nobody listens without one), a
// pair to tag under, queued packets and no sweep; paced to kMavUplinkMinMs
// while linked so the tunnel never crowds Telem or the OSD stream, unpaced
// on the update pair alone (no video to protect). Exit: once the update
// server asks to stop, wait for the queued replies to go, but at most
// kOtaExitGraceMs from the first request. The caller packs and sends.
class MavUplinkGate {
 public:
  static constexpr uint64_t kOtaExitGraceMs = 1000;

  bool heartbeat_due(uint64_t now, bool linked, bool sweeping) {
    if (!(linked && !sweeping && now - last_hb_ >= 1000)) return false;
    last_hb_ = now;
    return true;
  }
  bool uplink_due(uint64_t now, bool linked, bool update_open, bool have_pair, bool sweeping,
                  size_t outbound) {
    const uint64_t pace_ms = linked ? kMavUplinkMinMs : 0;
    if (!((linked || update_open) && have_pair && !sweeping && outbound > 0 &&
          now - last_up_ >= pace_ms))
      return false;
    last_up_ = now;
    return true;
  }
  bool exit_now(uint64_t now, bool exit_requested, size_t outbound) {
    if (!exit_requested) return false;
    if (!exit_seen_) exit_seen_ = now;
    return outbound == 0 || now - *exit_seen_ >= kOtaExitGraceMs;
  }

 private:
  uint64_t last_hb_ = 0, last_up_ = 0;
  std::optional<uint64_t> exit_seen_;
};

}  // namespace mabur
