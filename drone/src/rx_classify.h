// What the RX callback does with one frame body. A frame is ours when it
// parses at our RC version, or is a DISC / T_MAV of any version (frozen
// types: a DISC of another version is still answered, a T_MAV of another
// version may be the update lane, which the MAV thread decides). Ours is
// routed even when the CRC failed: the consumer verifies its own tag. But a
// corrupt frame's rssi/snr is no trustworthy sample, so only a CRC-clean one
// is `clean_own` (feeds the uplink EMAs and the own-frame count). Anything
// else counts as a foreign 802.11 frame only when CRC-clean: RC_MAGIC is two
// bytes, so ~1 in 65536 corrupt bodies match it by chance and must neither
// count as a neighbour nor print a version-mismatch scare. A foreign RC
// version is flagged on top. Header-inline: runs per received frame.
#pragma once
#include <cstddef>
#include <cstdint>
#include "mabur/rc_proto.h"

namespace mabur {

enum class RxRoute : uint8_t { Drop, Nack, Mav, Rc, Foreign };

struct RxVerdict {
  RxRoute route;
  bool clean_own;
  bool foreign_version;
};

inline RxVerdict classify_rx(const uint8_t* body, size_t len, bool crc_err) {
  const int rc_type = rc::frame_type(body, len);
  const int frozen = rc_type >= 0 ? rc_type : rc::frozen_type(body, len);
  if (rc_type >= 0 || frozen == rc::T_DISC || frozen == rc::T_MAV) {
    const RxRoute r = rc_type == rc::T_NACK ? RxRoute::Nack
                      : frozen == rc::T_MAV ? RxRoute::Mav
                                            : RxRoute::Rc;
    return {r, !crc_err, false};
  }
  if (!crc_err) return {RxRoute::Foreign, false, rc::is_foreign_rc_version(body, len)};
  return {RxRoute::Drop, false, false};
}

}  // namespace mabur
