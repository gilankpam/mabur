#pragma once

// The one include every mabur translation unit uses for MAVLink. The
// vendored tree under third_party/mavlink/ (see its VERSION file for the
// pymavlink version and the regenerate command) is mavgen's C output:
// generated headers whose pack/parse helpers are `static inline` with
// per-channel state held in function-local statics, keyed by channel
// index. That state is per translation unit, not
// per process, so a process that calls these helpers from more than one TU
// gets a separate copy of the channel state in each TU and parsing breaks
// across the split. Every process that speaks MAVLink therefore keeps ALL
// of its mavlink_msg_*_pack / mavlink_parse_char calls in exactly one TU
// (drone/src/mav_component.cpp, gs/src/mav_tunnel.cpp) and reaches the
// other TUs only through ordinary function calls, never by including this
// header a second time with different pack/parse call sites.
//
// The GS has two such TUs. gs/src/mav_tunnel.cpp parses on its own
// channels (kChanGsRx, kChanGsLocal below) because it needs to split an
// unbounded byte stream one byte at a time, carrying parse state across
// calls. gs/src/param_client.cpp packs with the plain `_pack` helpers (not
// `_pack_chan`, so it never touches kChanGsRx/kChanGsLocal either) and
// parses with the stateless mavlink_frame_char_buffer over its own
// mavlink_message_t/mavlink_status_t, fresh every call -- it is always
// handed one whole packet already split by the tunnel, so it never needs
// channel state to carry a parse across calls. Neither shares the other's
// channel state.
//
// The vendored headers are MIT-licensed generated output (see
// third_party/mavlink/LICENSE).

#include <cstdint>

#define MAVLINK_COMM_NUM_BUFFERS 4

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

// The mabur dialect is a superset of common: every common message still
// packs and decodes, plus the calibration messages (60000-60005).
#include <mavlink/mabur/mavlink.h>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace mabur::mav {

constexpr uint8_t kSysId = 1;
constexpr uint8_t kCompId = MAV_COMP_ID_ONBOARD_COMPUTER;

// One channel per owning TU's direction, so the drone and GS tunnels never
// share parser state with each other or with a local loopback user.
constexpr uint8_t kChanDroneRx = MAVLINK_COMM_0;
constexpr uint8_t kChanDroneUart = MAVLINK_COMM_1;
constexpr uint8_t kChanGsRx = MAVLINK_COMM_2;
constexpr uint8_t kChanGsLocal = MAVLINK_COMM_3;

}  // namespace mabur::mav
