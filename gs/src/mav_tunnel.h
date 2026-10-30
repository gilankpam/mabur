#pragma once
// MavTunnel — MAVLink packets over the RC link's T_MAV control frame, GS
// side. Shared by maburgs and the web GS core: pure data handling, the
// caller owns the sockets and the radio.
//
// Outbound (local MAVLink peer -> drone): push_outbound takes bytes cut at
// any boundary, a parser splits them into whole MAVLink 2 packets and
// queues each packet's wire bytes unchanged (re-serialising through the
// library would trim a non-truncating sender's zero tail while keeping its
// old CRC, corrupting the packet). flush() packs the queue greedily into
// as few T_MAV frames as fit kMavMaxPayload -- a packet never spans two
// frames -- and tags each with the session ctx and the next counter
// (TagCtx.seq32 = counter). The drone accepts only a counter above the
// last one it took this session, so the counter restarts at 1 whenever the
// session's vtx_nonce changes (a drone restart or re-rendezvous), the rule
// NackTracker::restart_counter follows. While the session is unlinked
// (vtx_nonce 0) outbound packets are dropped and counted: a ground app's
// heartbeats before link-up are noise, not something to replay later.
//
// Inbound (drone -> local peer): on_rc_frame takes an untagged T_MAV body,
// splits its payload and delivers each whole packet's wire bytes. A frame
// carrying the same drone counter as the last one taken is the other
// card's copy and is dropped (rx_dup); the counter memory clears when the
// session's vtx_nonce changes. A tagged T_MAV is the GS's own uplink heard
// by its other card (rx_uplink_heard), never a reject. The RC
// CRC already vouches for the bytes; the MAVLink CRC is the second check
// and is also what rejects a message id whose CRC_EXTRA the common dialect
// does not know. A partial packet at the end of a frame is dropped and the
// parser reset, so it never bleeds into the next frame.
//
// MAVLink 2 only: a v1 packet in either direction is dropped and counted.
//
// mabur/mav.h's helpers keep per-translation-unit channel state, so
// mav_tunnel.cpp is the GS's one TU that parses MAVLink. It parses on
// kChanGsLocal (outbound) and kChanGsRx (inbound); the channel state is
// process-wide, so one MavTunnel per process.
//
// Cal lane (push_cal/flush): the GS calibration kit's own MAVLink messages
// (the common dialect's 60000-60005 ids), each sent in a frame of its own
// kind -- never mixed with a live packet. The drone tells a cal-only frame
// from a live one by content: a frame mixing the two kinds is handed whole
// to the MAVLink component, which does not know the cal ids and silently
// drops them, so mixing would lose the calibration message rather than
// merely miscount it. A run's pair is frozen at the caller (the
// calibration session), passed as run_ctx on every push_cal, and used to
// tag the frame as-is -- unlike the live lane, set_session plays no part,
// so cal frames go out whether or not the GS has a live session at all
// (every sweep outlasts the link's failsafe). The lane keeps its own
// counter, restarting at 1 only when a frame's run_ctx.vtx_nonce differs
// from the last cal frame actually sent: a second run on the same pair
// keeps counting, because the drone checks the cal counter against its own
// per-pair high-water mark and would read a restart as a replay. Tracking
// only the last run's pair is enough because only one calibration run is
// ever live at a time. flush() drains the cal queue first, so a sweep's
// control traffic is never stuck behind a backlog of live MAVLink.
//
// Single-threaded: every call comes from the owner's core loop.
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

#include "mabur/link_key.h"
#include "mabur/rc_proto.h"

namespace maburgs {

struct MavTunnelStats {
  uint64_t tx_frames = 0, tx_packets = 0, rx_frames = 0, rx_packets = 0,
           drop_unlinked = 0,     // outbound packets with no session to tag them
           drop_bad_packet = 0,   // bad MAVLink CRC, v1, a partial or a T_MAV body that fails parse
           drop_oversize = 0,     // an outbound packet larger than kMavMaxPayload
           rx_dup = 0,            // a drone frame repeating the last counter (the other card's copy)
           rx_uplink_heard = 0,   // the GS's own tagged T_MAV, heard by its other card
           tx_cal_frames = 0,     // cal-lane frames sent (never counted in tx_frames/tx_packets)
           drop_cal_oversize = 0; // a cal packet larger than kMavMaxPayload, never queued
};

class MavTunnel {
 public:
  using SendFn = std::function<void(std::vector<uint8_t> frame)>;   // a packed, tagged T_MAV body
  using DeliverFn = std::function<void(const uint8_t* mav_bytes, size_t n)>;  // one whole packet

  // `cal_send`, when given, takes the cal lane's frames and `send` only the
  // live lane's: the owner sends the two through different gates (a live
  // frame waits on the RCF slotter like an RCF; a calibration message goes
  // out at once). Without it both lanes go through `send`.
  MavTunnel(mabur::LinkKey key, SendFn send, DeliverFn deliver, SendFn cal_send = nullptr);

  // The session the next frames are tagged under; ctx.seq32 is ignored
  // (the tunnel's counter fills it). vtx_nonce 0 = not linked: outbound is
  // dropped. A change of vtx_nonce restarts the counter: the next frame is 1,
  // and forgets the last inbound counter.
  void set_session(const mabur::rc::TagCtx& ctx);
  // Bytes from a local MAVLink peer, cut at any boundary. Whole packets are
  // queued (or dropped and counted while unlinked).
  void push_outbound(const uint8_t* bytes, size_t n);
  // Queues one calibration packet's wire bytes, tagged under run_ctx (frozen
  // by the caller for the run). Never merged with a live packet at flush.
  // A packet larger than kMavMaxPayload is dropped and counted, the same
  // as push_outbound's oversize guard.
  void push_cal(const std::vector<uint8_t>& packet, const mabur::rc::TagCtx& run_ctx);
  // Packs queued cal packets (per run_ctx), then queued live packets, into
  // as few T_MAV frames as fit kMavMaxPayload each, and calls send for
  // each -- cal frames first. Returns frames sent.
  int flush();
  // A T_MAV frame body from the drone (the rc-sink hook). Splits it and
  // delivers each packet, unless it is a duplicate or the GS's own uplink.
  void on_rc_frame(const uint8_t* body, size_t len);

  const MavTunnelStats& stats() const { return stats_; }
  uint32_t counter() const { return counter_; }   // last counter used
  uint32_t cal_counter() const { return cal_counter_; }   // last cal-lane counter used

 private:
  mabur::LinkKey key_;
  SendFn send_;
  DeliverFn deliver_;
  SendFn cal_send_;
  mabur::rc::TagCtx ctx_{};
  uint32_t vtx_seen_ = 0;
  uint32_t counter_ = 0;
  bool have_rx_ = false;   // rx_last_ holds a counter from this session
  uint32_t rx_last_ = 0;   // the drone counter of the last inbound frame taken
  std::deque<std::vector<uint8_t>> queue_;
  uint32_t cal_vtx_seen_ = 0;   // the last cal frame's run_ctx.vtx_nonce, for the restart rule
  uint32_t cal_counter_ = 0;
  std::deque<std::pair<std::vector<uint8_t>, mabur::rc::TagCtx>> cal_queue_;
  std::vector<uint8_t> out_acc_;   // wire bytes of the outbound packet being parsed
  std::vector<uint8_t> in_acc_;    // same, inbound
  MavTunnelStats stats_;
};

}  // namespace maburgs
