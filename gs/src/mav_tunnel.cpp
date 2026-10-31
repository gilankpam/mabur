#include "mav_tunnel.h"

#include <utility>

#include "mabur/mav.h"

namespace maburgs {

namespace {

bool idle(uint8_t chan) {
  return mavlink_get_channel_status(chan)->parse_state <= MAVLINK_PARSE_STATE_IDLE;
}

// Feeds bytes through `chan`'s parser while keeping each packet's own wire
// bytes in `acc` (from its start byte on), so a packet is forwarded exactly
// as it arrived. Calls on_packet(acc) for every MAVLink 2 packet whose CRC
// checks and on_bad() for a bad CRC, a v1 packet or a header the parser
// rejects mid-packet.
template <class OnPacket, class OnBad>
void split(uint8_t chan, std::vector<uint8_t>& acc, const uint8_t* p, size_t n,
           OnPacket&& on_packet, OnBad&& on_bad) {
  for (size_t i = 0; i < n; ++i) {
    const bool was_idle = idle(chan);
    if (was_idle) acc.clear();
    acc.push_back(p[i]);
    const uint8_t r = mavlink_frame_char(chan, p[i], nullptr, nullptr);
    if (r == MAVLINK_FRAMING_OK) {
      if (acc[0] == MAVLINK_STX) on_packet(acc);
      else on_bad();
    } else if (r != MAVLINK_FRAMING_INCOMPLETE) {
      on_bad();
    } else if (!was_idle && idle(chan)) {
      on_bad();
    }
  }
}

}  // namespace

MavTunnel::MavTunnel(mabur::LinkKey key, SendFn send, DeliverFn deliver, SendFn cal_send)
    : key_(key), send_(std::move(send)), deliver_(std::move(deliver)),
      cal_send_(std::move(cal_send)) {
  mavlink_reset_channel_status(mabur::mav::kChanGsLocal);
  mavlink_reset_channel_status(mabur::mav::kChanGsRx);
}

void MavTunnel::set_session(const mabur::rc::TagCtx& ctx) {
  if (ctx.vtx_nonce != 0 && ctx.vtx_nonce != vtx_seen_) {
    vtx_seen_ = ctx.vtx_nonce;
    counter_ = 0;
    have_rx_ = false;   // a restarted drone starts its own counter over
  }
  ctx_ = ctx;
}

void MavTunnel::push_outbound(const uint8_t* bytes, size_t n) {
  split(
      mabur::mav::kChanGsLocal, out_acc_, bytes, n,
      [&](const std::vector<uint8_t>& pkt) {
        if (ctx_.vtx_nonce == 0) {
          ++stats_.drop_unlinked;
        } else if (pkt.size() > mabur::rc::kMavMaxPayload) {
          ++stats_.drop_oversize;
        } else {
          queue_.push_back(pkt);
        }
      },
      [&] { ++stats_.drop_bad_packet; });
}

void MavTunnel::push_cal(const std::vector<uint8_t>& packet, const mabur::rc::TagCtx& run_ctx) {
  if (packet.size() > mabur::rc::kMavMaxPayload) {
    ++stats_.drop_cal_oversize;
    return;
  }
  cal_queue_.emplace_back(packet, run_ctx);
}

int MavTunnel::flush() {
  int frames = 0;
  while (!cal_queue_.empty()) {
    const mabur::rc::TagCtx run_ctx = cal_queue_.front().second;
    if (run_ctx.vtx_nonce != cal_vtx_seen_) {
      cal_vtx_seen_ = run_ctx.vtx_nonce;
      cal_counter_ = 0;
    }
    mabur::rc::Mav m;
    while (!cal_queue_.empty() && cal_queue_.front().second.vrx_nonce == run_ctx.vrx_nonce &&
           cal_queue_.front().second.vtx_nonce == run_ctx.vtx_nonce &&
           m.payload.size() + cal_queue_.front().first.size() <= mabur::rc::kMavMaxPayload) {
      const auto& pkt = cal_queue_.front().first;
      m.payload.insert(m.payload.end(), pkt.begin(), pkt.end());
      cal_queue_.pop_front();
    }
    // push_cal already rejects an oversized packet, but never spin on one
    // here either: a front packet that still doesn't fit an empty frame
    // is dropped and counted instead of sent (and counted) as an empty
    // frame forever.
    if (m.payload.empty()) {
      ++stats_.drop_cal_oversize;
      cal_queue_.pop_front();
      continue;
    }
    m.counter = ++cal_counter_;
    mabur::rc::TagCtx tag = run_ctx;
    tag.seq32 = cal_counter_;
    (cal_send_ ? cal_send_ : send_)(mabur::rc::pack_mav(m, true, key_, tag));
    ++stats_.tx_cal_frames;
    ++frames;
  }
  if (ctx_.vtx_nonce == 0) {
    stats_.drop_unlinked += queue_.size();
    queue_.clear();
    return frames;
  }
  while (!queue_.empty()) {
    mabur::rc::Mav m;
    size_t packets = 0;
    while (!queue_.empty() &&
           m.payload.size() + queue_.front().size() <= mabur::rc::kMavMaxPayload) {
      m.payload.insert(m.payload.end(), queue_.front().begin(), queue_.front().end());
      queue_.pop_front();
      ++packets;
    }
    m.counter = ++counter_;
    mabur::rc::TagCtx tag = ctx_;
    tag.seq32 = counter_;
    send_(mabur::rc::pack_mav(m, true, key_, tag));
    ++stats_.tx_frames;
    stats_.tx_packets += packets;
    ++frames;
  }
  return frames;
}

void MavTunnel::on_rc_frame(const uint8_t* body, size_t len) {
  // The GS's other card hears the GS's own tagged T_MAV. Checked first: an
  // untagged frame is at least 6 bytes short of the tagged structural
  // length even with a trailing 4-byte FCS, so it never parses as tagged,
  // while a tagged frame always passes its own CRC -- and could, once in
  // 2^16, also pass the untagged CRC on its tag bytes and be delivered.
  if (mabur::rc::parse_mav(body, len, /*tagged=*/true)) {
    ++stats_.rx_uplink_heard;
    return;
  }
  auto m = mabur::rc::parse_mav(body, len, false);
  if (!m) {
    ++stats_.drop_bad_packet;
    return;
  }
  // A two-card GS hands the rc sink each card's copy of a drone frame; the
  // drone's per-process counter tells the second copy from a new frame.
  if (have_rx_ && m->counter == rx_last_) {
    ++stats_.rx_dup;
    return;
  }
  have_rx_ = true;
  rx_last_ = m->counter;
  ++stats_.rx_frames;
  const uint8_t chan = mabur::mav::kChanGsRx;
  split(
      chan, in_acc_, m->payload.data(), m->payload.size(),
      [&](const std::vector<uint8_t>& pkt) {
        ++stats_.rx_packets;
        deliver_(pkt.data(), pkt.size());
      },
      [&] { ++stats_.drop_bad_packet; });
  if (!idle(chan)) {
    ++stats_.drop_bad_packet;
    mavlink_reset_channel_status(chan);
  }
  in_acc_.clear();
}

}  // namespace maburgs
