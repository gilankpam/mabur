#include "mabur/cal_mav.h"

#include <algorithm>
#include <cstring>
#include <type_traits>

namespace mabur::cal {

namespace {

// Applied.reason is cut to fit the wire's char[50] (49 chars + NUL), NUL
// padded over the rest so a stale tail byte can never read back as part of
// a shorter message.
void pack_reason(const std::string& reason, char out[50]) {
  std::memset(out, 0, 50);
  const size_t n = reason.size() < 49 ? reason.size() : 49;
  std::memcpy(out, reason.data(), n);
}

}  // namespace

std::vector<uint8_t> encode(const CalMsg& m, uint8_t sysid, uint8_t compid, uint8_t chan) {
  mavlink_message_t msg;
  std::visit(
      [&](auto&& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, SweepCmd>) {
          mavlink_msg_mabur_cal_sweep_pack_chan(
              sysid, compid, chan, &msg, v.nonce, v.seq, static_cast<uint8_t>(v.kind), v.bw,
              v.frames_per_cell, v.settle_ms, v.gap_us, v.dwell_ms, v.idx_lo, v.idx_hi,
              v.mcs_mask);
        } else if constexpr (std::is_same_v<T, SweepAck>) {
          mavlink_msg_mabur_cal_ack_pack_chan(sysid, compid, chan, &msg, v.nonce, v.seq,
                                              static_cast<uint8_t>(v.result), v.start_in_ms,
                                              v.flags);
        } else if constexpr (std::is_same_v<T, SweepDone>) {
          mavlink_msg_mabur_cal_sweep_done_pack_chan(sysid, compid, chan, &msg, v.nonce, v.seq,
                                                     v.offered.data(), v.blocked_pct.data());
        } else if constexpr (std::is_same_v<T, ApplyCmd>) {
          const uint8_t has = (v.has_walls ? 1u : 0u) | (v.has_eff ? 2u : 0u);
          mavlink_msg_mabur_cal_apply_pack_chan(sysid, compid, chan, &msg, v.nonce, v.bw, has,
                                                v.walls.data(), v.legacy, v.eff_x1000.data());
        } else if constexpr (std::is_same_v<T, Applied>) {
          char reason[50];
          pack_reason(v.reason, reason);
          mavlink_msg_mabur_cal_applied_pack_chan(sysid, compid, chan, &msg, v.nonce, v.bw,
                                                  v.ok ? 1u : 0u, reason);
        } else if constexpr (std::is_same_v<T, Abort>) {
          mavlink_msg_mabur_cal_abort_pack_chan(sysid, compid, chan, &msg, v.nonce);
        }
      },
      m);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  return std::vector<uint8_t>(buf, buf + n);
}

bool is_cal_msgid(uint32_t msgid) { return msgid >= kCalMsgIdLo && msgid <= kCalMsgIdHi; }

std::optional<CalMsg> decode(const mavlink_message_t& msg) {
  if (!is_cal_msgid(msg.msgid)) return std::nullopt;
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_MABUR_CAL_SWEEP: {
      mavlink_mabur_cal_sweep_t d;
      mavlink_msg_mabur_cal_sweep_decode(&msg, &d);
      SweepCmd c;
      c.nonce = d.nonce;
      c.seq = d.seq;
      c.kind = static_cast<Kind>(d.kind);
      c.bw = d.bw;
      c.frames_per_cell = d.frames_per_cell;
      c.settle_ms = d.settle_ms;
      c.gap_us = d.gap_us;
      c.dwell_ms = d.dwell_ms;
      c.idx_lo = d.idx_lo;
      c.idx_hi = d.idx_hi;
      c.mcs_mask = d.mcs_mask;
      return CalMsg{c};
    }
    case MAVLINK_MSG_ID_MABUR_CAL_ACK: {
      mavlink_mabur_cal_ack_t d;
      mavlink_msg_mabur_cal_ack_decode(&msg, &d);
      SweepAck a;
      a.nonce = d.nonce;
      a.seq = d.seq;
      a.result = static_cast<AckResult>(d.result);
      a.start_in_ms = d.start_in_ms;
      a.flags = d.flags;
      return CalMsg{a};
    }
    case MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE: {
      mavlink_mabur_cal_sweep_done_t d;
      mavlink_msg_mabur_cal_sweep_done_decode(&msg, &d);
      SweepDone s;
      s.nonce = d.nonce;
      s.seq = d.seq;
      std::copy(d.offered, d.offered + 8, s.offered.begin());
      std::copy(d.blocked_pct, d.blocked_pct + 8, s.blocked_pct.begin());
      return CalMsg{s};
    }
    case MAVLINK_MSG_ID_MABUR_CAL_APPLY: {
      mavlink_mabur_cal_apply_t d;
      mavlink_msg_mabur_cal_apply_decode(&msg, &d);
      ApplyCmd a;
      a.nonce = d.nonce;
      a.bw = d.bw;
      a.has_walls = (d.has & 1u) != 0;
      a.has_eff = (d.has & 2u) != 0;
      std::copy(d.walls, d.walls + 8, a.walls.begin());
      a.legacy = d.legacy;
      std::copy(d.eff, d.eff + 8, a.eff_x1000.begin());
      return CalMsg{a};
    }
    case MAVLINK_MSG_ID_MABUR_CAL_APPLIED: {
      mavlink_mabur_cal_applied_t d;
      mavlink_msg_mabur_cal_applied_decode(&msg, &d);
      Applied a;
      a.nonce = d.nonce;
      a.bw = d.bw;
      a.ok = d.ok != 0;
      a.reason = std::string(d.reason, strnlen(d.reason, sizeof(d.reason)));
      return CalMsg{a};
    }
    case MAVLINK_MSG_ID_MABUR_CAL_ABORT: {
      mavlink_mabur_cal_abort_t d;
      mavlink_msg_mabur_cal_abort_decode(&msg, &d);
      Abort a;
      a.nonce = d.nonce;
      return CalMsg{a};
    }
    default:
      return std::nullopt;
  }
}

bool all_packets_are_cal(const uint8_t* bytes, size_t n) {
  if (n == 0 || bytes == nullptr) return false;
  // Fresh, local parse state for this call only -- never a channel index,
  // so this never touches the per-TU statics the generated _chan helpers
  // keep (see mav.h's header comment).
  mavlink_message_t rx{};
  mavlink_status_t st{};
  size_t packets = 0;
  for (size_t i = 0; i < n; ++i) {
    mavlink_message_t out{};
    if (mavlink_frame_char_buffer(&rx, &st, bytes[i], &out, nullptr) == MAVLINK_FRAMING_OK) {
      ++packets;
      if (!is_cal_msgid(out.msgid)) return false;
    }
  }
  if (packets == 0) return false;
  // A partial packet still in flight at the end leaves the parser mid
  // frame rather than back at idle -- trailing garbage or a truncated tail
  // packet, either way not "every packet is cal".
  if (st.parse_state != MAVLINK_PARSE_STATE_IDLE) return false;
  return true;
}

std::vector<CalMsg> decode_all(const uint8_t* bytes, size_t n) {
  std::vector<CalMsg> out;
  if (bytes == nullptr) return out;
  // Same fresh, local parse state as all_packets_are_cal: no channel index.
  mavlink_message_t rx{};
  mavlink_status_t st{};
  for (size_t i = 0; i < n; ++i) {
    mavlink_message_t msg{};
    if (mavlink_frame_char_buffer(&rx, &st, bytes[i], &msg, nullptr) != MAVLINK_FRAMING_OK)
      continue;
    if (auto m = decode(msg)) out.push_back(std::move(*m));
  }
  return out;
}

}  // namespace mabur::cal
