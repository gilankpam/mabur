#include "param_client.h"

#include <utility>

#include "mabur/mav.h"

namespace maburgs {

namespace {

using mabur::cfgparam::kCount;

constexpr uint8_t kDroneSysId = 1;
constexpr uint8_t kDroneCompId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER

const char* type_name(mabur::cfgparam::Type t) {
  switch (t) {
    case mabur::cfgparam::Type::Bool: return "bool";
    case mabur::cfgparam::Type::Int32: return "int32";
    case mabur::cfgparam::Type::Real32: return "real32";
    case mabur::cfgparam::Type::Str: return "str";
  }
  return "?";
}

}  // namespace

ParamClient::ParamClient(SendFn send, uint8_t sysid, uint8_t compid)
    : send_(std::move(send)), sysid_(sysid), compid_(compid) {}

void ParamClient::send_list_request() {
  mavlink_message_t m;
  mavlink_msg_param_ext_request_list_pack(sysid_, compid_, &m, kDroneSysId, kDroneCompId);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
  send_(buf, n);
  ++stats_.tx;
}

void ParamClient::send_read_request(size_t idx) {
  mavlink_message_t m;
  char id[16] = {};  // blank: param_index selects the row, not the id
  mavlink_msg_param_ext_request_read_pack(sysid_, compid_, &m, kDroneSysId, kDroneCompId, id,
                                          static_cast<int16_t>(idx));
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
  send_(buf, n);
  ++stats_.tx;
}

void ParamClient::send_set_packet(size_t idx, const mabur::paramval::Value& v) {
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  mavlink_message_t m;
  mavlink_msg_param_ext_set_pack(sysid_, compid_, &m, kDroneSysId, kDroneCompId,
                                 mabur::cfgparam::kTable[idx].id, wire,
                                 mabur::cfgparam::mav_type(mabur::cfgparam::kTable[idx].type));
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
  send_(buf, n);
  ++stats_.tx;
}

void ParamClient::begin_list(uint64_t now_ms) {
  state_.list = ParamListState::Pending;
  for (bool& f : fresh_) f = false;
  list_last_req_ms_ = now_ms;
  list_rounds_ = 0;
  send_list_request();
}

void ParamClient::refuse(size_t row, const std::string& why, uint64_t now_ms) {
  if (row >= kCount) return;
  state_.rows[row].st = ParamRowState::Failed;
  state_.rows[row].status = why;
  state_.rows[row].result_ms = now_ms;
}

void ParamClient::timeout_pending(uint64_t now_ms) {
  const size_t idx = static_cast<size_t>(state_.pending_row);
  state_.rows[idx].st = ParamRowState::Timeout;
  state_.rows[idx].result_ms = now_ms;
  state_.pending_row = -1;
  set_in_progress_ = false;
  set_resends_ = 0;
  ++stats_.timeouts;
  send_read_request(idx);
}

void ParamClient::set_linked(bool linked, uint64_t now_ms) {
  if (linked == state_.linked) return;
  if (!linked && state_.pending_row != -1) {
    // The pending set can never get a final answer now. Mark it lost
    // before the reset below drops every row to its default -- so the
    // row's last word, however briefly held, is the true reason rather
    // than a silent disappearance.
    state_.rows[state_.pending_row].st = ParamRowState::Timeout;
    state_.rows[state_.pending_row].status = "link lost";
  }
  state_ = ParamClientState{};
  state_.linked = linked;
  alive_cached_ = false;
  for (bool& f : fresh_) f = false;
  list_rounds_ = 0;
  list_last_req_ms_ = now_ms;
  if (linked) {
    state_.list = ParamListState::Pending;
    send_list_request();
  }
  ++seq_;
}

void ParamClient::on_packet(const uint8_t* p, size_t n, uint64_t now_ms) {
  // Fresh buffers every call: mavlink_frame_char_buffer carries no state
  // of its own beyond what's passed in, so a packet cut off mid-way can
  // never bleed into the next on_packet call.
  mavlink_message_t rx{}, out{};
  mavlink_status_t st{};
  bool ok = false;
  for (size_t i = 0; i < n; ++i) {
    if (mavlink_frame_char_buffer(&rx, &st, p[i], &out, nullptr) == MAVLINK_FRAMING_OK) ok = true;
  }
  if (!ok) {
    ++stats_.drop_bad;
    return;
  }
  ++stats_.rx;
  if (out.sysid != kDroneSysId || out.compid != kDroneCompId) return;

  bool changed = false;
  switch (out.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      const bool was_alive = alive_cached_;
      state_.alive_ms = now_ms;
      alive_cached_ = true;
      if (!was_alive) changed = true;
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_VALUE: {
      mavlink_param_ext_value_t v;
      mavlink_msg_param_ext_value_decode(&out, &v);
      if (v.param_count != kCount && !state_.table_mismatch) {
        state_.table_mismatch = true;
        changed = true;
      }
      const int idx = mabur::cfgparam::find(v.param_id);
      if (idx < 0) {
        ++stats_.drop_unknown;
        break;
      }
      mabur::paramval::Value val;
      if (!mabur::paramval::decode_wire(static_cast<size_t>(idx), v.param_type, v.param_value,
                                        &val)) {
        ++stats_.drop_bad;
        break;
      }
      state_.rows[idx].value = val;
      state_.rows[idx].have_value = true;
      fresh_[idx] = true;
      changed = true;
      if (state_.list == ParamListState::Pending || state_.list == ParamListState::Partial) {
        bool all = true;
        for (size_t i = 0; i < kCount; ++i) {
          if (!fresh_[i]) {
            all = false;
            break;
          }
        }
        if (all) state_.list = ParamListState::Complete;
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_ACK: {
      mavlink_param_ext_ack_t a;
      mavlink_msg_param_ext_ack_decode(&out, &a);
      const int idx = mabur::cfgparam::find(a.param_id);
      if (idx < 0) {
        ++stats_.drop_unknown;
        break;
      }
      mabur::paramval::Value val;
      if (!mabur::paramval::decode_wire(static_cast<size_t>(idx), a.param_type, a.param_value,
                                        &val)) {
        ++stats_.drop_bad;
        break;
      }
      ParamRow& row = state_.rows[idx];
      row.value = val;
      row.have_value = true;
      fresh_[idx] = true;  // an ack carries the drone's value as of now too
      changed = true;
      if (static_cast<int>(idx) != state_.pending_row) break;  // not the row waiting on an ack
      switch (a.param_result) {
        case PARAM_ACK_IN_PROGRESS:
          if (!set_in_progress_) {
            set_in_progress_ = true;
            set_deadline_ms_ = now_ms + kParamFinalMs;  // the first one starts the clock
          }
          row.st = ParamRowState::InProgress;
          break;
        case PARAM_ACK_ACCEPTED:
          row.st = ParamRowState::Accepted;
          row.result_ms = now_ms;
          state_.pending_row = -1;
          set_in_progress_ = false;
          set_resends_ = 0;
          if (mabur::cfgparam::kTable[idx].apply == mabur::cfgparam::Apply::Rebuild) {
            begin_list(now_ms);  // a VENC_MODE set may have moved REC_SIZE along with it
          }
          break;
        case PARAM_ACK_FAILED:
          row.st = ParamRowState::Failed;
          row.result_ms = now_ms;
          state_.pending_row = -1;
          set_in_progress_ = false;
          set_resends_ = 0;
          break;
        case PARAM_ACK_VALUE_UNSUPPORTED:
          row.st = ParamRowState::Unsupported;
          row.result_ms = now_ms;
          state_.pending_row = -1;
          set_in_progress_ = false;
          set_resends_ = 0;
          break;
        default:
          break;
      }
      break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
      mavlink_statustext_t s;
      mavlink_msg_statustext_decode(&out, &s);
      size_t len = 0;
      while (len < sizeof(s.text) && s.text[len] != '\0') ++len;
      std::string text(s.text, len);
      if (s.id != 0) {
        // MAVLink 2's long form: chunks under one id, chunk_seq from 0,
        // the last one NUL-terminated. A row gets the whole text or none
        // of it: a gap drops what was gathered.
        if (s.chunk_seq == 0) {
          text_id_ = s.id;
          text_next_seq_ = 0;
          text_buf_.clear();
        }
        if (s.id != text_id_ || s.chunk_seq != text_next_seq_) {
          text_id_ = 0;
          text_buf_.clear();
          break;
        }
        text_buf_ += text;
        ++text_next_seq_;
        if (len == sizeof(s.text)) break;  // no NUL yet: more chunks to come
        text = std::move(text_buf_);
        text_buf_.clear();
        text_id_ = 0;
      }
      if (state_.pending_row != -1) {
        // The set's own ack may have been lost; the row that is waiting
        // on one is the only sensible destination.
        state_.rows[state_.pending_row].status = text;
        changed = true;
      } else {
        int best = -1;
        uint64_t best_ms = 0;
        for (size_t i = 0; i < kCount; ++i) {
          const ParamRow& row = state_.rows[i];
          // A refusal's reason follows its FAILED ack; a warning (the
          // RADIO_LDPC overdrive note) follows its ACCEPTED one.
          if (row.st != ParamRowState::Failed && row.st != ParamRowState::Unsupported &&
              row.st != ParamRowState::Accepted)
            continue;
          if (now_ms < row.result_ms || now_ms - row.result_ms > kParamStatusMs) continue;
          if (best < 0 || row.result_ms > best_ms) {
            best = static_cast<int>(i);
            best_ms = row.result_ms;
          }
        }
        if (best >= 0) {
          state_.rows[best].status = text;
          changed = true;
        }
      }
      break;
    }
    default:
      break;
  }
  if (changed) ++seq_;
}

void ParamClient::tick(uint64_t now_ms) {
  bool changed = false;
  if (alive_cached_ && !alive(now_ms)) {
    alive_cached_ = false;
    changed = true;
  }
  if (state_.pending_row != -1) {
    if (set_in_progress_) {
      if (now_ms >= set_deadline_ms_) {
        timeout_pending(now_ms);
        changed = true;
      }
    } else if (now_ms - set_send_ms_ >= kParamSetRetryMs) {
      if (set_resends_ < kParamSetRetries) {
        send_set_packet(static_cast<size_t>(state_.pending_row), set_value_);
        ++set_resends_;
        set_send_ms_ = now_ms;
        // A resend changes nothing observable through state(): no bump.
      } else {
        timeout_pending(now_ms);
        changed = true;
      }
    }
  }
  if (state_.list == ParamListState::Pending &&
      now_ms - list_last_req_ms_ >= kParamListRetryMs) {
    if (list_rounds_ >= kParamListRounds) {
      state_.list = ParamListState::Partial;
      changed = true;
    } else {
      bool any_missing = false;
      for (size_t i = 0; i < kCount; ++i) {
        if (!fresh_[i]) {
          send_read_request(i);
          any_missing = true;
        }
      }
      list_last_req_ms_ = now_ms;
      if (any_missing) {
        ++list_rounds_;
      } else {
        // Nothing left to re-read: the completion check in on_packet
        // should already have caught this, but a Complete transition
        // here too keeps tick() correct on its own.
        state_.list = ParamListState::Complete;
        changed = true;
      }
    }
  }
  if (changed) ++seq_;
}

void ParamClient::refresh(uint64_t now_ms) {
  if (!state_.linked) return;
  const bool changed = state_.list != ParamListState::Pending;
  begin_list(now_ms);
  if (changed) ++seq_;
}

bool ParamClient::set(size_t row, const std::string& text, uint64_t now_ms, std::string* why) {
  std::string reason;
  if (!state_.linked) {
    reason = "not linked";
  } else if (state_.pending_row != -1) {
    reason = "apply in progress";
  } else if (row >= kCount) {
    reason = "no such parameter";
  } else {
    mabur::paramval::Value v;
    const auto& tbl_row = mabur::cfgparam::kTable[row];
    if (!mabur::paramval::parse_text(tbl_row.type, text, &v)) {
      reason = std::string("bad value for ") + type_name(tbl_row.type);
    } else {
      send_set_packet(row, v);
      state_.rows[row].st = ParamRowState::Sending;
      // A new set owns the row's result: an earlier reason must not
      // survive onto an answer that brings none of its own.
      state_.rows[row].status.clear();
      state_.rows[row].result_ms = 0;
      state_.pending_row = static_cast<int>(row);
      set_value_ = v;
      set_send_ms_ = now_ms;
      set_resends_ = 0;
      set_in_progress_ = false;
      ++seq_;
      if (why) why->clear();
      return true;
    }
  }
  if (why) *why = reason;
  refuse(row, reason, now_ms);
  if (row < kCount) ++seq_;
  return false;
}

bool ParamClient::alive(uint64_t now_ms) const {
  return state_.alive_ms != 0 && now_ms - state_.alive_ms <= kParamAliveMs;
}

}  // namespace maburgs
