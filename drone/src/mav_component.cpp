#include "mav_component.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <exception>
#include <utility>

#include "config.h"
#include "config_apply.h"
#include "mabur/config_write.h"
#include "mabur/config_params.h"
#include "mabur/mav.h"
#include "mabur/param_value.h"
#include "mabur/rc_proto.h"
#include "mav_params.h"

namespace mabur {

struct MavComponent::Msg {
  mavlink_message_t m;
};

namespace {

using mav::kChanDroneRx;
using mav::kCompId;
using mav::kSysId;
using mavparam::Value;

constexpr size_t kStatusTextLen = 50;   // STATUSTEXT.text, single-chunk form

bool parser_idle() {
  return mavlink_get_channel_status(kChanDroneRx)->parse_state <= MAVLINK_PARSE_STATE_IDLE;
}

// 0 is broadcast: a target field addresses us when it is 0 or our own id.
bool for_us(uint8_t target_system, uint8_t target_component) {
  return (target_system == 0 || target_system == kSysId) &&
         (target_component == 0 || target_component == kCompId);
}

// A wire param_id for a log line: up to 16 bytes, NUL-terminated only when
// shorter, with anything unprintable shown as '?'.
std::string printable_id(const char id16[16]) {
  std::string s(id16, strnlen(id16, cfgparam::kMaxIdLen));
  for (char& c : s) {
    if (c < 0x20 || c > 0x7e) c = '?';
  }
  return s;
}

// The current value of row `idx` as the config file holds it. False with
// *err set when the file doesn't load.
bool current_value(const std::string& path, size_t idx, Value* out, std::string* err) {
  try {
    return mavparam::read(load_config(path), idx, out);
  } catch (const std::exception& e) {
    *err = loader_reason(e.what());
    return false;
  }
}

// The applier's own log wording for a path, for the completion log line.
const char* apply_path_name(ConfigApplyPath p) {
  switch (p) {
    case ConfigApplyPath::Knob: return "knob";
    case ConfigApplyPath::Reprogram: return "reprogram";
    case ConfigApplyPath::Rebuild: return "rebuild";
  }
  return "rebuild";
}

}  // namespace

MavComponent::MavComponent(std::string cfg_path, ApplyPort port,
                           std::function<void(const std::string&)> log)
    : cfg_path_(std::move(cfg_path)), port_(std::move(port)), log_(std::move(log)) {
  mavlink_reset_channel_status(kChanDroneRx);
}

uint8_t MavComponent::sys_id() { return mav::kSysId; }
uint8_t MavComponent::comp_id() { return mav::kCompId; }

void MavComponent::log(const std::string& line) const {
  if (log_) log_("[mav] " + line);
}

void MavComponent::queue(const Msg& m, bool list_value) {
  if (queue_.size() >= kMavQueueCap) {
    ++stats_.drop_queue_full;
    return;
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &m.m);
  queue_.push_back(Out{std::vector<uint8_t>(buf, buf + n), list_value});
  if (list_value) ++list_queued_;
}

void MavComponent::on_payload(const uint8_t* bytes, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    const bool was_idle = parser_idle();
    Msg msg;
    const uint8_t r = mavlink_frame_char(kChanDroneRx, bytes[i], &msg.m, nullptr);
    if (r == MAVLINK_FRAMING_OK) {
      if (msg.m.magic == MAVLINK_STX) {
        ++stats_.rx_packets;
        route(msg);
      } else {
        ++stats_.drop_bad_packet;   // MAVLink 1
      }
    } else if (r != MAVLINK_FRAMING_INCOMPLETE) {
      ++stats_.drop_bad_packet;     // bad CRC or signature
    } else if (!was_idle && parser_idle()) {
      ++stats_.drop_bad_packet;     // header rejected mid-packet
    }
  }
  if (!parser_idle()) ++stats_.drop_bad_packet;   // partial at the end of the payload
  mavlink_reset_channel_status(kChanDroneRx);
}

void MavComponent::route(const Msg& in) {
  const mavlink_message_t& m = in.m;
  switch (m.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return;
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
      mavlink_param_ext_request_list_t p;
      mavlink_msg_param_ext_request_list_decode(&m, &p);
      if (!for_us(p.target_system, p.target_component)) break;
      handle_list();
      return;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
      mavlink_param_ext_request_read_t p;
      mavlink_msg_param_ext_request_read_decode(&m, &p);
      if (!for_us(p.target_system, p.target_component)) break;
      handle_read(p.param_id, p.param_index);
      return;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      mavlink_param_ext_set_t p;
      mavlink_msg_param_ext_set_decode(&m, &p);
      if (!for_us(p.target_system, p.target_component)) break;
      handle_set(p.param_id, p.param_value, p.param_type);
      return;
    }
    default:
      break;
  }
  ++stats_.drop_not_for_us;
}

void MavComponent::queue_heartbeat() {
  Msg m;
  mavlink_msg_heartbeat_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m,
                                  MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0,
                                  MAV_STATE_ACTIVE);
  queue(m);
}

int MavComponent::take_outbound(std::vector<uint8_t>* out) {
  int taken = 0;
  while (!queue_.empty() && out->size() + queue_.front().bytes.size() <= rc::kMavMaxPayload) {
    const Out& o = queue_.front();
    out->insert(out->end(), o.bytes.begin(), o.bytes.end());
    if (o.list_value) --list_queued_;
    queue_.pop_front();
    ++taken;
  }
  return taken;
}

namespace {

void row_id(size_t idx, char id16[16]) {
  std::memset(id16, 0, cfgparam::kMaxIdLen);
  const char* id = cfgparam::kTable[idx].id;
  std::memcpy(id16, id, strnlen(id, cfgparam::kMaxIdLen));
}

}  // namespace

void MavComponent::queue_value(size_t idx, const Value& v, bool list_value) {
  char id[16], wire[128];
  row_id(idx, id);
  mavparam::encode_wire(v, wire);
  Msg m;
  mavlink_msg_param_ext_value_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m, id, wire,
                                        cfgparam::mav_type(v.type), cfgparam::kCount,
                                        static_cast<uint16_t>(idx));
  queue(m, list_value);
}

void MavComponent::queue_ack(const char id16[16], const Value* v, uint8_t mav_type,
                             uint8_t result) {
  char wire[128] = {};
  if (v) mavparam::encode_wire(*v, wire);
  Msg m;
  mavlink_msg_param_ext_ack_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m, id16, wire, mav_type,
                                      result);
  queue(m);
}

void MavComponent::queue_status(const std::string& text) {
  char buf[kStatusTextLen] = {};
  std::memcpy(buf, text.data(), std::min(text.size(), kStatusTextLen));
  Msg m;
  mavlink_msg_statustext_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m, MAV_SEVERITY_ERROR, buf, 0,
                                   0);
  queue(m);
}

void MavComponent::queue_warning(const std::string& text) {
  // One packet (id 0) when it fits; otherwise MAVLink 2's long-STATUSTEXT
  // form: 50-byte chunks sharing one nonzero id, chunk_seq from 0, the
  // last one NUL-terminated -- a text that fills its last chunk exactly
  // gets one more, empty, chunk to carry the NUL.
  if (text.size() <= kStatusTextLen) {
    char buf[kStatusTextLen] = {};
    std::memcpy(buf, text.data(), text.size());
    Msg m;
    mavlink_msg_statustext_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m, MAV_SEVERITY_WARNING,
                                     buf, 0, 0);
    queue(m);
    return;
  }
  if (++status_id_ == 0) status_id_ = 1;
  for (size_t off = 0, seq = 0; off <= text.size(); off += kStatusTextLen, ++seq) {
    char buf[kStatusTextLen] = {};
    const size_t n = std::min(text.size() - off, kStatusTextLen);
    std::memcpy(buf, text.data() + off, n);
    Msg m;
    mavlink_msg_statustext_pack_chan(kSysId, kCompId, kChanDroneRx, &m.m, MAV_SEVERITY_WARNING,
                                     buf, status_id_, static_cast<uint8_t>(seq));
    queue(m);
    if (n < kStatusTextLen) break;  // this chunk carried the terminating NUL
  }
}

void MavComponent::handle_list() {
  if (list_queued_ > 0) {
    ++stats_.list_coalesced;
    return;
  }
  Config cfg;
  try {
    cfg = load_config(cfg_path_);
  } catch (const std::exception& e) {
    log("list: config does not load: " + loader_reason(e.what()));
    return;
  }
  for (size_t idx = 0; idx < cfgparam::kCount; ++idx) {
    Value v;
    mavparam::read(cfg, idx, &v);
    queue_value(idx, v, /*list_value=*/true);
  }
  log("list: " + std::to_string(cfgparam::kCount) + " values");
}

void MavComponent::handle_read(const char id16[16], int16_t index) {
  const int idx = index >= 0 ? (index < static_cast<int>(cfgparam::kCount) ? index : -1)
                             : cfgparam::find(id16);
  if (idx < 0) {
    log("read " + (index >= 0 ? "#" + std::to_string(index) : printable_id(id16)) +
        ": no such parameter");
    return;
  }
  Value v;
  std::string err;
  if (!current_value(cfg_path_, static_cast<size_t>(idx), &v, &err)) {
    log("read " + std::string(cfgparam::kTable[idx].id) + ": config does not load: " + err);
    return;
  }
  queue_value(static_cast<size_t>(idx), v);
}

void MavComponent::handle_set(const char id16[16], const char value[128], uint8_t type) {
  const std::string shown = printable_id(id16);
  auto ack = [&](const Value* v, uint8_t mav_type, uint8_t result) {
    queue_ack(id16, v, mav_type, result);
  };
  auto status = [&](const std::string& text) { queue_status(text); };

  const int found = cfgparam::find(id16);
  if (found < 0) {
    ack(nullptr, type, PARAM_ACK_VALUE_UNSUPPORTED);
    ++stats_.sets_refused;
    log("set " + shown + ": no such parameter");
    return;
  }
  const size_t idx = static_cast<size_t>(found);
  const cfgparam::Row& row = cfgparam::kTable[idx];
  const uint8_t row_type = cfgparam::mav_type(row.type);
  const std::string field = std::string(row.section) + "." + row.key;

  // An apply in flight holds lock_config_writes() until it publishes, so
  // this is answered before the lock is touched: waiting for it would stop
  // the MAV loop's heartbeat and drain for the length of the apply. A repeat
  // of the pending set (a ground app re-sending after a lost IN_PROGRESS)
  // is told IN_PROGRESS again and is resolved by the one final ack; any
  // other set is refused, with the value the file holds, read without the
  // lock (the file is only ever replaced by rename).
  if (port_.busy && port_.busy()) {
    Value want;
    std::string rendered;
    const bool decoded =
        mavparam::decode_wire(idx, type, value, &want) && mavparam::render_toml(want, &rendered);
    if (decoded && has_pending_ && pending_.idx == idx && pending_.rendered == rendered) {
      ack(&pending_.cur, row_type, PARAM_ACK_IN_PROGRESS);
      log("set " + shown + " = " + rendered + " (" + field + ") in progress, repeat");
      return;
    }
    Value cur{row.type};
    bool have_cur = true;
    if (has_pending_ && pending_.idx == idx) {
      cur = pending_.cur;
    } else {
      std::string err;
      have_cur = current_value(cfg_path_, idx, &cur, &err);
    }
    ack(have_cur ? &cur : nullptr, row_type, PARAM_ACK_FAILED);
    status("apply in progress");
    ++stats_.sets_refused;
    log("set " + shown + " refused: apply in progress");
    return;
  }

  // Held from the read of the current file through the candidate write and
  // load and the hand-off to the applier, so the applier's own use of
  // <cfg_path>.new and a concurrent calibration publish can't interleave
  // with this read-patch-check sequence. The applier takes the same lock
  // again for its own re-read through its publish, once this function has
  // returned.
  auto lock = lock_config_writes();

  Config cur_cfg;
  try {
    cur_cfg = load_config(cfg_path_);
  } catch (const std::exception& e) {
    const std::string err = loader_reason(e.what());
    ack(nullptr, row_type, PARAM_ACK_FAILED);
    status(err);
    ++stats_.sets_refused;
    log("set " + shown + " refused: config does not load: " + err);
    return;
  }
  Value cur{row.type};
  mavparam::read(cur_cfg, idx, &cur);
  auto refuse = [&](uint8_t result, const std::string& why, bool tell) {
    ack(&cur, row_type, result);
    if (tell) status(why);
    ++stats_.sets_refused;
    log("set " + shown + " refused: " + why);
  };

  Value want;
  std::string rendered;
  if (!mavparam::decode_wire(idx, type, value, &want) || !mavparam::render_toml(want, &rendered)) {
    refuse(PARAM_ACK_VALUE_UNSUPPORTED,
           "value unsupported (sent type " + std::to_string(type) + ", row type " +
               std::to_string(row_type) + ")",
           false);
    return;
  }
  std::string text;
  if (!read_text_file(cfg_path_, &text)) {
    refuse(PARAM_ACK_FAILED, "config file unreadable", true);
    return;
  }
  std::string patched = text;
  std::string followed, patch_err;
  if (!mavparam::patch_row(&patched, idx, want, cur_cfg, &followed, &patch_err)) {
    refuse(PARAM_ACK_FAILED, patch_err, true);
    return;
  }
  const std::string followed_suffix = followed.empty() ? "" : " +" + followed;
  // Unchanged when the rendered line is already the file's, or when the
  // decoded value equals what the file loads as (a REAL32 line spelled
  // differently, "0.650"). Str rows are left to the text comparison: their
  // rendering is canonical, and REC_SIZE reads back venc.size for a "0x0"
  // line, which a set of that size must still be able to pin.
  const bool same_value = want.type != cfgparam::Type::Str && want.b == cur.b &&
                          want.i == cur.i && want.f == cur.f;
  if (patched == text || same_value) {
    ack(&cur, row_type, PARAM_ACK_ACCEPTED);
    ++stats_.sets_ok;
    log("set " + shown + " = " + rendered + " (" + field + followed_suffix + ", unchanged)");
    return;
  }
  // The candidate as the boot loader would see it, before handing this row
  // to the applier: a row the loader would refuse (an ordering rule, an
  // out-of-range value past render_toml's own checks) is caught here, so
  // the operator gets the loader's reason at once and the applier is never
  // asked to apply a candidate that can't boot. This is only a check
  // -- the applier re-reads, re-patches, re-verifies and publishes its own
  // candidate under its own hold of the lock -- so the scratch file is
  // removed right after the check, win or lose, the same as
  // publish_config and ConfigApplier::apply_locked do with theirs.
  const std::string tmp = cfg_path_ + ".new";
  if (!write_file_fsync(tmp, patched)) {
    std::remove(tmp.c_str());
    refuse(PARAM_ACK_FAILED, "failed to write " + tmp, true);
    return;
  }
  try {
    (void)load_config(tmp);
  } catch (const std::exception& e) {
    std::remove(tmp.c_str());
    refuse(PARAM_ACK_FAILED, loader_reason(e.what()), true);
    return;
  }
  std::remove(tmp.c_str());

  std::string why;
  if (!port_.submit(idx, want, &why)) {
    refuse(PARAM_ACK_FAILED, why, true);
    return;
  }
  // The file hasn't changed yet -- the ack reports the value still on
  // record until the applier's result lands in poll_results.
  std::memcpy(pending_.id16, id16, cfgparam::kMaxIdLen);
  pending_.idx = idx;
  pending_.row_type = row_type;
  pending_.cur = cur;
  pending_.rendered = rendered;
  pending_.field = field;
  has_pending_ = true;
  ack(&cur, row_type, PARAM_ACK_IN_PROGRESS);
  ++stats_.sets_in_progress;
  log("set " + shown + " = " + rendered + " (" + field + followed_suffix + ") in progress");
}

void MavComponent::poll_results() {
  ConfigApplyResult r;
  if (!port_.take_result(&r)) return;
  if (!has_pending_) {
    log("applier result with no set pending, dropped");
    return;
  }
  const Pending p = pending_;
  has_pending_ = false;
  const std::string shown = printable_id(p.id16);
  if (r.ok) {
    // The ack reports what the published file loads as; the applier just
    // published these bytes, so the fallback is only for a vanished file.
    Value now = p.cur;
    std::string err;
    (void)current_value(cfg_path_, p.idx, &now, &err);
    queue_ack(p.id16, &now, p.row_type, PARAM_ACK_ACCEPTED);
    for (const auto& n : r.notices) queue_warning(n);
    ++stats_.sets_ok;
    log("set " + shown + " = " + p.rendered + " (" + p.field + ") applied " +
        apply_path_name(r.path) + " " + std::to_string(r.ms) + " ms");
  } else {
    queue_ack(p.id16, &p.cur, p.row_type, PARAM_ACK_FAILED);
    queue_status(r.reason);
    ++stats_.sets_refused;
    log("set " + shown + " refused: " + r.reason);
  }
}

}  // namespace mabur
