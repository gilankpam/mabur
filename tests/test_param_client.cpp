// ParamClient: ground-side PARAM_EXT_* list/read/link state.
#include <cstring>
#include <vector>

#include "mabur/config_params.h"
#include "mabur/mav.h"
#include "mabur/param_value.h"
#include "mtest.h"
#include "param_client.h"

using namespace maburgs;
using Bytes = std::vector<uint8_t>;

static Bytes pkt(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  return Bytes(buf, buf + n);
}

static Bytes value_pkt(const char* id, uint8_t type, const char wire[128], uint16_t count,
                        uint16_t index) {
  char id16[16] = {};
  std::strncpy(id16, id, 16);
  mavlink_message_t msg;
  mavlink_msg_param_ext_value_pack(1, 191, &msg, id16, wire, type, count, index);
  return pkt(msg);
}

static Bytes hb_pkt() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 191, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0,
                             0, MAV_STATE_ACTIVE);
  return pkt(msg);
}

static Bytes ack_pkt(const char* id, uint8_t type, const char wire[128], uint8_t result) {
  char id16[16] = {};
  std::strncpy(id16, id, 16);
  mavlink_message_t msg;
  mavlink_msg_param_ext_ack_pack(1, 191, &msg, id16, wire, type, result);
  return pkt(msg);
}

static Bytes status_pkt(const std::string& text) {
  char buf[50] = {};
  std::memcpy(buf, text.data(), std::min(text.size(), static_cast<size_t>(50)));
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(1, 191, &msg, MAV_SEVERITY_ERROR, buf, 0, 0);
  return pkt(msg);
}

// A WARNING the way the drone's MavComponent::queue_warning sends it: one
// packet (id 0) when it fits, else MAVLink 2's long-STATUSTEXT form --
// 50-byte chunks under one nonzero id, chunk_seq from 0, the last one
// NUL-terminated (an extra empty chunk when the text fills its last one).
static std::vector<Bytes> warning_pkts(const std::string& text, uint16_t id) {
  std::vector<Bytes> out;
  for (size_t off = 0, seq = 0; off <= text.size(); off += 50, ++seq) {
    char buf[50] = {};
    const size_t n = std::min(text.size() - off, static_cast<size_t>(50));
    std::memcpy(buf, text.data() + off, n);
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(1, 191, &msg, MAV_SEVERITY_WARNING, buf,
                                text.size() <= 50 ? 0 : id, static_cast<uint8_t>(seq));
    out.push_back(pkt(msg));
    if (n < 50 || text.size() <= 50) break;
  }
  return out;
}

static std::vector<mavlink_message_t> parse(const Bytes& b) {
  mavlink_reset_channel_status(MAVLINK_COMM_1);
  std::vector<mavlink_message_t> out;
  mavlink_message_t msg;
  mavlink_status_t status;
  for (uint8_t c : b) {
    if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status) == MAVLINK_FRAMING_OK) {
      out.push_back(msg);
    }
  }
  return out;
}

// A plausible value for row `idx`, matching its own type -- VENC_MODE gets
// a shape decode_wire's loader-shape check accepts, every other Str row
// gets a plain printable string.
static mabur::paramval::Value sample_value(size_t idx) {
  const auto& row = mabur::cfgparam::kTable[idx];
  mabur::paramval::Value v{row.type};
  switch (row.type) {
    case mabur::cfgparam::Type::Bool:
      v.b = true;
      break;
    case mabur::cfgparam::Type::Int32:
      v.i = 42;
      break;
    case mabur::cfgparam::Type::Real32:
      v.f = 1.5f;
      break;
    case mabur::cfgparam::Type::Str:
      v.s = (idx == mabur::cfgparam::kVencModeRow) ? "60@1920x1080" : "0x0";
      break;
  }
  return v;
}

// Builds and feeds a PARAM_EXT_VALUE for row `idx` with a plausible value.
static void feed_row(ParamClient& c, size_t idx, uint64_t now_ms,
                      uint16_t count = mabur::cfgparam::kCount) {
  const auto& row = mabur::cfgparam::kTable[idx];
  const mabur::paramval::Value v = sample_value(idx);
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  const Bytes bytes = value_pkt(row.id, mabur::cfgparam::mav_type(row.type), wire, count,
                                 static_cast<uint16_t>(idx));
  c.on_packet(bytes.data(), bytes.size(), now_ms);
}

struct Rig {
  std::vector<Bytes> sent;
  ParamClient c{[this](const uint8_t* p, size_t n) { sent.emplace_back(p, p + n); }};
};

// Fixture for the set() tests: links `r.c` and feeds every row a value so
// the list reaches Complete, then clears `sent` so a test sees only its
// own traffic.
static void link_and_complete_list(Rig& r, uint64_t now_ms) {
  r.c.set_linked(true, now_ms);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) feed_row(r.c, i, now_ms + 1);
  r.sent.clear();
}

TEST(link_up_requests_the_list_once) {
  Rig r;
  const uint32_t seq0 = r.c.seq();
  r.c.set_linked(true, 1000);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST);
  mavlink_param_ext_request_list_t p;
  mavlink_msg_param_ext_request_list_decode(&msgs[0], &p);
  CHECK(p.target_system == 1 && p.target_component == 191);
  CHECK(r.c.state().list == ParamListState::Pending);
  CHECK(r.c.seq() == seq0 + 1);
}

TEST(twelve_values_complete_the_list) {
  Rig r;
  r.c.set_linked(true, 1000);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) feed_row(r.c, i, 1001);
  CHECK(r.c.state().list == ParamListState::Complete);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) CHECK(r.c.state().rows[i].have_value);
  CHECK(r.c.state().rows[mabur::cfgparam::kVencModeRow].value.s == "60@1920x1080");
  CHECK(!r.c.state().table_mismatch);
}

TEST(missing_index_is_reread_then_partial) {
  Rig r;
  r.c.set_linked(true, 1000);
  r.sent.clear();
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    if (i == 5) continue;
    feed_row(r.c, i, 1001);
  }
  REQUIRE(r.c.state().list == ParamListState::Pending);

  r.c.tick(3001);
  REQUIRE(r.sent.size() == 1);
  {
    auto msgs = parse(r.sent[0]);
    REQUIRE(msgs.size() == 1);
    CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ);
    mavlink_param_ext_request_read_t p;
    mavlink_msg_param_ext_request_read_decode(&msgs[0], &p);
    CHECK(p.param_index == 5);
  }

  r.sent.clear();
  r.c.tick(5002);
  CHECK(r.sent.size() == 1);
  CHECK(r.c.state().list == ParamListState::Pending);

  r.sent.clear();
  r.c.tick(7003);
  CHECK(r.sent.size() == 1);
  CHECK(r.c.state().list == ParamListState::Pending);

  r.sent.clear();
  r.c.tick(9004);
  CHECK(r.sent.empty());
  CHECK(r.c.state().list == ParamListState::Partial);
}

TEST(list_with_wrong_count_flags_mismatch) {
  Rig r;
  r.c.set_linked(true, 1000);
  feed_row(r.c, mabur::cfgparam::kVencModeRow, 1001, /*count=*/13);
  CHECK(r.c.state().table_mismatch);
  CHECK(r.c.state().rows[mabur::cfgparam::kVencModeRow].have_value);
  CHECK(r.c.state().rows[mabur::cfgparam::kVencModeRow].value.s == "60@1920x1080");
}

TEST(unknown_id_is_dropped_and_counted) {
  Rig r;
  r.c.set_linked(true, 1000);
  char wire[128] = {};
  const Bytes bytes = value_pkt("NOPE", MAV_PARAM_EXT_TYPE_UINT8, wire,
                                 mabur::cfgparam::kCount, 0);
  r.c.on_packet(bytes.data(), bytes.size(), 1001);
  CHECK(r.c.stats().drop_unknown == 1);
  CHECK(!r.c.state().table_mismatch);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) CHECK(!r.c.state().rows[i].have_value);
}

TEST(refresh_relists_without_clearing) {
  Rig r;
  r.c.set_linked(true, 1000);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) feed_row(r.c, i, 1001);
  REQUIRE(r.c.state().list == ParamListState::Complete);

  r.sent.clear();
  r.c.refresh(2000);
  CHECK(r.c.state().list == ParamListState::Pending);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) CHECK(r.c.state().rows[i].have_value);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST);
}

TEST(link_down_clears_everything) {
  Rig r;
  r.c.set_linked(true, 1000);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) feed_row(r.c, i, 1001);
  const uint32_t seq_before = r.c.seq();

  r.c.set_linked(false, 2000);
  CHECK(r.c.seq() == seq_before + 1);

  const ParamClientState def{};
  CHECK(r.c.state().linked == def.linked);
  CHECK(r.c.state().alive_ms == def.alive_ms);
  CHECK(r.c.state().table_mismatch == def.table_mismatch);
  CHECK(r.c.state().list == def.list);
  CHECK(r.c.state().pending_row == def.pending_row);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    CHECK(r.c.state().rows[i].have_value == def.rows[i].have_value);
    CHECK(r.c.state().rows[i].st == def.rows[i].st);
  }
}

TEST(heartbeat_stamps_alive_and_flips_once) {
  Rig r;
  const Bytes hb = hb_pkt();
  const uint32_t seq0 = r.c.seq();

  r.c.on_packet(hb.data(), hb.size(), 1000);
  CHECK(r.c.alive(1000));
  CHECK(r.c.seq() == seq0 + 1);

  const uint32_t seq1 = r.c.seq();
  r.c.on_packet(hb.data(), hb.size(), 1500);
  CHECK(r.c.seq() == seq1);  // still alive: refreshing alive_ms alone doesn't bump

  r.c.tick(4600);
  CHECK(!r.c.alive(4600));
  CHECK(r.c.seq() == seq1 + 1);  // the timeout flip does bump
}

TEST(bad_packet_is_counted) {
  Rig r;
  const Bytes hb = hb_pkt();
  const Bytes truncated(hb.begin(), hb.begin() + hb.size() - 3);  // never reaches a CRC
  const uint32_t seq0 = r.c.seq();

  r.c.on_packet(truncated.data(), truncated.size(), 1000);
  CHECK(r.c.stats().drop_bad == 1);
  CHECK(r.c.stats().rx == 0);
  CHECK(!r.c.alive(1000));
  CHECK(r.c.seq() == seq0);
}

TEST(seq_is_stable_when_nothing_changes) {
  Rig r;
  const uint32_t seq0 = r.c.seq();
  r.c.tick(1000);
  r.c.tick(2000);
  CHECK(r.c.seq() == seq0);
}

TEST(set_encodes_by_type_and_marks_sending) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  CHECK(r.c.set(4, "30000", 2000, &why));
  CHECK(why.empty());
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_SET);
  mavlink_param_ext_set_t p;
  mavlink_msg_param_ext_set_decode(&msgs[0], &p);
  CHECK(p.target_system == 1 && p.target_component == 191);
  CHECK(p.param_type == 6);  // MAV_PARAM_EXT_TYPE_INT32
  int32_t v;
  std::memcpy(&v, p.param_value, sizeof v);
  CHECK(v == 30000);
  CHECK(r.c.state().rows[4].st == ParamRowState::Sending);
  CHECK(r.c.state().pending_row == 4);
}

TEST(set_refuses_second_while_pending) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));
  r.sent.clear();

  std::string why2;
  CHECK(!r.c.set(5, "0.5", 2001, &why2));
  CHECK(why2 == "apply in progress");
  CHECK(r.c.state().rows[5].st == ParamRowState::Failed);
  CHECK(r.c.state().rows[5].status == "apply in progress");
  CHECK(r.sent.empty());
}

TEST(set_refuses_bad_text_and_records_it) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  CHECK(!r.c.set(4, "abc", 2000, &why));
  CHECK(why.rfind("bad value", 0) == 0);
  CHECK(r.c.state().rows[4].st == ParamRowState::Failed);
  CHECK(r.c.state().rows[4].status.rfind("bad value", 0) == 0);
  CHECK(r.sent.empty());
}

TEST(set_refuses_unlinked) {
  Rig r;
  link_and_complete_list(r, 1000);
  r.c.set_linked(false, 2000);

  std::string why;
  CHECK(!r.c.set(4, "30000", 2001, &why));
  CHECK(why == "not linked");
}

TEST(in_progress_then_accepted_updates_value) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  char wire[128];
  v.i = 24000;
  mabur::paramval::encode_wire(v, wire);
  Bytes a1 = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_IN_PROGRESS);
  r.c.on_packet(a1.data(), a1.size(), 2100);
  CHECK(r.c.state().rows[4].st == ParamRowState::InProgress);
  CHECK(r.c.state().rows[4].value.i == 24000);
  CHECK(r.c.state().pending_row == 4);

  v.i = 30000;
  mabur::paramval::encode_wire(v, wire);
  Bytes a2 = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a2.data(), a2.size(), 2200);
  CHECK(r.c.state().rows[4].st == ParamRowState::Accepted);
  CHECK(r.c.state().rows[4].value.i == 30000);
  CHECK(r.c.state().pending_row == -1);
  CHECK(r.c.state().rows[4].result_ms == 2200);
}

TEST(accepted_without_in_progress) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30000;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes a = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);
  CHECK(r.c.state().rows[4].st == ParamRowState::Accepted);
  CHECK(r.c.state().pending_row == -1);
}

TEST(failed_then_status_fills_status) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(8, "5", 2000, &why));

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes fail = ack_pkt("REC_FPS", 6, wire, PARAM_ACK_FAILED);
  r.c.on_packet(fail.data(), fail.size(), 2050);
  CHECK(r.c.state().rows[8].st == ParamRowState::Failed);
  CHECK(r.c.state().pending_row == -1);

  Bytes s1 = status_pkt("record.fps: must be in [1, venc.fps]");
  r.c.on_packet(s1.data(), s1.size(), 3050);  // 1 s after the ack: within the window
  CHECK(r.c.state().rows[8].status == "record.fps: must be in [1, venc.fps]");

  Bytes s2 = status_pkt("stale, must be dropped");
  r.c.on_packet(s2.data(), s2.size(), 4050);  // 2 s after the ack: outside the window
  CHECK(r.c.state().rows[8].status == "record.fps: must be in [1, venc.fps]");
}

TEST(new_set_clears_the_previous_status) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);

  REQUIRE(r.c.set(8, "5", 2000, &why));
  Bytes fail = ack_pkt("REC_FPS", 6, wire, PARAM_ACK_FAILED);
  r.c.on_packet(fail.data(), fail.size(), 2050);
  Bytes s1 = status_pkt("record.fps: must be in [1, venc.fps]");
  r.c.on_packet(s1.data(), s1.size(), 2060);
  REQUIRE(r.c.state().rows[8].status == "record.fps: must be in [1, venc.fps]");

  // A later set whose FAILED arrives with no STATUSTEXT (lost) must not
  // show the first set's unrelated reason.
  REQUIRE(r.c.set(8, "6", 5000, &why));
  CHECK(r.c.state().rows[8].status.empty());
  CHECK(r.c.state().rows[8].st == ParamRowState::Sending);
  r.c.on_packet(fail.data(), fail.size(), 5050);
  CHECK(r.c.state().rows[8].st == ParamRowState::Failed);
  CHECK(r.c.state().rows[8].status.empty());
  CHECK(r.c.state().rows[8].result_ms == 5050);
}

TEST(status_before_final_ack_lands_on_pending_row) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(8, "5", 2000, &why));

  Bytes s = status_pkt("record.fps: must be in [1, venc.fps]");
  r.c.on_packet(s.data(), s.size(), 2010);  // the ack was lost, this arrives instead
  CHECK(r.c.state().rows[8].status == "record.fps: must be in [1, venc.fps]");
  CHECK(r.c.state().rows[8].st == ParamRowState::Sending);

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes fail = ack_pkt("REC_FPS", 6, wire, PARAM_ACK_FAILED);
  r.c.on_packet(fail.data(), fail.size(), 2050);
  CHECK(r.c.state().rows[8].st == ParamRowState::Failed);
  CHECK(r.c.state().rows[8].status == "record.fps: must be in [1, venc.fps]");
  CHECK(r.c.state().pending_row == -1);
}

// The drone's RADIO_LDPC set that leaves a width's walls measured under
// the other LDPC setting: ACCEPTED, then the long warning in chunks.
static const std::string kLdpcWarning =
    "radio: 20 MHz walls were measured with ldpc on; radio.ldpc is off \xe2\x80\x94 re-run maburcal";

static Bytes ldpc_ack(uint8_t result) {
  mabur::paramval::Value v{mabur::cfgparam::Type::Bool};
  v.b = false;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  return ack_pkt("RADIO_LDPC", mabur::cfgparam::mav_type(mabur::cfgparam::Type::Bool), wire,
                 result);
}

TEST(a_warning_after_an_accepted_ack_lands_on_that_row_whole) {
  Rig r;
  link_and_complete_list(r, 1000);
  const int row = mabur::cfgparam::find("RADIO_LDPC");
  REQUIRE(row >= 0);
  std::string why;
  REQUIRE(r.c.set(static_cast<size_t>(row), "false", 2000, &why));
  const Bytes a = ldpc_ack(PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);
  REQUIRE(r.c.state().rows[row].st == ParamRowState::Accepted);
  const auto chunks = warning_pkts(kLdpcWarning, 3);
  REQUIRE(chunks.size() == 2);
  for (const auto& c : chunks) r.c.on_packet(c.data(), c.size(), 2060);
  CHECK(r.c.state().rows[row].status == kLdpcWarning);
  CHECK(r.c.state().rows[row].st == ParamRowState::Accepted);

  // Outside the window after the ack: dropped, the row keeps its warning.
  for (const auto& c : warning_pkts("unrelated, much later", 4))
    r.c.on_packet(c.data(), c.size(), 2050 + kParamStatusMs + 1);
  CHECK(r.c.state().rows[row].status == kLdpcWarning);
}

TEST(a_chunked_warning_is_reassembled_and_a_gap_drops_it) {
  Rig r;
  link_and_complete_list(r, 1000);
  const int row = mabur::cfgparam::find("RADIO_LDPC");
  REQUIRE(row >= 0);
  std::string why;
  REQUIRE(r.c.set(static_cast<size_t>(row), "false", 2000, &why));
  // Chunks reach the pending row (the ack lost) whole, not chunk by chunk.
  auto chunks = warning_pkts(kLdpcWarning, 5);
  REQUIRE(chunks.size() == 2);
  r.c.on_packet(chunks[0].data(), chunks[0].size(), 2010);
  CHECK(r.c.state().rows[row].status.empty());
  r.c.on_packet(chunks[1].data(), chunks[1].size(), 2011);
  CHECK(r.c.state().rows[row].status == kLdpcWarning);

  // A text that fills its last chunk exactly ends in an empty chunk.
  const std::string hundred(100, 'x');
  chunks = warning_pkts(hundred, 6);
  REQUIRE(chunks.size() == 3);
  for (const auto& c : chunks) r.c.on_packet(c.data(), c.size(), 2020);
  CHECK(r.c.state().rows[row].status == hundred);

  // A lost middle chunk: the text is never delivered half.
  const std::string three(120, 'y');
  chunks = warning_pkts(three, 7);
  REQUIRE(chunks.size() == 3);
  r.c.on_packet(chunks[0].data(), chunks[0].size(), 2030);
  r.c.on_packet(chunks[2].data(), chunks[2].size(), 2031);
  CHECK(r.c.state().rows[row].status == hundred);
}

TEST(resend_stops_at_first_ack) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));
  REQUIRE(r.sent.size() == 1);

  r.c.tick(2000 + 2001);
  CHECK(r.sent.size() == 2);
  r.c.tick(2000 + 4002);
  CHECK(r.sent.size() == 3);
  r.c.tick(2000 + 6003);
  CHECK(r.sent.size() == 4);
  r.c.tick(2000 + 8004);
  CHECK(r.sent.size() == 5);
  CHECK(r.c.state().rows[4].st == ParamRowState::Timeout);
  CHECK(r.c.state().rows[4].result_ms == 2000 + 8004);  // stamped, so its age means something
  CHECK(r.c.state().pending_row == -1);
  CHECK(r.c.stats().timeouts == 1);
  {
    auto msgs = parse(r.sent[4]);
    REQUIRE(msgs.size() == 1);
    CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ);
    mavlink_param_ext_request_read_t p;
    mavlink_msg_param_ext_request_read_decode(&msgs[0], &p);
    CHECK(p.param_index == 4);
  }

  // An IN_PROGRESS right after the first send stops every resend.
  Rig r2;
  link_and_complete_list(r2, 1000);
  std::string why2;
  REQUIRE(r2.c.set(9, "1920x1080", 3000, &why2));
  REQUIRE(r2.sent.size() == 1);

  mabur::paramval::Value v{mabur::cfgparam::Type::Str};
  v.s = "1920x1080";
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes ip = ack_pkt("REC_SIZE", 11, wire, PARAM_ACK_IN_PROGRESS);
  r2.c.on_packet(ip.data(), ip.size(), 3010);
  r2.sent.clear();

  // Well past several would-be resend intervals, but short of the
  // IN_PROGRESS deadline: still no resend, because the row is no longer
  // Sending.
  r2.c.tick(3010 + 10000);
  CHECK(r2.sent.empty());
  CHECK(r2.c.state().rows[9].st == ParamRowState::InProgress);
}

TEST(final_deadline_from_first_in_progress) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 24000;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes ip1 = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_IN_PROGRESS);
  r.c.on_packet(ip1.data(), ip1.size(), 2000);  // the first IN_PROGRESS

  Bytes ip2 = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_IN_PROGRESS);
  r.c.on_packet(ip2.data(), ip2.size(), 2000 + 10000);  // a repeat, 10 s later

  r.sent.clear();
  r.c.tick(2000 + 15000 - 1);  // just before 15 s after the first IN_PROGRESS
  CHECK(r.c.state().rows[4].st == ParamRowState::InProgress);
  CHECK(r.sent.empty());

  r.c.tick(2000 + 15001);  // just after
  CHECK(r.c.state().rows[4].st == ParamRowState::Timeout);
  CHECK(r.c.state().rows[4].result_ms == 2000 + 15001);
  CHECK(r.c.state().pending_row == -1);
  CHECK(r.c.stats().timeouts == 1);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ);
}

TEST(accepted_rebuild_row_relists) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(2, "90@1920x1080", 2000, &why));
  r.sent.clear();

  mabur::paramval::Value v{mabur::cfgparam::Type::Str};
  v.s = "90@1920x1080";
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes a = ack_pkt("VENC_MODE", 11, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);

  CHECK(r.c.state().rows[2].st == ParamRowState::Accepted);
  CHECK(r.c.state().list == ParamListState::Pending);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) CHECK(r.c.state().rows[i].have_value);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST);
}

// The re-list after a rebuild starts with every row already holding a
// value. A row whose answer to that list is lost (here REC_SIZE, the one
// a VENC_MODE set can move) must still be re-read by index -- its held
// value stays displayed meanwhile, but it is not this round's answer.
TEST(relist_rereads_rows_that_did_not_answer_this_round) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(2, "120@1728x816", 2000, &why));
  mabur::paramval::Value v{mabur::cfgparam::Type::Str};
  v.s = "120@1728x816";
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes a = ack_pkt("VENC_MODE", 11, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);
  REQUIRE(r.c.state().list == ParamListState::Pending);
  r.sent.clear();

  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    if (i == 9) continue;  // REC_SIZE's value is lost on the way
    feed_row(r.c, i, 2100);
  }
  CHECK(r.c.state().list == ParamListState::Pending);
  CHECK(r.c.state().rows[9].have_value);  // the earlier value is still shown

  r.c.tick(2050 + 2001);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ);
  mavlink_param_ext_request_read_t p;
  mavlink_msg_param_ext_request_read_decode(&msgs[0], &p);
  CHECK(p.param_index == 9);

  feed_row(r.c, 9, 4100);
  CHECK(r.c.state().list == ParamListState::Complete);
}

TEST(accepted_knob_row_does_not_relist) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));
  r.sent.clear();

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30000;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes a = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);

  CHECK(r.c.state().rows[4].st == ParamRowState::Accepted);
  CHECK(r.c.state().list == ParamListState::Complete);
  CHECK(r.sent.empty());
}

TEST(ack_for_other_row_updates_value_only) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));

  mabur::paramval::Value v{mabur::cfgparam::Type::Str};
  v.s = "1280x720";
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes a = ack_pkt("REC_SIZE", 11, wire, PARAM_ACK_ACCEPTED);
  r.c.on_packet(a.data(), a.size(), 2050);

  CHECK(r.c.state().rows[9].value.s == "1280x720");
  CHECK(r.c.state().rows[9].st == ParamRowState::Idle);
  CHECK(r.c.state().pending_row == 4);
}

TEST(link_down_mid_set_clears) {
  Rig r;
  link_and_complete_list(r, 1000);
  std::string why;
  REQUIRE(r.c.set(4, "30000", 2000, &why));

  r.c.set_linked(false, 2100);

  const ParamClientState def{};
  CHECK(r.c.state().linked == def.linked);
  CHECK(r.c.state().pending_row == def.pending_row);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    CHECK(r.c.state().rows[i].have_value == def.rows[i].have_value);
    CHECK(r.c.state().rows[i].st == def.rows[i].st);
  }
}

MTEST_MAIN
