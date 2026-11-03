// ParamHost: the hosted ParamClient's link policy, its sideport block and
// the `vtx set` verb.
#include <cstring>
#include <vector>

#include "mabur/config_params.h"
#include "mabur/mav.h"
#include "mabur/param_value.h"
#include "mtest.h"
#include "param_host.h"
#include "param_pkts.h"

using namespace maburgs;

namespace {

std::vector<mavlink_message_t> parse(const Bytes& b) {
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

struct Rig {
  std::vector<Bytes> sent;
  ParamHost h{[this](const uint8_t* p, size_t n) { sent.emplace_back(p, p + n); }};
};

// Builds and feeds a PARAM_EXT_VALUE for row `idx` with a plausible value.
void feed_row(Rig& r, size_t idx, uint64_t now_ms, uint16_t count = mabur::cfgparam::kCount) {
  const auto& row = mabur::cfgparam::kTable[idx];
  const auto v = sample_value(idx);
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  const Bytes bytes = value_pkt(row.id, mabur::cfgparam::mav_type(row.type), wire, count,
                                static_cast<uint16_t>(idx));
  r.h.on_packet(bytes.data(), bytes.size(), now_ms);
}

// Links `r.h` and feeds every row a value so the list reaches Complete,
// then clears `sent`/the log lines so a test sees only its own traffic.
void link_and_complete_list(Rig& r, uint32_t nonce, uint64_t now_ms) {
  r.h.tick(true, nonce, now_ms);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) feed_row(r, i, now_ms + 1);
  r.sent.clear();
  r.h.take_log_lines();
}

}  // namespace

TEST(a_session_edge_lists_and_a_drop_clears) {
  Rig r;
  r.h.tick(true, 0xA1, 1000);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST);
  CHECK(r.h.client().state().linked);

  r.h.tick(false, 0, 2000);
  CHECK(!r.h.client().state().linked);
  for (size_t i = 0; i < mabur::cfgparam::kCount; ++i) {
    CHECK(!r.h.client().state().rows[i].have_value);
  }
}

TEST(a_nonce_change_while_linked_relists) {
  Rig r;
  link_and_complete_list(r, 0xA1, 1000);
  REQUIRE(r.h.client().state().list == ParamListState::Complete);

  r.h.tick(true, 0xB2, 3000);
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST);
  CHECK(r.h.client().state().list == ParamListState::Pending);

  r.sent.clear();
  r.h.tick(true, 0xB2, 3100);
  CHECK(r.sent.empty());
}

TEST(the_sideport_block_mirrors_the_client) {
  Rig r;
  link_and_complete_list(r, 0xA1, 1000);

  StatsVtxIn s = r.h.sideport(5000);
  CHECK(s.present);
  CHECK(s.linked);
  CHECK(std::string(s.list) == "complete");
  REQUIRE(s.rows.size() == mabur::cfgparam::kCount);
  CHECK(std::string(s.rows[2].id) == "VENC_MODE");
  REQUIRE(s.rows[2].value.has_value());
  CHECK(*s.rows[2].value == mabur::paramval::format_text(sample_value(2)));
  CHECK(std::string(s.rows[2].st) == "idle");
  CHECK(!s.rows[2].age_ms.has_value());
  CHECK(!s.alive.has_value());  // no heartbeat yet

  Bytes hb = hb_pkt();
  r.h.on_packet(hb.data(), hb.size(), 5000);
  s = r.h.sideport(5000);
  REQUIRE(s.alive.has_value());
  CHECK(*s.alive == true);

  // A value_pkt whose count disagrees with the table's own row count flags
  // table_mismatch, same as the client itself.
  feed_row(r, mabur::cfgparam::kVencModeRow, 5100, /*count=*/13);
  s = r.h.sideport(5100);
  CHECK(s.table_mismatch);
}

TEST(vtx_set_sends_and_answers_ok_then_the_ack_lands_in_the_block) {
  Rig r;
  link_and_complete_list(r, 0xA1, 1000);

  const auto reply = r.h.handle_verb("vtx", "set ENC_BITRATE_MAX 12000", 2000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "ok");
  REQUIRE(r.sent.size() == 1);
  auto msgs = parse(r.sent[0]);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_SET);

  StatsVtxIn s = r.h.sideport(2000);
  CHECK(s.pending == 4);
  CHECK(std::string(s.rows[4].st) == "sending");

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 12000;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes ack = ack_pkt("ENC_BITRATE_MAX", 6, wire, PARAM_ACK_ACCEPTED);
  r.h.on_packet(ack.data(), ack.size(), 2000);

  s = r.h.sideport(2000);
  CHECK(std::string(s.rows[4].st) == "accepted");
  REQUIRE(s.rows[4].age_ms.has_value());
  CHECK(*s.rows[4].age_ms == 0);
  s = r.h.sideport(2500);
  REQUIRE(s.rows[4].age_ms.has_value());
  CHECK(*s.rows[4].age_ms == 500);

  const auto lines = r.h.take_log_lines();
  REQUIRE(lines.size() == 2);
  CHECK(lines[0] == "vtx set ENC_BITRATE_MAX 12000");
  CHECK(lines[1] == "vtx ENC_BITRATE_MAX accepted");
}

TEST(vtx_set_refusals_are_the_clients_reasons) {
  Rig r;
  auto reply = r.h.handle_verb("vtx", "set ENC_BITRATE_MAX 12000", 1000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "err not linked");

  link_and_complete_list(r, 0xA1, 1000);

  reply = r.h.handle_verb("vtx", "set NOPE 1", 2000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "err no such parameter");

  // ParamClient::set's own type_name() spells it lowercase ("bool", not
  // "Bool") -- the refusal text below is that function's actual wording.
  reply = r.h.handle_verb("vtx", "set RADIO_LDPC maybe", 2000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "err bad value for bool");

  reply = r.h.handle_verb("vtx", "set ENC_BITRATE_MAX 12000", 2000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "ok");

  reply = r.h.handle_verb("vtx", "set ENC_AIRTIME 0.5", 2001);
  REQUIRE(reply.has_value());
  CHECK(*reply == "err apply in progress");

  reply = r.h.handle_verb("vtx", "get X", 2001);
  REQUIRE(reply.has_value());
  CHECK(*reply == "err usage: vtx set <PARAM_ID> <value>");

  CHECK(!r.h.handle_verb("foo", "bar", 2001).has_value());
}

TEST(a_failed_set_logs_the_status) {
  Rig r;
  link_and_complete_list(r, 0xA1, 1000);

  const auto reply = r.h.handle_verb("vtx", "set REC_FPS 5", 2000);
  REQUIRE(reply.has_value());
  CHECK(*reply == "ok");
  r.h.take_log_lines();  // discard the submit line; only the terminal one matters below

  Bytes s1 = status_pkt("record.fps: must be in [1, venc.fps]");
  r.h.on_packet(s1.data(), s1.size(), 2010);  // the ack hasn't landed yet

  mabur::paramval::Value v{mabur::cfgparam::Type::Int32};
  v.i = 30;
  char wire[128];
  mabur::paramval::encode_wire(v, wire);
  Bytes fail = ack_pkt("REC_FPS", 6, wire, PARAM_ACK_FAILED);
  r.h.on_packet(fail.data(), fail.size(), 2050);

  const auto lines = r.h.take_log_lines();
  REQUIRE(!lines.empty());
  CHECK(lines.back() == "vtx REC_FPS failed record.fps: must be in [1, venc.fps]");
}

MTEST_MAIN
