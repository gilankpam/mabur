// MavComponent: the drone's MAVLink extended-parameter component over the
// thirteen cfgparam::kTable keys, against a real copy of the bundle config
// and the real loader.
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <future>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cal_apply.h"
#include "config.h"
#include "config_apply.h"
#include "config_write.h"
#include "mabur/config_params.h"
#include "mabur/mav.h"
#include "mabur/rc_proto.h"
#include "mav_component.h"
#include "mav_params.h"
#include "mtest.h"
#include "scratch.h"

using namespace mabur;

namespace {

constexpr uint8_t kGcsSys = 255, kGcsComp = 190;
// The test's own parser channel for replies; this TU's channel state is its
// own, but a channel the component never uses keeps that obvious.
constexpr uint8_t kChanTest = mav::kChanGsRx;

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void spit(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  f << content;
}

bool exists(const std::string& path) { return std::ifstream(path).good(); }

// A scratch copy of bundle/mabur.default.toml; removes the publisher's
// siblings (backup, scratch candidate) with it.
struct ScratchBundle {
  ScratchFile f{"test_mav_component", ".toml"};
  ScratchBundle() {
    const std::string t = slurp(std::string(MABUR_BUNDLE_DIR) + "/mabur.default.toml");
    REQUIRE(!t.empty());
    spit(f.path, t);
  }
  ~ScratchBundle() {
    std::remove((f.path + ".pre-param").c_str());
    std::remove((f.path + ".pre-param.tmp").c_str());
    std::remove((f.path + ".new").c_str());
  }
  const std::string& path() const { return f.path; }
};

std::vector<uint8_t> to_bytes(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  return std::vector<uint8_t>(buf, buf + n);
}

void feed(MavComponent& c, const mavlink_message_t& msg) {
  const auto b = to_bytes(msg);
  c.on_payload(b.data(), b.size());
}

// Parses one take_outbound buffer; REQUIREs that it holds whole packets only.
std::vector<mavlink_message_t> parse_all(const std::vector<uint8_t>& bytes) {
  std::vector<mavlink_message_t> out;
  mavlink_reset_channel_status(kChanTest);
  for (uint8_t b : bytes) {
    mavlink_message_t m;
    mavlink_status_t st;
    if (mavlink_parse_char(kChanTest, b, &m, &st)) out.push_back(m);
  }
  REQUIRE(mavlink_get_channel_status(kChanTest)->parse_state <= MAVLINK_PARSE_STATE_IDLE);
  return out;
}

std::vector<mavlink_message_t> drain(MavComponent& c) {
  std::vector<mavlink_message_t> all;
  for (;;) {
    std::vector<uint8_t> out;
    const int n = c.take_outbound(&out);
    if (n == 0) {
      CHECK(out.empty());
      break;
    }
    CHECK(out.size() <= rc::kMavMaxPayload);
    auto msgs = parse_all(out);
    CHECK(static_cast<int>(msgs.size()) == n);
    all.insert(all.end(), msgs.begin(), msgs.end());
  }
  CHECK(c.outbound_packets() == 0);
  return all;
}

void id16(const char* id, char out[16]) {
  std::memset(out, 0, 16);
  std::strncpy(out, id, 16);
}

std::string id_of(const char id[16]) { return std::string(id, strnlen(id, 16)); }

mavlink_message_t list_msg(uint8_t ts = mav::kSysId, uint8_t tc = mav::kCompId) {
  mavlink_message_t m;
  mavlink_msg_param_ext_request_list_pack(kGcsSys, kGcsComp, &m, ts, tc);
  return m;
}

mavlink_message_t read_msg(const char* id, int16_t index) {
  char pid[16];
  id16(id, pid);
  mavlink_message_t m;
  mavlink_msg_param_ext_request_read_pack(kGcsSys, kGcsComp, &m, mav::kSysId, mav::kCompId,
                                          pid, index);
  return m;
}

mavlink_message_t set_raw(const char* id, const char value[128], uint8_t type) {
  char pid[16];
  id16(id, pid);
  mavlink_message_t m;
  mavlink_msg_param_ext_set_pack(kGcsSys, kGcsComp, &m, mav::kSysId, mav::kCompId, pid, value,
                                 type);
  return m;
}

mavlink_message_t set_int(const char* id, int32_t v) {
  char w[128] = {};
  std::memcpy(w, &v, 4);
  return set_raw(id, w, MAV_PARAM_EXT_TYPE_INT32);
}

mavlink_message_t set_real(const char* id, float v) {
  char w[128] = {};
  std::memcpy(w, &v, 4);
  return set_raw(id, w, MAV_PARAM_EXT_TYPE_REAL32);
}

mavlink_message_t set_bool(const char* id, bool v) {
  char w[128] = {};
  w[0] = v ? 1 : 0;
  return set_raw(id, w, MAV_PARAM_EXT_TYPE_UINT8);
}

mavlink_message_t set_str(const char* id, const char* s) {
  char w[128] = {};
  std::strncpy(w, s, 127);
  return set_raw(id, w, MAV_PARAM_EXT_TYPE_CUSTOM);
}

int32_t wire_i32(const char v[128]) {
  int32_t i;
  std::memcpy(&i, v, 4);
  return i;
}

// One set round-trip: feeds the set, drains, REQUIREs a PARAM_EXT_ACK first.
struct SetReply {
  mavlink_param_ext_ack_t ack{};
  std::vector<mavlink_message_t> rest;
};

SetReply do_set(MavComponent& c, const mavlink_message_t& set) {
  feed(c, set);
  auto msgs = drain(c);
  REQUIRE(!msgs.empty());
  REQUIRE(msgs[0].msgid == MAVLINK_MSG_ID_PARAM_EXT_ACK);
  SetReply r;
  mavlink_msg_param_ext_ack_decode(&msgs[0], &r.ack);
  r.rest.assign(msgs.begin() + 1, msgs.end());
  return r;
}

std::string statustext(const mavlink_message_t& m, uint8_t* severity) {
  REQUIRE(m.msgid == MAVLINK_MSG_ID_STATUSTEXT);
  mavlink_statustext_t s;
  mavlink_msg_statustext_decode(&m, &s);
  if (severity) *severity = s.severity;
  CHECK(s.id == 0 && s.chunk_seq == 0);
  return std::string(s.text, strnlen(s.text, sizeof s.text));
}

std::string statustext_of(const mavlink_message_t& m) { return statustext(m, nullptr); }

mavlink_param_ext_ack_t ack_of(const mavlink_message_t& m) {
  REQUIRE(m.msgid == MAVLINK_MSG_ID_PARAM_EXT_ACK);
  mavlink_param_ext_ack_t a{};
  mavlink_msg_param_ext_ack_decode(&m, &a);
  return a;
}

// A fake ConfigApplier port: submit() records the row and decoded value and
// refuses with refuse_with when set; an accepted submit leaves it busy until
// take_result() hands the result over, as the real applier is; complete()
// makes the next take_result()
// return {ok, reason} and, for ok, also does the publish the real applier
// would -- read, mavparam::patch_row the submitted row, publish_config
// under lock_config_writes() -- so file-content assertions made after a
// set resolves still see the same bytes a real apply would have produced.
struct FakeApply {
  struct Submit {
    size_t row;
    mavparam::Value value;
  };
  std::string path;
  std::vector<Submit> submits;
  std::string refuse_with;
  bool has_result = false;
  bool busy = false;
  ConfigApplyResult result{};

  bool submit(size_t row, const mavparam::Value& value, std::string* why) {
    submits.push_back({row, value});
    if (!refuse_with.empty()) {
      if (why) *why = refuse_with;
      return false;
    }
    busy = true;
    return true;
  }

  bool take_result(ConfigApplyResult* out) {
    if (!has_result) return false;
    *out = result;
    has_result = false;
    busy = false;
    return true;
  }

  void complete(bool ok, const std::string& reason) {
    REQUIRE(!submits.empty());
    const Submit& s = submits.back();
    if (ok) {
      auto lock = lock_config_writes();
      std::string text;
      REQUIRE(read_text_file(path, &text));
      // mavparam::patch_row is the same function the real applier calls
      // against its own `live_`; here that's just what the file loads as,
      // since this fake never diverges from it.
      const Config current = load_config(path);
      std::string followed, err;
      REQUIRE(mavparam::patch_row(&text, s.row, s.value, current, &followed, &err));
      std::string publish_err;
      REQUIRE(publish_config(path, text, "pre-param", &publish_err) == PublishResult::Ok);
    }
    result = ConfigApplyResult{ok, reason, apply_path_for(s.row), 5};
    has_result = true;
  }

  ApplyPort port() {
    return ApplyPort{
        [this](size_t row, const mavparam::Value& value, std::string* why) {
          return submit(row, value, why);
        },
        [this](ConfigApplyResult* out) { return take_result(out); },
        [this] { return busy; },
    };
  }
};

std::vector<std::string> logs;
// Reset to a fresh state by every make(): the test that wants to drive it
// (check submits, call complete) reaches it as the plain file-scope `fake`.
FakeApply fake;

MavComponent make(const std::string& path) {
  fake = FakeApply{};
  fake.path = path;
  return MavComponent(path, fake.port(), [](const std::string& l) { logs.push_back(l); });
}

}  // namespace

TEST(list_returns_every_row_in_table_order) {
  ScratchBundle b;
  auto c = make(b.path());
  feed(c, list_msg());
  CHECK(c.outbound_packets() == cfgparam::kCount);
  auto msgs = drain(c);
  REQUIRE(msgs.size() == cfgparam::kCount);
  for (size_t i = 0; i < msgs.size(); ++i) {
    CHECK(msgs[i].msgid == MAVLINK_MSG_ID_PARAM_EXT_VALUE);
    CHECK(msgs[i].sysid == mav::kSysId && msgs[i].compid == mav::kCompId);
    mavlink_param_ext_value_t v;
    mavlink_msg_param_ext_value_decode(&msgs[i], &v);
    CHECK(v.param_count == cfgparam::kCount);
    CHECK(v.param_index == i);
    CHECK(id_of(v.param_id) == cfgparam::kTable[i].id);
    CHECK(v.param_type == cfgparam::mav_type(cfgparam::kTable[i].type));
    if (id_of(v.param_id) == "VENC_MODE") {
      CHECK(v.param_type == MAV_PARAM_EXT_TYPE_CUSTOM);
      CHECK(std::string(v.param_value) == "60@1920x1080");
    }
  }
  CHECK(c.stats().rx_packets == 1);
  CHECK(c.stats().drop_not_for_us == 0);
}

TEST(list_for_another_component_is_ignored) {
  ScratchBundle b;
  auto c = make(b.path());
  feed(c, list_msg(mav::kSysId, 1));
  CHECK(c.outbound_packets() == 0);
  CHECK(c.stats().drop_not_for_us == 1);
  feed(c, list_msg(2, mav::kCompId));
  CHECK(c.outbound_packets() == 0);
  CHECK(c.stats().drop_not_for_us == 2);
  // 0 is broadcast: both fields 0 is for us.
  feed(c, list_msg(0, 0));
  CHECK(c.outbound_packets() == cfgparam::kCount);
}

TEST(read_by_index_and_by_id) {
  ScratchBundle b;
  auto c = make(b.path());
  feed(c, read_msg("", 8));
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  mavlink_param_ext_value_t v;
  mavlink_msg_param_ext_value_decode(&msgs[0], &v);
  CHECK(id_of(v.param_id) == "REC_FPS");
  CHECK(v.param_index == 8 && v.param_count == cfgparam::kCount);
  CHECK(wire_i32(v.param_value) == 60);

  feed(c, read_msg("VENC_MODE", -1));
  msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  mavlink_msg_param_ext_value_decode(&msgs[0], &v);
  CHECK(id_of(v.param_id) == "VENC_MODE");
  CHECK(v.param_index == 2);
  CHECK(v.param_type == MAV_PARAM_EXT_TYPE_CUSTOM);
  CHECK(std::string(v.param_value) == "60@1920x1080");

  feed(c, read_msg("NOPE", -1));
  CHECK(drain(c).empty());
  feed(c, read_msg("", 12));   // one past the table
  CHECK(drain(c).empty());
  feed(c, read_msg("", 32767));
  CHECK(drain(c).empty());
}

TEST(set_rec_fps_writes_record_section_only_and_acks_new_value) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_int("REC_FPS", 30));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
  CHECK(id_of(r.ack.param_id) == "REC_FPS");
  CHECK(r.ack.param_type == MAV_PARAM_EXT_TYPE_INT32);
  CHECK(wire_i32(r.ack.param_value) == 60);   // the value still on record
  CHECK(r.rest.empty());
  fake.complete(true, "");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  const auto ack = ack_of(msgs[0]);
  CHECK(ack.param_result == PARAM_ACK_ACCEPTED);
  CHECK(id_of(ack.param_id) == "REC_FPS");
  CHECK(ack.param_type == MAV_PARAM_EXT_TYPE_INT32);
  CHECK(wire_i32(ack.param_value) == 30);
  const std::string t = slurp(b.path());
  CHECK(t.find("fps          = 30                 # at most venc.fps") != std::string::npos);
  CHECK(t.find("fps        = 60            # 1..120") != std::string::npos);   // [venc] fps
  CHECK(exists(b.path() + ".pre-param"));
  CHECK(c.stats().sets_in_progress == 1);
  CHECK(c.stats().sets_ok == 1 && c.stats().sets_refused == 0);
}

TEST(set_unknown_id_is_value_unsupported) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  auto r = do_set(c, set_int("NOPE", 30));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED);
  CHECK(id_of(r.ack.param_id) == "NOPE");
  bool zero = true;
  for (char ch : r.ack.param_value) zero = zero && ch == 0;
  CHECK(zero);
  CHECK(slurp(b.path()) == before);
  CHECK(c.stats().sets_refused == 1);
}

TEST(set_type_mismatch_is_value_unsupported_with_current_value) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  auto r = do_set(c, set_real("VENC_MODE", 30.0f));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED);
  CHECK(r.ack.param_type == MAV_PARAM_EXT_TYPE_CUSTOM);
  CHECK(std::string(r.ack.param_value) == "60@1920x1080");
  CHECK(slurp(b.path()) == before);
  // A bool byte other than 0/1 is malformed too.
  char w[128] = {};
  w[0] = 2;
  r = do_set(c, set_raw("MSP_ENABLE", w, MAV_PARAM_EXT_TYPE_UINT8));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED);
  CHECK(r.ack.param_value[0] == 1);
  CHECK(slurp(b.path()) == before);
  CHECK(c.stats().sets_refused == 2);
}

TEST(set_refused_by_loader_sends_failed_ack_then_statustext) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  const int writes = write_count_for_test();
  auto c = make(b.path());
  auto r = do_set(c, set_str("VENC_MODE", "100@1920x1080"));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED);
  CHECK(r.ack.param_type == MAV_PARAM_EXT_TYPE_CUSTOM);
  CHECK(std::string(r.ack.param_value) == "60@1920x1080");
  REQUIRE(r.rest.size() == 1);
  uint8_t sev = 0;
  const std::string text = statustext(r.rest[0], &sev);
  CHECK(sev == MAV_SEVERITY_ERROR);
  CHECK(text.rfind("venc.size", 0) == 0);   // prefix stripped: the field leads
  CHECK(text.find("config:") == std::string::npos);
  CHECK(slurp(b.path()) == before);
  CHECK(!exists(b.path() + ".new"));
  CHECK(write_count_for_test() == writes);
  CHECK(c.stats().sets_refused == 1 && c.stats().sets_ok == 0);
}

TEST(ordering_rules_are_enforced_by_the_loader) {
  ScratchBundle b;
  auto c = make(b.path());
  // A loader-rejected set never reaches the applier, so its ack is the
  // immediate FAILED; a legal one goes IN_PROGRESS first and is driven to
  // its final ACCEPTED here so the next (dependent) set sees the file it
  // produced.
  auto result = [&](const mavlink_message_t& m) {
    const auto r = do_set(c, m).ack.param_result;
    if (r != PARAM_ACK_IN_PROGRESS) return r;
    fake.complete(true, "");
    c.poll_results();
    auto msgs = drain(c);
    REQUIRE(msgs.size() == 1);
    return ack_of(msgs[0]).param_result;
  };
  // Bundle defaults: record.fps 60, so venc.fps can't drop below it first.
  CHECK(result(set_str("VENC_MODE", "30@1920x1080")) == PARAM_ACK_FAILED);
  CHECK(result(set_int("REC_FPS", 30)) == PARAM_ACK_ACCEPTED);
  CHECK(result(set_str("VENC_MODE", "30@1920x1080")) == PARAM_ACK_ACCEPTED);
  // 100 fps is the 1728x972 mode: size exceeds it at 1920x1080.
  CHECK(result(set_str("VENC_MODE", "100@1920x1080")) == PARAM_ACK_FAILED);
  CHECK(result(set_str("VENC_MODE", "100@1728x972")) == PARAM_ACK_ACCEPTED);
  // low_power needs msp for the arm state.
  CHECK(result(set_bool("LOWPWR_ENABLE", true)) == PARAM_ACK_ACCEPTED);
  CHECK(result(set_bool("MSP_ENABLE", false)) == PARAM_ACK_FAILED);
  CHECK(result(set_bool("LOWPWR_ENABLE", false)) == PARAM_ACK_ACCEPTED);
  CHECK(result(set_bool("MSP_ENABLE", false)) == PARAM_ACK_ACCEPTED);
  const Config cfg = load_config(b.path());
  CHECK(cfg.venc.core.fps == 100 && cfg.venc.core.width == 1728);
  CHECK(cfg.record.fps == 30 && !cfg.msp.enable && !cfg.low_power.enable);
}

TEST(set_same_value_is_accepted_and_file_unchanged) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  const int writes = write_count_for_test();
  auto r = do_set(c, set_int("REC_FPS", 60));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED && wire_i32(r.ack.param_value) == 60);
  r = do_set(c, set_int("REC_FPS", 60));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED && wire_i32(r.ack.param_value) == 60);
  CHECK(slurp(b.path()) == before);
  CHECK(write_count_for_test() == writes);
  CHECK(!exists(b.path() + ".pre-param"));
  CHECK(c.stats().sets_ok == 2);
}

TEST(set_real32_to_the_bundle_value_leaves_the_file_byte_identical) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  REQUIRE(before.find("airtime_budget     = 0.65  #") != std::string::npos);
  auto c = make(b.path());
  const int writes = write_count_for_test();
  auto r = do_set(c, set_real("ENC_AIRTIME", 0.65f));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED);
  float f;
  std::memcpy(&f, r.ack.param_value, 4);
  CHECK(f == 0.65f);
  CHECK(slurp(b.path()) == before);
  CHECK(write_count_for_test() == writes);
  CHECK(!exists(b.path() + ".pre-param"));
}

TEST(set_real32_equal_to_a_differently_spelled_line_writes_nothing) {
  ScratchBundle b;
  std::string t = slurp(b.path());
  const size_t at = t.find("airtime_budget     = 0.65  #");
  REQUIRE(at != std::string::npos);
  t.replace(at, std::strlen("airtime_budget     = 0.65"), "airtime_budget     = 0.650");
  spit(b.path(), t);
  auto c = make(b.path());
  const int writes = write_count_for_test();
  auto r = do_set(c, set_real("ENC_AIRTIME", 0.65f));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED);
  CHECK(slurp(b.path()) == t);
  CHECK(write_count_for_test() == writes);
  CHECK(c.stats().sets_ok == 1);
}

namespace {

std::vector<std::string> lines_of(const std::string& t) {
  std::vector<std::string> out;
  std::istringstream in(t);
  for (std::string l; std::getline(in, l);) out.push_back(l);
  return out;
}

// A PARAM_EXT_SET carrying `v` with its row's wire type.
mavlink_message_t set_value(const char* id, const mavparam::Value& v) {
  char w[128];
  mavparam::encode_wire(v, w);
  return set_raw(id, w, cfgparam::mav_type(v.type));
}

bool same(const mavparam::Value& a, const mavparam::Value& b) {
  return a.type == b.type && a.b == b.b && a.i == b.i && a.f == b.f && a.s == b.s;
}

mavparam::Value file_value(const std::string& path, size_t idx) {
  mavparam::Value v{cfgparam::kTable[idx].type};
  REQUIRE(mavparam::read(load_config(path), idx, &v));
  return v;
}

}  // namespace

// Every row: a set of the current value writes nothing; a set of another
// legal value (legal against the bundle with every other key unchanged)
// changes exactly one line and reads back; a set of the original value
// then restores the file byte for byte.
TEST(every_row_sets_reads_back_and_restores) {
  using mavparam::Value;
  using cfgparam::Type;
  struct Case {
    const char* id;
    Value other;
  };
  const Case cases[] = {
      {"RADIO_WALL_MARGN", Value{Type::Real32, false, 0, 2.0f}},
      {"RADIO_LDPC", Value{Type::Bool, false}},
      {"VENC_MODE", Value{Type::Str, false, 0, 0, "90@1280x720"}},
      {"VENC_SLICES", Value{Type::Int32, false, 2}},
      {"ENC_BITRATE_MAX", Value{Type::Int32, false, 20000}},
      {"ENC_AIRTIME", Value{Type::Real32, false, 0, 0.5f}},
      {"REC_ENABLE", Value{Type::Bool, false}},
      {"REC_BITRATE", Value{Type::Int32, false, 30000}},
      {"REC_FPS", Value{Type::Int32, false, 30}},
      {"REC_SIZE", Value{Type::Str, false, 0, 0, "1280x720"}},
      {"LOWPWR_ENABLE", Value{Type::Bool, true}},
      {"MSP_ENABLE", Value{Type::Bool, false}},
  };
  static_assert(sizeof(cases) / sizeof(cases[0]) == cfgparam::kCount, "one case per row");
  ScratchBundle b;
  const std::string bundle = slurp(b.path());
  const auto bundle_lines = lines_of(bundle);
  auto c = make(b.path());
  for (size_t idx = 0; idx < cfgparam::kCount; ++idx) {
    const Case& k = cases[idx];
    REQUIRE(std::string(cfgparam::kTable[idx].id) == k.id);
    const Value cur = file_value(b.path(), idx);
    REQUIRE(!same(cur, k.other));

    // (a) the current value: ACCEPTED, nothing written.
    const int writes = write_count_for_test();
    auto r = do_set(c, set_value(k.id, cur));
    CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED);
    CHECK(slurp(b.path()) == bundle);
    CHECK(write_count_for_test() == writes);

    // (b) another legal value: IN_PROGRESS first, then one line changed,
    // read back equal, once the applier's result is driven in.
    r = do_set(c, set_value(k.id, k.other));
    CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
    if (r.ack.param_result != PARAM_ACK_IN_PROGRESS) {
      for (const auto& m : r.rest)
        std::fprintf(stderr, "%s: %s\n", k.id, statustext(m, nullptr).c_str());
      continue;
    }
    fake.complete(true, "");
    c.poll_results();
    auto msgs = drain(c);
    REQUIRE(msgs.size() == 1);
    const auto ack = ack_of(msgs[0]);
    CHECK(ack.param_result == PARAM_ACK_ACCEPTED);
    const auto now_lines = lines_of(slurp(b.path()));
    REQUIRE(now_lines.size() == bundle_lines.size());
    int changed = 0;
    for (size_t i = 0; i < now_lines.size(); ++i) changed += now_lines[i] != bundle_lines[i];
    // VENC_MODE's one Str value spells both venc.fps and venc.size, so it
    // patches two lines; every other row still patches exactly one.
    CHECK(changed == (idx == cfgparam::kVencModeRow ? 2 : 1));
    CHECK(same(file_value(b.path(), idx), k.other));
    char want[128];
    mavparam::encode_wire(k.other, want);
    CHECK(std::memcmp(ack.param_value, want, sizeof want) == 0);

    // Restore: the file currently holds k.other, so this is a real change
    // again, not a same-value no-op.
    r = do_set(c, set_value(k.id, cur));
    CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
    fake.complete(true, "");
    c.poll_results();
    msgs = drain(c);
    REQUIRE(msgs.size() == 1);
    CHECK(ack_of(msgs[0]).param_result == PARAM_ACK_ACCEPTED);
    CHECK(slurp(b.path()) == bundle);
  }
}

// A calibration apply and parameter sets on two threads, same file: both
// hold the config-write lock across read -> patch -> publish, so neither
// loses the other's change and the file always loads.
TEST(calibration_and_parameter_sets_on_two_threads_lose_nothing) {
  ScratchBundle b;
  auto c = make(b.path());
  const std::array<int, 8> walls_a = {60, 60, 60, 45, 22, 6, 9, 5};
  const std::array<int, 8> walls_last = {58, 57, 56, 44, 21, 5, 8, 4};
  int cal_bad = 0, set_bad = 0;
  std::thread cal([&] {
    for (int i = 0; i < 20; ++i) {
      CalWrite w;
      w.walls = i == 19 ? walls_last : walls_a;
      w.legacy_wall = i == 19 ? 58 : 60;
      std::string err;
      if (apply_calibration(b.path(), w, 1.0, &err) != ApplyResult::Ok) ++cal_bad;
    }
  });
  std::thread set([&] {
    for (int i = 0; i < 20; ++i) {
      const int32_t kbps = i == 19 ? 30000 : (i % 2 ? 35000 : 25000);
      feed(c, set_int("REC_BITRATE", kbps));
      std::vector<uint8_t> out;
      c.take_outbound(&out);
      auto msgs = parse_all(out);
      mavlink_param_ext_ack_t ack{};
      if (msgs.size() != 1 || msgs[0].msgid != MAVLINK_MSG_ID_PARAM_EXT_ACK) {
        ++set_bad;
        continue;
      }
      mavlink_msg_param_ext_ack_decode(&msgs[0], &ack);
      if (ack.param_result != PARAM_ACK_IN_PROGRESS) {
        ++set_bad;
        continue;
      }
      // Drive the applier's result -- fake.complete() publishes under
      // lock_config_writes() too, the same as the real applier would from
      // its own thread -- then collect the final ack.
      fake.complete(true, "");
      c.poll_results();
      out.clear();
      c.take_outbound(&out);
      msgs = parse_all(out);
      if (msgs.size() != 1 || msgs[0].msgid != MAVLINK_MSG_ID_PARAM_EXT_ACK) {
        ++set_bad;
        continue;
      }
      mavlink_msg_param_ext_ack_decode(&msgs[0], &ack);
      if (ack.param_result != PARAM_ACK_ACCEPTED) ++set_bad;
    }
  });
  cal.join();
  set.join();
  CHECK(cal_bad == 0);
  CHECK(set_bad == 0);
  const Config cfg = load_config(b.path());
  CHECK(cfg.record.bitrate_kbps == 30000);
  CHECK(cfg.radio.rate_walls_rel_20 == walls_last);
  CHECK(cfg.radio.legacy_wall_rel == 58);
  CHECK(cfg.radio.power_mode == "offset");
  std::remove((b.path() + ".pre-cal").c_str());
  std::remove((b.path() + ".pre-cal.tmp").c_str());
}

TEST(commented_out_key_is_failed_with_statustext_naming_it) {
  ScratchBundle b;
  std::string t = slurp(b.path());
  const size_t msp = t.find("[msp]");
  REQUIRE(msp != std::string::npos);
  const size_t en = t.find("enable", msp);
  REQUIRE(en != std::string::npos);
  t.insert(en, "# ");
  spit(b.path(), t);
  auto c = make(b.path());
  auto r = do_set(c, set_bool("MSP_ENABLE", false));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED);
  CHECK(r.ack.param_value[0] == 0);   // what the live file loads as: the loader default, off
  REQUIRE(r.rest.size() == 1);
  const std::string text = statustext(r.rest[0], nullptr);
  CHECK(text.find("msp.enable") != std::string::npos);
  CHECK(slurp(b.path()) == t);
  CHECK(c.stats().sets_refused == 1);
}

TEST(custom_value_with_quote_or_no_nul_never_reaches_the_file) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  auto r = do_set(c, set_str("REC_SIZE", "19\"20x1080"));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED);
  CHECK(std::string(r.ack.param_value) == "1920x1080");
  char w[128];
  std::memset(w, 'A', sizeof w);   // no NUL anywhere
  r = do_set(c, set_raw("REC_SIZE", w, MAV_PARAM_EXT_TYPE_CUSTOM));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED);
  CHECK(slurp(b.path()) == before);
  CHECK(!exists(b.path() + ".pre-param"));
}

TEST(take_outbound_never_exceeds_600_bytes_and_never_splits_a_packet) {
  ScratchBundle b;
  auto c = make(b.path());
  feed(c, list_msg());
  size_t total = 0;
  int takes = 0;
  for (;;) {
    std::vector<uint8_t> out;
    const int n = c.take_outbound(&out);
    if (n == 0) break;
    ++takes;
    CHECK(out.size() <= rc::kMavMaxPayload);
    CHECK(parse_all(out).size() == static_cast<size_t>(n));
    total += static_cast<size_t>(n);
  }
  CHECK(total == cfgparam::kCount);
  CHECK(takes > 1);
  // A caller's partly filled buffer only gets what still fits.
  feed(c, list_msg());
  std::vector<uint8_t> out(rc::kMavMaxPayload - 10, 0);
  CHECK(c.take_outbound(&out) == 0);
  CHECK(out.size() == rc::kMavMaxPayload - 10);
  CHECK(c.outbound_packets() == cfgparam::kCount);
}

TEST(request_lists_coalesce_while_a_list_is_queued) {
  ScratchBundle b;
  auto c = make(b.path());
  std::vector<uint8_t> payload;
  const auto one = to_bytes(list_msg());
  for (int i = 0; i < 100; ++i) payload.insert(payload.end(), one.begin(), one.end());
  c.on_payload(payload.data(), payload.size());
  CHECK(c.stats().rx_packets == 100);
  CHECK(c.outbound_packets() <= kMavQueueCap);
  CHECK(c.outbound_packets() == cfgparam::kCount);
  CHECK(c.stats().list_coalesced == 99);
  CHECK(c.stats().drop_queue_full == 0);
  // Still coalesced after a partial drain; a fresh list once all are sent.
  std::vector<uint8_t> out;
  REQUIRE(c.take_outbound(&out) > 0);
  feed(c, list_msg());
  CHECK(c.stats().list_coalesced == 100);
  auto msgs = drain(c);
  CHECK(msgs.size() + parse_all(out).size() == cfgparam::kCount);
  feed(c, list_msg());
  CHECK(c.outbound_packets() == cfgparam::kCount);
  CHECK(c.stats().list_coalesced == 100);
}

// The cap applies to every packet alike: once the queue is full, a new
// heartbeat, value or set ack is dropped and counted; what is queued stays.
TEST(outbound_queue_is_capped_and_drops_the_new_packet) {
  ScratchBundle b;
  auto c = make(b.path());
  for (size_t i = 0; i < kMavQueueCap + 6; ++i) c.queue_heartbeat();
  CHECK(c.outbound_packets() == kMavQueueCap);
  CHECK(c.stats().drop_queue_full == 6);
  feed(c, read_msg("VENC_MODE", -1));
  CHECK(c.outbound_packets() == kMavQueueCap);
  CHECK(c.stats().drop_queue_full == 7);
  // A set on a full queue is still submitted; only its acks are dropped --
  // the immediate IN_PROGRESS, then the completion ack once it's driven in.
  feed(c, set_int("REC_FPS", 30));
  CHECK(c.stats().sets_in_progress == 1);
  CHECK(c.stats().drop_queue_full == 8);
  fake.complete(true, "");
  c.poll_results();
  CHECK(c.stats().sets_ok == 1);
  CHECK(c.stats().drop_queue_full == 9);
  CHECK(load_config(b.path()).record.fps == 30);
  auto msgs = drain(c);
  CHECK(msgs.size() == kMavQueueCap);
  for (const auto& m : msgs) CHECK(m.msgid == MAVLINK_MSG_ID_HEARTBEAT);
}

TEST(heartbeat_fields) {
  ScratchBundle b;
  auto c = make(b.path());
  c.queue_heartbeat();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].msgid == MAVLINK_MSG_ID_HEARTBEAT);
  CHECK(msgs[0].sysid == 1 && msgs[0].compid == 191);
  mavlink_heartbeat_t h;
  mavlink_msg_heartbeat_decode(&msgs[0], &h);
  CHECK(h.type == 18 && h.autopilot == 8 && h.base_mode == 0 && h.system_status == 4);
}

TEST(mavlink1_and_garbage_are_dropped) {
  ScratchBundle b;
  auto c = make(b.path());
  // A well-formed MAVLink 1 heartbeat: valid CRC, but v1 is dropped.
  mavlink_message_t m;
  mavlink_status_t* st = mavlink_get_channel_status(MAVLINK_COMM_0);
  st->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_msg_heartbeat_pack(kGcsSys, kGcsComp, &m, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);
  st->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  auto v1 = to_bytes(m);
  REQUIRE(v1[0] == MAVLINK_STX_MAVLINK1);
  c.on_payload(v1.data(), v1.size());
  CHECK(c.stats().drop_bad_packet == 1);
  CHECK(c.stats().rx_packets == 0);

  // A packet with a corrupted CRC, then a truncated one at the frame end.
  auto good = to_bytes(list_msg());
  auto bad = good;
  bad.back() ^= 0xFF;
  c.on_payload(bad.data(), bad.size());
  CHECK(c.stats().drop_bad_packet == 2);
  c.on_payload(good.data(), good.size() - 3);
  CHECK(c.stats().drop_bad_packet == 3);
  // The partial packet must not bleed into the next payload.
  c.on_payload(good.data(), good.size());
  CHECK(c.stats().drop_bad_packet == 3 && c.stats().rx_packets == 1);
  CHECK(c.outbound_packets() == cfgparam::kCount);
  drain(c);

  const uint8_t junk[] = {0x13, 0x37, 0xFD, 0x05, 0x00, 0x00, 0x01, 0x02, 0x03, 0xAA};
  c.on_payload(junk, sizeof junk);
  CHECK(c.stats().drop_bad_packet == 4);
  CHECK(c.outbound_packets() == 0);
  CHECK(c.stats().rx_packets == 1);
}

TEST(heartbeat_from_gcs_is_counted_and_ignored) {
  ScratchBundle b;
  auto c = make(b.path());
  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(kGcsSys, kGcsComp, &m, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);
  feed(c, m);
  CHECK(c.stats().rx_packets == 1);
  CHECK(c.stats().drop_not_for_us == 0);
  CHECK(c.outbound_packets() == 0);
  // A message with no handler here counts as not for us.
  mavlink_msg_param_request_list_pack(kGcsSys, kGcsComp, &m, 1, 191);
  feed(c, m);
  CHECK(c.stats().drop_not_for_us == 1);
}

TEST(log_lines_are_prefixed_and_null_log_is_allowed) {
  ScratchBundle b;
  logs.clear();
  auto c = make(b.path());
  (void)do_set(c, set_int("REC_FPS", 30));
  (void)do_set(c, set_str("VENC_MODE", "1x"));
  REQUIRE(!logs.empty());
  for (const auto& l : logs) {
    CHECK(l.rfind("[mav] ", 0) == 0);
    CHECK(l.find('\n') == std::string::npos);
  }
  FakeApply quiet_fake;
  quiet_fake.path = b.path();
  MavComponent quiet(b.path(), quiet_fake.port(), nullptr);
  auto r = do_set(quiet, set_int("REC_FPS", 45));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
  quiet_fake.complete(true, "");
  quiet.poll_results();
  auto msgs = drain(quiet);
  REQUIRE(msgs.size() == 1);
  CHECK(ack_of(msgs[0]).param_result == PARAM_ACK_ACCEPTED);
}

TEST(set_acks_in_progress_then_accepted_on_completion) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_int("REC_FPS", 30));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS && wire_i32(r.ack.param_value) == 60);
  CHECK(fake.submits.size() == 1 && fake.submits[0].row == 8 && fake.submits[0].value.i == 30);
  // handle_set's own candidate check writes and removes <path>.new; nothing
  // is left behind for a submit the applier is about to take over.
  CHECK(!exists(b.path() + ".new"));
  fake.complete(true, "");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  const auto ack = ack_of(msgs[0]);
  CHECK(ack.param_result == PARAM_ACK_ACCEPTED && wire_i32(ack.param_value) == 30);
  CHECK(c.stats().sets_in_progress == 1 && c.stats().sets_ok == 1);
}

TEST(set_acks_failed_with_statustext_on_failed_completion) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_int("REC_FPS", 30));
  REQUIRE(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
  fake.complete(false, "pipeline refused venc.size: restored previous");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 2);
  const auto ack = ack_of(msgs[0]);
  CHECK(ack.param_result == PARAM_ACK_FAILED && wire_i32(ack.param_value) == 60);   // current value
  CHECK(statustext_of(msgs[1]) == "pipeline refused venc.size: restored previous");
  CHECK(c.stats().sets_refused == 1);
}

TEST(submit_refusal_is_failed_at_once) {
  ScratchBundle b;
  auto c = make(b.path());
  fake.refuse_with = "apply in progress";
  auto r = do_set(c, set_int("REC_FPS", 30));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED && r.rest.size() == 1 &&
        statustext_of(r.rest[0]) == "apply in progress");
}

TEST(unchanged_value_never_submits) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_int("REC_FPS", 60));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED && fake.submits.empty());
}

TEST(decode_rejected_set_never_submits) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_str("VENC_MODE", "1x"));
  CHECK(r.ack.param_result == PARAM_ACK_VALUE_UNSUPPORTED && fake.submits.empty());
}

// Holds lock_config_writes() on its own thread until release(): a set that
// tried to take the lock would block behind it.
struct LockHolder {
  std::promise<void> held, go;
  std::thread t;
  LockHolder() {
    t = std::thread([this] {
      auto lock = lock_config_writes();
      held.set_value();
      go.get_future().wait();
    });
    held.get_future().wait();
  }
  void release() {
    if (t.joinable()) {
      go.set_value();
      t.join();
    }
  }
  ~LockHolder() { release(); }
};

// Runs one set against `c` on a worker thread while another thread holds
// the config-write lock; false when the set did not return within 2 s (it
// blocked on the lock).
bool set_without_lock(MavComponent& c, const mavlink_message_t& set, SetReply* r) {
  LockHolder holder;
  auto done = std::async(std::launch::async, [&] { feed(c, set); });
  const bool returned = done.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  holder.release();
  done.wait();
  auto msgs = drain(c);
  REQUIRE(!msgs.empty());
  r->ack = ack_of(msgs[0]);
  r->rest.assign(msgs.begin() + 1, msgs.end());
  return returned;
}

TEST(set_while_busy_is_refused_without_taking_the_lock) {
  ScratchBundle b;
  auto c = make(b.path());
  REQUIRE(do_set(c, set_int("REC_FPS", 30)).ack.param_result == PARAM_ACK_IN_PROGRESS);
  const std::string before = slurp(b.path());
  SetReply r;
  CHECK(set_without_lock(c, set_int("REC_BITRATE", 25000), &r));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED);
  REQUIRE(r.rest.size() == 1);
  CHECK(statustext_of(r.rest[0]) == "apply in progress");
  CHECK(fake.submits.size() == 1);   // never reached the applier
  CHECK(slurp(b.path()) == before);
  // A different value for the pending row is a different set: refused too.
  CHECK(set_without_lock(c, set_int("REC_FPS", 45), &r));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED && wire_i32(r.ack.param_value) == 60);
  CHECK(fake.submits.size() == 1);
  // The pending set still resolves normally.
  fake.complete(true, "");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  CHECK(ack_of(msgs[0]).param_result == PARAM_ACK_ACCEPTED);
  CHECK(wire_i32(ack_of(msgs[0]).param_value) == 30);
}

TEST(repeat_of_the_pending_set_is_acked_in_progress_again) {
  ScratchBundle b;
  auto c = make(b.path());
  REQUIRE(do_set(c, set_int("REC_FPS", 30)).ack.param_result == PARAM_ACK_IN_PROGRESS);
  const std::string before = slurp(b.path());
  SetReply r;
  CHECK(set_without_lock(c, set_int("REC_FPS", 30), &r));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS && wire_i32(r.ack.param_value) == 60);
  CHECK(r.rest.empty());   // no STATUSTEXT
  CHECK(fake.submits.size() == 1);
  CHECK(slurp(b.path()) == before);
  CHECK(!exists(b.path() + ".new"));
  CHECK(c.stats().sets_refused == 0);
  // One final ack answers both: the first completion resolves the set.
  fake.complete(true, "");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  CHECK(ack_of(msgs[0]).param_result == PARAM_ACK_ACCEPTED);
}

// The bundle fixture is 60@1920x1080 with the recorder on at an explicit
// 1920x1080 / 60 fps, so a VENC_MODE set that crosses the aspect ratio
// must carry record.size along in the same candidate (mavparam::patch_row,
// drone/src/mav_params.cpp).

TEST(venc_mode_set_crossing_aspect_patches_three_lines_in_one_set) {
  ScratchBundle b;
  auto c = make(b.path());
  auto r = do_set(c, set_str("VENC_MODE", "120@1728x816"));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);   // the loader accepted the candidate
  REQUIRE(fake.submits.size() == 1);
  CHECK(fake.submits[0].row == cfgparam::kVencModeRow);
  CHECK(fake.submits[0].value.s == "120@1728x816");
  fake.complete(true, "");
  c.poll_results();
  auto msgs = drain(c);
  REQUIRE(msgs.size() == 1);
  const auto ack = ack_of(msgs[0]);
  CHECK(ack.param_result == PARAM_ACK_ACCEPTED);
  CHECK(std::string(ack.param_value) == "120@1728x816");
  const std::string text = slurp(b.path());
  CHECK(text.find("size         = \"1728x816\"") != std::string::npos);   // [record]'s own line
}

TEST(venc_mode_fps_only_change_still_patches_both_lines) {
  ScratchBundle b;
  auto c = make(b.path());
  // Same aspect as the fixture's 1920x1080 record.size: no follow, but
  // still a real two-line change, never the unchanged shortcut.
  auto r = do_set(c, set_str("VENC_MODE", "90@1920x1080"));
  CHECK(r.ack.param_result == PARAM_ACK_IN_PROGRESS);
  REQUIRE(fake.submits.size() == 1);
  CHECK(fake.submits[0].value.s == "90@1920x1080");
}

TEST(venc_mode_lower_fps_refused_by_record_fps_rule) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  auto r = do_set(c, set_str("VENC_MODE", "30@1920x1080"));
  CHECK(r.ack.param_result == PARAM_ACK_FAILED);
  REQUIRE(r.rest.size() == 1);
  CHECK(statustext_of(r.rest[0]).rfind("record.fps", 0) == 0);
  CHECK(slurp(b.path()) == before);
  CHECK(fake.submits.empty());
}

TEST(venc_mode_equal_value_is_unchanged_shortcut) {
  ScratchBundle b;
  const std::string before = slurp(b.path());
  auto c = make(b.path());
  auto r = do_set(c, set_str("VENC_MODE", "60@1920x1080"));
  CHECK(r.ack.param_result == PARAM_ACK_ACCEPTED);
  CHECK(fake.submits.empty());
  CHECK(slurp(b.path()) == before);
}

MTEST_MAIN
