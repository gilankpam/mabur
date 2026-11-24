#include <cstdio>
#include <cstring>

#include "mabur/ota_mav.h"
#include "mtest.h"

using namespace mabur::ota;
using mabur::mav::kChanGsLocal;

namespace {

constexpr uint8_t kSys = 1, kComp = 191;

std::optional<mavlink_message_t> parse_one(const std::vector<uint8_t>& bytes) {
  mavlink_message_t rx{}, out{};
  mavlink_status_t st{};
  bool got = false;
  for (uint8_t b : bytes)
    if (mavlink_frame_char_buffer(&rx, &st, b, &out, nullptr) == MAVLINK_FRAMING_OK) got = true;
  if (!got) return std::nullopt;
  return out;
}

std::vector<uint8_t> unhex(const char* h) {
  std::vector<uint8_t> v;
  for (size_t i = 0; h[i] && h[i + 1]; i += 2) {
    unsigned x;
    std::sscanf(h + i, "%2x", &x);
    v.push_back(static_cast<uint8_t>(x));
  }
  return v;
}

template <class T>
T rt(const T& m) {
  auto b = encode(OtaMsg{m}, kSys, kComp, kChanGsLocal);
  auto all = decode_all(b.data(), b.size());
  REQUIRE(all.size() == 1);
  auto* g = std::get_if<T>(&all[0]);
  REQUIRE(g != nullptr);
  return *g;
}

Ftp ftp_of(size_t n) {
  Ftp f;
  f.seq = 0x1234; f.session = 3; f.opcode = 7; f.req_opcode = 6; f.burst_complete = 1;
  f.offset = 0xA0B0C0D0;
  for (size_t i = 0; i < n; ++i) f.data.push_back(static_cast<uint8_t>(i + 1));
  f.size = static_cast<uint8_t>(n);
  return f;
}

mabur::Sha256 sha_of() {
  mabur::Sha256 s{};
  for (size_t i = 0; i < 32; ++i) s[i] = static_cast<uint8_t>(0xA0 + i);
  return s;
}

// Compare ignoring byte 4 (per-channel sequence) and the trailing CRC it feeds.
bool same_masked(std::vector<uint8_t> a, std::vector<uint8_t> b) {
  if (a.size() != b.size() || a.size() < 5) return false;
  a[4] = b[4] = 0;
  a[a.size() - 1] = a[a.size() - 2] = b[b.size() - 1] = b[b.size() - 2] = 0;
  return a == b;
}

}  // namespace

TEST(every_message_round_trips) {
  rt(IdentReq{});
  Ident i{"v1.2.3-abc", "board-x", "rtl8812eu"};
  auto gi = rt(i);
  CHECK(gi.version == i.version && gi.board == i.board && gi.chip == i.chip);
  UpdateCmd u{Cmd::kStart, sha_of()};
  auto gu = rt(u);
  CHECK(gu.cmd == Cmd::kStart && gu.sha256 == u.sha256);
  Status s{State::kTrial, Reason::kNoVideo, 37};
  auto gs = rt(s);
  CHECK(gs.state == s.state && gs.reason == s.reason && gs.trial_s == 37);
  for (size_t n : {size_t(0), size_t(1), size_t(100), kFtpDataMax}) {
    Ftp f = ftp_of(n);
    auto g = rt(f);
    CHECK(g.seq == f.seq && g.session == f.session && g.opcode == f.opcode);
    CHECK(g.size == f.size && g.req_opcode == f.req_opcode);
    CHECK(g.burst_complete == f.burst_complete && g.offset == f.offset);
    CHECK(g.data == f.data);
  }
}

TEST(frozen_ota_goldens) {
  // FROZEN: a failing case is a broken peer, not a stale golden.
  const char* kIdentReq = "fd0100000801bf66ea0000a68f";
  const char* kIdent = "fd5600000901bf67ea00302e312e322d7465737400000000000000000000000000000000000000000000626f6172642d610000000000000000000000000000000000000000000000000000000000000000000000000000000000636869702d62b527";
  const char* kUpdate = "fd2100000a01bf68ea0001a0a1a2a3a4a5a6a7a8a9aaabacadaeafb0b1b2b3b4b5b6b7b8b9babbbcbdbebf0a93";
  const char* kStatus = "fd0300000b01bf69ea0004021d9437";
  auto enc = [](const OtaMsg& m) { return encode(m, kSys, kComp, kChanGsLocal); };
  auto b0 = enc(IdentReq{});
  auto b1 = enc(Ident{"0.1.2-test", "board-a", "chip-b"});
  auto b2 = enc(UpdateCmd{Cmd::kStart, sha_of()});
  auto b3 = enc(Status{State::kRolledBack, Reason::kNoLink, 29});
  CHECK(same_masked(b0, unhex(kIdentReq)));
  CHECK(same_masked(b1, unhex(kIdent)));
  CHECK(same_masked(b2, unhex(kUpdate)));
  CHECK(same_masked(b3, unhex(kStatus)));

  auto m0 = parse_one(unhex(kIdentReq));
  REQUIRE(m0.has_value());
  CHECK(m0->msgid == MAVLINK_MSG_ID_MABUR_IDENT_REQ && m0->sysid == kSys && m0->compid == kComp);
  auto m1 = parse_one(unhex(kIdent));
  REQUIRE(m1.has_value());
  CHECK(m1->msgid == 60007);
  {
    auto d = decode(*m1);
    REQUIRE(d.has_value());
    auto* g = std::get_if<Ident>(&*d);
    REQUIRE(g != nullptr);
    CHECK(g->version == "0.1.2-test" && g->board == "board-a" && g->chip == "chip-b");
  }
  auto m2 = parse_one(unhex(kUpdate));
  REQUIRE(m2.has_value());
  CHECK(m2->msgid == 60008);
  {
    auto g = std::get<UpdateCmd>(*decode(*m2));
    CHECK(g.cmd == Cmd::kStart && g.sha256 == sha_of());
  }
  auto m3 = parse_one(unhex(kStatus));
  REQUIRE(m3.has_value());
  CHECK(m3->msgid == 60009);
  {
    auto g = std::get<Status>(*decode(*m3));
    CHECK(g.state == State::kRolledBack && g.reason == Reason::kNoLink && g.trial_s == 29);
  }
}

TEST(ident_strings_are_cut_not_overflowed) {
  Ident i{std::string(40, 'v'), std::string(60, 'b'), std::string(20, 'c')};
  auto g = rt(i);
  CHECK(g.version == std::string(31, 'v'));
  CHECK(g.board == std::string(47, 'b'));
  CHECK(g.chip == std::string(15, 'c'));
}

TEST(uplink_lane_allowlist) {
  auto e = [](const OtaMsg& m) { return encode(m, kSys, kComp, kChanGsLocal); };
  auto ok = [](const std::vector<uint8_t>& b) { return all_packets_uplink_lane(b.data(), b.size()); };
  auto req = e(IdentReq{});
  auto upd = e(UpdateCmd{Cmd::kStart, sha_of()});
  CHECK(ok(req));
  CHECK(ok(upd));
  CHECK(ok(e(ftp_of(10))));
  CHECK(!ok(e(Ident{"a", "b", "c"})));
  mavlink_message_t msg;
  char id[16] = "X";
  char val[128] = "1";
  mavlink_msg_param_ext_set_pack_chan(kSys, kComp, kChanGsLocal, &msg, 1, 1, id, val, 1);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  std::vector<uint8_t> pes(buf, buf + mavlink_msg_to_send_buffer(buf, &msg));
  CHECK(!ok(pes));
  auto both = upd;
  both.insert(both.end(), pes.begin(), pes.end());
  CHECK(!ok(both));
  auto trunc = upd;
  trunc.insert(trunc.end(), req.begin(), req.end() - 2);
  CHECK(!ok(trunc));
  CHECK(!all_packets_uplink_lane(nullptr, 0));
  CHECK(is_uplink_lane_msgid(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL));
  CHECK(!is_uplink_lane_msgid(MAVLINK_MSG_ID_MABUR_UPDATE_STATUS));
}

TEST(reason_names_round_trip) {
  CHECK(std::string(reason_name(Reason::kNone)) == "");
  for (int r = 1; r <= 8; ++r) {
    const char* n = reason_name(static_cast<Reason>(r));
    CHECK(n[0] != 0);
    auto back = reason_from_name(n);
    REQUIRE(back.has_value());
    CHECK(*back == static_cast<Reason>(r));
  }
  CHECK(!reason_from_name("bogus").has_value());
  CHECK(std::string(reason_name(Reason::kWrongBoard)) == "wrong_board");
}

MTEST_MAIN
