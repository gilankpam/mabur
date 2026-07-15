#include "mtest.h"
#include "mabur/msp_dp.h"
#include <string>
using namespace mabur;

// Build one MSP v1 frame ($M< size cmd payload xor-checksum).
static std::vector<uint8_t> msp_frame(uint8_t cmd, std::vector<uint8_t> payload) {
  std::vector<uint8_t> f = {'$', 'M', '<', static_cast<uint8_t>(payload.size()), cmd};
  uint8_t cks = static_cast<uint8_t>(payload.size()) ^ cmd;
  for (uint8_t b : payload) { f.push_back(b); cks ^= b; }
  f.push_back(cks);
  return f;
}
// DisplayPort DRAW_STRING: sub=3, row, col, attrs, chars...
static std::vector<uint8_t> dp_draw_string(uint8_t row, uint8_t col, uint8_t attrs,
                                           const std::string& s) {
  std::vector<uint8_t> p = {3, row, col, attrs};
  for (char c : s) p.push_back(static_cast<uint8_t>(c));
  return msp_frame(182, p);
}

TEST(parser_recovers_message_across_split_feeds) {
  auto f = dp_draw_string(1, 2, 0, "HI");
  MspParser parser;
  // Feed byte-at-a-time; message completes only on the last byte.
  std::vector<MspMessage> msgs;
  for (size_t i = 0; i < f.size(); ++i) {
    auto out = parser.feed(&f[i], 1);
    for (auto& m : out) msgs.push_back(m);
  }
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].cmd == 182);
  REQUIRE(msgs[0].payload.size() == 6);   // 3,1,2,0,'H','I'
  CHECK(msgs[0].payload[0] == 3);
  CHECK(msgs[0].payload[4] == 'H');
}

TEST(parser_resyncs_after_garbage) {
  auto f = dp_draw_string(0, 0, 0, "X");
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x13};  // leading garbage bytes
  stream.insert(stream.end(), f.begin(), f.end());
  MspParser parser;
  auto msgs = parser.feed(stream.data(), stream.size());
  REQUIRE(msgs.size() == 1);
  CHECK(msgs[0].payload[0] == 3);
}

TEST(parser_drops_bad_checksum) {
  auto f = dp_draw_string(0, 0, 0, "X");
  f.back() ^= 0xFF;  // corrupt checksum
  MspParser parser;
  auto msgs = parser.feed(f.data(), f.size());
  CHECK(msgs.empty());
}

TEST(screen_draw_clear_and_complete) {
  MspScreen s;                              // default 50x18
  CHECK(s.cols() == 50);
  CHECK(s.rows() == 18);
  MspParser parser;
  auto draw = dp_draw_string(3, 5, 0, "AB");
  for (auto& m : parser.feed(draw.data(), draw.size()))
    CHECK(s.apply(m) == false);             // DRAW_STRING is not "complete"
  CHECK(s.cell(3, 5) == 'A');
  CHECK(s.cell(3, 6) == 'B');

  auto screen = msp_frame(182, {4});        // DRAW_SCREEN
  MspParser p2;
  bool complete = false;
  for (auto& m : p2.feed(screen.data(), screen.size())) complete = s.apply(m);
  CHECK(complete == true);

  auto clear = msp_frame(182, {2});         // CLEAR
  MspParser p3;
  for (auto& m : p3.feed(clear.data(), clear.size())) s.apply(m);
  CHECK(s.cell(3, 5) == 0);
}

TEST(serialize_snapshot_roundtrips_through_a_fresh_screen) {
  MspScreen a;
  MspParser parser;
  for (auto& blob : {dp_draw_string(1, 0, 0, "HELLO"),
                     dp_draw_string(2, 3, 0, "WORLD")})
    for (auto& m : parser.feed(blob.data(), blob.size())) a.apply(m);

  bool trunc = false;
  auto snap = a.serialize_snapshot(4096, &trunc);
  CHECK(trunc == false);
  REQUIRE(!snap.empty());

  // Replaying the snapshot into a blank screen reproduces the cells.
  MspScreen b;
  MspParser p2;
  for (auto& m : p2.feed(snap.data(), snap.size())) b.apply(m);
  CHECK(b.cell(1, 0) == 'H');
  CHECK(b.cell(1, 4) == 'O');
  CHECK(b.cell(2, 3) == 'W');
  CHECK(b.cell(2, 7) == 'D');
  CHECK(b.cell(0, 0) == 0);
}

TEST(serialize_snapshot_truncates_under_tight_budget) {
  MspScreen a;
  MspParser parser;
  auto blob = dp_draw_string(0, 0, 0, "ABCDEFGHIJ");
  for (auto& m : parser.feed(blob.data(), blob.size())) a.apply(m);
  bool trunc = false;
  auto snap = a.serialize_snapshot(8, &trunc);   // absurdly small
  CHECK(trunc == true);
  CHECK(snap.size() <= 8);
}

MTEST_MAIN
