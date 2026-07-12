#include <cstdio>
#include <vector>
#include "mtest.h"
#include "frame_file_source.h"
using namespace maburgs;

// Build a synthetic 3-frame file: 8-byte radiotap, 24-byte dot11 with a
// known seq, 5-byte body = {frame#, 'B','O','D','Y'}.
static std::string write_frames() {
  std::string path = "/tmp/maburgs_test_frames.bin";
  FILE* f = fopen(path.c_str(), "wb");
  for (uint8_t i = 0; i < 3; ++i) {
    std::vector<uint8_t> frame(8 + 24 + 5, 0);
    frame[2] = 8;                                   // radiotap len LE
    uint16_t seq_ctrl = static_cast<uint16_t>((100 + i) << 4);
    frame[8 + 22] = static_cast<uint8_t>(seq_ctrl & 0xFF);
    frame[8 + 23] = static_cast<uint8_t>(seq_ctrl >> 8);
    frame[8 + 24] = i; frame[8 + 25] = 'B'; frame[8 + 26] = 'O';
    frame[8 + 27] = 'D'; frame[8 + 28] = 'Y';
    uint32_t len = static_cast<uint32_t>(frame.size());
    fwrite(&len, 4, 1, f);
    fwrite(frame.data(), 1, frame.size(), f);
  }
  fclose(f);
  return path;
}

TEST(parses_frames_and_strips_headers) {
  FrameFileSource src(write_frames(), FrameFileSource::Options{1, 0, 1});
  REQUIRE(src.ok());
  for (uint8_t i = 0; i < 3; ++i) {
    auto m = src.next();
    REQUIRE(m.has_value());
    CHECK(m->card_id == 0);
    CHECK(m->mac_seq == 100 + i);
    CHECK(m->body.size() == 5);
    CHECK(m->body[0] == i);
    CHECK(m->crc_ok);
  }
  CHECK(!src.next().has_value());
  CHECK(src.frames_read() == 3);
}

TEST(two_cards_duplicate_in_card_order) {
  FrameFileSource src(write_frames(), FrameFileSource::Options{2, 0, 1});
  for (uint8_t i = 0; i < 3; ++i) {
    auto a = src.next(); auto b = src.next();
    REQUIRE(a && b);
    CHECK(a->card_id == 0); CHECK(b->card_id == 1);
    CHECK(a->mac_seq == b->mac_seq);
    CHECK(a->body == b->body);
  }
  CHECK(!src.next().has_value());
}

TEST(drops_are_deterministic_and_independent_per_card) {
  auto run = [&](uint32_t seed) {
    FrameFileSource src(write_frames(), FrameFileSource::Options{2, 50, seed});
    std::vector<std::pair<int, uint16_t>> got;
    while (auto m = src.next()) got.push_back({m->card_id, m->mac_seq});
    return got;
  };
  CHECK(run(7) == run(7));           // deterministic
  FrameFileSource src(write_frames(), FrameFileSource::Options{2, 100, 3});
  CHECK(!src.next().has_value());    // 100% drop -> nothing
  CHECK(src.dropped() == 6);
}

TEST(mono_us_same_per_air_frame_across_cards) {
  FrameFileSource src(write_frames(), FrameFileSource::Options{2, 0, 1});
  uint64_t prev_mono_us = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    auto card0 = src.next();
    auto card1 = src.next();
    REQUIRE(card0 && card1);
    // Two cards emitting the same air frame get equal mono_us
    CHECK(card0->mono_us == card1->mono_us);
    // mono_us is strictly increasing across successive air frames
    CHECK(card0->mono_us > prev_mono_us);
    prev_mono_us = card0->mono_us;
  }
  // Sanity: mono_us > 0 for the first frame
  src = FrameFileSource(write_frames(), FrameFileSource::Options{2, 0, 1});
  auto first = src.next();
  REQUIRE(first);
  CHECK(first->mono_us > 0);
}
MTEST_MAIN
