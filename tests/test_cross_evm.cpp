#include "mtest.h"
#include "cross_evm.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using mabur::CrossEvm;
using mabur::cross_evm;

namespace {

// A fixture holds the high dword of each capture word (tests/fixtures/crosspath/README.md).
std::vector<devourer::IqSample> load_hi32(const char* name) {
  std::ifstream f(std::string(MABUR_FIXTURE_DIR) + "/crosspath/" + name + ".hi32",
                  std::ios::binary);
  REQUIRE(f.good());
  std::vector<devourer::IqSample> s;
  uint8_t b[4];
  while (f.read(reinterpret_cast<char*>(b), 4)) {
    const uint32_t hi = b[0] | b[1] << 8 | b[2] << 16 | uint32_t(b[3]) << 24;
    s.push_back(devourer::cross_path_sample(uint64_t{hi} << 32));
  }
  return s;
}

struct Row {
  const char* name;
  uint8_t bw;
  bool frame;
  float evm_db, level_db, envelope_db;
  bool clipped;
  int flagged;
};

// Reference values: the spike's numpy measurement of the same captures.
const Row kRows[] = {
    {"c144_20_i028", 20, true, -30.60f, 50.16f, 51.20f, false, 0},
    {"c144_20_i060", 20, true, -16.78f, 57.06f, 58.01f, false, 0},
    {"c144_20_i124", 20, true, -8.12f, 59.60f, 60.07f, false, 0},
    {"c40_20_i060", 20, true, -32.00f, 52.77f, 53.97f, false, 0},
    {"c40_20_i060_weak", 20, true, -21.79f, 40.91f, 42.08f, false, 14},
    {"c144_20_i028_clip", 20, true, -22.38f, 61.86f, 62.98f, true, 0},
    {"c40_20_noise", 20, true, -2.80f, 12.30f, 15.06f, false, 94},
    {"c144_40_i028", 40, true, -29.86f, 49.20f, 50.20f, false, 0},
    {"c144_40_i076", 40, true, -11.85f, 58.68f, 59.16f, false, 0},
    {"c144_40_i124", 40, true, -7.27f, 59.84f, 60.04f, false, 0},
    {"c144_40_noframe", 40, false, 0, 0, 58.98f, false, 0},
};

} // namespace

TEST(matches_the_reference) {
  for (const Row& r : kRows) {
    const auto s = load_hi32(r.name);
    const CrossEvm e = cross_evm(s.data(), s.size(), r.bw);
    std::printf("  %-18s frame %d evm %7.2f level %6.2f env %6.2f clip %d flagged %d\n", r.name,
                e.frame, e.evm_db, e.level_db, e.envelope_db, e.clipped, e.flagged);
    CHECK(e.frame == r.frame);
    CHECK(std::fabs(e.envelope_db - r.envelope_db) <= 0.2f);
    CHECK(e.clipped == r.clipped);
    if (!r.frame) continue;
    CHECK(std::fabs(e.evm_db - r.evm_db) <= 0.2f);
    CHECK(std::fabs(e.level_db - r.level_db) <= 0.2f);
    CHECK(e.flagged == r.flagged);
  }
}

// Recorded from the exhaustive search (every symbol timing of every frame
// demodulated): the cheaper search keeps its frame, its level and its EVM.
struct Pin {
  const char* name;
  uint8_t bw;
  bool frame;
  size_t start;
  float evm_db, level_db;
};
const Pin kPins[] = {
    {"c144_20_i028", 20, true, 908, -30.5977f, 50.1552f},
    {"c144_20_i060", 20, true, 2808, -16.7838f, 57.0648f},
    {"c144_20_i124", 20, true, 3929, -8.1238f, 59.6049f},
    {"c40_20_i060", 20, true, 2988, -32.0028f, 52.7690f},
    {"c40_20_i060_weak", 20, true, 4330, -21.7906f, 40.9141f},
    {"c144_20_i028_clip", 20, true, 2128, -22.3813f, 61.8624f},
    {"c40_20_noise", 20, true, 1275, -2.8008f, 12.3008f},
    {"c144_40_i028", 40, true, 7780, -29.8555f, 49.2009f},
    {"c144_40_i076", 40, true, 1419, -11.8456f, 58.6848f},
    {"c144_40_i124", 40, true, 5218, -7.2659f, 59.8433f},
    {"c144_40_noframe", 40, false, 0, 0, 0},
};

TEST(keeps_the_exhaustive_search_answer) {
  for (const Pin& p : kPins) {
    const auto s = load_hi32(p.name);
    const CrossEvm e = cross_evm(s.data(), s.size(), p.bw);
    std::printf("  %-18s frame %d start %zu evm %8.4f level %8.4f\n", p.name, e.frame, e.start,
                e.evm_db, e.level_db);
    CHECK(e.frame == p.frame);
    if (!p.frame) continue;
    CHECK(e.start == p.start);
    CHECK(std::fabs(e.evm_db - p.evm_db) <= 0.05f);
    CHECK(std::fabs(e.level_db - p.level_db) <= 0.05f);
  }
}

TEST(too_short_or_empty_is_no_frame) {
  CHECK(!cross_evm(nullptr, 0, 20).frame);
  std::vector<devourer::IqSample> few(100);
  CHECK(!cross_evm(few.data(), few.size(), 20).frame);
}

TEST(the_no_signal_capture_reads_far_below_a_real_one) {
  const auto s = load_hi32("c40_20_noise");
  CHECK(cross_evm(s.data(), s.size(), 20).level_db < 30.0f);
}

TEST(time_per_40mhz_call) {
  const auto s = load_hi32("c144_40_i076");
  const int reps = 20;
  const auto t0 = std::chrono::steady_clock::now();
  float sink = 0;
  for (int i = 0; i < reps; ++i) sink += cross_evm(s.data(), s.size(), 40).evm_db;
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("  40 MHz call: %.2f ms (sink %.1f)\n", ms / reps, sink);
}

MTEST_MAIN
