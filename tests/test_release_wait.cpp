#include <cstdint>
#include <vector>

#include "mtest.h"
#include "release_wait.h"

namespace {

// A fake clock whose sleep advances it exactly; records every slice and
// the time of every poll, so a test can see what the loop was blind to.
struct Fake {
  uint64_t t = 100'000;
  std::vector<uint64_t> slices;
  std::vector<uint64_t> polls;
  uint64_t now() { return t; }
  void sleep(uint64_t us) { slices.push_back(us); t += us; }
  void poll() { polls.push_back(t); }
};

}  // namespace

TEST(polls_before_the_first_slice_and_between_slices) {
  // The decoder's output is only collected by poll(): a frame decoded
  // just before the wait must not sit through the whole wait, and one
  // decoded during it must be picked up within a step.
  Fake f;
  maburplay::wait_until_polling(
      f.t + 2'300, 500, [&] { return f.now(); }, [&](uint64_t us) { f.sleep(us); },
      [&] { f.poll(); });
  CHECK(f.t == 102'300);
  CHECK(f.slices.size() == 5);  // 500 500 500 500 300
  for (std::size_t i = 0; i + 1 < f.slices.size(); ++i) CHECK(f.slices[i] == 500);
  CHECK(f.slices.back() == 300);
  CHECK(f.polls.size() == 6);          // before each slice, and once at the deadline
  CHECK(f.polls.front() == 100'000);   // immediately, before sleeping at all
  CHECK(f.polls.back() == 102'300);
  for (std::size_t i = 1; i < f.polls.size(); ++i)
    CHECK(f.polls[i] - f.polls[i - 1] <= 500);
}

TEST(deadline_already_passed_polls_once_and_never_sleeps) {
  Fake f;
  maburplay::wait_until_polling(
      f.t - 1, 500, [&] { return f.now(); }, [&](uint64_t us) { f.sleep(us); },
      [&] { f.poll(); });
  CHECK(f.slices.empty());
  CHECK(f.polls.size() == 1);
  CHECK(f.t == 100'000);
}

TEST(short_wait_is_one_exact_slice) {
  Fake f;
  maburplay::wait_until_polling(
      f.t + 120, 500, [&] { return f.now(); }, [&](uint64_t us) { f.sleep(us); },
      [&] { f.poll(); });
  CHECK(f.slices.size() == 1);
  CHECK(f.slices[0] == 120);
  CHECK(f.t == 100'120);
  CHECK(f.polls.size() == 2);
}

MTEST_MAIN
