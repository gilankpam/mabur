// MenuViewFeed: when the presentation loop recomputes the menu's view, and
// how long a changed view stays owed to the screen.
#include <cstdint>
#include <string>

#include "menu_model.h"
#include "menu_view_feed.h"
#include "mtest.h"

using maburplay::MenuView;
using maburplay::MenuViewFeed;

namespace {

// A view source that counts its calls and shows whatever toast it is told.
struct Source {
  int calls = 0;
  std::string toast;
  MenuView operator()(uint64_t) {
    ++calls;
    MenuView v;
    v.toast = toast;
    return v;
  }
};

}  // namespace

TEST(the_first_call_computes_and_an_unchanged_view_owes_nothing) {
  MenuViewFeed f;
  Source src;
  auto compute = [&](uint64_t t) { return src(t); };
  CHECK(f.next(0, 0, compute) == nullptr);   // the same as the composer's empty view
  CHECK(src.calls == 1);
  src.toast = "hello";
  CHECK(f.next(100, 0, compute) != nullptr);
  CHECK(f.view().toast == "hello");
}

TEST(no_recompute_within_the_period_unless_the_generation_moves) {
  MenuViewFeed f;
  Source src;
  auto compute = [&](uint64_t t) { return src(t); };
  f.next(1000, 5, compute);
  for (uint64_t t = 1001; t < 1000 + MenuViewFeed::kPeriodMs; t += 2) f.next(t, 5, compute);
  CHECK(src.calls == 1);
  f.next(1010, 6, compute);   // a key
  CHECK(src.calls == 2);
  f.next(1010 + MenuViewFeed::kPeriodMs, 6, compute);   // an expiry nothing signals
  CHECK(src.calls == 3);
}

TEST(a_changed_view_stays_owed_until_composed) {
  MenuViewFeed f;
  Source src;
  auto compute = [&](uint64_t t) { return src(t); };
  f.next(0, 0, compute);
  src.toast = "x";
  const MenuView* v = f.next(0, 1, compute);
  REQUIRE(v != nullptr);
  CHECK(v->toast == "x");
  // The buffers were busy: no composition. Still owed, without recomputing.
  CHECK(f.next(1, 1, compute) == v);
  CHECK(src.calls == 2);
  f.composed();
  CHECK(!f.owed());
  CHECK(f.next(2, 1, compute) == nullptr);
  // Recomputed equal: nothing owed again.
  CHECK(f.next(2 + MenuViewFeed::kPeriodMs, 1, compute) == nullptr);
  CHECK(src.calls == 3);
}

MTEST_MAIN
