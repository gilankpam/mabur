#include <cstdint>
#include <map>
#include <vector>

#include "fb_cache.h"
#include "mtest.h"

using maburplay::FbCache;

namespace {

// A fake DRM: dma-buf fd -> handle is a fixed map (the kernel returns the
// same handle for the same dma-buf while it stays open); add_fb hands out
// increasing fb ids; rm/close are recorded.
struct Fake {
  std::map<int, uint32_t> fd_to_handle;
  uint32_t next_fb = 100;
  std::vector<uint32_t> rm, closed, added;
  bool fail_import = false, fail_add = false;
  FbCache::Ops ops() {
    return FbCache::Ops{
        [this](int fd, uint32_t* h) {
          if (fail_import) return false;
          auto it = fd_to_handle.find(fd);
          if (it == fd_to_handle.end()) return false;
          *h = it->second;
          return true;
        },
        [this](uint32_t h, const FbCache::Geom&, uint32_t* fb) {
          if (fail_add) return false;
          *fb = next_fb++;
          added.push_back(h);
          return true;
        },
        [this](uint32_t fb) { rm.push_back(fb); },
        [this](uint32_t h) { closed.push_back(h); }};
  }
};

FbCache::Geom geom(int w = 1920, int h = 1080) {
  FbCache::Geom g;
  g.width = w; g.height = h; g.stride = w; g.vstride = h; g.fourcc = 0x3231564e;
  return g;
}
bool none(uint32_t) { return false; }

}  // namespace

TEST(a_pool_buffer_seen_again_costs_no_add_and_no_free) {
  Fake f;
  f.fd_to_handle = {{10, 7}, {11, 8}, {12, 9}, {13, 10}};
  FbCache c(f.ops(), 32);
  uint32_t h = 0;
  const uint32_t fb0 = c.acquire(10, geom(), 1, none, &h);
  CHECK(fb0 == 100);
  CHECK(h == 7);
  for (int round = 0; round < 3; ++round)
    for (int fd = 10; fd <= 13; ++fd) c.acquire(fd, geom(), static_cast<uint64_t>(10 + round), none, nullptr);
  CHECK(c.acquire(10, geom(), 50, none, nullptr) == fb0);
  CHECK(c.adds() == 4);
  CHECK(c.size() == 4);
  CHECK(f.rm.empty());
  CHECK(f.closed.empty());
}

TEST(eviction_is_lru_and_skips_pinned_handles) {
  Fake f;
  f.fd_to_handle = {{10, 7}, {11, 8}, {12, 9}};
  FbCache c(f.ops(), 2);
  const uint32_t fb7 = c.acquire(10, geom(), 1, none, nullptr);
  const uint32_t fb8 = c.acquire(11, geom(), 2, none, nullptr);
  // Handle 7 is the LRU but still on screen: 8 goes instead.
  c.acquire(12, geom(), 3, [](uint32_t h) { return h == 7; }, nullptr);
  CHECK(c.size() == 2);
  CHECK(c.evictions() == 1);
  CHECK(f.rm == std::vector<uint32_t>{fb8});
  CHECK(f.closed == std::vector<uint32_t>{8});
  // Nothing pinned: the LRU (7) goes next.
  f.fd_to_handle[11] = 8;  // the kernel hands the reopened dma-buf the same handle
  c.acquire(11, geom(), 4, none, nullptr);
  CHECK(f.rm.size() == 2 && f.rm[1] == fb7);
  CHECK(f.closed.size() == 2 && f.closed[1] == 7);
}

TEST(everything_pinned_grows_instead_of_breaking_a_slot) {
  Fake f;
  f.fd_to_handle = {{10, 7}, {11, 8}, {12, 9}};
  FbCache c(f.ops(), 2);
  c.acquire(10, geom(), 1, none, nullptr);
  c.acquire(11, geom(), 2, none, nullptr);
  c.acquire(12, geom(), 3, [](uint32_t) { return true; }, nullptr);
  CHECK(c.size() == 3);
  CHECK(c.evictions() == 0);
}

TEST(new_geometry_on_a_known_handle_replaces_the_fb_and_keeps_the_handle) {
  Fake f;
  f.fd_to_handle = {{10, 7}};
  FbCache c(f.ops(), 32);
  const uint32_t fb_a = c.acquire(10, geom(1920, 1080), 1, none, nullptr);
  const uint32_t fb_b = c.acquire(10, geom(1280, 720), 2, none, nullptr);
  CHECK(fb_b != fb_a);
  CHECK(f.rm == std::vector<uint32_t>{fb_a});
  CHECK(f.closed.empty());
  CHECK(c.size() == 1);
}

TEST(failures_cache_nothing_and_leak_nothing) {
  Fake f;
  f.fd_to_handle = {{10, 7}};
  FbCache c(f.ops(), 32);
  f.fail_import = true;
  CHECK(c.acquire(10, geom(), 1, none, nullptr) == 0);
  CHECK(c.size() == 0);
  f.fail_import = false;
  f.fail_add = true;
  CHECK(c.acquire(10, geom(), 2, none, nullptr) == 0);
  CHECK(c.size() == 0);
  CHECK(f.closed == std::vector<uint32_t>{7});  // the imported handle is not left open
}

TEST(flush_frees_every_entry) {
  Fake f;
  f.fd_to_handle = {{10, 7}, {11, 8}};
  FbCache c(f.ops(), 32);
  c.acquire(10, geom(), 1, none, nullptr);
  c.acquire(11, geom(), 2, none, nullptr);
  c.flush();
  CHECK(c.size() == 0);
  CHECK(f.rm.size() == 2);
  CHECK(f.closed.size() == 2);
  c.flush();  // idempotent
  CHECK(f.rm.size() == 2);
}

MTEST_MAIN
