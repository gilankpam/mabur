#include "mtest.h"
#include "msp_renderer.h"
#include "shm_surface.h"
#include "mabur/msp_dp.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
using namespace maburgs;

// Stub font: 256 glyphs, 1x1 px; only 'A' (0x41) is opaque red.
static std::vector<uint32_t> stub_pixels(256, 0u);
static MspFont make_stub_font() {
  stub_pixels.assign(256, 0u);
  stub_pixels[0x41] = 0xffff0000;  // opaque red
  return MspFont{1, 1, 256, stub_pixels.data()};
}

static void make_region(const char* name, int w, int h) {
  shm_unlink(name);
  int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
  size_t sz = sizeof(ShmRegion) + (size_t)w * h * 4;
  (void)ftruncate(fd, sz);
  auto* r = static_cast<ShmRegion*>(mmap(nullptr, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  r->width = (uint16_t)w; r->height = (uint16_t)h;
  munmap(r, sz); close(fd);
}

// One self-contained snapshot: CLEAR, DRAW_STRING(row,col,text), DRAW_SCREEN.
static std::vector<uint8_t> snapshot(int row, int col, const std::string& text) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(s, 182, clr.data(), clr.size());
  std::vector<uint8_t> ds = {3, (uint8_t)row, (uint8_t)col, 0};
  for (char c : text) ds.push_back((uint8_t)c);
  mabur::msp_append_message(s, 182, ds.data(), ds.size());
  std::vector<uint8_t> scr = {4};
  mabur::msp_append_message(s, 182, scr.data(), scr.size());
  return s;
}

TEST(renders_glyph_into_correct_cell) {
  const char* name = "mabur_test_render";
  make_region(name, 50, 18);  // 1 px per cell -> cell grid maps 1:1
  MspFont font = make_stub_font();
  MspRenderer r(MspRenderCfg{name, 0, 0}, font);
  auto snap = snapshot(2, 3, "A");  // 'A' at row 2, col 3
  r.on_snapshot(snap.data(), snap.size());
  CHECK(r.frames_rendered() == 1);

  int fd = shm_open(name, O_RDONLY, 0);
  auto* reg = static_cast<ShmRegion*>(mmap(nullptr, sizeof(ShmRegion) + 50*18*4, PROT_READ, MAP_SHARED, fd, 0));
  auto* px = reinterpret_cast<uint32_t*>(reg->data);
  CHECK(px[2 * 50 + 3] == 0xffff0000);  // 'A' cell is red
  CHECK(px[2 * 50 + 4] == 0u);          // neighbor blank
  CHECK(px[0] == 0u);                    // (0,0) blank
  munmap(reg, sizeof(ShmRegion) + 50*18*4); close(fd);
  shm_unlink(name);
}

TEST(no_shm_region_drops_gracefully) {
  shm_unlink("mabur_test_norender");
  MspFont font = make_stub_font();
  MspRenderer r(MspRenderCfg{"mabur_test_norender", 0, 0}, font);
  auto snap = snapshot(0, 0, "A");
  r.on_snapshot(snap.data(), snap.size());  // must not crash
  CHECK(r.frames_rendered() == 0);
  CHECK(r.frames_dropped_no_shm() == 1);
}

MTEST_MAIN;
