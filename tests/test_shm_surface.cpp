#include "mtest.h"
#include "shm_surface.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>
using namespace maburgs;

// Create a PixelPilot-style region in /dev/shm and return its byte size.
static size_t make_region(const char* name, int w, int h) {
  shm_unlink(name);
  int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
  REQUIRE(fd >= 0);
  size_t sz = sizeof(ShmRegion) + (size_t)w * h * 4;
  REQUIRE(ftruncate(fd, sz) == 0);
  auto* r = static_cast<ShmRegion*>(mmap(nullptr, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  REQUIRE(r != MAP_FAILED);
  r->width = (uint16_t)w; r->height = (uint16_t)h;
  munmap(r, sz); close(fd);
  return sz;
}

TEST(acquire_reads_dims_and_writes_data) {
  const char* name = "mabur_test_shm";
  make_region(name, 8, 4);
  ShmSurface s(name);
  REQUIRE(s.acquire());
  CHECK(s.ok());
  CHECK(s.width() == 8);
  CHECK(s.height() == 4);
  CHECK(s.data_size() == (size_t)8 * 4 * 4);
  // Write a pixel and read it back through a fresh map.
  reinterpret_cast<uint32_t*>(s.data())[0] = 0xdeadbeef;
  int fd = shm_open(name, O_RDONLY, 0);
  REQUIRE(fd >= 0);
  auto* r = static_cast<ShmRegion*>(mmap(nullptr, sizeof(ShmRegion) + 8*4*4, PROT_READ, MAP_SHARED, fd, 0));
  CHECK(reinterpret_cast<uint32_t*>(r->data)[0] == 0xdeadbeef);
  munmap(r, sizeof(ShmRegion) + 8*4*4); close(fd);
  shm_unlink(name);
}

TEST(acquire_false_when_absent) {
  shm_unlink("mabur_test_absent");
  ShmSurface s("mabur_test_absent");
  CHECK(s.acquire() == false);
  CHECK(s.ok() == false);
}

MTEST_MAIN;
