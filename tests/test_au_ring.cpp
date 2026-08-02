#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

#include "au_ring.h"
#include "mtest.h"

using mabur::framewire::FrameHdr;
using mabur::framewire::kFlagIdr;

namespace {
std::string tmp_ring() {
  char tpl[] = "/tmp/test_au_ring_XXXXXX";
  int fd = mkstemp(tpl);
  REQUIRE(fd >= 0);
  close(fd);
  return std::string(tpl);
}
std::vector<uint8_t> au_bytes(size_t n, uint8_t seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + i * 7);
  return v;
}
}  // namespace

TEST(roundtrip_single_au) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {4096, 4}));

  FrameHdr h;
  h.frame_id = 7;
  h.flags = kFlagIdr;
  h.pts_us = 123456;
  const auto au = au_bytes(1000, 3);
  w.begin(h, 1);
  w.append(au.data(), 600);
  w.append(au.data() + 600, 400);
  CHECK(w.finish(true) == 0);
  CHECK(w.published() == 1);

  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  CHECK(r.geom().slot_bytes == 4096);
  CHECK(r.geom().slot_count == 4);
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.rec_no == 0);
  CHECK(m.sid == 1);
  CHECK(m.pts_us == 123456u);
  CHECK((m.flags & kFlagIdr) != 0);
  CHECK((m.flags & maburgs::kRecFlagComplete) != 0);
  CHECK(got == au);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kNone);
  unlink(path.c_str());
}

TEST(ordering_and_truncated_flag) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {4096, 4}));
  for (int i = 0; i < 3; ++i) {
    FrameHdr h;
    h.frame_id = static_cast<uint16_t>(i);
    h.pts_us = static_cast<uint32_t>(1000 * i);
    const auto au = au_bytes(100 + static_cast<size_t>(i), static_cast<uint8_t>(i));
    w.begin(h, i == 2 ? 3 : 1);
    w.append(au.data(), au.size());
    w.finish(i != 2);  // last one truncated
  }
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;
  for (uint64_t i = 0; i < 3; ++i) {
    CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
    CHECK(m.rec_no == i);
    CHECK(got.size() == 100 + i);
  }
  CHECK(m.sid == 3);
  CHECK((m.flags & maburgs::kRecFlagComplete) == 0);
  unlink(path.c_str());
}

MTEST_MAIN
