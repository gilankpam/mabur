#include <vector>
#include "mtest.h"
#include "frame_source.h"
using namespace mabur;

namespace {
const char* kName = "/mabur_test_frame_ring";
}

TEST(reads_frame_with_meta) {
  venc_frame_ring_t* prod = venc_frame_ring_create(kName, 16, 64 * 1024);
  REQUIRE(prod != nullptr);
  FrameSource src(kName, 10);
  std::vector<uint8_t> frame(20000);
  for (size_t i = 0; i < frame.size(); ++i) frame[i] = static_cast<uint8_t>(i);
  VencFrameMeta meta{};
  meta.pts = 123456;
  meta.codec = VENC_FRAME_CODEC_H265;
  meta.flags = VENC_FRAME_FLAG_IDR;
  REQUIRE(venc_frame_ring_write(prod, &meta, frame.data(),
                                static_cast<uint32_t>(frame.size())) == 0);
  std::vector<uint8_t> buf(VENC_FRAME_META_SIZE + 64 * 1024);
  VencFrameMeta got{};
  int n = src.read(buf.data(), buf.size(), 200, &got);
  REQUIRE(n == static_cast<int>(frame.size()));
  CHECK(got.pts == 123456u);
  CHECK(got.flags == VENC_FRAME_FLAG_IDR);
  CHECK(std::vector<uint8_t>(buf.begin() + VENC_FRAME_META_SIZE,
                             buf.begin() + VENC_FRAME_META_SIZE + n) == frame);
  venc_frame_ring_destroy(prod);
}

TEST(timeout_returns_zero) {
  venc_frame_ring_t* prod = venc_frame_ring_create(kName, 16, 64 * 1024);
  REQUIRE(prod != nullptr);
  FrameSource src(kName, 10);
  std::vector<uint8_t> buf(VENC_FRAME_META_SIZE + 64 * 1024);
  VencFrameMeta got{};
  CHECK(src.read(buf.data(), buf.size(), 20, &got) == 0);
  venc_frame_ring_destroy(prod);
}

TEST(producer_restart_reattaches) {
  venc_frame_ring_t* prod = venc_frame_ring_create(kName, 16, 64 * 1024);
  REQUIRE(prod != nullptr);
  FrameSource src(kName, 10);
  std::vector<uint8_t> buf(VENC_FRAME_META_SIZE + 64 * 1024);
  VencFrameMeta meta{}, got{};
  uint8_t payload[100] = {0x42};
  venc_frame_ring_write(prod, &meta, payload, sizeof payload);
  REQUIRE(src.read(buf.data(), buf.size(), 200, &got) == 100);
  // Producer restarts: destroy (unlinks shm) + recreate = new inode.
  venc_frame_ring_destroy(prod);
  prod = venc_frame_ring_create(kName, 16, 64 * 1024);
  REQUIRE(prod != nullptr);
  // First read after restart hits the stale mapping, detects the inode
  // change and detaches; a following read reattaches and delivers.
  int n = 0;
  for (int i = 0; i < 50 && n == 0; ++i) {
    venc_frame_ring_write(prod, &meta, payload, sizeof payload);
    n = src.read(buf.data(), buf.size(), 20, &got);
  }
  CHECK(n == 100);
  CHECK(src.reattach_count() >= 1);
  venc_frame_ring_destroy(prod);
}

// venc_frame_ring_fill_t is a snapshot of the *local handle's* counters (the
// vendored header calls it "Producer-side observability"), and only the write
// path touches writes/full_drops. maburd attaches as a consumer, so those two
// fields are structurally pinned at 0 in its process no matter what the
// producer does — the shm header carries write_idx/read_idx and no counters,
// so there is no path for them to cross. Locks that in so nobody re-adds them
// to maburd's stats line expecting a live number.
TEST(consumer_fill_reports_only_consumer_side_counters) {
  venc_frame_ring_t* prod = venc_frame_ring_create(kName, 16, 64 * 1024);
  REQUIRE(prod != nullptr);
  FrameSource src(kName, 10);
  std::vector<uint8_t> buf(VENC_FRAME_META_SIZE + 64 * 1024);
  VencFrameMeta meta{}, got{};
  uint8_t payload[100] = {0x42};
  for (int i = 0; i < 3; ++i) {
    REQUIRE(venc_frame_ring_write(prod, &meta, payload, sizeof payload) == 0);
    REQUIRE(src.read(buf.data(), buf.size(), 200, &got) == 100);
  }

  venc_frame_ring_fill_t consumer{};
  REQUIRE(src.fill(&consumer));
  CHECK(consumer.reads == 3);       // the consumer's own work: real
  CHECK(consumer.writes == 0);      // producer-only: always 0 here
  CHECK(consumer.full_drops == 0);  // producer-only: always 0 here

  // The same counters are live in the producer's handle, which is the process
  // that would have to report them.
  venc_frame_ring_fill_t producer{};
  REQUIRE(venc_frame_ring_get_fill(prod, &producer) == 0);
  CHECK(producer.writes == 3);
  CHECK(producer.reads == 0);

  venc_frame_ring_destroy(prod);
}

MTEST_MAIN
