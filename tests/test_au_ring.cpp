#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
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
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == 0);
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
    w.finish(i != 2, maburgs::AuLatMeta{});  // last one truncated
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

TEST(slot_bytes_alignment) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  // Request unaligned slot_bytes; should be rounded up to 1024.
  REQUIRE(w.open(path, {1000, 4}));

  FrameHdr h;
  h.frame_id = 42;
  h.pts_us = 999999;
  const auto au = au_bytes(500, 7);
  w.begin(h, 2);
  w.append(au.data(), au.size());
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == 0);

  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  CHECK(r.geom().slot_bytes == 1024);  // rounded up from 1000
  CHECK(r.geom().slot_count == 4);
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.frame_id64 == 42);
  CHECK(m.pts_us == 999999u);
  CHECK(m.sid == 2);
  CHECK(got == au);
  unlink(path.c_str());
}

TEST(writer_restart_detection) {
  const std::string path = tmp_ring();
  {
    maburgs::AuRingWriter w1;
    REQUIRE(w1.open(path, {4096, 4}));

    // Write 5 AUs. Reader will start at rec_no = wseq - slot_count = 5 - 4 = 1.
    for (int i = 0; i < 5; ++i) {
      FrameHdr h;
      h.frame_id = static_cast<uint16_t>(i);
      h.pts_us = static_cast<uint32_t>(i * 100);
      const auto au = au_bytes(200, static_cast<uint8_t>(i));
      w1.begin(h, 1);
      w1.append(au.data(), au.size());
      w1.finish(true, maburgs::AuLatMeta{});
    }
  }  // w1 goes out of scope, writer is closed

  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;

  // Read records 1, 2, 3 from first writer: cursor advances to 4.
  for (uint64_t i = 1; i < 4; ++i) {
    CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
    CHECK(m.rec_no == i);
  }
  CHECK(r.resyncs() == 0);

  // Simulate writer restart: create new writer on same path.
  const auto expected_au = au_bytes(150, 8);
  {
    maburgs::AuRingWriter w2;
    REQUIRE(w2.open(path, {4096, 4}));

    // Write 1 new AU (record 0).
    FrameHdr h;
    h.frame_id = 100;
    h.pts_us = 9999;
    w2.begin(h, 2);
    w2.append(expected_au.data(), expected_au.size());
    w2.finish(true, maburgs::AuLatMeta{});
  }  // w2 goes out of scope, writer is closed

  // Reader cursor is at 4, but wseq is now 1 (from the new writer).
  // This backward jump (1 < 4) triggers resync.
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kResync);
  CHECK(r.resyncs() == 1);

  // Next call should deliver the new record.
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.frame_id64 == 100);
  CHECK(m.pts_us == 9999u);
  CHECK(m.sid == 2);
  CHECK(got == expected_au);

  unlink(path.c_str());
}

namespace {
void write_n(maburgs::AuRingWriter& w, int n, size_t bytes, uint8_t sid = 1) {
  for (int i = 0; i < n; ++i) {
    FrameHdr h;
    h.frame_id = static_cast<uint16_t>(w.published());
    h.pts_us = static_cast<uint32_t>(w.published());
    const auto au = au_bytes(bytes, static_cast<uint8_t>(w.published()));
    w.begin(h, sid);
    w.append(au.data(), au.size());
    w.finish(true, maburgs::AuLatMeta{});
  }
}
}  // namespace

TEST(overwrite_oldest_reader_resyncs) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {1024, 4}));
  maburgs::AuRingReader r;
  write_n(w, 1, 64);
  REQUIRE(r.open(path));           // cursor at rec 0
  write_n(w, 9, 64);               // recs 1..9; recs 0..5 overwritten
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;
  // First read: slot 0 now holds rec 8 (8 % 4 == 0) -> overrun accepted.
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.rec_no >= 6);            // never a stale/garbage record
  CHECK(r.resyncs() == 1);
  uint64_t prev = m.rec_no;
  while (r.next(&m, &got) == maburgs::AuRingReader::Res::kOk) {
    CHECK(m.rec_no == prev + 1);
    CHECK(got == au_bytes(64, static_cast<uint8_t>(m.rec_no)));
    prev = m.rec_no;
  }
  CHECK(prev == 9);
  unlink(path.c_str());
}

TEST(oversize_au_dropped_whole) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {1024, 4}));
  FrameHdr h;
  const auto big = au_bytes(2000, 5);
  w.begin(h, 1);
  w.append(big.data(), big.size());
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == UINT64_MAX);
  CHECK(w.dropped_oversize() == 1);
  CHECK(w.published() == 0);
  write_n(w, 1, 100);  // ring still functional after a drop
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.rec_no == 0);
  CHECK(got.size() == 100);
  unlink(path.c_str());
}

TEST(reader_rejects_bad_magic) {
  const std::string path = tmp_ring();
  FILE* f = fopen(path.c_str(), "wb");
  REQUIRE(f != nullptr);
  std::vector<uint8_t> junk(8192, 0xAB);
  fwrite(junk.data(), 1, junk.size(), f);
  fclose(f);
  maburgs::AuRingReader r;
  CHECK(!r.open(path));
  unlink(path.c_str());
}

TEST(finish_without_begin_is_noop) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {1024, 4}));
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == UINT64_MAX);
  CHECK(w.published() == 0);
  unlink(path.c_str());
}

TEST(epoch_stamped_nonzero_and_latched) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {1024, 4}));
  write_n(w, 2, 64);
  auto read_epoch = [&]() -> uint64_t {
    FILE* f = fopen(path.c_str(), "rb");
    REQUIRE(f != nullptr);
    REQUIRE(fseek(f, 32, SEEK_SET) == 0);   // kOffEpoch: the wire contract
    uint64_t e = 0;
    REQUIRE(fread(&e, 8, 1, f) == 1);
    fclose(f);
    return e;
  };
  const uint64_t e1 = read_epoch();
  CHECK(e1 != 0);
  CHECK((e1 & 1) == 1);                     // |1 guarantee
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m; std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(read_epoch() == e1);                // stable across reads
  maburgs::AuRingWriter w2;
  REQUIRE(w2.open(path, {1024, 4}));        // restart
  const uint64_t e2 = read_epoch();
  CHECK(e2 != 0);
  CHECK(e2 != e1);                          // changes across restart
  unlink(path.c_str());
}

TEST(epoch_detects_missed_restart) {
  // The last_wseq_ blind spot: writer restarts and climbs PAST the reader's
  // cursor before the next poll. Epoch catches it; wseq comparison cannot.
  const std::string path = tmp_ring();
  auto w = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w->open(path, {1024, 4}));
  write_n(*w, 3, 64);
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m; std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  w.reset();
  auto w2 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w2->open(path, {1024, 4}));      // same geometry, new epoch
  write_n(*w2, 5, 64);                     // wseq 5 > reader cursor 1
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kResync);
  CHECK(r.resyncs() >= 1);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);  // new session
  CHECK(got == au_bytes(64, static_cast<uint8_t>(m.rec_no)));
  CHECK(!r.dead());
  unlink(path.c_str());
}

TEST(geometry_change_reopens_reader) {
  const std::string path = tmp_ring();
  auto w = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w->open(path, {1024, 4}));
  write_n(*w, 2, 64);
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m; std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  w.reset();
  auto w2 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w2->open(path, {2048, 8}));      // different geometry
  write_n(*w2, 1, 100);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kResync);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(r.geom().slot_bytes == 2048);
  CHECK(got.size() == 100);
  unlink(path.c_str());
}

TEST(reopen_retries_through_unreadable_header) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {1024, 4}));
  write_n(w, 2, 64);
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m; std::vector<uint8_t> got;
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  // Simulate mid-recreate: zero the magic+epoch words in place.
  FILE* f = fopen(path.c_str(), "rb+");
  REQUIRE(f != nullptr);
  uint8_t zeros[40] = {0};
  fwrite(zeros, 1, 40, f);
  fclose(f);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kResync);  // epoch 0 mismatch
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kNone);    // reopen failing, not dead
  CHECK(!r.dead());
  // Writer finishes the recreate.
  maburgs::AuRingWriter w2;
  REQUIRE(w2.open(path, {1024, 4}));
  write_n(w2, 3, 80);
  maburgs::AuRingReader::Res res;
  do { res = r.next(&m, &got); } while (res == maburgs::AuRingReader::Res::kNone && !r.dead());
  CHECK(res == maburgs::AuRingReader::Res::kResync);
  CHECK(r.next(&m, &got) == maburgs::AuRingReader::Res::kOk);
  CHECK(!r.dead());
  unlink(path.c_str());
}

TEST(hammer_writer_reader_integrity) {
  // Full-speed writer thread vs reader: every accepted record's payload must
  // match its rec_no pattern. Laps/resyncs allowed; corruption is not.
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {256, 4}));
  std::atomic<bool> stop{false};
  std::thread wr([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      FrameHdr h; h.frame_id = static_cast<uint16_t>(w.published());
      const auto au = au_bytes(64 + (w.published() % 128),
                               static_cast<uint8_t>(w.published() % 251));
      w.begin(h, 1); w.append(au.data(), au.size()); w.finish(true, maburgs::AuLatMeta{});
    }
  });
  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m; std::vector<uint8_t> got;
  uint64_t ok = 0, corrupt = 0;
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(2)) {
    if (r.next(&m, &got) != maburgs::AuRingReader::Res::kOk) continue;
    if (got != au_bytes(64 + (m.rec_no % 128), static_cast<uint8_t>(m.rec_no % 251)))
      ++corrupt;
    else ++ok;
  }
  stop = true; wr.join();
  CHECK(corrupt == 0);
  CHECK(ok > 1000);
  CHECK(r.resyncs() > 0);
  unlink(path.c_str());
}

TEST(au_ring_v2_lat_meta_roundtrip) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {4096, 4}));
  FrameHdr h;
  h.frame_id = 1;
  h.pts_us = 500;
  w.begin(h, 0);
  const uint8_t payload[4] = {1, 2, 3, 4};
  w.append(payload, sizeof(payload));
  maburgs::AuLatMeta lat;
  lat.t_first_us = 111111;
  lat.t_complete_us = 222222;
  lat.drone_q_ms = 7;
  lat.enc_us = 9001;
  lat.drone_air_ms = 21;
  CHECK(w.finish(true, lat) == 0);

  maburgs::AuRingReader r;
  REQUIRE(r.open(path));
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> au;
  CHECK(r.next(&m, &au) == maburgs::AuRingReader::Res::kOk);
  CHECK(m.t_first_us == 111111);
  CHECK(m.t_complete_us == 222222);
  CHECK(m.drone_q_ms == 7);
  CHECK(m.enc_us == 9001);
  CHECK(m.drone_air_ms == 21);
  unlink(path.c_str());
}

TEST(au_ring_v1_refused_by_reader) {
  const std::string path = tmp_ring();
  maburgs::AuRingWriter w;
  REQUIRE(w.open(path, {4096, 4}));
  write_n(w, 1, 64);

  // Flip the on-disk version dword (RingHdr offset 4) from kAuRingVersion
  // (2) down to 1, simulating a pre-SlotHdr-v2 ring: the reader must refuse
  // to open it rather than silently misreading the new fields.
  FILE* f = fopen(path.c_str(), "rb+");
  REQUIRE(f != nullptr);
  REQUIRE(fseek(f, 4, SEEK_SET) == 0);
  const uint32_t v1 = 1;
  REQUIRE(fwrite(&v1, sizeof(v1), 1, f) == 1);
  fclose(f);

  maburgs::AuRingReader r;
  CHECK(!r.open(path));
  unlink(path.c_str());
}

MTEST_MAIN
