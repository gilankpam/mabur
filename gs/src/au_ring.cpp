#include "au_ring.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cstring>

namespace maburgs {
namespace {

// RingHdr field offsets (see au_ring.h layout comment / ausniff.py mirror).
constexpr size_t kOffMagic = 0, kOffVersion = 4, kOffSlotBytes = 8,
                 kOffSlotCount = 12, kOffWriteSeq = 16, kOffDropped = 24,
                 kOffEpoch = 32;
// SlotHdr field offsets.
constexpr size_t kSOffLock = 0, kSOffLen = 4, kSOffRecNo = 8,
                 kSOffFrameId = 16, kSOffPts = 24, kSOffSid = 28,
                 kSOffFlags = 29, kSOffCodec = 30;

uint32_t load32(const uint8_t* p) {
  return __atomic_load_n(reinterpret_cast<const uint32_t*>(p), __ATOMIC_ACQUIRE);
}
uint32_t load32_relaxed(const uint8_t* p) {
  return __atomic_load_n(reinterpret_cast<const uint32_t*>(p), __ATOMIC_RELAXED);
}
uint64_t load64(const uint8_t* p) {
  return __atomic_load_n(reinterpret_cast<const uint64_t*>(p), __ATOMIC_ACQUIRE);
}
void store32(uint8_t* p, uint32_t v) {
  __atomic_store_n(reinterpret_cast<uint32_t*>(p), v, __ATOMIC_RELEASE);
}
void store32_relaxed(uint8_t* p, uint32_t v) {
  __atomic_store_n(reinterpret_cast<uint32_t*>(p), v, __ATOMIC_RELAXED);
}
void store64(uint8_t* p, uint64_t v) {
  __atomic_store_n(reinterpret_cast<uint64_t*>(p), v, __ATOMIC_RELEASE);
}
void put32(uint8_t* p, uint32_t v) { std::memcpy(p, &v, 4); }
void put64(uint8_t* p, uint64_t v) { std::memcpy(p, &v, 8); }
uint32_t get32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
uint64_t get64(const uint8_t* p) { uint64_t v; std::memcpy(&v, p, 8); return v; }

// Align slot_bytes up to the next multiple of 64 bytes to ensure lock word alignment.
uint32_t align_slot_bytes(uint32_t slot_bytes) {
  return ((slot_bytes + 63) / 64) * 64;
}

size_t ring_bytes(const AuRingGeom& g) {
  return kAuRingHdrBytes +
         static_cast<size_t>(g.slot_count) * (kAuSlotHdrBytes + g.slot_bytes);
}

uint64_t now_monotonic_ns() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
         static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t now_monotonic_ms() { return now_monotonic_ns() / 1000000ull; }

// Budget for a reader to keep retrying a failed reopen (unreadable/torn
// header) before giving up. Ring re-creation is non-atomic — ftruncate,
// then memset (which zeroes epoch AND magic for the memset's duration on a
// multi-MiB ring), then geometry, then epoch, then magic last/release — so
// a reader polling mid-recreate seeing a torn header is the expected,
// routine case, not a dead ring. 5 s comfortably covers that window plus
// unlink-then-recreate shutdown/restart sequences.
constexpr uint64_t kReopenBudgetMs = 5000;

// Writer-side: create/truncate to the exact geometry and map read-write.
uint8_t* map_file_rw(const std::string& path, size_t bytes, size_t* out_bytes) {
  int fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0644);
  if (fd < 0) return nullptr;
  if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
    ::close(fd);
    return nullptr;
  }
  void* m = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ::close(fd);
  if (m == MAP_FAILED) return nullptr;
  *out_bytes = bytes;
  return static_cast<uint8_t*>(m);
}

// Reader-side: never creates, never writes — PROT_READ only. Size comes from
// whatever the writer currently has on disk (fstat), so a reopen after a
// writer geometry change picks up the new extent automatically.
uint8_t* map_file_ro(const std::string& path, size_t* out_bytes) {
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) return nullptr;
  struct stat st;
  if (::fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) < kAuRingHdrBytes) {
    ::close(fd);
    return nullptr;
  }
  const size_t bytes = static_cast<size_t>(st.st_size);
  void* m = ::mmap(nullptr, bytes, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (m == MAP_FAILED) return nullptr;
  *out_bytes = bytes;
  return static_cast<uint8_t*>(m);
}

}  // namespace

AuRingWriter::~AuRingWriter() {
  if (map_) ::munmap(map_, map_bytes_);
}

bool AuRingWriter::open(const std::string& path, AuRingGeom geom) {
  if (geom.slot_bytes == 0 || geom.slot_count == 0) return false;
  // Round slot_bytes up to next multiple of 64 to ensure lock word alignment.
  geom.slot_bytes = align_slot_bytes(geom.slot_bytes);
  map_ = map_file_rw(path, ring_bytes(geom), &map_bytes_);
  if (!map_) return false;
  geom_ = geom;
  std::memset(map_, 0, map_bytes_);
  put32(map_ + kOffVersion, kAuRingVersion);
  put32(map_ + kOffSlotBytes, geom.slot_bytes);
  put32(map_ + kOffSlotCount, geom.slot_count);
  // Boot stamp, nonzero (OR 1): lets a reader tell this ring instance apart
  // from any other ever mapped at this path, independent of write_seq.
  put64(map_ + kOffEpoch, now_monotonic_ns() | 1);
  // Magic last, release: a reader that sees the magic sees the geometry.
  store32(map_ + kOffMagic, kAuRingMagic);
  return true;
}

uint8_t* AuRingWriter::slot_base_(uint64_t rec_no) const {
  return map_ + kAuRingHdrBytes +
         (rec_no % geom_.slot_count) *
             (kAuSlotHdrBytes + static_cast<size_t>(geom_.slot_bytes));
}

void AuRingWriter::begin(const mabur::framewire::FrameHdr& h, uint8_t sid) {
  hdr_ = h;
  sid_ = sid;
  au_.clear();
  in_au_ = true;
}

void AuRingWriter::append(const uint8_t* p, size_t n) {
  if (!in_au_) return;
  au_.insert(au_.end(), p, p + n);
}

uint64_t AuRingWriter::finish(bool complete) {
  if (!map_ || !in_au_) return UINT64_MAX;
  in_au_ = false;
  if (au_.size() > geom_.slot_bytes) {
    ++dropped_oversize_;
    store64(map_ + kOffDropped, dropped_oversize_);
    au_.clear();
    return UINT64_MAX;
  }
  const uint64_t n = published_;
  uint8_t* slot = slot_base_(n);
  const uint32_t lock = load32(slot + kSOffLock);
  store32_relaxed(slot + kSOffLock, lock + 1);  // odd: write in progress
  // Ensure odd lock value is visible to readers before payload writes.
  __atomic_thread_fence(__ATOMIC_RELEASE);
  std::memcpy(slot + kAuSlotHdrBytes, au_.data(), au_.size());
  put32(slot + kSOffLen, static_cast<uint32_t>(au_.size()));
  put64(slot + kSOffRecNo, n);
  // frame_id64: PR A publishes FrameStream's already-ordered stream, whose
  // u16 frame_id the writer unwraps monotonically here.
  const uint16_t prev = static_cast<uint16_t>(last_id_);
  const uint16_t d = static_cast<uint16_t>(hdr_.frame_id - prev);
  last_id_ = have_id_ ? last_id_ + static_cast<uint64_t>(d)
                      : static_cast<uint64_t>(hdr_.frame_id);
  have_id_ = true;
  put64(slot + kSOffFrameId, last_id_);
  put32(slot + kSOffPts, hdr_.pts_us);
  slot[kSOffSid] = sid_;
  slot[kSOffFlags] =
      static_cast<uint8_t>(hdr_.flags | (complete ? kRecFlagComplete : 0));
  slot[kSOffCodec] = hdr_.codec;
  store32(slot + kSOffLock, lock + 2);  // even: stable, release
  ++published_;
  store64(map_ + kOffWriteSeq, published_);
  au_.clear();
  return n;
}

AuRingReader::~AuRingReader() {
  if (map_) ::munmap(map_, map_bytes_);
}

bool AuRingReader::open(const std::string& path) {
  map_ = map_file_ro(path, &map_bytes_);
  if (!map_) return false;
  if (load32(map_ + kOffMagic) != kAuRingMagic ||
      get32(map_ + kOffVersion) != kAuRingVersion) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    return false;
  }
  geom_.slot_bytes = get32(map_ + kOffSlotBytes);
  geom_.slot_count = get32(map_ + kOffSlotCount);
  // Slot stride is 64 + slot_bytes; slot_bytes must be a multiple of 64 to
  // ensure the lock word at offset 0 of each slot is properly aligned.
  if (geom_.slot_bytes == 0 || geom_.slot_bytes % 64 != 0 || geom_.slot_count == 0) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    return false;
  }
  const size_t need = kAuRingHdrBytes + static_cast<size_t>(geom_.slot_count) *
                                            (kAuSlotHdrBytes + geom_.slot_bytes);
  if (map_bytes_ < need) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    return false;
  }
  path_ = path;
  // 0 means a pre-epoch (PR-A) ring; last_wseq_ below stays the only
  // restart detector for that case (see the fallback check in next()).
  epoch_ = get64(map_ + kOffEpoch);
  const uint64_t wseq = load64(map_ + kOffWriteSeq);
  cursor_ = wseq > geom_.slot_count ? wseq - geom_.slot_count : 0;
  last_wseq_ = wseq;
  // A successful (re)open means we have a good mapping again: clear any
  // stale failure state from a previous reopen attempt.
  dead_ = false;
  reopen_fail_ms_ = 0;
  return true;
}

const uint8_t* AuRingReader::slot_base_(uint64_t rec_no) const {
  return map_ + kAuRingHdrBytes +
         (rec_no % geom_.slot_count) *
             (kAuSlotHdrBytes + static_cast<size_t>(geom_.slot_bytes));
}

AuRingReader::Res AuRingReader::next(AuRecordMeta* meta,
                                     std::vector<uint8_t>* payload) {
  if (dead_) return Res::kNone;
  if (!map_) {
    // A previous reopen attempt (below, or here) failed to (re)map the
    // ring — expected transiently while a writer is mid-recreate (see the
    // non-atomic re-creation note in au_ring.h / kReopenBudgetMs above).
    // Retry every poll rather than latching dead_ immediately; only give
    // up once retries have failed continuously past the budget, which also
    // covers an unlink-then-recreate shutdown/restart sequence.
    const std::string p = path_;  // local copy: open() writes path_ itself
    if (open(p)) return Res::kResync;  // recovered: caller sees a discontinuity
    const uint64_t now = now_monotonic_ms();
    if (reopen_fail_ms_ == 0) reopen_fail_ms_ = now;
    if (now - reopen_fail_ms_ > kReopenBudgetMs) dead_ = true;
    return Res::kNone;
  }
  const uint64_t ep = get64(map_ + kOffEpoch);
  // ep == epoch_ == 0 falls through here (pre-epoch ring, see the wseq
  // fallback below); any other change — including 0 -> nonzero, a reader
  // that started against a legacy writer outliving it into an
  // epoch-stamped one — is a real ring re-creation.
  if (ep != epoch_) {
    // Writer re-created the ring (restart). Unlike the wseq check below,
    // this also catches the case where the new writer's wseq climbs PAST
    // our cursor before we poll again — a same-or-higher wseq that the
    // regression check can't see is happening from a *different* writer
    // instance. Geometry may have changed with it, and the mapping may
    // even be a different size (ftruncate to a new extent). Re-open from
    // scratch — cheapest safe path, and it re-latches epoch, geometry,
    // and cursor together whether or not geometry actually changed.
    ++resyncs_;
    const std::string p = path_;  // local copy: open() writes path_ itself
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    if (!open(p)) {
      // Re-creation is non-atomic; an unreadable header here just means we
      // caught the writer mid-recreate. Stay unmapped — the retry block at
      // the top of next() above keeps trying on every subsequent poll, and
      // only escalates to dead_ past kReopenBudgetMs. This call already
      // observed a real discontinuity (the epoch moved), so report it now.
      const uint64_t now = now_monotonic_ms();
      if (reopen_fail_ms_ == 0) reopen_fail_ms_ = now;
      return Res::kResync;
    }
    return Res::kResync;
  }
  const uint64_t wseq = load64(map_ + kOffWriteSeq);
  // Fallback for epoch_ == 0 (pre-epoch, e.g. PR-A maburgs still deployed):
  // write_seq went backwards against itself, not against cursor_. Cursor can
  // legitimately exceed wseq after an overrun (writer stores lock before
  // write_seq, so reader can lap-ahead). Only write_seq regressing signals
  // ring re-creation here — and it still misses the same missed-restart
  // window the epoch check above closes for epoch-stamped rings.
  if (wseq < last_wseq_) {
    ++resyncs_;
    cursor_ = wseq > geom_.slot_count ? wseq - geom_.slot_count : 0;
    last_wseq_ = wseq;
    return Res::kResync;
  }
  last_wseq_ = wseq;
  if (cursor_ >= wseq) return Res::kNone;
  const uint8_t* slot = slot_base_(cursor_);
  const uint32_t l1 = load32(slot + kSOffLock);
  if (l1 & 1) return Res::kNone;  // mid-write; caller retries
  AuRecordMeta m;
  m.len = get32(slot + kSOffLen);
  m.rec_no = get64(slot + kSOffRecNo);
  m.frame_id64 = get64(slot + kSOffFrameId);
  m.pts_us = get32(slot + kSOffPts);
  m.sid = slot[kSOffSid];
  m.flags = slot[kSOffFlags];
  m.codec = slot[kSOffCodec];
  if (m.len > geom_.slot_bytes) {  // torn beyond repair
    ++resyncs_;
    cursor_ = wseq > geom_.slot_count ? wseq - geom_.slot_count : 0;
    return Res::kResync;
  }
  payload->assign(slot + kAuSlotHdrBytes, slot + kAuSlotHdrBytes + m.len);
  // Ensure payload copy completes before checking l2.
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  const uint32_t l2 = load32_relaxed(slot + kSOffLock);
  if (l1 != l2) {  // writer landed on this slot mid-copy: overrun by a lap
    ++resyncs_;
    cursor_ = wseq > geom_.slot_count ? wseq - geom_.slot_count : 0;
    return Res::kResync;
  }
  if (m.rec_no > cursor_) {
    // Slot already holds a newer lap: records [cursor_, m.rec_no) are gone.
    ++resyncs_;
    *meta = m;
    cursor_ = m.rec_no + 1;
    return Res::kOk;
  }
  if (m.rec_no < cursor_) return Res::kNone;  // stale slot; wait for writer
  *meta = m;
  ++cursor_;
  return Res::kOk;
}

}  // namespace maburgs
