#include "au_ring.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cstring>
#include <utility>

namespace maburgs {
namespace {

// RingHdr field offsets (see au_ring.h layout comment / ausniff.py mirror).
constexpr size_t kOffMagic = 0, kOffVersion = 4, kOffSlotBytes = 8,
                 kOffSlotCount = 12, kOffWriteSeq = 16, kOffDropped = 24,
                 kOffEpoch = 32, kOffOpenRec = 40;
// SlotHdr field offsets.
constexpr size_t kSOffLock = 0, kSOffLen = 4, kSOffRecNo = 8,
                 kSOffFrameId = 16, kSOffPts = 24, kSOffSid = 28,
                 kSOffFlags = 29;
// SlotHdr v2/v3 additions.
constexpr size_t kSOffTFirst = 32, kSOffTComplete = 40, kSOffDroneQ = 48,
                 kSOffEncUs = 50, kSOffAirMs = 52;
// Ring v4: open-slot publication.
constexpr size_t kSOffState = 54, kSOffNslices = 55, kSOffValidLen = 56;

uint8_t load8(const uint8_t* p) { return __atomic_load_n(p, __ATOMIC_ACQUIRE); }
void store8(uint8_t* p, uint8_t v) { __atomic_store_n(p, v, __ATOMIC_RELEASE); }
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
void put16(uint8_t* p, uint16_t v) { std::memcpy(p, &v, 2); }
void put32(uint8_t* p, uint32_t v) { std::memcpy(p, &v, 4); }
void put64(uint8_t* p, uint64_t v) { std::memcpy(p, &v, 8); }
uint16_t get16(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
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
uint64_t now_monotonic_us() { return now_monotonic_ns() / 1000ull; }

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

void AuRingWriter::close_slot_(uint8_t state) {
  store8(slot_ + kSOffState, state);
  store32(slot_ + kSOffLock, lock_ + 1);  // even: stable, release
  store64(map_ + kOffOpenRec, 0);
  in_au_ = false;
}

void AuRingWriter::begin(const mabur::framewire::FrameHdr& h, uint8_t sid, uint8_t nslices) {
  if (!map_) return;
  if (in_au_) close_slot_(kSlotAborted);  // begin without finish: that AU never publishes
  hdr_ = h;
  sid_ = sid;
  nslices_ = nslices;
  // frame_id64: FrameStream's already-ordered stream, whose u16 frame_id the
  // writer unwraps monotonically. Committed to last_id_ only at a successful
  // finish(), so an aborted AU does not move the unwrap reference.
  const uint16_t prev = static_cast<uint16_t>(last_id_);
  const uint16_t d = static_cast<uint16_t>(h.frame_id - prev);
  cur_id64_ = have_id_ ? last_id_ + static_cast<uint64_t>(d)
                       : static_cast<uint64_t>(h.frame_id);
  slot_ = slot_base_(published_);
  const uint32_t old = load32(slot_ + kSOffLock);
  lock_ = (old & 1u) ? old + 2 : old + 1;  // odd: write in progress, for the whole AU
  store32_relaxed(slot_ + kSOffLock, lock_);
  // The odd lock is visible before any slot write (seqlock writer side).
  __atomic_thread_fence(__ATOMIC_RELEASE);
  put32(slot_ + kSOffLen, 0);
  put64(slot_ + kSOffRecNo, published_);
  put64(slot_ + kSOffFrameId, cur_id64_);
  put32(slot_ + kSOffPts, h.pts_us);
  slot_[kSOffSid] = sid;
  slot_[kSOffFlags] = h.flags;
  slot_[kSOffNslices] = nslices;
  store32(slot_ + kSOffValidLen, 0);
  store8(slot_ + kSOffState, kSlotOpen);
  copied_ = 0;
  valid_ = 0;
  overflow_ = false;
  in_au_ = true;
  // Release: a reader that sees open_rec sees the odd lock and the header.
  store64(map_ + kOffOpenRec, published_ + 1);
}

bool AuRingWriter::copy_(const uint8_t* p, size_t n) {
  if (!in_au_ || overflow_ || n == 0) return false;
  if (static_cast<size_t>(copied_) + n > geom_.slot_bytes) {
    // Never truncated into the slot: abort now so a reader following this
    // AU ends it at once, not at finish().
    overflow_ = true;
    store8(slot_ + kSOffState, kSlotAborted);
    return false;
  }
  std::memcpy(slot_ + kAuSlotHdrBytes + copied_, p, n);
  copied_ += static_cast<uint32_t>(n);
  return true;
}

void AuRingWriter::append(const uint8_t* p, size_t n) {
  if (!copy_(p, n)) return;
  valid_ = copied_;
  store32(slot_ + kSOffValidLen, valid_);  // release: the bytes above first
}

void AuRingWriter::append_unaligned(const uint8_t* p, size_t n) {
  copy_(p, n);  // valid_len stays put: an open-slot reader never sees these bytes
}

uint64_t AuRingWriter::finish(bool complete, const AuLatMeta& lat) {
  if (!map_ || !in_au_) return UINT64_MAX;
  if (overflow_) {
    ++dropped_oversize_;
    store64(map_ + kOffDropped, dropped_oversize_);
    close_slot_(kSlotAborted);  // write_seq does not move: nothing published
    return UINT64_MAX;
  }
  const uint64_t n = published_;
  const uint8_t rec_flags = static_cast<uint8_t>(
      hdr_.flags | (complete ? kRecFlagComplete : 0) |
      (!complete && lat.slice.salvaged ? kRecFlagSliceSalvaged : 0));
  AuLatMeta l = lat;
  if (l.t_complete_us == 0) l.t_complete_us = now_monotonic_us();
  put32(slot_ + kSOffLen, copied_);
  slot_[kSOffFlags] = rec_flags;
  put64(slot_ + kSOffTFirst, l.t_first_us);
  put64(slot_ + kSOffTComplete, l.t_complete_us);
  put16(slot_ + kSOffDroneQ, l.drone_q_ms);
  put16(slot_ + kSOffEncUs, l.enc_us);
  put16(slot_ + kSOffAirMs, l.drone_air_ms);
  store8(slot_ + kSOffState, kSlotClosed);
  store32(slot_ + kSOffLock, lock_ + 1);  // even: stable, release
  in_au_ = false;
  last_id_ = cur_id64_;
  have_id_ = true;
  ++published_;
  bytes_published_ += copied_;
  store64(map_ + kOffWriteSeq, published_);
  store64(map_ + kOffOpenRec, 0);
  last_.rec_no = n;
  last_.frame_id64 = cur_id64_;
  last_.pts_us = hdr_.pts_us;
  last_.len = copied_;
  last_.sid = sid_;
  last_.flags = rec_flags;
  last_.nslices = nslices_;
  last_.t_first_us = l.t_first_us;
  last_.t_complete_us = l.t_complete_us;
  last_.drone_q_ms = l.drone_q_ms;
  last_.enc_us = l.enc_us;
  last_.drone_air_ms = l.drone_air_ms;
  last_.slice = lat.slice;
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
  // Ring v4: slot (wseq % slot_count) may be one maburgs is filling (open)
  // or one whose AU it aborted -- either way it no longer holds record
  // wseq - slot_count. Start one later rather than open on a lap.
  if (wseq >= geom_.slot_count &&
      (load64(map_ + kOffOpenRec) != 0 || load8(slot_base_(wseq) + kSOffState) != kSlotClosed))
    cursor_ = wseq - geom_.slot_count + 1;
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

uint64_t AuRingReader::debug_wseq() const {
  return map_ ? load64(map_ + kOffWriteSeq) : 0;
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
  if (l1 & 1) return Res::kNone;  // mid-write (v4: possibly a whole AU long); caller retries
  AuRecordMeta m;
  m.len = get32(slot + kSOffLen);
  m.rec_no = get64(slot + kSOffRecNo);
  m.frame_id64 = get64(slot + kSOffFrameId);
  m.pts_us = get32(slot + kSOffPts);
  m.sid = slot[kSOffSid];
  m.flags = slot[kSOffFlags];
  m.nslices = slot[kSOffNslices];
  m.t_first_us = get64(slot + kSOffTFirst);
  m.t_complete_us = get64(slot + kSOffTComplete);
  m.drone_q_ms = get16(slot + kSOffDroneQ);
  m.enc_us = get16(slot + kSOffEncUs);
  m.drone_air_ms = get16(slot + kSOffAirMs);
  const uint8_t state = slot[kSOffState];
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
  if (state != kSlotClosed) {
    // Ring v4: a stable slot that is not a closed record is one claimed for
    // rec wseq whose AU overflowed (aborted). The record the cursor wanted
    // from it is gone; the next slot_count - 1 are intact.
    ++resyncs_;
    cursor_ = wseq >= geom_.slot_count ? wseq - geom_.slot_count + 1 : wseq;
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

int AuRingReader::grow_(const uint8_t* slot, uint32_t lock, OpenView* v) const {
  const size_t have = v->bytes.size();
  const uint32_t vlen = load32(slot + kSOffValidLen);
  if (vlen > geom_.slot_bytes) return -1;
  if (vlen > have)
    v->bytes.insert(v->bytes.end(), slot + kAuSlotHdrBytes + have, slot + kAuSlotHdrBytes + vlen);
  // Bytes below valid_len never change within one open instance; the lock
  // re-check proves the instance (and the header read before it) held still.
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  if (load32_relaxed(slot + kSOffLock) != lock) {
    v->bytes.resize(have);
    return -1;
  }
  if (vlen <= have) return 0;
  v->meta.len = vlen;
  return 1;
}

AuRingReader::OpenRes AuRingReader::peek_open(OpenView* v) {
  if (dead_ || !map_) return OpenRes::kNone;
  if (get64(map_ + kOffEpoch) != epoch_) return OpenRes::kNone;      // next() resyncs first
  if (cursor_ != load64(map_ + kOffWriteSeq)) return OpenRes::kNone; // closed records first
  if (v->lock != 0 && v->meta.rec_no != cursor_) *v = OpenView{};    // caller skipped a reset
  const uint8_t* slot = slot_base_(cursor_);
  const uint64_t open_rec = load64(map_ + kOffOpenRec);
  const uint32_t l1 = load32(slot + kSOffLock);
  const uint8_t state = load8(slot + kSOffState);
  if (v->lock != 0) {
    const bool same = l1 == v->lock;
    // Ended without closing: overflowed in place (same instance, aborted),
    // aborted and stable, or re-begun by the next AU (another odd lock).
    if ((same && state == kSlotAborted) || (!same && ((l1 & 1u) != 0 || state == kSlotAborted))) {
      v->bytes.clear();
      v->lock = 0;
      return OpenRes::kAborted;
    }
    if (!same) return OpenRes::kNone;  // closed: next() delivers it
    return grow_(slot, l1, v) > 0 ? OpenRes::kGrow : OpenRes::kNone;
  }
  if (open_rec != cursor_ + 1 || (l1 & 1u) == 0 || state != kSlotOpen) return OpenRes::kNone;
  OpenView fresh;
  fresh.meta.rec_no = get64(slot + kSOffRecNo);
  fresh.meta.frame_id64 = get64(slot + kSOffFrameId);
  fresh.meta.pts_us = get32(slot + kSOffPts);
  fresh.meta.sid = slot[kSOffSid];
  fresh.meta.flags = slot[kSOffFlags];
  fresh.meta.nslices = slot[kSOffNslices];
  fresh.lock = l1;
  if (grow_(slot, l1, &fresh) < 0) return OpenRes::kNone;  // moved on mid-read
  if (fresh.meta.rec_no != cursor_) return OpenRes::kNone;
  *v = std::move(fresh);
  return OpenRes::kOpen;
}

}  // namespace maburgs
