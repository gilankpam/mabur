#include "au_ring.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>

namespace maburgs {
namespace {

// RingHdr field offsets (see au_ring.h layout comment / ausniff.py mirror).
constexpr size_t kOffMagic = 0, kOffVersion = 4, kOffSlotBytes = 8,
                 kOffSlotCount = 12, kOffWriteSeq = 16, kOffDropped = 24;
// SlotHdr field offsets.
constexpr size_t kSOffLock = 0, kSOffLen = 4, kSOffRecNo = 8,
                 kSOffFrameId = 16, kSOffPts = 24, kSOffSid = 28,
                 kSOffFlags = 29, kSOffCodec = 30;

uint32_t load32(const uint8_t* p) {
  return __atomic_load_n(reinterpret_cast<const uint32_t*>(p), __ATOMIC_ACQUIRE);
}
uint64_t load64(const uint8_t* p) {
  return __atomic_load_n(reinterpret_cast<const uint64_t*>(p), __ATOMIC_ACQUIRE);
}
void store32(uint8_t* p, uint32_t v) {
  __atomic_store_n(reinterpret_cast<uint32_t*>(p), v, __ATOMIC_RELEASE);
}
void store64(uint8_t* p, uint64_t v) {
  __atomic_store_n(reinterpret_cast<uint64_t*>(p), v, __ATOMIC_RELEASE);
}
void put32(uint8_t* p, uint32_t v) { std::memcpy(p, &v, 4); }
void put64(uint8_t* p, uint64_t v) { std::memcpy(p, &v, 8); }
uint32_t get32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
uint64_t get64(const uint8_t* p) { uint64_t v; std::memcpy(&v, p, 8); return v; }

size_t ring_bytes(const AuRingGeom& g) {
  return kAuRingHdrBytes +
         static_cast<size_t>(g.slot_count) * (kAuSlotHdrBytes + g.slot_bytes);
}

uint8_t* map_file(const std::string& path, size_t bytes, bool create, size_t* out_bytes) {
  int fd = ::open(path.c_str(), create ? (O_RDWR | O_CREAT) : O_RDWR, 0644);
  if (fd < 0) return nullptr;
  if (create && ::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
    ::close(fd);
    return nullptr;
  }
  if (!create) {
    struct stat st;
    if (::fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) < kAuRingHdrBytes) {
      ::close(fd);
      return nullptr;
    }
    bytes = static_cast<size_t>(st.st_size);
  }
  void* m = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
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
  map_ = map_file(path, ring_bytes(geom), /*create=*/true, &map_bytes_);
  if (!map_) return false;
  geom_ = geom;
  std::memset(map_, 0, map_bytes_);
  put32(map_ + kOffVersion, kAuRingVersion);
  put32(map_ + kOffSlotBytes, geom.slot_bytes);
  put32(map_ + kOffSlotCount, geom.slot_count);
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
  store32(slot + kSOffLock, lock + 1);  // odd: write in progress
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
  store32(slot + kSOffLock, lock + 2);  // even: stable
  ++published_;
  store64(map_ + kOffWriteSeq, published_);
  au_.clear();
  return n;
}

AuRingReader::~AuRingReader() {
  if (map_) ::munmap(map_, map_bytes_);
}

bool AuRingReader::open(const std::string& path) {
  map_ = map_file(path, 0, /*create=*/false, &map_bytes_);
  if (!map_) return false;
  if (load32(map_ + kOffMagic) != kAuRingMagic ||
      get32(map_ + kOffVersion) != kAuRingVersion) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    return false;
  }
  geom_.slot_bytes = get32(map_ + kOffSlotBytes);
  geom_.slot_count = get32(map_ + kOffSlotCount);
  const size_t need = kAuRingHdrBytes + static_cast<size_t>(geom_.slot_count) *
                                            (kAuSlotHdrBytes + geom_.slot_bytes);
  if (geom_.slot_bytes == 0 || geom_.slot_count == 0 || map_bytes_ < need) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    return false;
  }
  const uint64_t wseq = load64(map_ + kOffWriteSeq);
  cursor_ = wseq > geom_.slot_count ? wseq - geom_.slot_count : 0;
  return true;
}

const uint8_t* AuRingReader::slot_base_(uint64_t rec_no) const {
  return map_ + kAuRingHdrBytes +
         (rec_no % geom_.slot_count) *
             (kAuSlotHdrBytes + static_cast<size_t>(geom_.slot_bytes));
}

AuRingReader::Res AuRingReader::next(AuRecordMeta* meta,
                                     std::vector<uint8_t>* payload) {
  if (!map_) return Res::kNone;
  const uint64_t wseq = load64(map_ + kOffWriteSeq);
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
  if (m.len > geom_.slot_bytes) return Res::kResync;  // torn beyond repair
  payload->assign(slot + kAuSlotHdrBytes, slot + kAuSlotHdrBytes + m.len);
  const uint32_t l2 = load32(slot + kSOffLock);
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
