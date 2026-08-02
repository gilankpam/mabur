#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/frame_wire.h"

namespace maburgs {

// Shared-memory access-unit ring: maburgs publishes whole reassembled AUs,
// maburplay/ausniff consume. Fixed-slot seqlock design; layout is mirrored
// byte-for-byte by tools/bench/ausniff.py — change both together.
// Spec: docs/superpowers/specs/2026-08-02-gs-player-au-ring-design.md.
//
// RingHdr (little-endian, kAuRingHdrBytes-byte header page):
//    0 u32 magic            — 'AUBM' (kAuRingMagic)
//    4 u32 version           — kAuRingVersion
//    8 u32 slot_bytes
//   12 u32 slot_count
//   16 u64 write_seq         — records ever published; release-stored last
//   24 u64 dropped_oversize
//   32 u64 epoch             — writer boot stamp (CLOCK_MONOTONIC ns | 1),
//                              nonzero; re-stamped whenever the ring file is
//                              (re)created, independent of write_seq — this
//                              is what lets a reader detect a writer restart
//                              that climbed past its cursor before the next
//                              poll (see AuRingReader::next, kResync). 0 in
//                              the header means a pre-epoch (PR-A) writer.
//                              Ring re-creation is NOT atomic (ftruncate,
//                              then memset, then geometry, then epoch, then
//                              magic last/release): a reader polling mid-
//                              recreate can see a torn header, and if the
//                              new geometry is a SHRINK, a reader still
//                              holding the old (larger) mapping is not
//                              guaranteed a consistent view until it
//                              re-opens on the next epoch-mismatch poll.
// SlotHdr (64-byte header per slot, offsets relative to the slot base):
//    0 u32 lock              — seqlock word; odd = write in progress
//    4 u32 len
//    8 u64 rec_no
//   16 u64 frame_id64
//   24 u32 pts_us
//   28 u8  sid
//   29 u8  flags             — idr|discont|complete (kRecFlagComplete)
//   30 u8  codec
// followed by slot_bytes of Annex-B AU payload.
inline constexpr uint32_t kAuRingMagic = 0x4D425541;  // "AUBM" LE
inline constexpr uint32_t kAuRingVersion = 1;
inline constexpr size_t kAuRingHdrBytes = 4096;
inline constexpr size_t kAuSlotHdrBytes = 64;
// ORed into the record flags byte next to framewire kFlagIdr/kFlagDiscont.
// ring-local bit, deliberately at the TOP of the byte; low bits remain
// framewire's namespace (kFlagIdr 0x01, kFlagDiscont 0x02, future framewire
// bits grow upward from there).
inline constexpr uint8_t kRecFlagComplete = 0x80;

struct AuRingGeom {
  uint32_t slot_bytes = 512 * 1024;
  uint32_t slot_count = 16;
};

struct AuRecordMeta {
  uint64_t rec_no = 0;
  uint64_t frame_id64 = 0;
  uint32_t pts_us = 0;
  uint32_t len = 0;
  uint8_t sid = 0;
  uint8_t flags = 0;
  uint8_t codec = 0;
};

// Creates/truncates the ring file and publishes AUs accumulated between
// begin()/finish(). Never blocks on readers: slot rec_no % slot_count is
// overwritten unconditionally. An AU larger than slot_bytes is dropped whole
// (never truncated into a slot) and counted.
class AuRingWriter {
 public:
  AuRingWriter() = default;
  ~AuRingWriter();
  AuRingWriter(const AuRingWriter&) = delete;
  AuRingWriter& operator=(const AuRingWriter&) = delete;

  bool open(const std::string& path, AuRingGeom geom);
  AuRingGeom geom() const { return geom_; }  // effective (post-alignment) geometry — the header/hello contract
  void begin(const mabur::framewire::FrameHdr& h, uint8_t sid);
  void append(const uint8_t* p, size_t n);
  uint64_t finish(bool complete);  // rec_no, or UINT64_MAX if dropped/no-op
  bool ok() const { return map_ != nullptr; }
  uint64_t published() const { return published_; }
  uint64_t dropped_oversize() const { return dropped_oversize_; }

 private:
  uint8_t* slot_base_(uint64_t rec_no) const;
  uint8_t* map_ = nullptr;
  size_t map_bytes_ = 0;
  AuRingGeom geom_;
  std::vector<uint8_t> au_;
  mabur::framewire::FrameHdr hdr_;
  uint8_t sid_ = 0;
  bool in_au_ = false;
  uint64_t published_ = 0;
  uint64_t dropped_oversize_ = 0;
  uint64_t last_id_ = 0;
  bool have_id_ = false;
};

// Maps an existing ring read-only and iterates records seqlock-safely.
// open() positions the cursor at the oldest record still guaranteed
// retained (write_seq - slot_count), so a post-hoc reader sees everything
// a fresh ring holds and a live reader starts near the tail.
class AuRingReader {
 public:
  enum class Res { kNone, kOk, kResync };
  AuRingReader() = default;
  ~AuRingReader();
  AuRingReader(const AuRingReader&) = delete;
  AuRingReader& operator=(const AuRingReader&) = delete;

  bool open(const std::string& path);
  Res next(AuRecordMeta* meta, std::vector<uint8_t>* payload);
  uint64_t resyncs() const { return resyncs_; }
  AuRingGeom geom() const { return geom_; }
  bool dead() const { return dead_; }

 private:
  const uint8_t* slot_base_(uint64_t rec_no) const;
  uint8_t* map_ = nullptr;
  size_t map_bytes_ = 0;
  AuRingGeom geom_;
  uint64_t cursor_ = 0;
  uint64_t resyncs_ = 0;
  uint64_t last_wseq_ = 0;
  uint64_t epoch_ = 0;      // latched at open; 0 = ring predates epochs
  bool dead_ = false;
  std::string path_;        // for geometry-change reopen
  // CLOCK_MONOTONIC ms timestamp of the first consecutive failed reopen
  // attempt; 0 = not currently failing. Ring re-creation is non-atomic (see
  // the layout comment above), so a failed reopen is expected transiently
  // during a writer restart — dead_ only latches after reopen keeps failing
  // past a budget (see next()).
  uint64_t reopen_fail_ms_ = 0;
};

}  // namespace maburgs
