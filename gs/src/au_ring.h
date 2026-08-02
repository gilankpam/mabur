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
inline constexpr uint32_t kAuRingMagic = 0x4D425541;  // "AUBM" LE
inline constexpr uint32_t kAuRingVersion = 1;
inline constexpr size_t kAuRingHdrBytes = 4096;
inline constexpr size_t kAuSlotHdrBytes = 64;
// ORed into the record flags byte next to framewire kFlagIdr/kFlagDiscont.
inline constexpr uint8_t kRecFlagComplete = 0x04;

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

 private:
  const uint8_t* slot_base_(uint64_t rec_no) const;
  uint8_t* map_ = nullptr;
  size_t map_bytes_ = 0;
  AuRingGeom geom_;
  uint64_t cursor_ = 0;
  uint64_t resyncs_ = 0;
};

}  // namespace maburgs
