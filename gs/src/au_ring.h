#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/frame_wire.h"

namespace maburgs {

// Shared-memory access-unit ring: maburgs publishes reassembled AUs,
// maburplay/ausniff consume. Fixed-slot seqlock design; layout is mirrored
// byte-for-byte by tools/bench/ausniff.py and tools/bench/aucadence.py --
// change all three together.
// Spec: docs/superpowers/specs/2026-08-02-gs-player-au-ring-design.md;
// ring v4 (a slot is readable while maburgs fills it):
// docs/superpowers/specs/2026-10-10-h265-slices-design.md §6.2.
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
//   40 u64 open_rec          — v4: rec_no + 1 of the slot maburgs is filling
//                              right now, 0 = none. Store-released after the
//                              slot's header, so a reader that sees it sees
//                              the odd lock and the header too.
// SlotHdr (64-byte header per slot, offsets relative to the slot base):
//    0 u32 lock              — seqlock word; odd = write in progress. v4: it
//                              stays odd from begin() to finish(), the whole
//                              time the AU fills
//    4 u32 len               — the closed record's length (0 while open)
//    8 u64 rec_no
//   16 u64 frame_id64
//   24 u32 pts_us
//   28 u8  sid
//   29 u8  flags             — idr|discont from begin(); complete
//                              (kRecFlagComplete) | salvaged at finish()
//   30 u8  padding           — the codec byte until v4
//   31 u8  padding
//   32 u64 t_first_us        — SlotHdr v2 (kAuRingVersion 2): mono µs at the
//                              first body of the AU (0 = unknown)
//   40 u64 t_complete_us     — mono µs at finish(); writer stamps now() if
//                              the caller passed 0
//   48 u16 drone_q_ms        — from SBI q_ms via the AU's first fragment
//   50 u16 enc_us            — from SBI enc_us, same latch
//   52 u16 drone_air_ms      — from SBI air_ms, same latch (SlotHdr v3, 2026-09-06)
//   54 u8  state             — v4: SlotState (closed/empty, open, aborted)
//   55 u8  nslices           — v4: slices of the picture, 0/1 = whole AU
//   56 u32 valid_len         — v4: payload [0, valid_len) is final; store-
//                              released after every append(), never shrinks
//                              within one open instance (one lock value).
//                              For nslices >= 2 it always ends on a NAL
//                              boundary (writer contract; append_unaligned()
//                              bytes count in len, not in valid_len)
// (60..63 padding; SlotHdr stays 64 B)
// followed by slot_bytes of Annex-B AU payload.
inline constexpr uint32_t kAuRingMagic = 0x4D425541;  // "AUBM" LE
inline constexpr uint32_t kAuRingVersion = 4;
inline constexpr size_t kAuRingHdrBytes = 4096;
inline constexpr size_t kAuSlotHdrBytes = 64;
// ORed into the record flags byte next to framewire kFlagIdr/kFlagDiscont.
// ring-local bit, deliberately at the TOP of the byte; low bits remain
// framewire's namespace (kFlagIdr 0x01, kFlagDiscont 0x02, future framewire
// bits grow upward from there).
inline constexpr uint8_t kRecFlagComplete = 0x80;

// Slice salvage (spec 2026-10-10-h265-slices §5): an AU FrameStream could
// not complete, rebuilt as a decodable picture -- every slice that arrived
// whole, plus one synthetic all-skip fill per lost slice. Ring-local flag,
// next to kRecFlagComplete at the top of the byte. A salvaged AU is never
// also complete; consumers decode either (au_decodable), and only a
// complete one may arm a decoder.
enum SliceFallback : uint8_t { kSliceFbNone = 0, kSliceFbNoParams, kSliceFbUnsupported,
                               kSliceFbISlice, kSliceFbNoTemplate, kSliceFbGeometry, kSliceFbCount };
struct SliceSalvage { bool salvaged = false; uint8_t slices = 0, kept = 0, filled = 0,
                      kept_after_hole = 0; uint8_t fallback = kSliceFbNone; };
inline constexpr uint8_t kRecFlagSliceSalvaged = 0x40;
inline bool au_decodable(uint8_t flags) {
  return (flags & (kRecFlagComplete | kRecFlagSliceSalvaged)) != 0; }

// SlotHdr state byte (v4). closed = a published record, or a slot never
// used; open = begin() done, finish() not yet; aborted = the AU outgrew the
// slot (or a begin() came without a finish()) and was never published.
enum SlotState : uint8_t { kSlotClosed = 0, kSlotOpen = 1, kSlotAborted = 2 };

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
  uint8_t nslices = 0;  // v4: slices of the picture, 0/1 = whole AU
  uint64_t t_first_us = 0;
  uint64_t t_complete_us = 0;
  uint16_t drone_q_ms = 0;
  uint16_t enc_us = 0;
  uint16_t drone_air_ms = 0;
  // Writer side only: last_record() copies it from AuLatMeta; readers leave
  // it default.
  SliceSalvage slice;
};

// GS-side per-AU latency stamps (SlotHdr v2). Passed to finish() so the
// writer stamps them inside the same seqlock window as the rest of the
// slot header; Task 8 (FrameStream) is the real producer, Task 9
// (PtsAnchor) and Task 10 (sideport) the consumers.
struct AuLatMeta {
  uint64_t t_first_us = 0;    // mono µs, first body of the AU (0 = unknown)
  uint64_t t_complete_us = 0; // mono µs at finish (writer stamps if 0)
  // mono µs, LAST body arrival of the AU (max over fragments' body_mono_us;
  // same latch site as t_first_us). au_tail gauge only (usb-feed probe
  // 2026-09-01): splits fec into arrival span (t_last - t_first) vs
  // publish tail (finish - t_last: repair/decode/assembly/ring write).
  // Not written to the SlotHdr — internal to the writer's gauge.
  uint64_t t_last_arr_us = 0;
  uint16_t drone_q_ms = 0;    // from SBI q_ms via the AU's first fragment
  uint16_t enc_us = 0;        // from SBI enc_us, same latch
  uint16_t drone_air_ms = 0;  // from SBI air_ms, same latch as drone_q_ms
  // fragment 0 of this AU arrived retx-marked (fec-nack, Task 5/7): a NACK
  // retransmit filled the AU's header fragment, so the latency-anchor guard
  // (Task 7) must not trust this AU's arrival time as a clean anchor sample.
  bool hdr_retx = false;
  // Slice salvage (spec 2026-10-10-h265-slices §5): FrameStream's verdict
  // for this AU. finish() turns slice.salvaged into kRecFlagSliceSalvaged;
  // the counts ride to AuLog through last_record(). Not in the SlotHdr.
  SliceSalvage slice;
};

// Creates/truncates the ring file and publishes AUs. Never blocks on
// readers: slot rec_no % slot_count is overwritten unconditionally. v4: the
// slot is claimed at begin() and readable while it fills (peek_open); an AU
// larger than slot_bytes is aborted as soon as it outgrows the slot (never
// truncated into it), counted, and never published.
class AuRingWriter {
 public:
  AuRingWriter() = default;
  ~AuRingWriter();
  AuRingWriter(const AuRingWriter&) = delete;
  AuRingWriter& operator=(const AuRingWriter&) = delete;

  bool open(const std::string& path, AuRingGeom geom);
  AuRingGeom geom() const { return geom_; }  // effective (post-alignment) geometry — the header/hello contract
  // Claims slot published() and publishes it OPEN (state, header, open_rec).
  // nslices: the picture's slice count when it arrives slice by slice, 0/1 =
  // whole AU. A begin() while another AU is open aborts that one.
  void begin(const mabur::framewire::FrameHdr& h, uint8_t sid, uint8_t nslices = 1);
  // Copies into the slot and store-releases valid_len = every byte copied so
  // far. The append that would outgrow slot_bytes aborts the AU instead
  // (state = aborted, nothing more copied); finish() then counts it
  // dropped_oversize and publishes nothing.
  // Contract (spec §6.2): for an AU begun with nslices >= 2, every append()
  // ends on a NAL boundary, so valid_len of a split AU always does.
  void append(const uint8_t* p, size_t n);
  // Copies into the slot without advancing valid_len: part of the closed
  // record (len), never visible to an open-slot reader. For the one piece
  // that may end mid-NAL -- SliceAssembler's passthrough remainder at finish.
  // Same overflow rule as append().
  void append_unaligned(const uint8_t* p, size_t n);
  // rec_no, or UINT64_MAX if dropped/no-op. No default AuLatMeta: every
  // caller must decide (pass AuLatMeta{} to stamp nothing but t_complete_us).
  uint64_t finish(bool complete, const AuLatMeta& lat);
  bool ok() const { return map_ != nullptr; }
  // Records published so far -- also the rec_no of the AU being filled.
  uint64_t published() const { return published_; }
  uint64_t dropped_oversize() const { return dropped_oversize_; }
  uint64_t bytes_published() const { return bytes_published_; }
  // The meta of the most recently published record, exactly as written into
  // the slot (flags include kRecFlagComplete; t_complete_us is post
  // zero-substitution). Undefined before the first successful finish().
  // AuLog's row source: the writer already unwrapped frame_id to 64 bits
  // here, and duplicating that in a second place would be a second copy of a
  // subtle invariant.
  const AuRecordMeta& last_record() const { return last_; }

 private:
  uint8_t* slot_base_(uint64_t rec_no) const;
  void close_slot_(uint8_t state);  // even lock, open_rec 0; write_seq untouched
  bool copy_(const uint8_t* p, size_t n);  // into the slot at copied_; false = no-op/overflow
  uint8_t* map_ = nullptr;
  size_t map_bytes_ = 0;
  AuRingGeom geom_;
  mabur::framewire::FrameHdr hdr_;
  uint8_t sid_ = 0;
  uint8_t nslices_ = 0;
  bool in_au_ = false;
  bool overflow_ = false;
  uint8_t* slot_ = nullptr;   // the slot being filled (valid while in_au_)
  uint32_t lock_ = 0;         // its odd seqlock value
  uint32_t copied_ = 0;       // payload bytes in the slot (the record's len)
  uint32_t valid_ = 0;        // of them, published as valid_len
  uint64_t cur_id64_ = 0;     // frame_id64 of the AU being filled (committed at finish)
  uint64_t published_ = 0;
  uint64_t dropped_oversize_ = 0;
  uint64_t bytes_published_ = 0;
  uint64_t last_id_ = 0;
  bool have_id_ = false;
  AuRecordMeta last_;
};

// Maps an existing ring read-only and iterates records seqlock-safely.
// open() positions the cursor at the oldest record still guaranteed
// retained (write_seq - slot_count, one later when that slot is being
// filled or was aborted), so a post-hoc reader sees everything a fresh ring
// holds and a live reader starts near the tail.
class AuRingReader {
 public:
  enum class Res { kNone, kOk, kResync };
  enum class OpenRes { kNone, kOpen, kGrow, kAborted };
  // The record at the read cursor while maburgs is still filling it.
  struct OpenView {
    AuRecordMeta meta;           // len = bytes.size(); flags: framewire bits only
    std::vector<uint8_t> bytes;  // payload [0, valid_len) copied so far
    uint32_t lock = 0;           // seqlock value of the instance followed; 0 = none
  };
  AuRingReader() = default;
  ~AuRingReader();
  AuRingReader(const AuRingReader&) = delete;
  AuRingReader& operator=(const AuRingReader&) = delete;

  bool open(const std::string& path);
  Res next(AuRecordMeta* meta, std::vector<uint8_t>* payload);
  // Ring v4. Call after next() returned kNone, with the same view each time:
  //  kOpen    first sight of the record being filled; *v = its meta and
  //           the bytes so far (possibly none)
  //  kGrow    more bytes landed; v->bytes extended (cumulative)
  //  kAborted the record v followed will never close (it overflowed, or its
  //           slot was re-begun); v->meta still names it, bytes/lock cleared
  //  kNone    nothing new: no open record, no new bytes, or it just closed
  //           (next() delivers it)
  // Reset the view (*v = {}) whenever next() returns anything but kNone.
  OpenRes peek_open(OpenView* v);
  uint64_t resyncs() const { return resyncs_; }
  AuRingGeom geom() const { return geom_; }
  // Stall diagnostics only (racy reads are fine for logging).
  bool ok() const { return map_ != nullptr; }
  uint64_t debug_cursor() const { return cursor_; }
  uint64_t debug_wseq() const;
  bool dead() const { return dead_; }

 private:
  const uint8_t* slot_base_(uint64_t rec_no) const;
  // Copies payload [v->bytes.size(), valid_len) of the open instance `lock`.
  // +1 grew, 0 nothing new, -1 the slot moved on mid-copy (copy discarded).
  int grow_(const uint8_t* slot, uint32_t lock, OpenView* v) const;
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
