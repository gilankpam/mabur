#ifndef MABUR_PLAYER_LAT_TRACKER_H_
#define MABUR_PLAYER_LAT_TRACKER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "au_ring.h"     // maburgs::AuRecordMeta
#include "pts_anchor.h"  // maburgs::PtsAnchor -- the ONE anchor impl

namespace maburplay {

// Per-frame tail latency stamps + the additive 8-segment chain:
// enc,dq,air,fec,dec,reg,dsp,e2e (indices 0..7), same order as maburgs'
// head-segment sideport export and the OSD LAT row. enc/dq/air/fec are
// computed at on_submit() from AuRecordMeta the same way gs/src/main.cpp
// computes its own head segments (own PtsAnchor instance -- daemon and player
// are separate processes and do not share warm-up state, even though they run
// on the same box and the same CLOCK_MONOTONIC domain). dec/reg/dsp are filled
// in as the frame moves through decode -> regulator release -> vsync flip;
// e2e is the running sum. A frame that never completes the whole chain
// (dropped, flushed, or a wedged decoder that ages it out of the bounded
// map) is excluded from percentiles and simply forgotten -- there is
// nothing to count it against in the per-segment output.
//
// Keyed by pts_us (u32): MPP round-trips the submitted pts through decode,
// so it is the only correlator on_decoded()/on_present()/on_flip() have.
// The inflight map is a small bounded array (kMaxInflight slots,
// drop-oldest by insertion order) so a wedged decoder or a presenter that
// stops flipping cannot grow it -- a stuck pipeline just ages entries out.
class LatTracker {
 public:
  void on_submit(const maburgs::AuRecordMeta& m, uint64_t mono_us);
  void on_decoded(uint32_t pts_us, uint64_t mono_us);
  void on_present(uint32_t pts_us, uint64_t mono_us);  // t_release
  // Right after the presenter took the frame: when its atomic commit ioctl
  // returned (0 = no commit, the frame was parked) and the presenter's
  // commit bits (DrmPresenter::CommitInfo). Splits dsp into the commit's
  // own cost and the kernel's latch.
  void on_commit(uint32_t pts_us, uint64_t commit_mono_us, uint8_t bits);
  void on_flip(uint32_t pts_us, uint64_t flip_mono_us, bool exact);
  void on_drop(uint32_t pts_us);  // displaced/flushed: forget, count
  void flush_all();               // discont/flush points

  // One completed frame, as frame.log records it: the AU meta copied at
  // submit (join key pts, plus what au.log also carries so a row reads on
  // its own), every absolute stamp of the chain (mono µs) and the segment
  // tuple. t_submit precedes t_complete for a streamed frame (slices fed
  // as they arrive), so dec = t_decoded - t_complete can be split into the
  // ring-to-decoder hand-off and the decoder's own time.
  struct FrameRow {
    uint32_t pts_us = 0;
    uint64_t frame_id64 = 0;
    uint8_t sid = 0;
    uint32_t len = 0;
    uint8_t flags = 0;
    uint8_t nslices = 0;
    uint64_t t_submit_us = 0;
    uint64_t t_complete_us = 0;
    uint64_t t_decoded_us = 0;
    uint64_t t_release_us = 0;
    uint64_t t_flip_us = 0;
    bool exact = false;  // the flip stamp came from a real vblank event
    uint32_t seg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint64_t t_commit_us = 0;  // atomic commit returned (0 = parked, never committed)
    uint8_t pbits = 0;         // presenter commit bits (DrmPresenter::CommitInfo)
  };

  // 1 Hz consumption: aggregates + clears the completed-frame window. The
  // window's frames leave as `rows`, the same set the percentiles rank, so
  // frame.log and the lat: line always describe the same frames.
  struct Line {
    int n = 0;  // per-seg p50/p99 µs: enc,dq,air,fec,dec,reg,dsp,e2e
    uint32_t p50[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t p99[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double chk_ms = 0.0;   // mean of (t_flip - map(pts64)) - Σsegs
    bool anchor_ok = false;
    bool dsp_exact = false;
    std::vector<FrameRow> rows;
  };
  Line flush_line();

  // OSD rows: the REAL frame at p99- and p50-by-e2e rank from the last
  // window -- never a mix of per-segment percentiles (those can each come
  // from a different frame and would not sum to anything meaningful).
  // Both are ranked the SAME way for the same reason, so the two rows are
  // directly comparable segment by segment: p50 is the typical frame, p99
  // the tail, and each row's own segments sum to its own headline.
  struct Breakdown {
    bool valid = false;
    uint32_t ms[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // rounded ms, same order
  };
  Breakdown p99_frame() const;
  Breakdown p50_frame() const;

  static constexpr std::size_t kMaxInflight = 64;

  // link-rtt: the tracker's own anchor, read-only, for the
  // absolute-floor combination with the sideport pts offset
  // (maburgs::floor_us_from). Each process combines the offset with its
  // OWN anchor — the daemon's floor_ms export used ITS anchor and the two
  // warm up independently, so borrowing the exported floor here would mix
  // anchor domains.
  const maburgs::PtsAnchor& anchor() const { return anchor_; }

 private:
  struct Entry {
    bool valid = false;
    uint32_t pts_us = 0;
    uint64_t pts64 = 0;         // anchor-unwrapped pts, for map_us() at flip
    uint64_t t_complete_us = 0;
    uint64_t t_submit_us = 0;   // diagnostic only -- dec folds t_submit..t_complete in
    uint64_t t_decoded_us = 0;
    uint64_t t_release_us = 0;
    uint64_t t_commit_us = 0;
    uint8_t pbits = 0;
    uint64_t seq = 0;           // insertion order, for drop-oldest eviction
    bool anchor_ok_at_submit = false;
    bool has_decoded = false;
    bool has_present = false;
    uint32_t seg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // AU meta carried through to the FrameRow (not used by the chain).
    uint64_t frame_id64 = 0;
    uint32_t len = 0;
    uint8_t sid = 0;
    uint8_t flags = 0;
    uint8_t nslices = 0;
  };

  int find_(uint32_t pts_us) const;
  int alloc_(uint32_t pts_us);
  void invalidate_(int idx);
  static uint32_t percentile_(std::vector<uint32_t>& v, int pct);

  std::array<Entry, kMaxInflight> map_{};
  uint64_t next_seq_ = 0;

  maburgs::PtsAnchor anchor_;

  // Completed-frame window, since the last flush_line() (or construction).
  std::vector<FrameRow> completed_;
  double chk_sum_us_ = 0.0;
  int chk_n_ = 0;
  bool dsp_exact_window_ = true;

  Breakdown p99_frame_, p50_frame_;
};

// frame.log's format marker and one row's payload (everything after the
// writer's t_flip column), in the order docs/observability.md lists.
// snprintf semantics: returns the length the full row needs.
inline constexpr const char* kFrameLogHeader = "# framelog 3";
int format_frame_row(const LatTracker::FrameRow& r, char* buf, size_t n);

}  // namespace maburplay

#endif  // MABUR_PLAYER_LAT_TRACKER_H_
