#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/uep_encoder.h"

// venc_frame_ring.h (vendored, C header) — see frame_source.h for why the
// include is wrapped.
extern "C" {
#include "venc_frame_ring.h"
}

namespace mabur {

// The per-frame ingest step shared by maburd's hot thread and its --dry-run
// replay, so both drive byte-identical wire units: classify the Annex-B frame,
// protect up on the producer's IDR flag, stamp the FrameHdr over the meta
// region, and hand the frame unit to the UEP encoder.
//
// Not thread-safe: one instance per ingest thread (frame_id and the
// discontinuity latch are per-stream state).
class FramePipeline {
 public:
  // buf: VENC_FRAME_META_SIZE bytes of meta followed by payload_len Annex-B
  // bytes — exactly how FrameSource::read fills it. The meta region is
  // OVERWRITTEN in place with the FrameHdr (kFrameHdrLen ==
  // VENC_FRAME_META_SIZE), so the unit (header + Annex-B) stays contiguous
  // with zero copies. Returns the bodies the encoder produced.
  std::vector<UepBody> encode(UepEncoder& uep, uint8_t* buf, size_t payload_len,
                              const VencFrameMeta& meta, uint64_t now_ms);

  // Marks a pts/frame_id re-base point: start of stream, or a reattach that
  // joined a new producer's ring mid-GOP. kFlagDiscont then rides on EVERY
  // frame for the next kDiscontStickyMs — a one-shot flag is sent right at
  // radio bring-up and is systematically lost, wedging the GS's emit cursor
  // above the new epoch for up to a full 16-bit id wrap
  // (docs/gs-frame-stall-after-drone-restart-handoff.md). The GS rebases once
  // per flagged run and its decoder recovers at the next IDR.
  //
  // Also re-anchors the vanish tracker: a new producer may re-base pts (and
  // even change frame rate), so the period must be re-confirmed before any
  // hole can be booked, and a pre-restart pending self-IDR is moot.
  void mark_discontinuity() {
    discont_pending_ = true;
    have_prev_pts_ = false;
    period_samples_ = 0;
    period_us_ = 0.0;
    self_idr_pending_ = false;
  }

  // Frames whose Annex-B scan disagreed with the producer's IDR flag. A bug
  // signal (producer vs. scanner), surfaced in maburd's stats line — never a
  // silent choice.
  uint64_t idr_disagreements() const { return idr_disagree_; }

  // Frames where the producer ENHANCE flag and the TRAIL_N scan disagreed.
  // Same contract as idr_disagreements(): a bug signal, never a silent
  // choice — the frame protects up to base (spec 2026-07-26 svct-enable).
  uint64_t enhance_disagreements() const { return enhance_disagree_; }
  uint16_t next_frame_id() const { return next_frame_id_; }

  // ── venc-ring vanish detection ──────────────────────────────────────────
  // (docs/venc-ring-vanish-findings-2026-08-12.md) Frames dropped between
  // waybeam's encoder and this reader never get a frame_id — the wire closes
  // seamlessly over the hole, so the pts step between consecutive ring reads
  // is the ONLY read-side evidence. A delta > 1.5x the observed period books
  // the missing frames here, classified from the neighbours' producer ENHANCE
  // flags (strict base/enhance alternation). Shed cannot fire this: shedding
  // happens downstream of the ring read, so encode() sees every frame's pts.
  uint64_t vanished_base() const { return vanished_base_; }
  uint64_t vanished_enhance() const { return vanished_enh_; }

  // A vanished BASE frame silently corrupts the GS decoder until an IDR, so
  // it latches this level; any IDR passing through clears it (the heal makes
  // the request moot). The caller reconciles the level into
  // RcAgent::request_self_idr on its own cadence.
  bool self_idr_pending() const { return self_idr_pending_; }

  // Re-seed loop guard: the IDR's own ~10x frame-size burst is the likely
  // vanish trigger, so a base vanish detected within kSelfIdrGuardMs of the
  // last IDR read is NOT latched (it would re-trigger at cooldown rate,
  // forever) — it is counted here instead so the loop stays visible.
  uint64_t self_idr_refused() const { return self_idr_refused_; }

  static constexpr uint64_t kDiscontStickyMs = 1000;
  static constexpr uint64_t kSelfIdrGuardMs = 500;

 private:
  uint16_t next_frame_id_ = 0;
  bool discont_pending_ = true;    // first frame after start anchors the window
  uint64_t discont_until_ms_ = 0;  // flag rides on frames until this deadline
  uint64_t idr_disagree_ = 0;
  uint64_t enhance_disagree_ = 0;

  // Vanish tracker state (see accessors above). The period is an EMA over
  // "normal" deltas only — never hardcoded, so any encoder frame rate works —
  // and detection stays off until kMinPeriodSamples deltas have confirmed it.
  static constexpr int kMinPeriodSamples = 4;
  static constexpr double kVanishFactor = 1.5;
  // Deltas at/above this are an encoder clock discontinuity, not a plausible
  // ring vanish (the ring holds ~133 ms): re-anchor instead of counting.
  static constexpr uint32_t kResyncUs = 1000000;
  // Bounds one event's booking so a sub-kResyncUs stall can't flood the
  // counters (a real ring vanish is 1-2 frames).
  static constexpr int kMaxVanishPerEvent = 16;

  bool have_prev_pts_ = false;
  uint32_t prev_pts_ = 0;
  bool prev_enhance_ = false;
  double period_us_ = 0.0;
  int period_samples_ = 0;
  bool have_last_idr_ = false;
  uint64_t last_idr_ms_ = 0;
  uint64_t vanished_base_ = 0;
  uint64_t vanished_enh_ = 0;
  uint64_t self_idr_refused_ = 0;
  bool self_idr_pending_ = false;

  void track_vanish(const VencFrameMeta& meta, bool meta_idr, bool meta_enhance,
                    uint64_t now_ms);
};

}  // namespace mabur
