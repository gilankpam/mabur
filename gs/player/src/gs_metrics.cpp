#include "gs_metrics.h"

namespace maburplay {

void AuJitter::on_au(uint64_t now_ms) {
  if (last_au_ms_ != 0) {
    const double iv = (double)(now_ms - last_au_ms_);
    if (iv >= (double)kStallMs) {
      // A stall is a stall, not jitter. Both the interval and the EMA are
      // dropped: the first interval after a gap is not comparable to the
      // last one before it either.
      last_interval_ms_ = -1.0;
      ema_ms_ = 0.0;
    } else if (last_interval_ms_ >= 0.0) {
      const double d =
          iv > last_interval_ms_ ? iv - last_interval_ms_ : last_interval_ms_ - iv;
      ema_ms_ += (d - ema_ms_) / kAlphaDiv;
      last_interval_ms_ = iv;
    } else {
      last_interval_ms_ = iv;  // first interval of a run: no delta to take yet
    }
  }
  last_au_ms_ = now_ms;
}

void AuJitter::on_tick(uint64_t now_ms) {
  if (last_au_ms_ == 0) return;  // nothing has ever arrived; 0.0 already
  if (now_ms - last_au_ms_ < kStallMs) return;
  last_interval_ms_ = -1.0;
  ema_ms_ = 0.0;
}

RecState RecTracker::update(const Inputs& in, uint64_t now_ms) {
  RecState out;
  // low_space is judged BEFORE `open`, so a card that is already full when
  // the session starts reads FAULT rather than sitting at ARMED forever
  // waiting for a file that will never be written.
  if (in.broken || in.low_space) {
    out.kind = RecState::Kind::kFault;
    return out;
  }
  if (!in.open) {
    out.kind = RecState::Kind::kArmed;
    return out;
  }

  // Either progress, or nothing to make progress FROM, resets the stall
  // clock. Without the second half a link outage -- or a truncation storm,
  // where AUs keep arriving but none is recordable -- reads as a recording
  // fault, which is a lie about the one subsystem still working.
  if (in.samples != samples_ || in.feed == feed_) stall_since_ms_ = now_ms;
  samples_ = in.samples;
  feed_ = in.feed;

  // stall_since_ms_ is always set by the time samples > 0: samples_ starts
  // at 0, so the first call that sees any sample takes the != branch above.
  if (in.samples > 0 && now_ms - stall_since_ms_ >= kStallMs) {
    out.kind = RecState::Kind::kFault;
    return out;
  }
  if (in.samples == 0) {
    // Open, but nothing written yet. RECORDING must never show while no
    // bytes are moving.
    out.kind = RecState::Kind::kArmed;
    return out;
  }
  if (!started_) {
    started_ = true;
    start_ms_ = now_ms;
  }
  out.kind = RecState::Kind::kRecording;
  out.elapsed_s = (int)((now_ms - start_ms_) / 1000);
  return out;
}

void RecTracker::reset() {
  samples_ = 0;
  feed_ = 0;
  stall_since_ms_ = 0;
  start_ms_ = 0;
  started_ = false;
}

VtxRecTracker::Out VtxRecTracker::update(const Inputs& in, uint64_t now_ms) {
  Out o;
  if (!in.requested) {
    reset();
    return o;
  }
  // No current drone report: the drone keeps its last state (no decay), so
  // say WAIT rather than guess. The clock origin survives the gap.
  if (!in.fresh || !in.state) {
    o.leg = RecState::Vtx::kWait;
    return o;
  }
  switch (*in.state) {
    case 1:
      if (!started_) {
        started_ = true;
        start_ms_ = now_ms;
      }
      o.leg = RecState::Vtx::kRecording;
      o.elapsed_s = static_cast<int>((now_ms - start_ms_) / 1000);
      return o;
    case 2:
      switch (in.err.value_or(0)) {
        case 1: o.leg = RecState::Vtx::kOff; break;
        case 2: case 3: case 4: o.leg = RecState::Vtx::kNoCard; break;
        case 5: o.leg = RecState::Vtx::kFull; break;
        default: o.leg = RecState::Vtx::kFault; break;
      }
      return o;
    default:   // 0: armed on the drone, waiting for its first IDR
      o.leg = RecState::Vtx::kWait;
      return o;
  }
}

void VtxRecTracker::reset() {
  started_ = false;
  start_ms_ = 0;
}

}  // namespace maburplay
