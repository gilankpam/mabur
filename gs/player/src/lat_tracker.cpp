#include "lat_tracker.h"

#include <algorithm>
#include <cstdlib>

namespace maburplay {

int LatTracker::find_(uint32_t pts_us) const {
  for (std::size_t i = 0; i < kMaxInflight; ++i) {
    if (map_[i].valid && map_[i].pts_us == pts_us) return static_cast<int>(i);
  }
  return -1;
}

int LatTracker::alloc_(uint32_t pts_us) {
  // Reuse a slot already keyed to this pts (a resubmission -- should not
  // happen in practice, but overwriting beats leaking a stale entry).
  const int existing = find_(pts_us);
  if (existing >= 0) return existing;

  for (std::size_t i = 0; i < kMaxInflight; ++i) {
    if (!map_[i].valid) return static_cast<int>(i);
  }
  // Full: evict the oldest in-flight entry (drop-oldest) so a wedged
  // decoder or a presenter that stopped flipping cannot grow this map.
  std::size_t oldest = 0;
  for (std::size_t i = 1; i < kMaxInflight; ++i) {
    if (map_[i].seq < map_[oldest].seq) oldest = i;
  }
  return static_cast<int>(oldest);
}

void LatTracker::invalidate_(int idx) {
  if (idx < 0) return;
  map_[static_cast<std::size_t>(idx)] = Entry{};
}

uint32_t LatTracker::percentile_(std::vector<uint32_t>& v, int pct) {
  if (v.empty()) return 0;
  const std::size_t n = v.size();
  std::size_t idx = (n * static_cast<std::size_t>(pct)) / 100;
  if (idx >= n) idx = n - 1;
  std::nth_element(v.begin(), v.begin() + static_cast<long>(idx), v.end());
  return v[idx];
}

void LatTracker::on_submit(const maburgs::AuRecordMeta& m, uint64_t mono_us) {
  const int idx = alloc_(m.pts_us);
  Entry& e = map_[static_cast<std::size_t>(idx)];
  e = Entry{};
  e.valid = true;
  e.pts_us = m.pts_us;
  e.t_complete_us = m.t_complete_us;
  e.t_submit_us = mono_us;
  e.seq = next_seq_++;

  const auto obs = anchor_.observe(m.pts_us, m.t_first_us);
  e.pts64 = obs.pts64;

  // Same clamp-order math as gs/src/main.cpp's head-segment accounting:
  // enc first, then dq, against the anchored span; the remainder is air.
  // Only meaningful once the anchor is warm and this sample did not itself
  // re-seed it (a discontinuity's span is against a floor that no longer
  // means anything) -- in that case the segments stay 0 rather than lie.
  if (!obs.discont && anchor_.usable()) {
    e.anchor_ok_at_submit = true;
    const int64_t span = static_cast<int64_t>(m.t_first_us) -
                          static_cast<int64_t>(anchor_.map_us(obs.pts64));
    int64_t rem = span > 0 ? span : 0;
    const uint32_t enc = static_cast<uint32_t>(std::min<int64_t>(m.enc_us, rem));
    rem -= enc;
    const uint32_t dq = static_cast<uint32_t>(
        std::min<int64_t>(static_cast<int64_t>(m.drone_q_ms) * 1000, rem));
    rem -= dq;
    const uint32_t fec = static_cast<uint32_t>(
        m.t_complete_us > m.t_first_us ? m.t_complete_us - m.t_first_us : 0);
    e.seg[0] = enc;
    e.seg[1] = dq;
    e.seg[2] = static_cast<uint32_t>(rem);
    e.seg[3] = fec;
  }
}

void LatTracker::on_decoded(uint32_t pts_us, uint64_t mono_us) {
  const int idx = find_(pts_us);
  if (idx < 0) return;
  Entry& e = map_[static_cast<std::size_t>(idx)];
  // dec folds t_submit..t_complete's handoff in: it is t_decoded -
  // t_complete directly, not t_decoded - t_submit (see class comment).
  e.seg[4] = static_cast<uint32_t>(mono_us > e.t_complete_us ? mono_us - e.t_complete_us : 0);
  e.t_decoded_us = mono_us;
  e.has_decoded = true;
}

void LatTracker::on_present(uint32_t pts_us, uint64_t mono_us) {
  const int idx = find_(pts_us);
  if (idx < 0) return;
  Entry& e = map_[static_cast<std::size_t>(idx)];
  if (!e.has_decoded) return;
  e.seg[5] = static_cast<uint32_t>(mono_us > e.t_decoded_us ? mono_us - e.t_decoded_us : 0);
  e.t_release_us = mono_us;
  e.has_present = true;
}

void LatTracker::on_flip(uint32_t pts_us, uint64_t flip_mono_us, bool exact) {
  const int idx = find_(pts_us);
  if (idx < 0) return;
  Entry& e = map_[static_cast<std::size_t>(idx)];
  if (!e.has_present) {
    invalidate_(idx);
    return;
  }
  e.seg[6] =
      static_cast<uint32_t>(flip_mono_us > e.t_release_us ? flip_mono_us - e.t_release_us : 0);
  uint32_t e2e = 0;
  for (int i = 0; i < 7; ++i) e2e += e.seg[i];
  e.seg[7] = e2e;

  // A cold/discontinuous anchor at submit time means on_submit left
  // enc/dq/air/fec at 0 rather than computing real values (see there) --
  // the WHOLE frame is excluded from the window here, not just chk, so a
  // post-flush_all() warm-up run never blends fabricated zero head-segments
  // into the aggregates. Matches gs/src/main.cpp's own head-segment gate
  // (Task 10's !obs.discont && lat_anchor.usable() condition). The frame
  // still retires from the in-flight map below either way.
  if (e.anchor_ok_at_submit) {
    const int64_t ideal =
        static_cast<int64_t>(flip_mono_us) - static_cast<int64_t>(anchor_.map_us(e.pts64));
    chk_sum_us_ += static_cast<double>(ideal - static_cast<int64_t>(e2e));
    ++chk_n_;
    dsp_exact_window_ = dsp_exact_window_ && exact;

    std::array<uint32_t, 8> snap{};
    for (int i = 0; i < 8; ++i) snap[static_cast<std::size_t>(i)] = e.seg[i];
    completed_.push_back(snap);
  }

  invalidate_(idx);
}

void LatTracker::on_drop(uint32_t pts_us) {
  const int idx = find_(pts_us);
  if (idx >= 0) invalidate_(idx);
}

void LatTracker::flush_all() {
  for (auto& e : map_) e = Entry{};
  // New session's pts space is unrelated to the old one's -- same
  // reasoning as gs/src/main.cpp's lat_anchor.reset() on a discontinuity.
  anchor_.reset();
  // Drop pre-reset samples too, so they cannot silently mix into the next
  // flush_line() (same rationale as LatWindow::clear() on the daemon side).
  completed_.clear();
  chk_sum_us_ = 0.0;
  chk_n_ = 0;
  dsp_exact_window_ = true;
}

LatTracker::Line LatTracker::flush_line() {
  Line L{};
  L.n = static_cast<int>(completed_.size());
  L.anchor_ok = anchor_.usable();
  L.dsp_exact = dsp_exact_window_;

  if (L.n > 0) {
    for (int s = 0; s < 8; ++s) {
      std::vector<uint32_t> col(completed_.size());
      for (std::size_t i = 0; i < completed_.size(); ++i)
        col[i] = completed_[i][static_cast<std::size_t>(s)];
      L.p50[s] = percentile_(col, 50);
      L.p99[s] = percentile_(col, 99);
    }
    L.chk_ms = chk_n_ > 0 ? (chk_sum_us_ / chk_n_) / 1000.0 : 0.0;

    // p99-by-e2e frame: rank on e2e (seg 7), then report THAT frame's own
    // full breakdown -- never a mix of independently-ranked per-segment
    // percentiles, which would not sum to anything meaningful.
    std::vector<uint32_t> e2es(completed_.size());
    for (std::size_t i = 0; i < completed_.size(); ++i) e2es[i] = completed_[i][7];
    const uint32_t target = percentile_(e2es, 99);
    for (const auto& c : completed_) {
      if (c[7] == target) {
        p99_frame_.valid = true;
        for (int i = 0; i < 8; ++i)
          p99_frame_.ms[i] =
              static_cast<uint32_t>((c[static_cast<std::size_t>(i)] + 500) / 1000);
        break;
      }
    }
  }

  completed_.clear();
  chk_sum_us_ = 0.0;
  chk_n_ = 0;
  dsp_exact_window_ = true;
  return L;
}

LatTracker::Breakdown LatTracker::p99_frame() const { return p99_frame_; }

}  // namespace maburplay
