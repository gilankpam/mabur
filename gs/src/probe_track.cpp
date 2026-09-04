#include "probe_track.h"

#include <algorithm>
#include <cmath>

namespace maburgs {

ProbeTrack::ProbeTrack(ProbeTrackCfg cfg) : cfg_(cfg) {}

void ProbeTrack::set_commanded(uint8_t profile, double /*now_ms*/) {
  commanded_ = profile;
}
uint8_t ProbeTrack::commanded() const { return commanded_; }

void ProbeTrack::on_enh_au(double now_ms) {
  if (commanded_ == 0xFF) return;
  au_pending_.push_back(now_ms);
}

void ProbeTrack::on_body(int card, const mabur::probe::ProbeRx& rx, double snr_db,
                          double evm_db, double now_ms) {
  if (card < 0 || card >= 8) return;
  auto it = std::find_if(ring_.begin(), ring_.end(),
                          [&](const Pending& p) { return p.seq == rx.hdr.seq; });
  if (it == ring_.end()) {
    Pending p;
    p.seq = rx.hdr.seq;
    p.profile = rx.hdr.profile;
    p.enh_fid = rx.hdr.enh_fid;
    p.first_ms = now_ms;
    p.snr.fill(std::nan(""));
    p.evm.fill(std::nan(""));
    ring_.push_back(p);
    it = std::prev(ring_.end());
    if (ring_.size() > kRingMax) {
      // Bounded ring: force the oldest entry to finalize now rather than
      // grow without limit -- a body count is meant to settle within
      // finalize_ms anyway, so an entry that has been outrun by 64 newer
      // seqs is settled in every practical sense already.
      finalize_body(ring_.front(), now_ms);
      ring_.pop_front();
      it = std::prev(ring_.end());
    }
  }
  const size_t c = static_cast<size_t>(card);
  it->bitmap |= rx.survivors;
  it->card_bits[c] |= rx.survivors;
  it->card_mask |= 1u << card;
  it->snr[c] = snr_db;
  it->evm[c] = evm_db;
}

void ProbeTrack::finalize_au() {
  union_.expected_blocks += static_cast<uint64_t>(cfg_.bpb);
  for (int c = 0; c < cfg_.max_cards && c < 8; ++c)
    cards_[static_cast<size_t>(c)].expected_blocks += static_cast<uint64_t>(cfg_.bpb);
}

void ProbeTrack::finalize_body(Pending& p, double now_ms) {
  const uint64_t bpb = static_cast<uint64_t>(cfg_.bpb);
  if (p.profile != commanded_) {
    // The AU that carried this body was booked as an expectation before
    // its profile was known (on_enh_au fires ahead of the probe body
    // landing), but a probe sent at a stale/candidate profile isn't a
    // sample of the commanded profile's loss -- un-book it rather than
    // score it as either a hit or a miss. Saturate at arrived_blocks so
    // a burst of off-profile bodies can never push expected below what
    // has already been credited as arrived.
    ++off_profile_;
    auto unbook = [&](ProbeCounts& c) {
      c.expected_blocks =
          c.expected_blocks >= c.arrived_blocks + bpb ? c.expected_blocks - bpb : c.arrived_blocks;
    };
    unbook(union_);
    for (int c = 0; c < cfg_.max_cards && c < 8; ++c) unbook(cards_[static_cast<size_t>(c)]);
    return;
  }
  union_.arrived_blocks += static_cast<uint64_t>(__builtin_popcount(p.bitmap));
  ++union_.bodies_rx;
  for (int c = 0; c < cfg_.max_cards && c < 8; ++c) {
    const uint32_t b = p.card_bits[static_cast<size_t>(c)];
    cards_[static_cast<size_t>(c)].arrived_blocks += static_cast<uint64_t>(__builtin_popcount(b));
    if (b) ++cards_[static_cast<size_t>(c)].bodies_rx;
  }
  ProbeFinalized f;
  f.t_ms = now_ms;
  f.seq = p.seq;
  f.profile = p.profile;
  f.enh_fid = p.enh_fid;
  f.blocks_ok = __builtin_popcount(p.bitmap);
  f.card_mask = p.card_mask;
  for (size_t c = 0; c < 8; ++c) {
    f.snr_db[c] = p.snr[c];
    f.evm_db[c] = p.evm[c];
  }
  finalized_.push_back(f);
}

void ProbeTrack::tick(double now_ms) {
  while (!au_pending_.empty() && now_ms - au_pending_.front() >= cfg_.finalize_ms) {
    au_pending_.pop_front();
    finalize_au();
  }
  while (!ring_.empty() && now_ms - ring_.front().first_ms >= cfg_.finalize_ms) {
    finalize_body(ring_.front(), now_ms);
    ring_.pop_front();
  }
}

const ProbeCounts& ProbeTrack::union_counts() const { return union_; }
const ProbeCounts& ProbeTrack::card_counts(int card) const {
  return cards_[static_cast<size_t>(std::clamp(card, 0, 7))];
}
uint64_t ProbeTrack::off_profile() const { return off_profile_; }

std::vector<ProbeFinalized> ProbeTrack::take_finalized() {
  std::vector<ProbeFinalized> out;
  out.swap(finalized_);
  return out;
}

}  // namespace maburgs
