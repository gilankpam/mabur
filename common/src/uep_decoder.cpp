#include "mabur/uep_decoder.h"

#include "mabur/sbi.h"

namespace mabur {

namespace {
// Transition boundaries older than this are force-closed and disarmed: the
// u16 FRAG watermark would eventually wrap into false stale matches, and a
// straggler this old is not transition debris. Structural, not config.
constexpr uint64_t kBoundaryExpiryMs = 1000;
// Wrap-aware u16 "a <= b" over the half-space, same idiom as the gap walk.
bool seq16_le(uint16_t a, uint16_t b) {
  return static_cast<int16_t>(static_cast<uint16_t>(a - b)) <= 0;
}
bool seq16_gt(uint16_t a, uint16_t b) { return !seq16_le(a, b); }
}  // namespace

UepDecoder::UepDecoder(const std::array<UepLayerCfg, 2>& layers,
                       uint64_t decode_deadline_ms, uint32_t seq_horizon)
    : layers_{Layer(layers[0], seq_horizon), Layer(layers[1], seq_horizon)},
      decode_deadline_ms_(decode_deadline_ms) {}

void UepDecoder::mark_transition(int sid, uint8_t new_mcs, uint64_t now_ms) {
  if (sid < 0 || sid > 1) return;
  Layer& L = layers_[static_cast<size_t>(sid)];
  const uint8_t prev = L.cur_mcs;
  L.cur_mcs = new_mcs;
  L.pkt_wm = L.last_seq;
  L.pkt_wm_valid = L.has_last_seq;
  L.bnd_armed = true;
  L.bnd_arm_ms = now_ms;
  L.bnd_post_floor_set = false;
  // Open only when the PHY rate actually changes and we know what it was:
  // a same-MCS (overhead-only) step cannot kill in-flight bodies at the
  // radio, so the plain highest-seen snapshot is sufficient there.
  L.bnd_open = prev != kMcsUnknown && new_mcs != kMcsUnknown && new_mcs != prev;
  L.sw.mark_transition();
  if (!L.bnd_open) L.sw.close_boundary();
}

void UepDecoder::note_delivery(Layer& l, uint16_t seq) {
  // Delivery window: forward FRAG-seq gap = packets that will never
  // complete; backward/duplicate (gap 0 or > 0x8000) = reorder, count
  // delivered only. Monster gaps are an outage, not per-packet info —
  // cap at 512, same bounded-walk stance as gs/src/aggregator.cpp's
  // kMaxSeqGap.
  // A unit is pre-transition debris iff the boundary is still open (the
  // new-op stream hasn't been heard yet, so nothing can be the new rung's
  // fault) or it sits at/below the packet watermark.
  auto is_stale = [&](uint16_t u) {
    if (l.bnd_open) return true;
    if (!l.bnd_armed) return false;
    if (l.pkt_wm_valid && seq16_le(u, l.pkt_wm)) return true;
    // A late FEC recovery can regress last_seq (preserved verbatim below),
    // which would otherwise re-count an already-delivered unit as a fresh
    // current-rung expectation on the next forward gap. Once a boundary is
    // armed, treat anything at/below the high-water mark already reached
    // as accounted-for rather than a new current-rung miss — a reorder
    // artifact, not attribution, but it only needs handling once the
    // stale/cur split is in play (unarmed layers never touch either
    // stale bucket, so this never fires for them).
    return l.win_hwm_valid && seq16_le(u, l.win_hwm);
  };
  uint64_t exp = 0, exp_stale = 0;
  if (!l.has_last_seq) {
    exp = 1;
  } else {
    uint16_t gap = static_cast<uint16_t>(seq - l.last_seq);
    if (gap == 0 || gap > 0x8000) gap = 0;       // reorder/dup
    if (gap > 512 || gap == 0) {
      exp = 1;
    } else {
      exp = gap;
      // Classify each missing unit in (last_seq, seq) individually — the
      // FRAG seqs of a gap are exactly known, so there is no straddle.
      for (uint16_t i = 1; i < gap; ++i)
        if (is_stale(static_cast<uint16_t>(l.last_seq + i))) ++exp_stale;
    }
  }
  const bool unit_stale = is_stale(seq);
  l.win_expected += exp;
  l.win_delivered += 1;
  l.win_expected_stale += exp_stale + (unit_stale ? 1 : 0);
  l.win_delivered_stale += unit_stale ? 1 : 0;
  l.has_last_seq = true;
  l.last_seq = seq;
  // (keep the original last_seq update semantics exactly: the two lines
  //  above replace the originals, nothing else moves)
  // win_hwm is updated unconditionally on every completion, armed or not —
  // never gated behind bnd_armed/bnd_open. That means it is already
  // correct (covers full history) at the moment a boundary next arms, and
  // it always tracks the recent head going forward, so it can never carry
  // a value stale enough for the u16 wrap-aware compare to misfire. Only
  // the is_stale() *comparison* above is gated on l.bnd_armed — an unarmed
  // layer never consults win_hwm at all, so this update is a no-op for it.
  if (!l.win_hwm_valid || seq16_gt(seq, l.win_hwm)) {
    l.win_hwm = seq;
    l.win_hwm_valid = true;
  }
}

std::vector<DecodedFrag> UepDecoder::add_body(const uint8_t* body, size_t len,
                                              uint64_t now_ms,
                                              uint8_t rx_mcs) {
  const int sid = sbi_peek_stream_id(body, len);
  if (sid < 0 || sid > 1) {
    ++bodies_misrouted_;
    return {};
  }
  Layer& L = layers_[static_cast<size_t>(sid)];
  ++L.bodies;
  SwBoundary hint = SwBoundary::kNone;
  if (L.bnd_armed) {
    if (now_ms > L.bnd_arm_ms && now_ms - L.bnd_arm_ms > kBoundaryExpiryMs) {
      L.bnd_armed = false;
      L.bnd_open = false;
      L.pkt_wm_valid = false;
      L.sw.close_boundary();
    } else if (rx_mcs != kMcsUnknown && L.cur_mcs != kMcsUnknown) {
      hint = rx_mcs == L.cur_mcs ? SwBoundary::kPost : SwBoundary::kPre;
    }
  }
  if (hint == SwBoundary::kPost && L.bnd_open) {
    L.bnd_open = false;
    if (L.bnd_arm_ms <= now_ms)
      L.bnd_close_ms = static_cast<double>(now_ms - L.bnd_arm_ms);
  }
  const SbiUnpackResult r = sbi_unpack(body, len, L.env_size);
  L.subblocks_failed += static_cast<uint64_t>(r.n_failed);
  std::vector<DecodedFrag> out;
  for (const auto& env : r.survivors) {
    for (const auto& pkt : L.sw.add_symbol(env.data(), env.size(), now_ms, hint)) {
      if (pkt.size() < Fragmenter::kHdrLen) continue;
      const uint16_t fseq = static_cast<uint16_t>(pkt[0] | (pkt[1] << 8));
      const uint16_t idx = static_cast<uint16_t>(pkt[2] | (pkt[3] << 8));
      const uint16_t count = static_cast<uint16_t>(pkt[4] | (pkt[5] << 8));
      // Delivery window: count a unit on last-fragment arrival, by FRAG-seq
      // continuity. Approximation (a unit missing a MIDDLE fragment but
      // keeping its tail still counts delivered); acceptable for the
      // controller signal, documented here deliberately.
      if (static_cast<uint16_t>(idx + 1) == count) {
        // Packet-space watermark maintenance from the completing body's
        // generation: kPre completions advance it (provably old units);
        // the FIRST kPost completion floors it at (fseq - 1) — everything
        // below the first new-op unit is pre-transition (fseq is
        // sender-monotonic per stream).
        if (hint == SwBoundary::kPre &&
            (!L.pkt_wm_valid || seq16_gt(fseq, L.pkt_wm))) {
          L.pkt_wm = fseq;
          L.pkt_wm_valid = true;
        }
        if (hint == SwBoundary::kPost && !L.bnd_post_floor_set) {
          L.bnd_post_floor_set = true;
          const uint16_t floor = static_cast<uint16_t>(fseq - 1);
          if (!L.pkt_wm_valid || seq16_gt(floor, L.pkt_wm)) {
            L.pkt_wm = floor;
            L.pkt_wm_valid = true;
          }
        }
        note_delivery(L, fseq);
      }
      out.push_back(DecodedFrag{static_cast<uint8_t>(sid), pkt});
    }
  }
  return out;
}

void UepDecoder::poll(uint64_t now_ms) {
  for (auto& L : layers_) L.sw.expire_rows_older_than(decode_deadline_ms_, now_ms);
}

void UepDecoder::reset_continuity() {
  for (auto& l : layers_) {
    l.has_last_seq = false;
    l.bnd_armed = false;
    l.bnd_open = false;
    l.pkt_wm_valid = false;
    l.win_hwm_valid = false;
    l.cur_mcs = kMcsUnknown;
    l.sw.close_boundary();
  }
}

UepDecoder::LayerStats UepDecoder::stats(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return LayerStats{L.bodies,              L.subblocks_failed,
                    L.sw.syms_delivered(), L.sw.syms_recovered(),
                    L.sw.syms_recovered_arrived(),
                    L.sw.syms_abandoned(), L.sw.packets_out(),
                    L.sw.symbols_in(),     L.sw.symbols_dropped_stale(),
                    L.sw.symbols_dropped_bad_cfg(), L.sw.rows_in_flight(),
                    L.sw.syms_abandoned_stale()};
}

int UepDecoder::window_delivery_pct(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  if (L.win_expected == 0) return 100;
  const uint64_t pct = L.win_delivered * 100 / L.win_expected;
  return static_cast<int>(pct > 100 ? 100 : pct);
}

void UepDecoder::reset_window() {
  for (auto& L : layers_) {
    L.win_delivered = L.win_expected = 0;
    L.win_delivered_stale = L.win_expected_stale = 0;
    // keep has_last_seq/last_seq: continuity spans windows
  }
}

std::pair<uint64_t, uint64_t> UepDecoder::window_counts(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return {L.win_delivered, L.win_expected};
}

std::pair<uint64_t, uint64_t> UepDecoder::window_counts_cur(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return {L.win_delivered - L.win_delivered_stale,
          L.win_expected - L.win_expected_stale};
}

double UepDecoder::last_boundary_close_ms(int sid) const {
  return layers_[static_cast<size_t>(sid)].bnd_close_ms;
}

}  // namespace mabur
