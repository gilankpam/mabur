#include "mabur/uep_decoder.h"

#include "mabur/sbi.h"

namespace mabur {

namespace {
// Transition boundaries older than this are force-closed and disarmed: a
// straggler this old is not transition debris. Structural, not config.
// (Pre-2026-09-02 this also bounded a u16 FRAG watermark against wrapping
// into false stale matches; that watermark died with the delivery window,
// and SwDecoder's own boundary tracking is virtual-seq based.)
constexpr uint64_t kBoundaryExpiryMs = 1000;
}  // namespace

UepDecoder::UepDecoder(const std::array<UepLayerCfg, 2>& layers,
                       uint32_t seq_horizon)
    : layers_{Layer(layers[0], seq_horizon), Layer(layers[1], seq_horizon)} {}

void UepDecoder::mark_transition(int sid, uint8_t new_mcs, uint64_t now_ms) {
  if (sid < 0 || sid > 1) return;
  Layer& L = layers_[static_cast<size_t>(sid)];
  const uint8_t prev = L.cur_mcs;
  L.cur_mcs = new_mcs;
  L.bnd_armed = true;
  L.bnd_arm_ms = now_ms;
  // Open only when the PHY rate actually changes and we know what it was:
  // a same-MCS (overhead-only) step cannot kill in-flight bodies at the
  // radio, so the plain highest-seen snapshot is sufficient there.
  L.bnd_open = prev != kMcsUnknown && new_mcs != kMcsUnknown && new_mcs != prev;
  L.sw.mark_transition();
  if (!L.bnd_open) L.sw.close_boundary();
}

std::vector<DecodedFrag> UepDecoder::add_body(const uint8_t* body, size_t len,
                                              uint64_t now_ms,
                                              uint8_t rx_mcs,
                                              uint64_t body_mono_us,
                                              bool body_crc_ok) {
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
      // q_ms/enc_us are outside the per-block CRCs: only an FCS-clean body
      // may vouch for them (0 = unknown downstream, header comment).
      out.push_back(DecodedFrag{static_cast<uint8_t>(sid), pkt, body_mono_us,
                                body_crc_ok ? r.q_ms : static_cast<uint16_t>(0),
                                body_crc_ok ? r.enc_us : static_cast<uint16_t>(0),
                                body_crc_ok ? r.air_ms : static_cast<uint16_t>(0)});
    }
  }
  return out;
}

uint64_t UepDecoder::newest_seq(int sid) const {
  if (sid < 0 || sid > 1) return 0;
  return layers_[static_cast<size_t>(sid)].sw.newest_seq();
}

int UepDecoder::repair_window(int sid) const {
  if (sid < 0 || sid > 1) return 0;
  return layers_[static_cast<size_t>(sid)].sw.repair_window();
}

void UepDecoder::reset_continuity() {
  for (auto& l : layers_) {
    l.bnd_armed = false;
    l.bnd_open = false;
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
                    L.sw.syms_abandoned_stale(),
                    L.sw.arr_expected(),   L.sw.arr_arrived(),
                    L.sw.arr_expected_stale(), L.sw.arr_arrived_stale(),
                    L.sw.arr_late()};
}

double UepDecoder::last_boundary_close_ms(int sid) const {
  return layers_[static_cast<size_t>(sid)].bnd_close_ms;
}

}  // namespace mabur
