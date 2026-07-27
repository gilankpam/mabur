#include "mabur/uep_decoder.h"

#include "mabur/sbi.h"

namespace mabur {

UepDecoder::UepDecoder(const std::array<UepLayerCfg, 4>& layers,
                       uint64_t decode_deadline_ms, uint32_t seq_horizon)
    : layers_{Layer(layers[0], seq_horizon), Layer(layers[1], seq_horizon),
              Layer(layers[2], seq_horizon), Layer(layers[3], seq_horizon)},
      decode_deadline_ms_(decode_deadline_ms) {}

void UepDecoder::note_delivery(Layer& l, uint16_t seq) {
  // Delivery window: forward FRAG-seq gap = packets that will never
  // complete; backward/duplicate (gap 0 or > 0x8000) = reorder, count
  // delivered only. Monster gaps are an outage, not per-packet info —
  // cap at 512, same bounded-walk stance as gs/src/aggregator.cpp's
  // kMaxSeqGap.
  if (!l.has_last_seq) {
    l.win_expected += 1;
  } else {
    uint16_t gap = static_cast<uint16_t>(seq - l.last_seq);
    if (gap == 0 || gap > 0x8000) gap = 0;       // reorder/dup
    l.win_expected += gap > 512 ? 1 : (gap == 0 ? 1 : gap);
  }
  l.win_delivered += 1;
  l.has_last_seq = true;
  l.last_seq = seq;
}

std::vector<DecodedFrag> UepDecoder::add_body(const uint8_t* body, size_t len,
                                              uint64_t now_ms) {
  const int sid = sbi_peek_stream_id(body, len);
  if (sid < 0 || sid > 3) {
    ++bodies_misrouted_;
    return {};
  }
  Layer& L = layers_[static_cast<size_t>(sid)];
  ++L.bodies;
  const SbiUnpackResult r = sbi_unpack(body, len, L.env_size);
  L.subblocks_failed += static_cast<uint64_t>(r.n_failed);
  std::vector<DecodedFrag> out;
  for (const auto& env : r.survivors) {
    for (const auto& pkt : L.sw.add_symbol(env.data(), env.size(), now_ms)) {
      if (pkt.size() < Fragmenter::kHdrLen) continue;
      const uint16_t fseq = static_cast<uint16_t>(pkt[0] | (pkt[1] << 8));
      const uint16_t idx = static_cast<uint16_t>(pkt[2] | (pkt[3] << 8));
      const uint16_t count = static_cast<uint16_t>(pkt[4] | (pkt[5] << 8));
      // Delivery window: count a unit on last-fragment arrival, by FRAG-seq
      // continuity. Approximation (a unit missing a MIDDLE fragment but
      // keeping its tail still counts delivered); acceptable for the
      // controller signal, documented here deliberately.
      if (static_cast<uint16_t>(idx + 1) == count) note_delivery(L, fseq);
      out.push_back(DecodedFrag{static_cast<uint8_t>(sid), pkt});
    }
  }
  return out;
}

void UepDecoder::poll(uint64_t now_ms) {
  for (auto& L : layers_) L.sw.expire_rows_older_than(decode_deadline_ms_, now_ms);
}

void UepDecoder::reset_continuity() {
  for (auto& l : layers_) l.has_last_seq = false;
}

UepDecoder::LayerStats UepDecoder::stats(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return LayerStats{L.bodies,              L.subblocks_failed,
                    L.sw.syms_delivered(), L.sw.syms_recovered(),
                    L.sw.syms_recovered_arrived(),
                    L.sw.syms_abandoned(), L.sw.packets_out(),
                    L.sw.symbols_in(),     L.sw.symbols_dropped_stale(),
                    L.sw.symbols_dropped_bad_cfg(), L.sw.rows_in_flight()};
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
    // keep has_last_seq/last_seq: continuity spans windows
  }
}

std::pair<uint64_t, uint64_t> UepDecoder::window_counts(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return {L.win_delivered, L.win_expected};
}

}  // namespace mabur
