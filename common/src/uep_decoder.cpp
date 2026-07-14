#include "mabur/uep_decoder.h"

#include "mabur/sbi.h"

namespace mabur {

UepDecoder::UepDecoder(const std::array<UepLayerCfg, 4>& layers,
                       uint64_t decode_deadline_ms, uint32_t seq_horizon)
    : layers_{Layer(layers[0], decode_deadline_ms, seq_horizon),
              Layer(layers[1], decode_deadline_ms, seq_horizon),
              Layer(layers[2], decode_deadline_ms, seq_horizon),
              Layer(layers[3], decode_deadline_ms, seq_horizon)},
      decode_deadline_ms_(decode_deadline_ms) {}

std::vector<DecodedRtp> UepDecoder::add_body(const uint8_t* body, size_t len,
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
  std::vector<DecodedRtp> out;
  for (const auto& env : r.survivors) {
    for (const auto& pkt : L.sw.add_symbol(env.data(), env.size(), now_ms)) {
      for (auto& done : L.reasm.add(pkt.data(), pkt.size(), now_ms)) {
        // Delivery window: forward FRAG-seq gap = packets that will never
        // complete; backward/duplicate (gap 0 or > 0x8000) = reorder, count
        // delivered only. Monster gaps are an outage, not per-packet info —
        // cap at 512 like the RungWindow's bounded walk.
        if (!L.has_last_seq) {
          L.win_expected += 1;
        } else {
          uint16_t gap = static_cast<uint16_t>(done.seq - L.last_seq);
          if (gap == 0 || gap > 0x8000) gap = 0;       // reorder/dup
          L.win_expected += gap > 512 ? 1 : (gap == 0 ? 1 : gap);
        }
        L.win_delivered += 1;
        L.has_last_seq = true;
        L.last_seq = done.seq;
        out.push_back(DecodedRtp{static_cast<uint8_t>(sid), std::move(done.pkt)});
      }
    }
  }
  return out;
}

void UepDecoder::poll(uint64_t now_ms) {
  for (auto& L : layers_) L.sw.expire_rows_older_than(decode_deadline_ms_, now_ms);
}

UepDecoder::LayerStats UepDecoder::stats(int sid) const {
  const Layer& L = layers_[static_cast<size_t>(sid)];
  return LayerStats{L.bodies,               L.subblocks_failed,
                    L.sw.syms_delivered(),  L.sw.syms_recovered(),
                    L.sw.syms_abandoned(),  L.sw.packets_out(),
                    L.reasm.evicted(),      L.sw.symbols_in(),
                    L.sw.symbols_dropped_stale(),
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
