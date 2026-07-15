#include "mabur/msp_source.h"
#include "mabur/sw_wire.h"  // sw::kSwHeaderLen

namespace mabur {
namespace {
SwConfig make_sw_cfg(const MspSourceCfg& c) {
  SwConfig s;
  s.symbol_size = c.symbol_size;
  s.window = c.window;
  s.overhead = c.overhead;
  return s;
}
}  // namespace

MspSource::MspSource(const MspSourceCfg& cfg, EmitFn emit, uint32_t initial_seq)
    : cfg_(cfg),
      emit_(std::move(emit)),
      enc_(make_sw_cfg(cfg), initial_seq),
      // One SW envelope per SBI body: block_payload = symbol_size + header,
      // blocks_per_body = 1 (each frame carries one symbol -> source and repair
      // ride separate air frames).
      packer_(cfg.symbol_size + static_cast<int>(sw::kSwHeaderLen), 1, kMspStreamId) {}

void MspSource::on_serial_bytes(const uint8_t* p, size_t n, uint64_t now_ms) {
  for (auto& m : parser_.feed(p, n)) {
    if (!screen_.apply(m)) continue;  // not a completed screen
    double period_ms = 1000.0 / (cfg_.update_rate_hz > 0 ? cfg_.update_rate_hz : 1.0);
    if (!have_forwarded_ ||
        static_cast<double>(now_ms - last_forward_ms_) >= period_ms) {
      forward_snapshot(now_ms);
      last_forward_ms_ = now_ms;
      have_forwarded_ = true;
    } else {
      ++snapshots_gated_;
    }
  }
}

void MspSource::forward_snapshot(uint64_t) {
  bool trunc = false;
  auto snap = screen_.serialize_snapshot(
      static_cast<size_t>(cfg_.symbol_size - 2), &trunc);
  if (trunc) ++truncated_;
  if (snap.empty()) return;

  auto emit_env = [&](const std::vector<uint8_t>& env) {
    for (auto& body : packer_.add(env.data(), env.size())) emit_(body.data(), body.size());
  };

  // One packet == one snapshot == one symbol (atomic, order-independent).
  for (auto& env : enc_.add_packet(snap.data(), snap.size())) emit_env(env);
  // Seal the partial symbol and emit its tail repair.
  for (auto& env : enc_.flush()) emit_env(env);
  for (auto& body : packer_.flush()) emit_(body.data(), body.size());

  ++snapshots_sent_;
}

}  // namespace mabur
