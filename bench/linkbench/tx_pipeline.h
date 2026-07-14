#pragma once
// TX side of the bench: app packets → SwEncoder → SbiPacker → complete SBI
// radio bodies. Mirrors UepEncoder's single-layer wiring (common/src/
// uep_encoder.cpp pack_envs/drain_layer) minus the UEP ladder — the bench
// measures the link through the same encode path with one deliberate
// stream. No Fragmenter: bench packets are already sized to fit one symbol.
#include <cstdint>
#include <utility>
#include <vector>

#include "bench_wire.h"
#include "mabur/sbi.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"

namespace linkbench {

struct FecParams {
  double overhead = 0.5;  // repair symbols per source symbol (wfb-ng default)
  int symbol_size = 64;
  int window = 128;  // sliding-window repair span, in symbols
  int bpb = 16;       // envelopes per SBI body

  mabur::SwConfig sw() const {
    mabur::SwConfig c;
    c.symbol_size = symbol_size;
    c.window = window;
    c.overhead = overhead;
    return c;
  }
  int envelope_len() const {
    return static_cast<int>(mabur::sw::kSwHeaderLen) + symbol_size;
  }
};

class TxPipeline {
 public:
  explicit TxPipeline(const FecParams& p)
      : p_(p),
        sw_(p.sw()),
        packer_(p.envelope_len(), p.bpb, kBenchStreamId) {}

  // Feeds one app packet; appends any completed SBI bodies to out.
  void add_packet(const uint8_t* data, size_t len,
                  std::vector<std::vector<uint8_t>>& out) {
    auto envs = sw_.add_packet(data, len);
    for (auto& env : envs)
      for (auto& b : packer_.add(env.data(), env.size()))
        out.push_back(std::move(b));
  }

  // Seals the pending partial symbol (and its tail repair, if one is due)
  // and drains the packer.
  void flush(std::vector<std::vector<uint8_t>>& out) {
    auto envs = sw_.flush();
    for (auto& env : envs)
      for (auto& b : packer_.add(env.data(), env.size()))
        out.push_back(std::move(b));
    for (auto& b : packer_.flush()) out.push_back(std::move(b));
  }

  uint64_t sources_sent() const { return sw_.sources_out(); }
  uint64_t repairs_sent() const { return sw_.repairs_out(); }
  size_t oversize_drops() const { return sw_.oversize_drops(); }

 private:
  FecParams p_;
  mabur::SwEncoder sw_;
  mabur::SbiPacker packer_;
};

}  // namespace linkbench
