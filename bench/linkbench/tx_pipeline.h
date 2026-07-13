#pragma once
// TX side of the bench: app packets → RsEncoder → (SymbolInterleaver) →
// SbiPacker → complete SBI radio bodies. Mirrors UepEncoder's single-layer
// wiring (common/src/uep_encoder.cpp pack_block/drain_layer) minus the UEP
// ladder — the bench measures the link through the same encode path with
// one deliberate stream.
#include <cstdint>
#include <utility>
#include <vector>

#include "bench_wire.h"
#include "mabur/interleaver.h"
#include "mabur/rs_encoder.h"
#include "mabur/sbi.h"

namespace linkbench {

struct FecParams {
  int k = 8;
  double overhead = 0.5;  // n = k + ceil(k*overhead); 0.5 → 8/12, wfb-ng's default
  int symbol_size = 64;
  int bpb = 16;        // envelopes per SBI body
  int interleave = 0;  // 0 = off; >0 = symbol interleaving depth

  mabur::RsConfig rs() const {
    mabur::RsConfig c;
    c.k = k;
    c.symbol_size = symbol_size;
    c.overhead = overhead;
    return c;
  }
  int envelope_len() const { return 11 + symbol_size; }
  // With interleaving, the packer's bpb must equal the interleave depth so
  // each body carries one symbol from `depth` DIFFERENT blocks (see
  // interleaver.h); mirror UepEncoder's max(depth, bpb) rule.
  int effective_bpb() const {
    if (interleave <= 0) return bpb;
    return interleave > bpb ? interleave : bpb;
  }
};

class TxPipeline {
 public:
  explicit TxPipeline(const FecParams& p)
      : p_(p),
        rs_(p.rs()),
        il_(p.effective_bpb()),
        packer_(p.envelope_len(), p.effective_bpb(), kBenchStreamId) {}

  // Feeds one app packet; appends any completed SBI bodies to out.
  void add_packet(const uint8_t* data, size_t len,
                  std::vector<std::vector<uint8_t>>& out) {
    auto envs = rs_.add_packet(data, len);
    if (envs.empty()) return;
    ++blocks_;
    pack_block(std::move(envs), out);
  }

  // Seals the pending partial block and drains the interleaver window and
  // packer. Sub-depth interleaver rounds each close as their own short body
  // (a depth-sized packer fed a sub-depth round would misalign — see
  // SymbolInterleaver::drain_round()).
  void flush(std::vector<std::vector<uint8_t>>& out) {
    auto envs = rs_.flush();
    if (!envs.empty()) {
      ++blocks_;
      pack_block(std::move(envs), out);
    }
    if (p_.interleave > 0) {
      for (;;) {
        auto round = il_.drain_round();
        if (round.empty()) break;
        for (auto& env : round)
          for (auto& b : packer_.add(env.data(), env.size()))
            out.push_back(std::move(b));
        for (auto& b : packer_.flush()) out.push_back(std::move(b));
      }
    }
    for (auto& b : packer_.flush()) out.push_back(std::move(b));
  }

  uint64_t blocks_encoded() const { return blocks_; }
  size_t oversize_drops() const { return rs_.oversize_drops(); }

 private:
  void pack_block(std::vector<std::vector<uint8_t>> envs,
                  std::vector<std::vector<uint8_t>>& out) {
    if (p_.interleave > 0) {
      for (auto& env : il_.add_block(std::move(envs)))
        for (auto& b : packer_.add(env.data(), env.size()))
          out.push_back(std::move(b));
      return;
    }
    for (auto& env : envs)
      for (auto& b : packer_.add(env.data(), env.size()))
        out.push_back(std::move(b));
  }

  FecParams p_;
  mabur::RsEncoder rs_;
  mabur::SymbolInterleaver il_;
  mabur::SbiPacker packer_;
  uint64_t blocks_ = 0;
};

}  // namespace linkbench
