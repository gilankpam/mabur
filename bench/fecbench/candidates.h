// A/B candidate registries for the SW-FEC optimization campaign.
//
// The BASELINE for every table is mabur's shipped code, linked unmodified
// from common/ — candidates live HERE (bench-local) until one wins on the
// SSC338Q, at which point it graduates into common/ as its own change.
//
// Two seams, matching where the optimizations under study apply:
//   - KernelCandidate: a gf::lincomb-compatible GF(256) muladd kernel.
//   - RepairGen: the unit SwEncoder owns — window storage + "emit N repair
//     envelopes". Stateful so candidates can bring their own storage layout
//     (flat aligned ring), fused kernels, and worker threads.
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace fecbench {

// dst-accumulating GF(256) multiply-add over one source symbol; must be
// byte-exact with mabur::gf::lincomb for every (len, coeff, data).
using LincombFn = void (*)(uint8_t* acc, const uint8_t* sym, uint8_t coeff,
                           size_t len);

struct KernelCandidate {
  const char* name;
  LincombFn fn;
};

// Stateful repair-path candidate, driven exactly like SwEncoder drives its
// window_: one on_seal per sealed source symbol (candidate evicts its oldest
// once `window` symbols are held), make_repairs at the credit cadence.
// Envelopes must byte-match the baseline's for the same seal/repair sequence
// (SET equality — a multicore candidate may reorder envelopes within one
// make_repairs call, never across calls).
class RepairGen {
 public:
  virtual ~RepairGen() = default;
  virtual void on_seal(const uint8_t* sym) = 0;  // exactly symbol_size bytes
  virtual void make_repairs(uint32_t next_seq, uint32_t* repair_key, int nrep,
                            std::vector<std::vector<uint8_t>>* out) = 0;
};

struct RepairGenCandidate {
  const char* name;
  std::unique_ptr<RepairGen> (*create)(int symbol_size, int window);
};

const std::vector<KernelCandidate>& kernel_candidates();
const std::vector<RepairGenCandidate>& repair_candidates();

}  // namespace fecbench
