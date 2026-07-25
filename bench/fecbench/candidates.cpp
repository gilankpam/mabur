#include "candidates.h"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

#include "mabur/gf256.h"
#include "mabur/sw_wire.h"

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

namespace fecbench {
namespace {

namespace sw = mabur::sw;

// ------------------------------------------------------------ GF tables --
// Bench-local copies (mabur's NibbleTables are file-private in gf256.cpp).

struct GfTables {
  uint8_t exp[512], log[256];
  alignas(16) uint8_t lo[256][16];  // lo[c][x] = c*x, x in 0..15
  alignas(16) uint8_t hi[256][16];  // hi[c][x] = c*(x<<4)
  GfTables() {
    int x = 1;
    for (int i = 0; i < 255; ++i) {
      exp[i] = (uint8_t)x;
      log[x] = (uint8_t)i;
      x <<= 1;
      if (x & 0x100) x ^= 0x11D;
    }
    for (int i = 255; i < 512; ++i) exp[i] = exp[i - 255];
    auto mul = [&](int a, int b) -> uint8_t {
      if (!a || !b) return 0;
      return exp[(size_t)log[a] + log[b]];
    };
    for (int c = 0; c < 256; ++c)
      for (int v = 0; v < 16; ++v) {
        lo[c][v] = mul(c, v);
        hi[c][v] = mul(c, v << 4);
      }
  }
};
const GfTables& gft() {
  static const GfTables t;
  return t;
}

#if (defined(__ARM_NEON) || defined(__ARM_NEON__)) && !defined(__aarch64__)
// ARMv7: 16-lane nibble-table multiply, same technique as gf256.cpp mul16.
inline uint8x16_t mul16(uint8x16_t s, uint8x16_t tlo, uint8x16_t thi,
                        uint8x16_t mask) {
  uint8x16_t lo = vandq_u8(s, mask);
  uint8x16_t hi = vshrq_n_u8(s, 4);
  __asm__("vtbl.8 %e[v], {%q[t]}, %e[v]\n\t"
          "vtbl.8 %f[v], {%q[t]}, %f[v]"
          : [v] "+w"(lo)
          : [t] "w"(tlo));
  __asm__("vtbl.8 %e[v], {%q[t]}, %e[v]\n\t"
          "vtbl.8 %f[v], {%q[t]}, %f[v]"
          : [v] "+w"(hi)
          : [t] "w"(thi));
  return veorq_u8(lo, hi);
}
#endif

// --------------------------------------------------------- fused kernels --
// acc ^= c1*s1 ^ c2*s2 in ONE pass: halves the accumulator load/store
// traffic vs two gf::lincomb calls. GF(256) addition is XOR, so any
// grouping/order is byte-exact with the sequential form.
//
// Two inner-loop widths to isolate the unroll effect on the in-order A7
// (mabur's single-source loop is 64 B wide; the first fused attempt was
// 32 B wide and LOST to baseline at ss=1312 on target — 2026-07-17 run).

using Lc2 = void (*)(uint8_t* acc, const uint8_t* s1, uint8_t c1,
                     const uint8_t* s2, uint8_t c2, size_t len);

void lincomb2_scalar_tail(uint8_t* acc, const uint8_t* s1, uint8_t c1,
                          const uint8_t* s2, uint8_t c2, size_t i, size_t len) {
  const GfTables& t = gft();
  const uint8_t l1 = t.log[c1], l2 = t.log[c2];
  for (; i < len; ++i) {
    uint8_t v = acc[i];
    if (s1[i]) v ^= t.exp[(size_t)l1 + t.log[s1[i]]];
    if (s2[i]) v ^= t.exp[(size_t)l2 + t.log[s2[i]]];
    acc[i] = v;
  }
}

template <int BYTES>  // 32 or 64: main-loop width
void lincomb2(uint8_t* acc, const uint8_t* s1, uint8_t c1, const uint8_t* s2,
              uint8_t c2, size_t len) {
  if (c1 == 0) {
    mabur::gf::lincomb(acc, s2, c2, len);
    return;
  }
  if (c2 == 0) {
    mabur::gf::lincomb(acc, s1, c1, len);
    return;
  }
  size_t i = 0;
#if defined(__aarch64__)
  {
    const GfTables& t = gft();
    const uint8x16_t tlo1 = vld1q_u8(t.lo[c1]), thi1 = vld1q_u8(t.hi[c1]);
    const uint8x16_t tlo2 = vld1q_u8(t.lo[c2]), thi2 = vld1q_u8(t.hi[c2]);
    const uint8x16_t mask = vdupq_n_u8(0x0F);
    for (; i + 16 <= len; i += 16) {
      const uint8x16_t v1 = vld1q_u8(s1 + i), v2 = vld1q_u8(s2 + i);
      uint8x16_t p = veorq_u8(vqtbl1q_u8(tlo1, vandq_u8(v1, mask)),
                              vqtbl1q_u8(thi1, vshrq_n_u8(v1, 4)));
      p = veorq_u8(p, veorq_u8(vqtbl1q_u8(tlo2, vandq_u8(v2, mask)),
                               vqtbl1q_u8(thi2, vshrq_n_u8(v2, 4))));
      vst1q_u8(acc + i, veorq_u8(vld1q_u8(acc + i), p));
    }
  }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
  {
    const GfTables& t = gft();
    const uint8x16_t tlo1 = vld1q_u8(t.lo[c1]), thi1 = vld1q_u8(t.hi[c1]);
    const uint8x16_t tlo2 = vld1q_u8(t.lo[c2]), thi2 = vld1q_u8(t.hi[c2]);
    const uint8x16_t mask = vdupq_n_u8(0x0F);
    if (BYTES >= 64) {
      for (; i + 64 <= len; i += 64) {
        const uint8x16_t a0 = vld1q_u8(acc + i), a1 = vld1q_u8(acc + i + 16),
                         a2 = vld1q_u8(acc + i + 32),
                         a3 = vld1q_u8(acc + i + 48);
        vst1q_u8(acc + i,
                 veorq_u8(a0, veorq_u8(mul16(vld1q_u8(s1 + i), tlo1, thi1, mask),
                                       mul16(vld1q_u8(s2 + i), tlo2, thi2, mask))));
        vst1q_u8(acc + i + 16,
                 veorq_u8(a1, veorq_u8(mul16(vld1q_u8(s1 + i + 16), tlo1, thi1, mask),
                                       mul16(vld1q_u8(s2 + i + 16), tlo2, thi2, mask))));
        vst1q_u8(acc + i + 32,
                 veorq_u8(a2, veorq_u8(mul16(vld1q_u8(s1 + i + 32), tlo1, thi1, mask),
                                       mul16(vld1q_u8(s2 + i + 32), tlo2, thi2, mask))));
        vst1q_u8(acc + i + 48,
                 veorq_u8(a3, veorq_u8(mul16(vld1q_u8(s1 + i + 48), tlo1, thi1, mask),
                                       mul16(vld1q_u8(s2 + i + 48), tlo2, thi2, mask))));
      }
    } else {
      for (; i + 32 <= len; i += 32) {
        const uint8x16_t a0 = vld1q_u8(acc + i), a1 = vld1q_u8(acc + i + 16);
        const uint8x16_t p0 =
            veorq_u8(mul16(vld1q_u8(s1 + i), tlo1, thi1, mask),
                     mul16(vld1q_u8(s2 + i), tlo2, thi2, mask));
        const uint8x16_t p1 =
            veorq_u8(mul16(vld1q_u8(s1 + i + 16), tlo1, thi1, mask),
                     mul16(vld1q_u8(s2 + i + 16), tlo2, thi2, mask));
        vst1q_u8(acc + i, veorq_u8(a0, p0));
        vst1q_u8(acc + i + 16, veorq_u8(a1, p1));
      }
    }
    for (; i + 16 <= len; i += 16) {
      const uint8x16_t p = veorq_u8(mul16(vld1q_u8(s1 + i), tlo1, thi1, mask),
                                    mul16(vld1q_u8(s2 + i), tlo2, thi2, mask));
      vst1q_u8(acc + i, veorq_u8(vld1q_u8(acc + i), p));
    }
  }
#endif
  lincomb2_scalar_tail(acc, s1, c1, s2, c2, i, len);
}

void xor_bytes(uint8_t* dst, const uint8_t* src, size_t len) {
  size_t i = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  for (; i + 16 <= len; i += 16)
    vst1q_u8(dst + i, veorq_u8(vld1q_u8(dst + i), vld1q_u8(src + i)));
#endif
  for (; i < len; ++i) dst[i] ^= src[i];
}

// ---------------------------------------------------------------- shared --

// Envelope skeleton identical to SwEncoder::make_repair's: packed header,
// zeroed payload; returns payload offset in *off.
std::vector<uint8_t> repair_env(int ss, uint32_t next_seq, int wl,
                                uint32_t key, size_t* off) {
  sw::SwHeader h;
  h.repair = true;
  h.symbol_size = (uint16_t)ss;
  h.seq = next_seq - (uint32_t)wl;
  h.window_len = (uint8_t)wl;
  h.repair_key = key;
  std::vector<uint8_t> env;
  env.reserve(sw::kSwHeaderLen + (size_t)ss);
  sw::pack_header(env, h);
  *off = env.size();
  env.insert(env.end(), (size_t)ss, 0);
  return env;
}

// ------------------------------------------------- baseline: SwEncoder's --
// Storage and math exactly as common/src/sw_encoder.cpp: deque of
// heap-allocated vectors, one gf::lincomb per window symbol.

class BaselineDeque : public RepairGen {
 public:
  BaselineDeque(int ss, int window) : ss_(ss), window_(window) {}
  void on_seal(const uint8_t* sym) override {
    win_.emplace_back(sym, sym + ss_);
    while (win_.size() > (size_t)window_) win_.pop_front();
  }
  void make_repairs(uint32_t next_seq, uint32_t* repair_key, int nrep,
                    std::vector<std::vector<uint8_t>>* out) override {
    const int wl = (int)win_.size();
    for (int r = 0; r < nrep; ++r) {
      size_t off;
      auto env = repair_env(ss_, next_seq, wl, (*repair_key)++, &off);
      uint8_t coeffs[sw::kMaxWindow];
      sw::repair_coeffs((*repair_key) - 1, wl, coeffs);
      for (int i = 0; i < wl; ++i)
        mabur::gf::lincomb(env.data() + off, win_[(size_t)i].data(), coeffs[i],
                           (size_t)ss_);
      out->push_back(std::move(env));
    }
  }

 private:
  int ss_, window_;
  std::deque<std::vector<uint8_t>> win_;
};

// ----------------------------------------------------- flat aligned ring --
// One contiguous 16B-aligned allocation, one memcpy per seal (production
// would hand SwEncoder's sealed symbol straight to the ring instead), rows
// iterated oldest-first like the deque.

class FlatRingBase : public RepairGen {
 public:
  FlatRingBase(int ss, int window)
      : ss_((size_t)ss),
        stride_(((size_t)ss + 15) & ~(size_t)15),
        cap_((size_t)window) {
    raw_.resize(stride_ * cap_ + 15);
    base_ = raw_.data();
    base_ += (16 - ((uintptr_t)base_ & 15)) & 15;
  }
  void on_seal(const uint8_t* sym) override {
    std::memcpy(base_ + next_ * stride_, sym, ss_);
    next_ = (next_ + 1) % cap_;
    if (count_ < cap_) ++count_;
  }

 protected:
  const uint8_t* row(size_t oldest_i) const {  // 0 = oldest held symbol
    const size_t start = (next_ + cap_ - count_) % cap_;
    return base_ + ((start + oldest_i) % cap_) * stride_;
  }
  size_t ss_, stride_, cap_, count_ = 0, next_ = 0;
  std::vector<uint8_t> raw_;
  uint8_t* base_;
};

class FlatRing : public FlatRingBase {
 public:
  using FlatRingBase::FlatRingBase;
  void make_repairs(uint32_t next_seq, uint32_t* repair_key, int nrep,
                    std::vector<std::vector<uint8_t>>* out) override {
    const int wl = (int)count_;
    for (int r = 0; r < nrep; ++r) {
      size_t off;
      auto env = repair_env((int)ss_, next_seq, wl, (*repair_key)++, &off);
      uint8_t coeffs[sw::kMaxWindow];
      sw::repair_coeffs((*repair_key) - 1, wl, coeffs);
      for (int i = 0; i < wl; ++i)
        mabur::gf::lincomb(env.data() + off, row((size_t)i), coeffs[i], ss_);
      out->push_back(std::move(env));
    }
  }
};

// -------------------------------------------- flat ring + fused kernel ----

class FusedBase : public FlatRingBase {
 public:
  FusedBase(int ss, int window, Lc2 k2, bool prefetch)
      : FlatRingBase(ss, window), k2_(k2), pf_(prefetch) {}

  void make_repairs(uint32_t next_seq, uint32_t* repair_key, int nrep,
                    std::vector<std::vector<uint8_t>>* out) override {
    const int wl = (int)count_;
    for (int r = 0; r < nrep; ++r) {
      size_t off;
      auto env = repair_env((int)ss_, next_seq, wl, (*repair_key)++, &off);
      uint8_t coeffs[sw::kMaxWindow];
      sw::repair_coeffs((*repair_key) - 1, wl, coeffs);
      repair_into(env.data() + off, coeffs, wl);
      out->push_back(std::move(env));
    }
  }

 protected:
  virtual void repair_into(uint8_t* acc, const uint8_t* coeffs, int wl) {
    fold(acc, coeffs, 0, wl, 0, ss_);
  }
  // Folds window rows [i0, i1) into acc over byte range [b0, b1).
  // k2_ == nullptr selects the unfused baseline kernel (one gf::lincomb per
  // row) — used to isolate the fused kernel's effect inside the mt variants.
  void fold(uint8_t* acc, const uint8_t* coeffs, int i0, int i1, size_t b0,
            size_t b1) const {
    if (!k2_) {
      for (int i = i0; i < i1; ++i)
        mabur::gf::lincomb(acc + b0, row((size_t)i) + b0, coeffs[i], b1 - b0);
      return;
    }
    int i = i0;
    for (; i + 2 <= i1; i += 2) {
#if defined(__GNUC__)
      if (pf_ && i + 3 < i1) {
        __builtin_prefetch(row((size_t)i + 2) + b0, 0, 0);
        __builtin_prefetch(row((size_t)i + 3) + b0, 0, 0);
      }
#endif
      k2_(acc + b0, row((size_t)i) + b0, coeffs[i], row((size_t)i + 1) + b0,
          coeffs[i + 1], b1 - b0);
    }
    if (i < i1)
      mabur::gf::lincomb(acc + b0, row((size_t)i) + b0, coeffs[i], b1 - b0);
  }
  Lc2 k2_;
  bool pf_;
};

// BYTES == 0 selects the unfused baseline kernel (see fold()).
template <int BYTES>
constexpr Lc2 kern() {
  if constexpr (BYTES == 0)
    return nullptr;
  else
    return &lincomb2<BYTES>;
}

template <int BYTES, bool PF>
class FlatFused : public FusedBase {
 public:
  FlatFused(int ss, int window) : FusedBase(ss, window, kern<BYTES>(), PF) {}
};

// ------------------------------- flat ring + fused kernel + second core ---
// Persistent worker, fork-join per repair. Two split shapes:
//   - byte-half: both threads stream ALL window rows, each owning half the
//     accumulator bytes (doubles source-read bandwidth demand).
//   - row-half: each thread folds HALF the rows — worker into a private
//     scratch accumulator, XOR-merged after (halves per-thread source
//     traffic at the cost of one extra ss-sized XOR + scratch buffer).
// Both are byte-exact: GF addition is XOR, disjoint bytes / disjoint rows.

class MtBase : public FusedBase {
 public:
  MtBase(int ss, int window, Lc2 k2, bool pf) : FusedBase(ss, window, k2, pf) {
    worker_ = std::thread([this] { worker_loop(); });
  }
  ~MtBase() override {
    {
      std::lock_guard<std::mutex> l(m_);
      quit_ = true;
    }
    cv_job_.notify_one();
    worker_.join();
  }

 protected:
  struct Job {
    uint8_t* acc;
    const uint8_t* coeffs;
    int i0, i1;
    size_t b0, b1;
  };
  void run_async(const Job& j) {
    {
      std::lock_guard<std::mutex> l(m_);
      job_ = j;
      have_job_ = true;
      done_ = false;
    }
    cv_job_.notify_one();
  }
  void wait_done() {
    std::unique_lock<std::mutex> l(m_);
    cv_done_.wait(l, [this] { return done_; });
  }

 private:
  void worker_loop() {
    std::unique_lock<std::mutex> l(m_);
    for (;;) {
      cv_job_.wait(l, [this] { return have_job_ || quit_; });
      if (quit_) return;
      Job j = job_;
      have_job_ = false;
      l.unlock();
      fold(j.acc, j.coeffs, j.i0, j.i1, j.b0, j.b1);
      l.lock();
      done_ = true;
      cv_done_.notify_one();
    }
  }
  std::thread worker_;
  std::mutex m_;
  std::condition_variable cv_job_, cv_done_;
  Job job_{};
  bool have_job_ = false, done_ = true, quit_ = false;
};

template <int BYTES, bool PF>
class MtByteHalf : public MtBase {
 public:
  MtByteHalf(int ss, int window) : MtBase(ss, window, kern<BYTES>(), PF) {}

 protected:
  void repair_into(uint8_t* acc, const uint8_t* coeffs, int wl) override {
    const size_t half = ((ss_ / 2) + 15) & ~(size_t)15;
    if (half >= ss_ || wl < 4) {
      fold(acc, coeffs, 0, wl, 0, ss_);
      return;
    }
    run_async({acc, coeffs, 0, wl, half, ss_});
    fold(acc, coeffs, 0, wl, 0, half);
    wait_done();
  }
};

template <int BYTES, bool PF>
class MtRowHalf : public MtBase {
 public:
  MtRowHalf(int ss, int window)
      : MtBase(ss, window, kern<BYTES>(), PF), scratch_(stride_ + 15) {
    sbase_ = scratch_.data();
    sbase_ += (16 - ((uintptr_t)sbase_ & 15)) & 15;
  }

 protected:
  void repair_into(uint8_t* acc, const uint8_t* coeffs, int wl) override {
    if (wl < 4) {
      fold(acc, coeffs, 0, wl, 0, ss_);
      return;
    }
    const int mid = wl / 2;
    std::memset(sbase_, 0, ss_);
    run_async({sbase_, coeffs, mid, wl, 0, ss_});
    fold(acc, coeffs, 0, mid, 0, ss_);
    wait_done();
    xor_bytes(acc, sbase_, ss_);
  }

 private:
  std::vector<uint8_t> scratch_;
  uint8_t* sbase_;
};

// Row-split with ATOMIC SPIN handoff instead of mutex/condvar: the ~40 µs
// residual gap between mt2-row and ideal 2x at w128 is futex wake latency
// paid twice per repair. The worker burns its core while idle — acceptable
// on a quiesced bench; a production version would need a bounded
// spin-then-sleep. Measures the sync-latency ceiling of the fork-join shape.
template <int BYTES>
class MtRowSpin : public FusedBase {
 public:
  MtRowSpin(int ss, int window)
      : FusedBase(ss, window, kern<BYTES>(), false), scratch_(stride_ + 15) {
    sbase_ = scratch_.data();
    sbase_ += (16 - ((uintptr_t)sbase_ & 15)) & 15;
    worker_ = std::thread([this] { worker_loop(); });
  }
  ~MtRowSpin() override {
    quit_.store(true, std::memory_order_release);
    worker_.join();
  }

 protected:
  void repair_into(uint8_t* acc, const uint8_t* coeffs, int wl) override {
    if (wl < 4) {
      fold(acc, coeffs, 0, wl, 0, ss_);
      return;
    }
    const int mid = wl / 2;
    std::memset(sbase_, 0, ss_);
    coeffs_ = coeffs;
    i0_ = mid;
    i1_ = wl;
    const uint32_t ticket = job_.load(std::memory_order_relaxed) + 1;
    job_.store(ticket, std::memory_order_release);
    fold(acc, coeffs, 0, mid, 0, ss_);
    // relaxed spin + one fence: an acquire load here is LDR+DMB on ARMv7 and
    // the barrier storm from spinning slows the other core (see mt_encoder.h)
    while (done_.load(std::memory_order_relaxed) != ticket) { /* spin */ }
    std::atomic_thread_fence(std::memory_order_acquire);
    xor_bytes(acc, sbase_, ss_);
  }

 private:
  void worker_loop() {
    uint32_t seen = 0;
    for (;;) {
      uint32_t j;
      while ((j = job_.load(std::memory_order_relaxed)) == seen) {
        if (quit_.load(std::memory_order_relaxed)) return;
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      seen = j;
      fold(sbase_, coeffs_, i0_, i1_, 0, ss_);
      done_.store(seen, std::memory_order_release);
    }
  }
  std::vector<uint8_t> scratch_;
  uint8_t* sbase_;
  const uint8_t* coeffs_ = nullptr;
  int i0_ = 0, i1_ = 0;
  std::atomic<uint32_t> job_{0}, done_{0};
  std::atomic<bool> quit_{false};
  std::thread worker_;
};

// ---------------------------------------------------------------- wiring --

void baseline_lincomb(uint8_t* acc, const uint8_t* sym, uint8_t coeff,
                      size_t len) {
  mabur::gf::lincomb(acc, sym, coeff, len);
}

template <class T>
std::unique_ptr<RepairGen> make_gen(int ss, int window) {
  return std::make_unique<T>(ss, window);
}

}  // namespace

const std::vector<KernelCandidate>& kernel_candidates() {
  static const std::vector<KernelCandidate> k = {
      {"baseline/gf::lincomb", baseline_lincomb},
  };
  return k;
}

const std::vector<RepairGenCandidate>& repair_candidates() {
  static const std::vector<RepairGenCandidate> r = {
      {"baseline/deque", make_gen<BaselineDeque>},
      {"flat-ring", make_gen<FlatRing>},
      {"fused32+pf", make_gen<FlatFused<32, true>>},
      {"fused64", make_gen<FlatFused<64, false>>},
      {"fused64+pf", make_gen<FlatFused<64, true>>},
      {"mt2-byte/fused64", make_gen<MtByteHalf<64, false>>},
      {"mt2-row/fused64", make_gen<MtRowHalf<64, false>>},
      {"mt2-row/lincomb", make_gen<MtRowHalf<0, false>>},
      {"mt2-row/spin", make_gen<MtRowSpin<0>>},
  };
  return r;
}

}  // namespace fecbench
