// Bench-side wiring of the winning repair shape (mt2-row/spin, RESULTS.md)
// into the FULL encode pipeline — zero mabur changes. Reuses Fragmenter,
// SbiPacker and classify_rtp unmodified; only the sliding-window engine is
// swapped:
//
//   SpinWorker    one shared worker thread, atomic spin handoff (measures
//                 the sync-latency ceiling; production needs spin-then-sleep)
//   SwEncoderMt   SwEncoder replicated byte-for-byte, window in a flat
//                 aligned ring, repairs row-split across both cores
//   UepEncoderT   uep_encoder.cpp's composition, templated on the engine and
//                 taking FIXED initial seqs so UepEncoderT<SwStock> vs
//                 UepEncoderT<SwEncoderMt> body streams compare byte-exact
//                 (the real UepEncoder draws random seqs, so it can only be
//                 perf-compared, not byte-compared)
#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#include "mabur/frag.h"
#include "mabur/gf256.h"
#include "mabur/nal.h"
#include "mabur/sbi.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mabur/uep_encoder.h"  // UepLayerCfg, UepBody, uep_layer_overhead

namespace fecbench {

// Pins the calling thread to a CPU (no-op off Linux / on failure).
inline void pin_to_cpu(int cpu) {
#if defined(__linux__)
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  pthread_setaffinity_np(pthread_self(), sizeof set, &set);
#else
  (void)cpu;
#endif
}

// One shared worker with BOUNDED spin-then-sleep pickup and optional core
// pinning. Two drone lessons are baked in (encoder runs, 2026-07-17):
//   1. Spin bodies use RELAXED loads (plain LDR on a cached line) with one
//      acquire fence after the change is seen — an acquire load in the loop
//      is LDR+DMB on ARMv7 and the barrier storm from an idle spinner slowed
//      the other core 10-22%.
//   2. The worker must SLEEP when idle: a never-sleeping spinner keeps the
//      load balancer migrating the main thread between cores, which erased
//      the mt gain at low repair rates even with relaxed spins. It spins
//      kSpinIters (~50-100 us) for the fast handoff, then futex-waits.
class SpinWorker {
 public:
  explicit SpinWorker(int cpu = -1) : cpu_(cpu), th_([this] { loop(); }) {}
  ~SpinWorker() {
    quit_.store(true, std::memory_order_seq_cst);
    {
      std::lock_guard<std::mutex> l(m_);
    }
    cv_.notify_one();
    th_.join();
  }

  // Runs fn(ctx) on the worker. One job in flight; wait() before the next.
  void submit(void (*fn)(void*), void* ctx) {
    fn_ = fn;
    ctx_ = ctx;
    job_.store(job_.load(std::memory_order_relaxed) + 1,
               std::memory_order_seq_cst);
    if (sleeping_.load(std::memory_order_seq_cst)) {
      {
        std::lock_guard<std::mutex> l(m_);
      }
      cv_.notify_one();
    }
  }

  // Caller-side wait is always short (the worker is folding its half-window)
  // — pure relaxed spin.
  void wait() const {
    const uint32_t t = job_.load(std::memory_order_relaxed);
    while (done_.load(std::memory_order_relaxed) != t) { /* spin */ }
    std::atomic_thread_fence(std::memory_order_acquire);
  }

 private:
  static constexpr long kSpinIters = 16384;  // ~50-100 us on the 800MHz A7

  void loop() {
    if (cpu_ >= 0) pin_to_cpu(cpu_);
    uint32_t seen = 0;
    for (;;) {
      uint32_t j;
      long spins = 0;
      while ((j = job_.load(std::memory_order_relaxed)) == seen) {
        if (quit_.load(std::memory_order_relaxed)) return;
        if (++spins >= kSpinIters) {
          std::unique_lock<std::mutex> l(m_);
          sleeping_.store(true, std::memory_order_seq_cst);
          cv_.wait(l, [&] {
            return job_.load(std::memory_order_relaxed) != seen ||
                   quit_.load(std::memory_order_relaxed);
          });
          sleeping_.store(false, std::memory_order_seq_cst);
          spins = 0;
        }
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      seen = j;
      fn_(ctx_);
      done_.store(seen, std::memory_order_release);
    }
  }
  int cpu_;
  void (*fn_)(void*) = nullptr;
  void* ctx_ = nullptr;
  std::atomic<uint32_t> job_{0}, done_{0};
  std::atomic<bool> quit_{false}, sleeping_{false};
  std::mutex m_;
  std::condition_variable cv_;
  std::thread th_;
};

// common/src/sw_encoder.cpp replicated; every emitted envelope byte-exact.
// Differences: window rows live in one flat 16B-aligned ring, and repair
// GF folds are row-split with the shared SpinWorker (nullptr = single-core).
class SwEncoderMt {
 public:
  using Worker = SpinWorker;
  SwEncoderMt(const mabur::SwConfig& cfg, uint32_t initial_seq,
              SpinWorker* worker)
      : cfg_(cfg), worker_(worker), next_seq_(initial_seq) {
    if (cfg_.window < 2) cfg_.window = 2;
    if (cfg_.window > mabur::sw::kMaxWindow) cfg_.window = mabur::sw::kMaxWindow;
    stride_ = ((size_t)cfg_.symbol_size + 15) & ~(size_t)15;
    ring_raw_.resize(stride_ * (size_t)cfg_.window + 15);
    ring_ = align16(ring_raw_.data());
    scratch_raw_.resize(stride_ + 15);
    scratch_ = align16(scratch_raw_.data());
  }

  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* data, size_t len) {
    std::vector<std::vector<uint8_t>> out;
    if ((int)len > cfg_.max_packet_size()) {
      ++oversize_drops_;
      return out;
    }
    const size_t needed = 2 + len;
    const size_t remaining = (size_t)cfg_.symbol_size - current_symbol_.size();
    if (needed > remaining) seal_current(out);
    const uint16_t ln = (uint16_t)len;
    current_symbol_.push_back((uint8_t)(ln & 0xFF));
    current_symbol_.push_back((uint8_t)((ln >> 8) & 0xFF));
    current_symbol_.insert(current_symbol_.end(), data, data + len);
    return out;
  }

  std::vector<std::vector<uint8_t>> flush() {
    std::vector<std::vector<uint8_t>> out;
    seal_current(out);
    if (tail_repair_pending_ && count_ > 0) out.push_back(make_repair());
    return out;
  }

  void set_overhead(double overhead) { cfg_.overhead = overhead; }
  bool has_pending() const { return !current_symbol_.empty(); }

 private:
  static uint8_t* align16(uint8_t* p) {
    return p + ((16 - ((uintptr_t)p & 15)) & 15);
  }
  const uint8_t* row(size_t oldest_i) const {  // 0 = oldest held symbol
    const size_t start = (next_slot_ + (size_t)cfg_.window - count_) %
                         (size_t)cfg_.window;
    return ring_ + ((start + oldest_i) % (size_t)cfg_.window) * stride_;
  }

  void seal_current(std::vector<std::vector<uint8_t>>& out) {
    if (current_symbol_.empty()) return;
    const size_t ss = (size_t)cfg_.symbol_size;
    current_symbol_.resize(ss, 0);

    mabur::sw::SwHeader h;
    h.repair = false;
    h.symbol_size = (uint16_t)cfg_.symbol_size;
    h.seq = next_seq_;
    std::vector<uint8_t> env;
    env.reserve(mabur::sw::kSwHeaderLen + ss);
    mabur::sw::pack_header(env, h);
    env.insert(env.end(), current_symbol_.begin(), current_symbol_.end());
    out.push_back(std::move(env));

    std::memcpy(ring_ + next_slot_ * stride_, current_symbol_.data(), ss);
    next_slot_ = (next_slot_ + 1) % (size_t)cfg_.window;
    if (count_ < (size_t)cfg_.window) ++count_;
    current_symbol_.clear();
    ++next_seq_;
    tail_repair_pending_ = true;

    credit_ += cfg_.overhead;
    while (credit_ >= 1.0) {
      out.push_back(make_repair());
      credit_ -= 1.0;
    }
  }

  struct FoldCtx {
    SwEncoderMt* self;
    const uint8_t* coeffs;
    int i0, i1;
  };
  static void fold_job(void* p) {
    auto* c = (FoldCtx*)p;
    SwEncoderMt* s = c->self;
    std::memset(s->scratch_, 0, (size_t)s->cfg_.symbol_size);
    s->fold(s->scratch_, c->coeffs, c->i0, c->i1);
  }
  void fold(uint8_t* acc, const uint8_t* coeffs, int i0, int i1) const {
    for (int i = i0; i < i1; ++i)
      mabur::gf::lincomb(acc, row((size_t)i), coeffs[i],
                         (size_t)cfg_.symbol_size);
  }

  std::vector<uint8_t> make_repair() {
    const size_t ss = (size_t)cfg_.symbol_size;
    const int wl = (int)count_;
    mabur::sw::SwHeader h;
    h.repair = true;
    h.symbol_size = (uint16_t)cfg_.symbol_size;
    h.seq = next_seq_ - (uint32_t)wl;
    h.window_len = (uint8_t)wl;
    h.repair_key = repair_key_++;

    std::vector<uint8_t> env;
    env.reserve(mabur::sw::kSwHeaderLen + ss);
    mabur::sw::pack_header(env, h);
    const size_t off = env.size();
    env.insert(env.end(), ss, 0);
    uint8_t* acc = env.data() + off;

    uint8_t coeffs[mabur::sw::kMaxWindow];
    mabur::sw::repair_coeffs(h.repair_key, wl, coeffs);

    if (!worker_ || wl < 8) {
      fold(acc, coeffs, 0, wl);
    } else {
      const int mid = wl / 2;
      FoldCtx ctx{this, coeffs, mid, wl};
      worker_->submit(&fold_job, &ctx);
      fold(acc, coeffs, 0, mid);
      worker_->wait();
      size_t i = 0;
      for (; i < ss; ++i) acc[i] ^= scratch_[i];
    }

    tail_repair_pending_ = false;
    return env;
  }

  mabur::SwConfig cfg_;
  SpinWorker* worker_;
  size_t stride_ = 0, count_ = 0, next_slot_ = 0;
  std::vector<uint8_t> ring_raw_, scratch_raw_;
  uint8_t* ring_ = nullptr;
  uint8_t* scratch_ = nullptr;
  std::vector<uint8_t> current_symbol_;
  uint32_t next_seq_ = 0, repair_key_ = 0;
  double credit_ = 0.0;
  bool tail_repair_pending_ = false;
  size_t oversize_drops_ = 0;
};

// ------------------------------------------------------------- async mt ---
// The fork-join shape above measured PARITY end-to-end on the drone: repairs
// arrive every 0.3-1 ms, so a bounded-spin worker is asleep for most
// submissions and the futex wake latency eats the row-split gain (always-
// spinning measured worse still). The shape that fits the cadence is a
// PIPELINE: the hot thread enqueues a repair job (window coordinates only —
// the ring carries kSlackRows so in-flight rows aren't overwritten by later
// seals) and continues; the worker builds the whole envelope alone and
// delivers it at the hot thread's next drain point. Repairs trail their
// sources by a beat (decoder is order-agnostic; envelope BYTES stay
// identical, so verification compares sorted envelope sets).

class SwEncoderMtAsync;

class AsyncFecWorker {
 public:
  explicit AsyncFecWorker(int cpu = -1) : cpu_(cpu), th_([this] { loop(); }) {}
  ~AsyncFecWorker() {
    quit_.store(true, std::memory_order_seq_cst);
    {
      std::lock_guard<std::mutex> l(m_);
    }
    cv_.notify_one();
    th_.join();
  }

  struct Job {
    SwEncoderMtAsync* eng;
    uint32_t repair_key;
    uint32_t header_seq;  // h.seq (window start) captured at credit time
    int wl;
    size_t start_slot;  // ring slot of the window's oldest row at credit time
  };

  // Single producer (the hot thread owns every layer engine). False = queue
  // full, caller must run the job inline.
  bool try_enqueue(const Job& j) {
    const uint32_t t = tail_.load(std::memory_order_relaxed);
    if (t - head_.load(std::memory_order_acquire) >= kQueue) return false;
    q_[t % kQueue] = j;
    tail_.store(t + 1, std::memory_order_seq_cst);
    if (sleeping_.load(std::memory_order_seq_cst)) {
      {
        std::lock_guard<std::mutex> l(m_);
      }
      cv_.notify_one();
    }
    return true;
  }

 private:
  static constexpr uint32_t kQueue = 256;
  static constexpr long kSpinIters = 16384;

  void loop();  // defined after SwEncoderMtAsync (calls eng->execute_job)

  int cpu_;
  std::array<Job, kQueue> q_{};
  std::atomic<uint32_t> head_{0}, tail_{0};
  std::atomic<bool> quit_{false}, sleeping_{false};
  std::mutex m_;
  std::condition_variable cv_;
  std::thread th_;
};

// SwEncoder semantics with repairs built asynchronously on AsyncFecWorker.
// Envelope bytes are identical to stock SwEncoder's; only emission ORDER
// changes (repairs surface at the next add_packet/flush drain). flush()
// joins: every queued repair for this engine completes and is emitted, so
// the tail-repair contract and idle-flush semantics survive.
class SwEncoderMtAsync {
 public:
  using Worker = AsyncFecWorker;
  static constexpr size_t kSlackRows = 64;

  SwEncoderMtAsync(const mabur::SwConfig& cfg, uint32_t initial_seq,
                   AsyncFecWorker* worker)
      : cfg_(cfg), worker_(worker), next_seq_(initial_seq) {
    if (cfg_.window < 2) cfg_.window = 2;
    if (cfg_.window > mabur::sw::kMaxWindow) cfg_.window = mabur::sw::kMaxWindow;
    stride_ = ((size_t)cfg_.symbol_size + 15) & ~(size_t)15;
    cap_ = (size_t)cfg_.window + kSlackRows;
    ring_raw_.resize(stride_ * cap_ + 15);
    ring_ = ring_raw_.data();
    ring_ += (16 - ((uintptr_t)ring_ & 15)) & 15;
  }
  ~SwEncoderMtAsync() { join(); }

  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* data, size_t len) {
    std::vector<std::vector<uint8_t>> out;
    drain_done(out);
    if ((int)len > cfg_.max_packet_size()) return out;
    const size_t needed = 2 + len;
    const size_t remaining = (size_t)cfg_.symbol_size - current_symbol_.size();
    if (needed > remaining) seal_current(out);
    const uint16_t ln = (uint16_t)len;
    current_symbol_.push_back((uint8_t)(ln & 0xFF));
    current_symbol_.push_back((uint8_t)((ln >> 8) & 0xFF));
    current_symbol_.insert(current_symbol_.end(), data, data + len);
    return out;
  }

  std::vector<std::vector<uint8_t>> flush() {
    std::vector<std::vector<uint8_t>> out;
    seal_current(out);
    if (tail_repair_pending_ && count_ > 0) enqueue_repair();
    join();
    drain_done(out);
    return out;
  }

  void set_overhead(double overhead) { cfg_.overhead = overhead; }

  // Called on the worker thread.
  void execute_job(const AsyncFecWorker::Job& j) {
    const size_t ss = (size_t)cfg_.symbol_size;
    mabur::sw::SwHeader h;
    h.repair = true;
    h.symbol_size = (uint16_t)cfg_.symbol_size;
    h.seq = j.header_seq;
    h.window_len = (uint8_t)j.wl;
    h.repair_key = j.repair_key;
    std::vector<uint8_t> env;
    env.reserve(mabur::sw::kSwHeaderLen + ss);
    mabur::sw::pack_header(env, h);
    const size_t off = env.size();
    env.insert(env.end(), ss, 0);
    uint8_t coeffs[mabur::sw::kMaxWindow];
    mabur::sw::repair_coeffs(j.repair_key, j.wl, coeffs);
    for (int i = 0; i < j.wl; ++i)
      mabur::gf::lincomb(env.data() + off,
                         ring_ + ((j.start_slot + (size_t)i) % cap_) * stride_,
                         coeffs[i], ss);
    {
      std::lock_guard<std::mutex> l(done_m_);
      done_.push_back(std::move(env));
    }
    outstanding_.fetch_sub(1, std::memory_order_release);
  }

 private:
  void seal_current(std::vector<std::vector<uint8_t>>& out) {
    if (current_symbol_.empty()) return;
    const size_t ss = (size_t)cfg_.symbol_size;
    current_symbol_.resize(ss, 0);

    mabur::sw::SwHeader h;
    h.repair = false;
    h.symbol_size = (uint16_t)cfg_.symbol_size;
    h.seq = next_seq_;
    std::vector<uint8_t> env;
    env.reserve(mabur::sw::kSwHeaderLen + ss);
    mabur::sw::pack_header(env, h);
    env.insert(env.end(), current_symbol_.begin(), current_symbol_.end());
    out.push_back(std::move(env));

    // A row stays readable for kSlackRows further seals after leaving the
    // window; join() if a queued job might still reference the row this
    // seal overwrites (backstop — at bench rates the queue never gets there).
    if (outstanding_.load(std::memory_order_acquire) &&
        seals_since_oldest_job_++ >= (long)kSlackRows - 2)
      join();
    std::memcpy(ring_ + next_slot_ * stride_, current_symbol_.data(), ss);
    next_slot_ = (next_slot_ + 1) % cap_;
    if (count_ < (size_t)cfg_.window) ++count_;
    current_symbol_.clear();
    ++next_seq_;
    tail_repair_pending_ = true;

    credit_ += cfg_.overhead;
    while (credit_ >= 1.0) {
      enqueue_repair();
      credit_ -= 1.0;
    }
  }

  void enqueue_repair() {
    const int wl = (int)count_;
    AsyncFecWorker::Job j;
    j.eng = this;
    j.repair_key = repair_key_++;
    j.header_seq = next_seq_ - (uint32_t)wl;
    j.wl = wl;
    j.start_slot = (next_slot_ + cap_ - (size_t)wl) % cap_;
    tail_repair_pending_ = false;
    if (!outstanding_.load(std::memory_order_relaxed))
      seals_since_oldest_job_ = 0;
    outstanding_.fetch_add(1, std::memory_order_relaxed);
    if (!worker_ || !worker_->try_enqueue(j)) {
      execute_job(j);  // queue full or no worker: inline fallback
    }
  }

  void join() {
    while (outstanding_.load(std::memory_order_acquire) != 0) { /* spin */ }
    seals_since_oldest_job_ = 0;
  }

  void drain_done(std::vector<std::vector<uint8_t>>& out) {
    std::lock_guard<std::mutex> l(done_m_);
    for (auto& e : done_) out.push_back(std::move(e));
    done_.clear();
  }

  mabur::SwConfig cfg_;
  AsyncFecWorker* worker_;
  size_t stride_ = 0, cap_ = 0, count_ = 0, next_slot_ = 0;
  long seals_since_oldest_job_ = 0;
  std::vector<uint8_t> ring_raw_;
  uint8_t* ring_ = nullptr;
  std::vector<uint8_t> current_symbol_;
  uint32_t next_seq_ = 0, repair_key_ = 0;
  double credit_ = 0.0;
  bool tail_repair_pending_ = false;
  std::atomic<int> outstanding_{0};
  std::mutex done_m_;
  std::vector<std::vector<uint8_t>> done_;
};

inline void AsyncFecWorker::loop() {
  if (cpu_ >= 0) pin_to_cpu(cpu_);
  uint32_t seen = 0;
  for (;;) {
    long spins = 0;
    while (tail_.load(std::memory_order_relaxed) == seen) {
      if (quit_.load(std::memory_order_relaxed)) return;
      if (++spins >= kSpinIters) {
        std::unique_lock<std::mutex> l(m_);
        sleeping_.store(true, std::memory_order_seq_cst);
        cv_.wait(l, [&] {
          return tail_.load(std::memory_order_relaxed) != seen ||
                 quit_.load(std::memory_order_relaxed);
        });
        sleeping_.store(false, std::memory_order_seq_cst);
        spins = 0;
      }
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    Job j = q_[seen % kQueue];
    head_.store(seen + 1, std::memory_order_release);
    ++seen;
    j.eng->execute_job(j);
  }
}

// Stock SwEncoder behind the same 3-arg constructor so UepEncoderT can host
// either engine.
class SwStock {
 public:
  using Worker = SpinWorker;
  SwStock(const mabur::SwConfig& cfg, uint32_t initial_seq, SpinWorker*)
      : e_(cfg, initial_seq) {}
  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* d, size_t l) {
    return e_.add_packet(d, l);
  }
  std::vector<std::vector<uint8_t>> flush() { return e_.flush(); }
  void set_overhead(double ov) { e_.set_overhead(ov); }

 private:
  mabur::SwEncoder e_;
};

// common/src/uep_encoder.cpp's composition with deterministic initial seqs.
template <class Sw>
class UepEncoderT {
 public:
  UepEncoderT(const std::array<mabur::UepLayerCfg, 4>& layers, int flush_ms,
              const std::array<uint32_t, 4>& seqs,
              typename Sw::Worker* worker)
      : layers_{Layer(layers[0], 0, seqs[0], worker),
                Layer(layers[1], 1, seqs[1], worker),
                Layer(layers[2], 2, seqs[2], worker),
                Layer(layers[3], 3, seqs[3], worker)},
        flush_ms_(flush_ms) {}

  std::vector<mabur::UepBody> add_rtp(const uint8_t* pkt, size_t len,
                                      uint64_t now_ms) {
    std::vector<mabur::UepBody> out;
    int sid = mabur::classify_rtp(pkt, len);
    Layer& layer = layers_[(size_t)sid];
    layer.last_activity_ms = now_ms;
    layer.has_activity = true;
    auto frags = layer.frag.fragment(pkt, len, layer.usable);
    for (auto& f : frags) {
      auto envs = layer.sw.add_packet(f.data(), f.size());
      if (envs.empty()) continue;
      pack_envs(layer, (uint8_t)sid, std::move(envs), out);
    }
    return out;
  }

  std::vector<mabur::UepBody> poll(uint64_t now_ms) {
    std::vector<mabur::UepBody> out;
    for (int sid = 0; sid < 4; ++sid) {
      Layer& layer = layers_[(size_t)sid];
      if (!layer.has_activity) continue;
      if (now_ms - layer.last_activity_ms < (uint64_t)flush_ms_) continue;
      drain_layer(layer, (uint8_t)sid, out);
    }
    return out;
  }

 private:
  struct Layer {
    mabur::SwConfig fec;
    mabur::Fragmenter frag;
    Sw sw;
    mabur::SbiPacker packer;
    int usable;
    uint64_t last_activity_ms = 0;
    bool has_activity = false;
    Layer(const mabur::UepLayerCfg& cfg, uint8_t sid, uint32_t initial_seq,
          typename Sw::Worker* worker)
        : fec(cfg.fec),
          sw(cfg.fec, initial_seq, worker),
          packer((int)mabur::sw::kSwHeaderLen + cfg.fec.symbol_size,
                 cfg.blocks_per_body, sid),
          usable(cfg.fec.max_packet_size() - 4) {}
  };

  void pack_envs(Layer& layer, uint8_t sid,
                 std::vector<std::vector<uint8_t>> envs,
                 std::vector<mabur::UepBody>& out) {
    for (auto& env : envs)
      for (auto& b : layer.packer.add(env.data(), env.size()))
        out.push_back(mabur::UepBody{sid, std::move(b)});
  }
  void drain_layer(Layer& layer, uint8_t sid, std::vector<mabur::UepBody>& out) {
    pack_envs(layer, sid, layer.sw.flush(), out);
    for (auto& b : layer.packer.flush())
      out.push_back(mabur::UepBody{sid, std::move(b)});
  }

  std::array<Layer, 4> layers_;
  int flush_ms_;
};

}  // namespace fecbench
