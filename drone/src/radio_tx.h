#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>

#include "mabur/profile.h"
#include "mabur/uep_encoder.h"

namespace mabur {

// Sink for a fully-built 802.11 frame (radiotap + MAC header + body). Owned
// by whatever drives the actual radio (devourer's send_packet in the real
// build, a capture double in tests).
class FrameSink {
 public:
  struct View {
    const uint8_t* data;
    size_t len;
  };

  virtual ~FrameSink() = default;
  virtual bool send(const uint8_t* frame, size_t len) = 0;

  // Batch submit; returns how many frames the sink accepted. The default is
  // a per-frame send() loop; sinks with a cheaper bulk path (devourer's
  // send_packets USB TX aggregation packs up to 3 frames per bulk-OUT URB
  // on Jaguar3) override it.
  virtual size_t send_many(const View* frames, size_t n) {
    size_t ok = 0;
    for (size_t i = 0; i < n; ++i)
      if (frames[i].data && send(frames[i].data, frames[i].len)) ++ok;
    return ok;
  }
};

// Builds `radiotap(layer.bw) | 24-byte 802.11 header | body` frames for the
// 2-slot adaptive-link ladder and hands them to a FrameSink. Every frame
// transmits at the ladder's configured width — there is no per-seq
// bandwidth-probe schedule (removed 2026-07-27, SDD ladder-controller
// Task 5: the ladder controller never varies bw independently of the
// commanded rung, so probing alternate widths had no consumer left).
//
// Radiotap headers are prebuilt per layer in set_ladder() — called from the
// agent thread whenever the ladder changes — and cached in a
// std::shared_ptr<Cache> that send_body() reads via atomic load. This keeps
// the hot thread (send_body, ~1350 fps) from ever observing a torn
// combination of new radiotap bytes paired with a stale (or vice versa)
// cache generation: one atomic shared_ptr swap in set_ladder() covers the
// whole table consistently.
//
// send_body() itself is NOT thread-safe against concurrent calls from
// multiple threads — it owns a reusable scratch buffer for the outgoing
// frame to avoid a per-frame heap allocation on the hot path, so it must
// only ever be called from one (the same) thread.
//
// power_offset_db (LayerTxSpec) is carried through the ladder but not
// emitted anywhere in v1 — there is no DBM_TX_POWER radiotap field written.
// A future task can add it once the radio's per-packet power path exists.
class RadioTx {
 public:
  explicit RadioTx(FrameSink& sink);

  // Rebuilds the radiotap cache for the new ladder: prebuilds one radiotap
  // header per layer, at that layer's bw. Safe to call concurrently with
  // send_body() (agent thread vs hot thread) — the cache is swapped in
  // atomically once fully built.
  void set_ladder(const std::array<rc::LayerTxSpec, 2>& ladder);

  // Builds and sends one frame for `stream_id` mapped to a ladder slot:
  // stream 0→slot0, 1→slot1, anything else→slot0 (MSP OSD + side-channels
  // ride the robust base rate). Returns whatever sink.send() returned. The
  // sequence counter is consumed (incremented, mod 4096) regardless of the
  // sink's return value, so a ground-station gap detector sees the loss
  // represented as a skipped sequence number.
  // NOTE: send_body() before set_ladder() drops frames (missing radiotap cache).
  bool send_body(uint8_t stream_id, const uint8_t* body, size_t len);

  // Batch variant: builds one frame per body, then hands them all to
  // sink.send_many() in a single call (URB-aggregating sinks pack several
  // frames per bulk transfer). Framing, seq consumption and sent/drops
  // accounting match the equivalent send_body() loop. Same
  // single-caller-thread contract as send_body (shares its buffer pool).
  size_t send_bodies(const std::vector<UepBody>& bodies);

  uint16_t seq() const { return seq_; }
  uint64_t sent() const { return sent_; }
  uint64_t drops() const { return drops_; }

 private:
  // Per-layer radiotap bytes at that layer's configured bw — everything
  // send_body() needs for one layer, read together from one
  // atomically-swapped-in snapshot. Empty (default-constructed) until the
  // first set_ladder() call.
  struct LayerCache {
    std::vector<uint8_t> radiotap;
  };
  struct Cache {
    std::array<LayerCache, 2> layers;
  };

  // Builds `radiotap | dot11(seq_) | body` into out, consuming seq_. False
  // (drop counted) when the radiotap cache has no entry for the effective
  // bw (send before set_ladder).
  bool build_frame(const Cache& cache, uint8_t stream_id, const uint8_t* body,
                   size_t len, std::vector<uint8_t>& out);

  FrameSink& sink_;
  std::atomic<std::shared_ptr<Cache>> cache_;
  uint16_t seq_ = 0;
  uint64_t sent_ = 0;
  uint64_t drops_ = 0;
  std::vector<uint8_t> scratch_;  // reusable frame buffer (single-threaded use)
  std::vector<std::vector<uint8_t>> pool_;  // send_bodies frame buffers
};

}  // namespace mabur
