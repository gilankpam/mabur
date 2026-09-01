#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "mabur/fec_worker.h"
#include "mabur/frag.h"
#include "mabur/sbi.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"

namespace mabur {

// Per-layer configuration: the sliding-window scheme knobs plus how many FEC
// envelopes SbiPacker groups into one radio body for this layer.
struct UepLayerCfg {
  SwConfig fec;
  int blocks_per_body = 4;
};

// One radio-bound body plus which layer's SBI stream it belongs to.
struct UepBody {
  uint8_t stream_id;
  std::vector<uint8_t> body;
  // Steady-clock ms at TxQueue push; the tx thread turns it into the SBI
  // q_ms duration at pop. 0 = never stamped (dry-run, tests) → q_ms 0 =
  // unknown on the wire. Drone-local, never serialized itself.
  uint32_t enqueued_ms = 0;
  // Steady-clock µs sampled immediately before the push (unlike enqueued_ms,
  // which inherits the hot loop's top-of-iteration stamp and therefore spans
  // the venc-ring wait + FEC CPU too — dq-spike finding 2026-08-31). Feeds
  // the drone-local dq_split gauge only; 0 = never stamped.
  uint64_t pushed_us = 0;
  // True only for the body carrying fragment index 0 of its AU — the one
  // whose q_ms the GS latches as the frame's dq. Lets the tx thread report
  // a queue-wait figure directly comparable to the GS dq segment.
  bool au_first = false;
};

// Receives each body the instant its SBI group seals, while later FEC blocks
// of the same frame are still being packed. The hot loop's sink hands the
// body straight to the TxQueue so the radio drains in parallel with the
// remaining GF256/SBI CPU (dq-spike 2026-08-31: the accumulate-then-push
// shape serialized ~3.4 ms of that CPU in front of an idle radio).
using UepBodySink = std::function<void(UepBody&&)>;

// Composes Fragmenter + SwEncoder + SbiPacker into one independent pipeline
// per SVC temporal layer (stream_id 0..1), giving each layer its own
// fragmentation sequence, sliding-window FEC redundancy, and SBI sub-block
// framing.
//
// No reorder-buffer stage: SwEncoder's overlapping repair windows carry the
// time diversity that a block-FEC time-diversity buffer used to buy, so
// sources ship in the body they were sealed in instead of waiting.
//
// Each layer's SwEncoder starts at a random seq (std::random_device seeding
// one std::mt19937, one draw per layer) rather than 0: a restarted maburd
// re-sending seq 0 within SwDecoder's kResetSpan of the predecessor's last
// seq is otherwise dropped as stale for that predecessor's entire lifetime
// (final-review Critical). This only affects live process startup — it does
// not change any wire byte for a given seq.
class UepEncoder {
 public:
  static constexpr int kNumStreams = 2;

  // worker: optional shared async FEC worker handed to every layer's
  // SwEncoder (see sw_encoder.h). nullptr = fully synchronous, today's
  // exact behavior.
  UepEncoder(const std::array<UepLayerCfg, 2>& layers, int flush_ms = 15,
             FecWorker* worker = nullptr);

  // Frame-unit ingest: data = FrameHdr + Annex-B frame, stream_id already
  // chosen by the caller (classify_frame + the producer's IDR flag). Drops the
  // frame if that stream is shed (counted in dropped()), otherwise fragments
  // (usable = fec.max_packet_size() - Fragmenter::kHdrLen), feeds each
  // fragment through that layer's sliding-window encoder and SBI packer, and
  // seals the window at frame end (SwEncoder::flush) so tail symbols + their
  // repair ship now instead of at next-frame arrival (spec 2026-07-22).
  std::vector<UepBody> add_frame(int stream_id, const uint8_t* data, size_t len,
                                 uint64_t now_ms);

  // Streaming variant of the above: identical bodies in identical order,
  // delivered through sink as each one seals instead of accumulated into a
  // vector (see UepBodySink). The vector overload is this one plus a
  // push_back sink.
  void add_frame(int stream_id, const uint8_t* data, size_t len,
                 uint64_t now_ms, const UepBodySink& sink);

  // Flushes any layer that has pending (unflushed) data and has been idle
  // (no add_frame activity) for >= flush_ms.
  std::vector<UepBody> poll(uint64_t now_ms);

  // Flushes every layer in stream_id ascending order: sliding-window flush,
  // feed each resulting envelope to the SBI packer, then SBI flush.
  std::vector<UepBody> flush_all();

  // Sets both layers' FEC overhead to ov, literal — no scaling, no clamp
  // beyond SwEncoder's own.
  void set_overhead(double ov);

  // Sets one layer's FEC overhead to ov, literal (apply_op_to_uep's and the
  // debug-HTTP override's knob).
  void set_layer_overhead(int stream_id, double ov);

  void set_shed(int stream_id, bool shed);

  // True — and the drop is booked in dropped(stream_id) — when that layer is
  // shed. Lets callers skip per-frame work for shed frames BEFORE committing
  // resources (FramePipeline checks this before allocating a frame_id, so
  // sustained shed never punches id gaps into the GS FrameStream; spec
  // 2026-07-26 svct-enable). Same sid clamping as add_frame.
  bool drop_if_shed(int stream_id);

  uint64_t dropped(int stream_id) const;

  // Per-layer async-FEC gauge passthrough (fec-compute handover 2026-09-01);
  // see SwEncoder::SwFecGauge for field semantics + the window-max reset.
  // Same sid clamping as add_frame; hot-thread-only like the rest.
  SwEncoder::SwFecGauge take_fec_gauge(int stream_id);

 private:
  struct Layer {
    SwConfig fec;
    Fragmenter frag;
    SwEncoder sw;
    SbiPacker packer;
    int usable;
    bool shed = false;
    uint64_t dropped_count = 0;
    uint64_t last_activity_ms = 0;
    bool has_activity = false;

    Layer(const UepLayerCfg& cfg, uint8_t sid, uint32_t initial_seq,
          FecWorker* worker)
        : fec(cfg.fec),
          sw(cfg.fec, initial_seq, worker),
          packer(static_cast<int>(sw::kSwHeaderLen) + cfg.fec.symbol_size,
                 cfg.blocks_per_body, sid),
          usable(cfg.fec.max_packet_size() -
                 static_cast<int>(Fragmenter::kHdrLen)) {}
  };

  // Feeds a batch of sliding-window envelopes toward layer's SBI packer.
  void pack_envs(Layer& layer, uint8_t sid,
                 std::vector<std::vector<uint8_t>> envs,
                 const UepBodySink& sink);
  // Flush tail: sliding-window flush then the SBI packer flush.
  void drain_layer(Layer& layer, uint8_t sid, std::vector<UepBody>& out);

  std::array<Layer, 2> layers_;
  int flush_ms_;
};

}  // namespace mabur
