#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/frag.h"
#include "mabur/rs_encoder.h"
#include "mabur/sbi.h"

namespace mabur {

// Reference overhead ladder per stream_id (0=critical .. 3=lightest),
// mirrors devourer's svc_uep_fec.default_uep_policy overhead assignments.
inline constexpr double kUepRefOverhead[4] = {1.00, 0.75, 0.50, 0.25};

// Scales stream_id's reference RS overhead by cmd_overhead/0.25 (0.25 is the
// "baseline" command value), clamped to [0.125, 2.0]. Byte-exact port of the
// per-layer FEC-rate scaling devourer's rate-control uses to trade outer-code
// redundancy for goodput under a single scalar "how much overhead can we
// afford" signal from the link's RC channel.
double uep_layer_overhead(int stream_id, double cmd_overhead);

// Per-layer configuration: the RS scheme knobs plus how many FEC envelopes
// SbiPacker groups into one radio body for this layer.
struct UepLayerCfg {
  RsConfig fec;
  int blocks_per_body = 4;
};

// One radio-bound body plus which layer's SBI stream it belongs to.
struct UepBody {
  uint8_t stream_id;
  std::vector<uint8_t> body;
};

// Composes Fragmenter + RsEncoder + SbiPacker into one independent pipeline
// per SVC temporal layer (stream_id 0..3), giving each layer its own
// fragmentation sequence, RS redundancy, and SBI sub-block framing. Byte-
// exact port of devourer's tools/precoder/svc_uep_fec.py's SvcUepEncoder
// (fragment=True) composition order.
class UepEncoder {
 public:
  UepEncoder(const std::array<UepLayerCfg, 4>& layers, int flush_ms = 15);

  // Classifies pkt via classify_rtp, drops it if that stream is shed
  // (counted in dropped()), otherwise fragments (usable = fec.max_packet_size()
  // - 4) and feeds each fragment through that layer's RS encoder and SBI
  // packer. Stamps the layer's last_activity_ms to now_ms.
  std::vector<UepBody> add_rtp(const uint8_t* pkt, size_t len, uint64_t now_ms);

  // Flushes any layer that has pending (unflushed) data and has been idle
  // (no add_rtp activity) for >= flush_ms.
  std::vector<UepBody> poll(uint64_t now_ms);

  // Flushes every layer in stream_id ascending order: RS flush, feed each
  // resulting envelope to the SBI packer, then SBI flush.
  std::vector<UepBody> flush_all();

  // Rescales every layer's RS overhead via uep_layer_overhead(sid, cmd_overhead).
  void set_overhead_scale(double cmd_overhead);

  void set_shed(int stream_id, bool shed);

  uint64_t dropped(int stream_id) const;

 private:
  struct Layer {
    RsConfig fec;
    Fragmenter frag;
    RsEncoder rs;
    SbiPacker packer;
    int usable;
    bool shed = false;
    uint64_t dropped_count = 0;
    uint64_t last_activity_ms = 0;
    bool has_activity = false;

    Layer(const UepLayerCfg& cfg, uint8_t sid)
        : fec(cfg.fec),
          rs(cfg.fec),
          packer(11 + cfg.fec.symbol_size, cfg.blocks_per_body, sid),
          usable(cfg.fec.max_packet_size() - 4) {}
  };

  std::array<Layer, 4> layers_;
  int flush_ms_;
};

}  // namespace mabur
