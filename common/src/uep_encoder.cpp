#include "mabur/uep_encoder.h"

#include <algorithm>

#include "mabur/nal.h"

namespace mabur {

double uep_layer_overhead(int stream_id, double cmd_overhead) {
  double ref = kUepRefOverhead[stream_id];
  double v = ref * (cmd_overhead / 0.25);
  return std::clamp(v, 0.125, 2.0);
}

UepEncoder::UepEncoder(const std::array<UepLayerCfg, 4>& layers, int flush_ms)
    : layers_{Layer(layers[0], 0), Layer(layers[1], 1), Layer(layers[2], 2), Layer(layers[3], 3)},
      flush_ms_(flush_ms) {}

std::vector<UepBody> UepEncoder::add_rtp(const uint8_t* pkt, size_t len, uint64_t now_ms) {
  std::vector<UepBody> out;
  int sid = classify_rtp(pkt, len);
  Layer& layer = layers_[static_cast<size_t>(sid)];

  layer.last_activity_ms = now_ms;
  layer.has_activity = true;

  if (layer.shed) {
    ++layer.dropped_count;
    return out;
  }

  auto frags = layer.frag.fragment(pkt, len, layer.usable);
  for (auto& f : frags) {
    auto envs = layer.rs.add_packet(f.data(), f.size());
    for (auto& env : envs) {
      auto bodies = layer.packer.add(env.data(), env.size());
      for (auto& b : bodies) out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
    }
  }
  return out;
}

std::vector<UepBody> UepEncoder::poll(uint64_t now_ms) {
  std::vector<UepBody> out;
  for (int sid = 0; sid < 4; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    if (!layer.has_activity) continue;
    if (now_ms - layer.last_activity_ms < static_cast<uint64_t>(flush_ms_)) continue;

    for (auto& env : layer.rs.flush()) {
      for (auto& b : layer.packer.add(env.data(), env.size()))
        out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
    }
    for (auto& b : layer.packer.flush())
      out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
  }
  return out;
}

std::vector<UepBody> UepEncoder::flush_all() {
  std::vector<UepBody> out;
  for (int sid = 0; sid < 4; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    for (auto& env : layer.rs.flush()) {
      for (auto& b : layer.packer.add(env.data(), env.size()))
        out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
    }
    for (auto& b : layer.packer.flush())
      out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
  }
  return out;
}

void UepEncoder::set_overhead_scale(double cmd_overhead) {
  for (int sid = 0; sid < 4; ++sid)
    layers_[static_cast<size_t>(sid)].rs.set_overhead(uep_layer_overhead(sid, cmd_overhead));
}

void UepEncoder::set_shed(int stream_id, bool shed) {
  layers_[static_cast<size_t>(stream_id)].shed = shed;
}

uint64_t UepEncoder::dropped(int stream_id) const {
  return layers_[static_cast<size_t>(stream_id)].dropped_count;
}

}  // namespace mabur
