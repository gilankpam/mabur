#include "mabur/uep_encoder.h"

#include <algorithm>

#include "mabur/nal.h"

namespace mabur {

double uep_layer_overhead(int stream_id, double cmd_overhead) {
  double ref = kUepRefOverhead[stream_id];
  double v = ref * (cmd_overhead / 0.25);
  return std::clamp(v, 0.125, 2.0);
}

UepEncoder::UepEncoder(const std::array<UepLayerCfg, 4>& layers, int flush_ms, bool interleave)
    : layers_{Layer(layers[0], 0), Layer(layers[1], 1), Layer(layers[2], 2), Layer(layers[3], 3)},
      flush_ms_(flush_ms),
      interleave_(interleave) {}

void UepEncoder::pack_block(Layer& layer, uint8_t sid,
                            std::vector<std::vector<uint8_t>> envs,
                            std::vector<UepBody>& out) {
  if (interleave_) {
    for (auto& env : layer.il.add_block(std::move(envs)))
      for (auto& b : layer.packer.add(env.data(), env.size()))
        out.push_back(UepBody{sid, std::move(b)});
    return;
  }
  for (auto& env : envs)
    for (auto& b : layer.packer.add(env.data(), env.size()))
      out.push_back(UepBody{sid, std::move(b)});
}

void UepEncoder::drain_layer(Layer& layer, uint8_t sid, std::vector<UepBody>& out) {
  pack_block(layer, sid, layer.rs.flush(), out);
  if (interleave_) {
    // Each sub-depth round becomes its own short body so every body still
    // carries at most one symbol per block (see drain_round()).
    for (;;) {
      auto round = layer.il.drain_round();
      if (round.empty()) break;
      for (auto& env : round)
        for (auto& b : layer.packer.add(env.data(), env.size()))
          out.push_back(UepBody{sid, std::move(b)});
      for (auto& b : layer.packer.flush())
        out.push_back(UepBody{sid, std::move(b)});
    }
  }
  for (auto& b : layer.packer.flush())
    out.push_back(UepBody{sid, std::move(b)});
}

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
    if (envs.empty()) continue;
    pack_block(layer, static_cast<uint8_t>(sid), std::move(envs), out);
  }
  return out;
}

std::vector<UepBody> UepEncoder::poll(uint64_t now_ms) {
  std::vector<UepBody> out;
  for (int sid = 0; sid < 4; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    if (!layer.has_activity) continue;
    if (now_ms - layer.last_activity_ms < static_cast<uint64_t>(flush_ms_)) continue;

    drain_layer(layer, static_cast<uint8_t>(sid), out);
  }
  return out;
}

std::vector<UepBody> UepEncoder::flush_all() {
  std::vector<UepBody> out;
  for (int sid = 0; sid < 4; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    drain_layer(layer, static_cast<uint8_t>(sid), out);
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
