#include "mabur/uep_encoder.h"

#include <algorithm>

namespace mabur {
namespace {
// One random seq per layer so a restarted encoder lands far from its
// predecessor's stream (see uep_encoder.h class comment / SwEncoder's
// initial_seq doc). Seeded once per UepEncoder instance from
// std::random_device, not per layer, so the four draws aren't correlated by
// a shared reseed.
std::array<uint32_t, 4> random_initial_seqs() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist;
  std::array<uint32_t, 4> seqs;
  for (auto& s : seqs) s = dist(gen);
  return seqs;
}
}  // namespace

double uep_layer_overhead(int stream_id, double cmd_overhead) {
  double ref = kUepRefOverhead[stream_id];
  double v = ref * (cmd_overhead / 0.25);
  return std::clamp(v, 0.125, 2.0);
}

UepEncoder::UepEncoder(const std::array<UepLayerCfg, 4>& layers, int flush_ms,
                       FecWorker* worker)
    : layers_{[&] {
        const auto seq = random_initial_seqs();
        return std::array<Layer, 4>{Layer(layers[0], 0, seq[0], worker),
                                     Layer(layers[1], 1, seq[1], worker),
                                     Layer(layers[2], 2, seq[2], worker),
                                     Layer(layers[3], 3, seq[3], worker)};
      }()},
      flush_ms_(flush_ms) {}

void UepEncoder::pack_envs(Layer& layer, uint8_t sid,
                           std::vector<std::vector<uint8_t>> envs,
                           std::vector<UepBody>& out) {
  for (auto& env : envs)
    for (auto& b : layer.packer.add(env.data(), env.size()))
      out.push_back(UepBody{sid, std::move(b)});
}

void UepEncoder::drain_layer(Layer& layer, uint8_t sid, std::vector<UepBody>& out) {
  pack_envs(layer, sid, layer.sw.flush(), out);
  for (auto& b : layer.packer.flush())
    out.push_back(UepBody{sid, std::move(b)});
}

std::vector<UepBody> UepEncoder::add_frame(int stream_id, const uint8_t* data,
                                           size_t len, uint64_t now_ms) {
  std::vector<UepBody> out;
  int sid = stream_id < 0 ? 0 : (stream_id > 3 ? 3 : stream_id);
  Layer& layer = layers_[static_cast<size_t>(sid)];
  if (layer.shed) {
    ++layer.dropped_count;
    return out;
  }
  auto frags = layer.frag.fragment(data, len, layer.usable);
  for (auto& f : frags)
    pack_envs(layer, static_cast<uint8_t>(sid),
              layer.sw.add_packet(f.data(), f.size()), out);
  // Frame-end seal: flush() seals the partial tail symbol and emits one
  // tail repair; idle re-flush is a no-op so back-to-back empty frames
  // cannot spam repairs. Also flush the SBI packer's pending group as a
  // short final body — otherwise the tail envelope(s) just sealed above sit
  // buffered until a future frame's envelopes happen to fill the group,
  // defeating the "ship now" point of the frame-end seal.
  pack_envs(layer, static_cast<uint8_t>(sid), layer.sw.flush(), out);
  for (auto& b : layer.packer.flush())
    out.push_back(UepBody{static_cast<uint8_t>(sid), std::move(b)});
  layer.last_activity_ms = now_ms;
  layer.has_activity = true;
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
    layers_[static_cast<size_t>(sid)].sw.set_overhead(uep_layer_overhead(sid, cmd_overhead));
}

void UepEncoder::set_shed(int stream_id, bool shed) {
  layers_[static_cast<size_t>(stream_id)].shed = shed;
}

uint64_t UepEncoder::dropped(int stream_id) const {
  return layers_[static_cast<size_t>(stream_id)].dropped_count;
}

}  // namespace mabur
