#include "mabur/uep_encoder.h"

#include <algorithm>

namespace mabur {
namespace {
// One random seq per layer so a restarted encoder lands far from its
// predecessor's stream (see uep_encoder.h class comment / SwEncoder's
// initial_seq doc). Seeded once per UepEncoder instance from
// std::random_device, not per layer, so the two draws aren't correlated by
// a shared reseed.
std::array<uint32_t, 2> random_initial_seqs() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist;
  std::array<uint32_t, 2> seqs;
  for (auto& s : seqs) s = dist(gen);
  return seqs;
}
}  // namespace

UepEncoder::UepEncoder(const std::array<UepLayerCfg, 2>& layers, int flush_ms,
                       FecWorker* worker)
    : layers_{[&] {
        const auto seq = random_initial_seqs();
        return std::array<Layer, 2>{Layer(layers[0], 0, seq[0], worker),
                                     Layer(layers[1], 1, seq[1], worker)};
      }()},
      flush_ms_(flush_ms) {}

SwEnvSink UepEncoder::env_sink(Layer& layer, const UepBodySink& sink) {
  return [&layer, &sink](const uint8_t* env, size_t n) {
    auto b = layer.packer.add_one(env, n);
    if (!b.empty()) sink(UepBody{layer.sid, std::move(b)});
  };
}

void UepEncoder::emit_flush(Layer& layer, const UepBodySink& sink) {
  auto b = layer.packer.flush_one();
  if (!b.empty()) sink(UepBody{layer.sid, std::move(b)});
}

void UepEncoder::drain_layer(Layer& layer, std::vector<UepBody>& out, bool join) {
  const UepBodySink sink = [&](UepBody&& b) { out.push_back(std::move(b)); };
  const SwEnvSink es = env_sink(layer, sink);
  layer.sw.flush(es);
  if (join)
    for (auto& e : layer.sw.finish()) es(e.data(), e.size());
  emit_flush(layer, sink);
}

void UepEncoder::add_frame(int stream_id, const uint8_t* data, size_t len,
                           uint64_t now_ms, const UepBodySink& sink) {
  int sid = std::clamp(stream_id, 0, kNumStreams - 1);
  Layer& layer = layers_[static_cast<size_t>(sid)];
  if (layer.shed) {
    ++layer.dropped_count;
    return;
  }
  const SwEnvSink es = env_sink(layer, sink);
  // Fragment header + frame bytes go straight into the envelope buffer as
  // two spans: no fragment vector, no envelope vector, no packer copies.
  layer.frag.fragment(data, len, layer.usable,
                      [&](const uint8_t* hdr, const uint8_t* chunk, size_t n) {
                        layer.sw.add_packet(hdr, Fragmenter::kHdrLen, chunk, n, es);
                      });
  // Frame-end seal: flush() seals the partial tail symbol and emits one
  // tail repair; idle re-flush is a no-op so back-to-back empty frames
  // cannot spam repairs. Also flush the SBI packer's pending group as a
  // short final body — otherwise the tail envelope(s) just sealed above sit
  // buffered until a future frame's envelopes happen to fill the group,
  // defeating the "ship now" point of the frame-end seal.
  layer.sw.flush(es);
  emit_flush(layer, sink);
  layer.last_activity_ms = now_ms;
  layer.has_activity = true;
}

std::vector<UepBody> UepEncoder::add_frame(int stream_id, const uint8_t* data,
                                           size_t len, uint64_t now_ms) {
  std::vector<UepBody> out;
  add_frame(stream_id, data, len, now_ms,
            [&](UepBody&& b) { out.push_back(std::move(b)); });
  return out;
}

void UepEncoder::collect(const UepBodySink& sink) {
  for (int sid = 0; sid < kNumStreams; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    // Read idle BEFORE the drain: idle means every repair is already in
    // the done list, so the flush below ships a complete group. Reading it
    // after could see a job finish between the two and strand its envelope
    // as a one-block body at the next harvest.
    const bool idle = !layer.sw.repairs_outstanding();
    layer.sw.collect(env_sink(layer, sink));
    if (idle) emit_flush(layer, sink);
  }
}

std::vector<UepBody> UepEncoder::poll(uint64_t now_ms) {
  std::vector<UepBody> out;
  for (int sid = 0; sid < kNumStreams; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    if (!layer.has_activity) continue;
    if (now_ms - layer.last_activity_ms < static_cast<uint64_t>(flush_ms_)) continue;

    drain_layer(layer, out, /*join=*/false);
  }
  return out;
}

std::vector<UepBody> UepEncoder::flush_all() {
  std::vector<UepBody> out;
  for (int sid = 0; sid < kNumStreams; ++sid) {
    Layer& layer = layers_[static_cast<size_t>(sid)];
    drain_layer(layer, out, /*join=*/true);
  }
  return out;
}

void UepEncoder::set_overhead(double ov) {
  for (auto& l : layers_) l.sw.set_overhead(ov);
}

void UepEncoder::set_layer_overhead(int stream_id, double ov) {
  int sid = std::clamp(stream_id, 0, kNumStreams - 1);
  layers_[static_cast<size_t>(sid)].sw.set_overhead(ov);
}

void UepEncoder::set_shed(int stream_id, bool shed) {
  layers_[static_cast<size_t>(stream_id)].shed = shed;
}

bool UepEncoder::drop_if_shed(int stream_id) {
  int sid = std::clamp(stream_id, 0, kNumStreams - 1);
  Layer& layer = layers_[static_cast<size_t>(sid)];
  if (!layer.shed) return false;
  ++layer.dropped_count;
  return true;
}

uint64_t UepEncoder::dropped(int stream_id) const {
  return layers_[static_cast<size_t>(stream_id)].dropped_count;
}

SwEncoder::SwFecGauge UepEncoder::take_fec_gauge(int stream_id) {
  int sid = std::clamp(stream_id, 0, kNumStreams - 1);
  return layers_[static_cast<size_t>(sid)].sw.take_fec_gauge();
}

}  // namespace mabur
