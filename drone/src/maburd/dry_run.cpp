// Dry-run mode: file-driven, no radio. host_e2e byte-checks its output.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "debug_http.h"
#include "frame_pipeline.h"
#include "probe_source.h"
#include "real_actuator.h"

namespace mabur::maburd {

using namespace mabur;

namespace {


// ---------------------------------------------------------------------------
// Sinks
// ---------------------------------------------------------------------------

// Writes u32_le len | frame records to a file (dry-run --out).
struct FileSink : mabur::FrameSink {
  FILE* f = nullptr;

  bool send(const uint8_t* p, size_t n) override {
    if (!f) return false;
    uint8_t hdr[4] = {
        static_cast<uint8_t>(n & 0xFF),
        static_cast<uint8_t>((n >> 8) & 0xFF),
        static_cast<uint8_t>((n >> 16) & 0xFF),
        static_cast<uint8_t>((n >> 24) & 0xFF),
    };
    if (std::fwrite(hdr, 1, 4, f) != 4) return false;
    if (n > 0 && std::fwrite(p, 1, n, f) != n) return false;
    return true;
  }
};

// u32_le len | body — same record framing as FileSink, for --msp-out.
void write_len_prefixed(FILE* f, const uint8_t* p, size_t n) {
  uint8_t hdr[4] = {static_cast<uint8_t>(n & 0xFF),
                    static_cast<uint8_t>((n >> 8) & 0xFF),
                    static_cast<uint8_t>((n >> 16) & 0xFF),
                    static_cast<uint8_t>((n >> 24) & 0xFF)};
  std::fwrite(hdr, 1, 4, f);
  if (n > 0) std::fwrite(p, 1, n, f);
}

// ---------------------------------------------------------------------------
// Dry-run mode
// ---------------------------------------------------------------------------

struct RcInRecord {
  // Wire field (u32-LE in the --rc-in file): deliver this frame once that many
  // video frames have been consumed.
  uint32_t after_frame_index;
  std::vector<uint8_t> body;
};

// One record of a --dry-run frame file: the whole-frame records waybeam's
// frame-shm ring publishes, serialized as
//   u32-LE record length | VencFrameMeta (8 B) | Annex-B frame
// The buffer keeps the meta room up front exactly as FrameSource::read fills
// it, so FramePipeline can stamp the FrameHdr over it in place.
struct DryRunFrame {
  VencFrameMeta meta{};
  std::vector<uint8_t> buf;  // VENC_FRAME_META_SIZE + payload
  size_t payload_len() const { return buf.size() - VENC_FRAME_META_SIZE; }
};

std::vector<DryRunFrame> read_frame_file(const std::string& path) {
  std::vector<DryRunFrame> out;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return out;
  while (true) {
    uint8_t lenb[4];
    if (std::fread(lenb, 1, 4, f) != 4) break;
    const uint32_t len = static_cast<uint32_t>(lenb[0]) |
                         (static_cast<uint32_t>(lenb[1]) << 8) |
                         (static_cast<uint32_t>(lenb[2]) << 16) |
                         (static_cast<uint32_t>(lenb[3]) << 24);
    if (len < VENC_FRAME_META_SIZE) break;
    DryRunFrame fr;
    fr.buf.resize(len);
    if (std::fread(fr.buf.data(), 1, len, f) != len) break;
    std::memcpy(&fr.meta, fr.buf.data(), VENC_FRAME_META_SIZE);
    out.push_back(std::move(fr));
  }
  std::fclose(f);
  return out;
}

std::vector<uint8_t> read_whole_file(const std::string& path) {
  std::vector<uint8_t> v;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return v;
  uint8_t buf[4096];
  size_t r;
  while ((r = std::fread(buf, 1, sizeof buf, f)) > 0) v.insert(v.end(), buf, buf + r);
  std::fclose(f);
  return v;
}

std::vector<RcInRecord> read_rc_in(const std::string& path) {
  std::vector<RcInRecord> recs;
  if (path.empty()) return recs;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::fprintf(stderr, "warning: cannot open --rc-in '%s'\n", path.c_str());
    return recs;
  }
  while (true) {
    uint8_t hdr[8];
    if (std::fread(hdr, 1, 8, f) != 8) break;
    uint32_t after = static_cast<uint32_t>(hdr[0]) | (static_cast<uint32_t>(hdr[1]) << 8) |
                     (static_cast<uint32_t>(hdr[2]) << 16) | (static_cast<uint32_t>(hdr[3]) << 24);
    uint32_t len = static_cast<uint32_t>(hdr[4]) | (static_cast<uint32_t>(hdr[5]) << 8) |
                   (static_cast<uint32_t>(hdr[6]) << 16) | (static_cast<uint32_t>(hdr[7]) << 24);
    std::vector<uint8_t> body(len);
    if (len > 0 && std::fread(body.data(), 1, len, f) != len) break;
    recs.push_back(RcInRecord{after, std::move(body)});
  }
  std::fclose(f);
  return recs;
}

}  // namespace

int run_dry_run(const Config& cfg, uint8_t start_ch, const std::string& in_path,
                const std::string& out_path, const std::string& rc_in_path,
                const std::string& msp_in_path, const std::string& msp_out_path) {
  FileSink file_sink;
  file_sink.f = std::fopen(out_path.c_str(), "wb");
  if (!file_sink.f) {
    std::fprintf(stderr, "error: cannot open --out '%s'\n", out_path.c_str());
    return 1;
  }

  RadioTx tx(file_sink);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.sink = &file_sink;
  actuator.shared_op = &shared_op;
  actuator.dev = nullptr;
  actuator.dry_run = true;

  RcAgent agent(cfg, actuator, nullptr, start_ch);
  agent.install_session_for_replay(1, 1);  // --rc-in frames are tagged under (1,1) by tests/integration/mabur_rc.py
  // Debug endpoint is startable here too (no MABUR_HAVE_VENC on a host
  // build, so every route just answers "disabled") -- keeps host/dry-run
  // and real mode on one code path instead of special-casing it out.
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality);
  // Deterministic replay output: the async FEC worker is never attached in
  // dry-run mode (repair emission order would depend on thread timing).
  UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms);

  // Probe stream: same FEC geometry as the enh layer
  // (block_payload/bpb), so a probe body is the same wire size as a video
  // body at that rung.
  const auto probe_layer = cfg.uep_layers()[1];
  ProbeSource probe_src(probe_layer.blocks_per_body,
                        static_cast<int>(mabur::sw::kSwHeaderLen) + probe_layer.fec.symbol_size,
                        std::random_device{}());

  auto frames = read_frame_file(in_path);
  FramePipeline pipe;
  pipe.set_slice_geometry(
      venc_cfg_ctb64_rows(static_cast<uint16_t>(cfg.venc.core.height)),
      venc_cfg_slice_rows(static_cast<uint16_t>(cfg.venc.core.height), cfg.venc.core.slices));
  auto rc_recs = read_rc_in(rc_in_path);
  size_t rc_idx = 0;

  std::shared_ptr<const AppliedOp> last_applied_op;

  uint64_t sent_bodies = 0;
  uint64_t consumed_frames = 0;

  // First tick: BOOT -> RENDEZVOUS, applies MAX_RANGE op.
  agent.tick(now_steady_ms(), RadioHealth{});

  // Detects "a new AppliedOp was published" by shared_ptr IDENTITY, not by
  // generation: reapply_with_shed() (congestion) publishes a fresh AppliedOp
  // via apply_op() WITHOUT bumping generation (by design — generation
  // tracks new operating points, not shed adjustments to the current one),
  // but every apply_op() call, including reapplies, always stores a
  // brand-new shared_ptr<const AppliedOp>. Comparing against generation
  // alone would silently miss local congestion shed changes whenever no new
  // RCF/DISC/failsafe op happened in between — the exact bug this
  // identity-compare fixes.
  auto pump_op_change = [&]() {
    auto op = shared_op.load();
    if (op && op != last_applied_op) {
      apply_op_to_uep(*op, uep);
      // Dry-run deliberately does not call apply_op_to_clock: the air
      // clock is not priced here, so replayed bodies carry air_ms 0.
      last_applied_op = op;
    }
  };
  pump_op_change();

  // Drain semantics: deliver every RC record whose after_frame_index is
  // <= consumed_frames, in file order, regardless of whether that exact
  // count was ever hit as a distinct step. A strict `==` check (the
  // previous implementation) blocks forever on a record whose index is 0
  // (never equal to consumed_frames, which is incremented starting from
  // 1), duplicated, out-of-order, or >= frames.size() — and because rc_idx
  // only ever advances past a match, one stuck record wedges every
  // subsequent record too. Draining on `<=` delivers exact matches at the
  // same point as before (e.g. after_frame_index=3 still lands right
  // after the 3rd frame is consumed) while guaranteeing every record is
  // eventually delivered — before the loop for index 0, inline as the
  // frame count catches up, and at EOF for anything left over.
  auto drain_rc_records = [&](uint64_t now, bool drain_all = false) {
    uint64_t gate = drain_all ? UINT64_MAX : consumed_frames;
    while (rc_idx < rc_recs.size() && rc_recs[rc_idx].after_frame_index <= gate) {
      agent.on_rc_frame(rc_recs[rc_idx].body.data(), rc_recs[rc_idx].body.size(), now);
      std::fprintf(stderr, "[dry-run] rc-in delivered record %zu (after_frame_index=%u) at consumed_frames=%llu\n",
                  rc_idx, rc_recs[rc_idx].after_frame_index,
                  static_cast<unsigned long long>(consumed_frames));
      ++rc_idx;
    }
  };

  // Deliver any records due before the first frame (after_frame_index=0).
  drain_rc_records(now_steady_ms());

  for (size_t i = 0; i < frames.size(); ++i) {
    uint64_t now = now_steady_ms();

    pump_op_change();

    // Same ingest step as the real hot thread (classify, IDR protect-up,
    // FrameHdr stamp), so replayed bytes are the bytes the drone would send.
    auto bodies = pipe.encode(uep, frames[i].buf.data(), frames[i].payload_len(),
                              frames[i].meta, now);
    for (auto& b : bodies) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    // Probe stream (per AU): one body at the tail of every video AU's burst,
    // base or enh, sent right after the AU's own bodies so it lands at the
    // tail of the burst on the wire too. No AU on air (shed) => no probe
    // (probe_follows).
    if (!bodies.empty()) {
      const int au_sid = bodies.back().stream_id;
      auto op_now = shared_op.load();
      if (op_now && probe_follows(au_sid, op_now->probe_profile != rc::kNoProbeProfile,
                                  au_sid < UepEncoder::kNumStreams && op_now->shed[static_cast<size_t>(au_sid)])) {
        UepBody pb = probe_src.build(op_now->probe_profile,
                                     static_cast<uint16_t>(pipe.next_frame_id() - 1));
        tx.send_body(pb.stream_id, pb.body.data(), pb.body.size());
        ++sent_bodies;
      }
    }

    auto polled = uep.poll(now);
    for (auto& b : polled) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    ++consumed_frames;

    // Deliver any RC records due at or before this frame index, then tick
    // the agent (simulated radio health: empty/no drops).
    drain_rc_records(now);
    agent.tick(now, RadioHealth{});
    pump_op_change();
  }

  // EOF: deliver any records left over (duplicate/out-of-order/>=
  // frames.size() indices), flush every layer, send whatever falls out, then
  // stop.
  uint64_t now = now_steady_ms();
  drain_rc_records(now, true);
  auto flushed = uep.flush_all();
  for (auto& b : flushed) {
    tx.send_body(b.stream_id, b.body.data(), b.body.size());
    ++sent_bodies;
  }

  std::fclose(file_sink.f);

  if (!msp_in_path.empty() && !msp_out_path.empty()) {
    FILE* mf = std::fopen(msp_out_path.c_str(), "wb");
    if (mf) {
      MspSource msp(to_msp_source_cfg(cfg.msp),
                    [&](const uint8_t* body, size_t n){ write_len_prefixed(mf, body, n); });
      auto bytes = read_whole_file(msp_in_path);
      // Advance the clock past the rate-gate period per feed chunk so every
      // captured screen forwards (gate correctness is unit-tested separately).
      uint64_t clk = 0;
      for (size_t off = 0; off < bytes.size(); off += 64, clk += 10000)
        msp.on_serial_bytes(bytes.data() + off, std::min<size_t>(64, bytes.size() - off), clk);
      std::fclose(mf);
      std::fprintf(stderr, "[dry-run] msp: snapshots_sent=%llu\n",
                   static_cast<unsigned long long>(msp.snapshots_sent()));
    }
  }

  std::fprintf(stderr,
              "maburd dry-run stats: frames_in=%zu bodies_out=%llu seq=%u sent=%llu drops=%llu "
              "agent_state=%d rc_records=%zu/%zu idr_disagree=%llu enhance_disagree=%llu\n",
              frames.size(), static_cast<unsigned long long>(sent_bodies), tx.seq(),
              static_cast<unsigned long long>(tx.sent()), static_cast<unsigned long long>(tx.drops()),
              static_cast<int>(agent.state()), rc_idx, rc_recs.size(),
              static_cast<unsigned long long>(pipe.idr_disagreements()),
              static_cast<unsigned long long>(pipe.enhance_disagreements()));
  return 0;
}

}  // namespace mabur::maburd
