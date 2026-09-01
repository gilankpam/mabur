// maburd — drone-side daemon: reads whole encoded frames off the in-process
// venc core's frame-shm ring (or, in --dry-run, a fixture file), runs them
// through the UEP/FEC pipeline, and hands radio-bound bodies to the
// adaptive-link-controlled RadioTx. A parallel agent thread runs RcAgent against inbound RC
// frames + periodic radio-health ticks, publishing AppliedOp changes the hot
// path picks up via a lock-free shared_ptr handoff.
//
// Two modes:
//   maburd -c /etc/mabur.json                     — real mode (devourer USB radio)
//   maburd -c cfg.json --dry-run --in F --out F [--rc-in F]  — file-driven, no radio
//
// Dry-run is the tested path (see tests/fixtures/frame_stream.bin smoke test);
// real mode must compile and be structurally sound but is not exercised here
// (no bench hardware in this environment).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>  // _exit() — see the venc on_fault handler

#include "air_feed.h"
#include "config.h"
#include "debug_http.h"
#include "frame_pipeline.h"
#include "frame_source.h"
#include "mabur/frame_wire.h"
#include "mabur/msp_source.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/fec_worker.h"
#include "mabur/uep_encoder.h"
#include "msp_serial.h"
#include "power_plan.h"
#include "radio_tx.h"
#include "rc_agent.h"
#include "telemetry.h"
#include "tick_gate.h"
#include "tx_queue.h"
#include "usb_tx_pool.h"
#ifdef MABUR_HAVE_VENC
#include "venc_core.h"  // ARM only: drone/venc is not compiled on host builds
#endif

#if defined(MABUR_DRY_RUN_ONLY)
// Not used; real mode is always compiled in, guarded at runtime by --dry-run
// so the same binary runs (in dry-run) on a machine with no dongle attached.
#endif

#include "AmpduMode.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

#include <libusb.h>

namespace {

using namespace mabur;

// A GS at a different RC_VERSION is refused by rc::frame_type(), so its RCFs
// never reach the agent -- and, because the uplink RSSI/SNR EMAs are fed
// inside that same accepted-frame branch, drone.uplink.snr_* goes stale too.
// From the drone's side that is indistinguishable from "no GS is talking",
// and from the GS's side it looks like the stale-caps restart deadlock, which
// sends the operator to `restart maburd` -- which cannot help. So say it out
// loud, but rarely: a mismatched peer transmits continuously and /tmp is
// tmpfs, hence the once-per-5 s gate.
// Not thread-safe; only rx_callback (the RX thread) calls it.
void log_foreign_rc_version(uint8_t peer_ver) {
  using clock = std::chrono::steady_clock;
  static clock::time_point last{};
  const auto now = clock::now();
  if (last.time_since_epoch().count() != 0 &&
      now - last < std::chrono::seconds(5))
    return;
  last = now;
  std::fprintf(stderr,
               "maburd: heard an RC frame at RC_VERSION %u but this build "
               "speaks %u -- ignoring it (rate-limited to 1/5s). The pair is "
               "half-deployed: there is no control link and no video in "
               "either direction. Finish the deploy on BOTH ends; restarting "
               "maburd will not help.\n",
               static_cast<unsigned>(peer_ver),
               static_cast<unsigned>(rc::RC_VERSION));
}

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

MspSourceCfg to_msp_source_cfg(const MspCfg& m) {
  MspSourceCfg c;
  c.update_rate_hz = m.update_rate_hz;
  c.symbol_size = m.symbol_size;
  c.window = m.window;
  c.overhead = m.overhead;
  return c;
}

// Wraps IRtlDevice::send_packet with a mutex — shared between the hot
// thread (video bodies) and the agent thread (send_control / DISC_ACK).
struct DevourerSink : mabur::FrameSink {
  IRtlDevice* dev = nullptr;
  std::mutex m;
  // Opened true only after InitWrite() finishes device bring-up (power-on,
  // firmware download, TX-path enable). Until then every send is dropped:
  // pushing frames into the chip's bulk-OUT FIFO *during* bring-up fills it
  // with packets the not-yet-booted MAC cannot transmit, which then starves
  // devourer's own reserved-page firmware download (its bulk-OUT to the same
  // endpoint times out) — the FW never boots and TX is bricked for the whole
  // session. Bench-confirmed on the SSC338Q: devourer's `doctor` brings the
  // same dongle up HEALTHY in isolation, while maburd's concurrent hot/agent
  // sends made DLFW fail after ~3 frames.
  std::atomic<bool>* ready = nullptr;

  // Parallel USB sender pool (radio.tx_threads > 1): send_many submits
  // frames here and returns immediately; N pool threads each block in
  // their own sync bulk transfer, keeping ~N URBs in flight (the chip
  // flow-controls sync URB acceptance — one blocking sender caps air at
  // ~26 Mbps regardless of MCS). Null = direct synchronous path.
  mabur::UsbTxPool* pool = nullptr;

  bool send(const uint8_t* p, size_t n) override {
    if (ready && !ready->load(std::memory_order_acquire)) return false;
    std::lock_guard<std::mutex> l(m);
    return dev->send_packet(p, n);
  }

  // Batch path: devourer's send_packets packs consecutive frames into
  // shared bulk-OUT URBs when tx.usb_agg_max > 0 (one transfer per batch
  // instead of one per frame). Same ready-gate as send(). With a pool,
  // "accepted" means queued to the senders (frames are copied; real air
  // failures surface via GetTxStats + the pool's own counters).
  size_t send_many(const View* frames, size_t n) override {
    if (ready && !ready->load(std::memory_order_acquire)) return 0;
    if (pool) {
      size_t ok = 0;
      for (size_t i = 0; i < n; ++i)
        if (pool->submit(frames[i].data, frames[i].len)) ++ok;
      return ok;
    }
    std::vector<TxPacketView> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = {frames[i].data, frames[i].len};
    std::lock_guard<std::mutex> l(m);
    return dev->send_packets(v.data(), n);
  }
};

// ---------------------------------------------------------------------------
// RealActuator — bridges RcAgent to the radio/UEP/encoder world.
// ---------------------------------------------------------------------------

constexpr uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr size_t kDot11HeaderLen = 24;

std::vector<uint8_t> build_dot11_header(uint16_t seq) {
  std::vector<uint8_t> h(kDot11HeaderLen, 0);
  h[0] = 0x40;
  h[1] = 0x00;
  h[2] = 0x00;
  h[3] = 0x00;
  std::memset(h.data() + 4, 0xff, 6);
  std::memcpy(h.data() + 10, kCanonicalSa, 6);
  std::memcpy(h.data() + 16, kCanonicalSa, 6);
  uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  h[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  h[23] = static_cast<uint8_t>((seq_ctl >> 8) & 0xff);
  return h;
}

// devourer::TxMode for the MAX_RANGE control-channel rate (mirrors
// radio_tx.cpp's to_tx_mode helper — kept local since RadioTx doesn't expose
// its private conversion). DISC_ACK and any other control frame must fly at
// the same robustness as the MAX_RANGE data profile: MCS0/20MHz WITH
// LDPC+STBC. Flying control frames without them (the pre-fix behavior) was
// weaker than MAX_RANGE's own data profile despite control traffic needing
// to be at least as robust — a DISC_ACK lost at exactly the range where
// MAX_RANGE is needed defeats the whole point of the robust floor.
devourer::TxMode control_tx_mode() {
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.sgi = false;
  m.ldpc = true;
  m.stbc = true;
  return m;
}

// RealActuator bridges RcAgent's Actuator interface to the radio (RadioTx +
// FrameSink), the hot-thread-owned UepEncoder (via the shared_op handoff),
// and the in-process encoder (venc_core.h's verbs, called directly — the
// HTTP control plane and its WaybeamClient went away with the fold-in,
// spec 2026-08-28 venc-foldin).
//
// It deliberately has NO TX-power knob. Power is constant: bring-up programs
// the wall-equalized per-rate diff table and zeroes the global offset once
// (the `power_mode == "offset"` block in run_real_mode()), and nothing
// touches power again for the life of the process. Spec
// 2026-08-12-constant-txpower-design.md.
//
// Threading: apply_op()/send_control()/set_bitrate_kbps()/set_roi_qp()/
// request_idr() are all called from the agent thread only (RcAgent's
// contract). apply_op() publishes the new AppliedOp into shared_op via an
// atomic store of a fresh shared_ptr — the hot thread picks it up with an
// atomic load, so there is no lock and no torn read.
struct RealActuator : mabur::Actuator {
  mabur::RadioTx* tx = nullptr;
  mabur::FrameSink* sink = nullptr;
  std::atomic<std::shared_ptr<const mabur::AppliedOp>>* shared_op = nullptr;
  IRtlDevice* dev = nullptr;  // nullptr in dry-run
  bool dry_run = false;

  std::vector<uint8_t> control_radiotap;  // built once; control channel is fixed
  uint16_t control_seq = 0;

  // Last values commanded to the encoder — read by the telemetry collector
  // (agent thread only; RcAgent's contract calls these setters from the
  // agent thread exclusively, same thread the collector runs on, so plain
  // ints are safe with no lock).
  int last_bitrate_kbps = 0;
  int last_roi_qp = 0;
  // Lifetime count of encoder verbs the venc core refused. Replaces the
  // deleted waybeam_failures on the 1 Hz stats line: without it a drone
  // whose encoder silently rejects every set_bitrate looks identical from
  // the ground to one tracking the ladder perfectly. Same agent-thread-only
  // access as the two above, so a plain counter is safe.
  uint64_t venc_verb_failures = 0;

  void apply_op(const AppliedOp& op) override {
    tx->set_ladder(op.ladder);
    // Applying an op is a ladder + FEC + shed change and nothing else — see
    // the struct comment: there is no per-op power step to do in real mode.
    if (!dev && dry_run) {
      std::fprintf(stderr, "[dry-run] fec_ov_base=%.3f fec_ov_enh=%.3f gen=%llu\n",
                   op.fec_ov_base, op.fec_ov_enh,
                   static_cast<unsigned long long>(op.generation));
    }
    shared_op->store(std::make_shared<const AppliedOp>(op));
  }

  void send_control(const std::vector<uint8_t>& body) override {
    if (control_radiotap.empty()) {
      control_radiotap = devourer::build_stream_radiotap(control_tx_mode());
    }
    std::vector<uint8_t> frame;
    frame.reserve(control_radiotap.size() + kDot11HeaderLen + body.size());
    frame.insert(frame.end(), control_radiotap.begin(), control_radiotap.end());
    auto hdr = build_dot11_header(control_seq);
    control_seq = static_cast<uint16_t>((control_seq + 1) & 0xFFF);
    frame.insert(frame.end(), hdr.begin(), hdr.end());
    frame.insert(frame.end(), body.begin(), body.end());
    sink->send(frame.data(), frame.size());
  }

  // No retry loop lives here. A failed bitrate/ROI verb is reported UP (the
  // bool return): RcAgent declines to latch it and re-issues the same value
  // on its next policy tick, which is the only place that knows what the
  // current operating point should be. Retrying inside the actuator would
  // block the agent thread on a sick MI layer instead. A failed IDR is not
  // reported — nothing latches on it, and the next chain break or LINKED
  // re-entry raises another.
  //
  // last_bitrate_kbps/last_roi_qp record the last value ATTEMPTED, not the
  // last one accepted, because they exist to answer "what did the ladder
  // ask for" on the telemetry row; venc_verb_failures next to them is what
  // says whether the encoder is actually keeping up with those asks.
  //
  // On host builds (no MABUR_HAVE_VENC) both verbs report success without
  // doing anything: there is no encoder to diverge from, and reporting
  // failure would make RcAgent retry forever. Tests drive MockActuator.
  bool set_bitrate_kbps(int k) override {
    last_bitrate_kbps = k;
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_bitrate_kbps(%d)\n", k);
      return true;
    }
#ifdef MABUR_HAVE_VENC
    if (venc_set_bitrate_kbps(k) != 0) {
      ++venc_verb_failures;
      std::fprintf(stderr, "venc: set_bitrate(%d) FAILED (retry next tick)\n", k);
      return false;
    }
#endif
    return true;
  }

  bool set_roi_qp(int q) override {
    last_roi_qp = q;
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_roi_qp(%d)\n", q);
      return true;
    }
#ifdef MABUR_HAVE_VENC
    if (venc_set_roi_qp(q) != 0) {
      ++venc_verb_failures;
      std::fprintf(stderr, "venc: set_roi_qp(%d) FAILED (retry next tick)\n", q);
      return false;
    }
#endif
    return true;
  }

  void request_idr() override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] request_idr()\n");
      return;
    }
#ifdef MABUR_HAVE_VENC
    if (venc_request_idr() != 0) {
      ++venc_verb_failures;
      std::fprintf(stderr, "venc: request_idr FAILED\n");
    }
#endif
  }
};

uint64_t now_steady_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// µs sibling for the dq_split gauge: the intervals it separates (venc-ring
// wait 0–5 ms, FEC/SBI CPU, queue wait) are each of the same order as
// now_steady_ms()'s 1 ms quantum, so a ms clock cannot split them.
uint64_t now_steady_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Applies a freshly-published AppliedOp (detected via shared_ptr identity,
// not generation — see the callers' pump-loop comments) to the
// hot-thread-owned UepEncoder: the commanded overhead PAIR applied directly
// per layer (Task 6, RC_VERSION 5 — no ladder/scale translation) plus
// per-layer shed. Called from the hot thread only. The 2-slot AppliedOp's
// indices [0]/[1] map 1:1 onto the 2-stream UepEncoder's sids.
void apply_op_to_uep(const AppliedOp& op, UepEncoder& uep) {
  uep.set_layer_overhead(0, op.fec_ov_base);
  uep.set_layer_overhead(1, op.fec_ov_enh);
  for (int i = 0; i < 2; ++i) uep.set_shed(i, op.shed[static_cast<size_t>(i)]);
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

int run_dry_run(const Config& cfg, const std::string& in_path, const std::string& out_path,
                const std::string& rc_in_path, const std::string& msp_in_path,
                const std::string& msp_out_path) {
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

  RcAgent agent(cfg, actuator);
  // Debug endpoint is startable here too (no MABUR_HAVE_VENC on a host
  // build, so every route just answers "disabled") -- keeps host/dry-run
  // and real mode on one code path instead of special-casing it out.
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality);
  // Deterministic replay output: the async FEC worker is never attached in
  // dry-run mode (repair emission order would depend on thread timing).
  UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms);

  auto frames = read_frame_file(in_path);
  FramePipeline pipe;
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

// ---------------------------------------------------------------------------
// Real mode
// ---------------------------------------------------------------------------

std::atomic<bool> g_sigusr1_flag{false};

void handle_sigusr1(int) { g_sigusr1_flag.store(true); }

// SIGINT/SIGTERM shutdown: devourer's IRtlDevice::Init() runs a blocking RX
// loop that only observes the library-global g_devourer_should_stop (see
// ../devourer/src/SignalStop.h and how examples/rx, examples/tx, and
// examples/doctor all call install_devourer_signal_handlers() instead of
// installing their own SIGINT/SIGTERM handlers). A locally-scoped flag set by
// a handler main.cpp installs itself is never consulted by Init()'s loop, so
// Ctrl-C/SIGTERM would never unblock it. The hot/agent thread loops below
// read g_devourer_should_stop directly too, so all three (Init()'s RX loop,
// hot_thread, agent_thread) stop on the same signal.

// Small mutex-guarded deque standing in for an SPSC queue — RC control
// traffic arrives at ~10 Hz, far below anything a mutex can't absorb.
struct RcQueue {
  std::mutex m;
  std::deque<std::vector<uint8_t>> q;

  void push(const uint8_t* p, size_t n) {
    std::lock_guard<std::mutex> l(m);
    q.emplace_back(p, p + n);
  }
  bool pop(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> l(m);
    if (q.empty()) return false;
    out = std::move(q.front());
    q.pop_front();
    return true;
  }
};

uint16_t open_usb_and_get_pid(uint16_t vid, uint16_t configured_pid,
                              libusb_context* ctx, libusb_device_handle** out_handle) {
  std::vector<uint16_t> pids;
  if (configured_pid != 0) {
    pids.push_back(configured_pid);
  } else {
    pids = {0xa81a, 0x881a, 0x8812};
  }
  for (uint16_t pid : pids) {
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (h) {
      *out_handle = h;
      return pid;
    }
  }
  *out_handle = nullptr;
  return 0;
}

int run_real_mode(const Config& cfg) {
  std::signal(SIGUSR1, handle_sigusr1);
  // Installs the SIGINT/SIGTERM handlers that set g_devourer_should_stop —
  // the flag IRtlDevice::Init()'s blocking RX loop actually watches.
  install_devourer_signal_handlers();

  auto logger = std::make_shared<Logger>();
  // devourer has two independent output channels and this daemon wants both
  // quiet. set_level() gates only the human diagnostics (logger->info/warn/…);
  // the JSON event stream is gated solely by EventSink::enabled(), which
  // defaults to stdout + enabled + flush-per-line. S96mabur redirects stdout
  // into the RAM-backed /tmp/mabur.log, so jaguar3's per-URB "tx.agg" event
  // (~600/s at video rate) wrote ~1.5 MB/min and filled the drone's 45 MB
  // /tmp in ~30 min — after which every log write failed silently. Nothing is
  // lost by muting the stream: our stats line already carries tx_failed=.
  logger->set_level(Logger::Level::Warn);
  logger->events().disable();

  libusb_context* usb_ctx = nullptr;
  int rc = libusb_init(&usb_ctx);
  if (rc < 0) {
    std::fprintf(stderr, "error: libusb_init failed (%d)\n", rc);
    return 1;
  }

  libusb_device_handle* handle = nullptr;
  uint16_t pid = open_usb_and_get_pid(cfg.radio.usb_vid, cfg.radio.usb_pid, usb_ctx, &handle);
  if (!handle) {
    std::fprintf(stderr, "error: no radio found under VID 0x%04x\n", cfg.radio.usb_vid);
    libusb_exit(usb_ctx);
    return 1;
  }
  std::fprintf(stderr, "opened device %04x:%04x\n", cfg.radio.usb_vid, pid);

  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_then_reset(handle, 0, logger, /*do_reset=*/true, usb_lock);
  if (rc != 0) {
    std::fprintf(stderr, "error: claim_interface_then_reset failed (%d)\n", rc);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }

  // Jaguar3 TX+RX on one claimed handle: enable_with_tx makes InitWrite keep
  // the RX filters open so a later StartRxLoop can run concurrently with TX
  // (mirrors devourer's doctor/streamtx examples). Retrofitting RX after a
  // plain InitWrite is unreliable on this chip.
  devourer::DeviceConfig dev_cfg;
  dev_cfg.rx.enable_with_tx = true;
  // USB TX aggregation: pack up to 3 frames (the HalMAC per-transfer
  // descriptor limit) into one bulk-OUT URB via send_packets — amortizes
  // the per-URB tax that capped inline per-frame injection at ~2500 fps.
  dev_cfg.tx.usb_agg_max = 3;
  // MAC carrier sense OFF. The FPV downlink owns its channel, so CSMA backoff
  // only stutters it: devourer measured injection deferring 41-45% to a
  // co-channel 802.11 transmitter on this same Jaguar3 family, recovered ~1.5x
  // by clearing primary CCA 0x520[14] (tests/dis_cca_tx_onair.sh). The frames
  // are late, not lost, so the cost lands as TxQueue backpressure and aborted
  // slice tails that the loss-driven ladder cannot see. This is the MAC TX gate
  // only -- SetCcaMode deliberately skips the vendor BB CCA-off writes, which
  // deafen the receiver (measured: delivery 6800 -> 10 frames). Deliberate
  // side effect: the same flag latches _cca_disabled, which suppresses
  // phydm's periodic EDCCA re-tracking (RtlJaguar3Device.cpp) so it stops
  // fighting the disable by rewriting the 0x84c energy-detect thresholds
  // every ~2 s.
  dev_cfg.tuning.disable_cca = true;

  WiFiDriver wifi_driver{logger};
  auto rtl_device = wifi_driver.CreateRtlDevice(handle, usb_ctx, usb_lock, dev_cfg);
  if (!rtl_device) {
    std::fprintf(stderr, "error: CreateRtlDevice failed (unsupported chip or already in use)\n");
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }

  // Gate for DevourerSink: stays false until InitWrite() completes bring-up.
  std::atomic<bool> device_ready{false};

  DevourerSink dev_sink;
  dev_sink.dev = rtl_device.get();
  dev_sink.ready = &device_ready;

  // Capacity 6 frames per sender: enough to keep every sender's next ≤3-
  // frame URB staged while it blocks in the current one, small enough that
  // backlog still lands in TxQueue (whose drop-oldest policy is the
  // FEC-recoverable erasure path).
  mabur::UsbTxPool tx_pool(
      [dev = rtl_device.get()](const std::vector<std::vector<uint8_t>>& b) {
        std::vector<TxPacketView> v(b.size());
        for (size_t i = 0; i < b.size(); ++i) v[i] = {b[i].data(), b[i].size()};
        return dev->send_packets(v.data(), v.size());
      },
      cfg.radio.tx_threads,
      static_cast<size_t>(cfg.radio.tx_threads) * 6);
  if (cfg.radio.tx_threads > 1) dev_sink.pool = &tx_pool;

  RadioTx tx(dev_sink);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  // Cross-thread feed from the hot thread's AirFeed to the agent thread
  // (bitrate policy input + telemetry snapshot) — see AirFeedOut's doc
  // comment in rc_agent.h. Must outlive both hot_thread and agent_thread,
  // so it lives in this outer scope beside shared_op, not inside either
  // thread's lambda.
  AirFeedOut air_feed_out;

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.sink = &dev_sink;
  actuator.shared_op = &shared_op;
  actuator.dev = rtl_device.get();
  actuator.dry_run = false;
  // Encoder starts at the "normal" ROI QP (RcAgent only calls set_roi_qp on
  // a low<->normal transition — see run_bitrate_policy's roi_low_ default),
  // so the telemetry collector needs this seeded to reflect what's actually
  // commanded before the first transition ever happens.
  actuator.last_roi_qp = cfg.encoder.roi_qp_normal;

  RcAgent agent(cfg, actuator, &air_feed_out);

#ifdef MABUR_HAVE_VENC
  // Boot the encoder BEFORE the radio and before any thread starts: it
  // creates the frame-shm ring the hot thread reads, and RcAgent's very
  // first tick (BOOT -> MAX_RANGE) commands a bitrate through the verbs
  // above, which no-op until the core is up. `agent` is constructed on the
  // line above and outlives every venc thread (venc_core_stop() joins them
  // before this scope ends), so handing its address to the callbacks is
  // safe without a file-scope indirection.
  VencCallbacks vcb{};
  vcb.on_chain_break = [](void* u) {
    static_cast<RcAgent*>(u)->note_chain_break();  // atomic set; acted on at tick
  };
  // Fault policy: log and _exit(3) for the wrapper to respawn. Deliberately
  // NOT venc_core_stop() — this runs ON the encoder thread and stop() joins
  // that same thread (venc_core.c's contract), so calling it here would
  // self-deadlock the very failure it is meant to escape.
  //
  // _exit(), not std::exit(): this fires on the ENCODER thread of a live
  // multi-threaded process whose other threads (agent, hot TX, debug HTTP,
  // MSP) keep running through the teardown. std::exit() runs atexit
  // handlers and static destructors on that thread while the rest of the
  // process still touches the same objects — a hang there (a destructor
  // blocking on a lock another thread holds, or on the same MI call that
  // just faulted) leaves a wedged, video-less maburd that the wrapper never
  // gets to respawn, which is the exact outcome this policy exists to
  // avoid. stderr is unbuffered, and the explicit fflush covers the case
  // where something upstream has set a buffer on it, so skipping _exit()'s
  // omitted flush-at-exit costs no diagnostics.
  vcb.on_fault = [](void*, const char* what) {
    std::fprintf(stderr, "venc FATAL: %s — exiting for wrapper respawn\n", what);
    std::fflush(stderr);
    _exit(3);
  };
  vcb.user = &agent;
  if (venc_core_start(&cfg.venc.core, &vcb) != 0) {
    // Boot failure, not a transient: the wrapper's 2 s respawn is the retry.
    // Release the USB device on the way out (same shape as the
    // CreateRtlDevice failure path above) — the radio is not up yet, so
    // there is nothing else to unwind.
    std::fprintf(stderr, "venc_core_start failed — exiting\n");
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 3;
  }
#endif
  // After venc_core_start: RcAgent's first tick (below) already commands a
  // bitrate through the verbs, so the ring/stats the debug endpoint reads
  // are live from here on. localhost-only, always on -- bind failure logs
  // and disables itself, never fatal (see debug_http.h).
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality,
                   &air_feed_out);

  RcQueue rc_queue;
  std::atomic<uint64_t> rx_beat{0};
  std::atomic<uint64_t> hot_beat{0};

  // Uplink RSSI/SNR EMAs, fed from rx_callback (RX thread) on CRC-clean RC
  // frames, read by the agent thread's 1 Hz telemetry collector (spec
  // 2026-07-26 drone-telemetry). Thread-safe per UplinkTrack's own mutex.
  UplinkTrack uplink_track;

  // Cumulative encoder/ring counters (spec 2026-07-26 drone-telemetry):
  // written by the hot thread, read by the agent thread's telemetry
  // collector. FramePipeline/FrameSource don't track these themselves (see
  // frame_ring stats block below), so maburd tracks them here. Two
  // different patterns live in this group: enc_frames/enc_bytes/ring_drops
  // are computed right here (fetch_add) because nothing else tracks them,
  // while idr_disagree_total/enhance_disagree_total are relaxed-published
  // MIRRORS (store, not fetch_add) of counters FramePipeline already owns
  // and updates on the hot thread — see pipe.idr_disagreements() below.
  std::atomic<uint64_t> enc_frames_total{0};
  std::atomic<uint64_t> enc_bytes_total{0};
  std::atomic<uint64_t> idr_disagree_total{0};
  std::atomic<uint64_t> enhance_disagree_total{0};
  std::atomic<uint64_t> ring_drops_total{0};
  // venc-ring vanish detection (docs/venc-ring-vanish-findings-2026-08-12.md):
  // relaxed-published mirrors of FramePipeline's counters (the
  // idr_disagree_total pattern). Detection-only port of 65c94fd: the
  // pipeline's self-IDR latch level is deliberately NOT consumed here — the
  // self-IDR mechanism needs the redesign queued in that doc (kill switch,
  // GOP-aware suppression, rate-based guard) before it returns.
  std::atomic<uint64_t> vanished_base_total{0};
  std::atomic<uint64_t> vanished_enh_total{0};
  std::atomic<uint64_t> self_idr_refused_total{0};
  // TxQueue wait window max (spec 2026-08-30 latency-accounting, Task 4):
  // tx thread publishes the largest push→pop delay it saw since the last
  // 1 Hz telemetry read; the agent thread's collector (Task 5) exchanges it
  // back to 0 so each tick reports its own window, not a running max.
  std::atomic<uint32_t> txq_wait_max_ms{0};
  // Agent thread -> hot thread: link came up from BOOT/RENDEZVOUS, so every
  // frame encoded so far died before the air — re-mark the discontinuity
  // window so the GS gets the re-base signal on frames that can actually
  // land (docs/gs-frame-stall-after-drone-restart-handoff.md).
  std::atomic<bool> link_up_discont{false};

  // RX callback: pulls RC frames (rc::frame_type >= 0) off the air and
  // queues them for the agent thread. Runs on the main thread (inside
  // rtl_device->Init's blocking RX loop).
  auto rx_callback = [&](const Packet& pkt) {
    rx_beat.fetch_add(1, std::memory_order_relaxed);
    if (pkt.Data.size() < kDot11HeaderLen + 4) return;
    const uint8_t* body = pkt.Data.data() + kDot11HeaderLen;
    size_t body_len = pkt.Data.size() - kDot11HeaderLen;
    if (rc::frame_type(body, body_len) >= 0) {
      rc_queue.push(body, body_len);
      // Uplink EMAs feed off CRC-clean RC frames only — a corrupt frame's
      // attrib (rssi/snr) is not a trustworthy sample.
      if (!pkt.RxAtrib.crc_err)
        uplink_track.on_rc_frame(pkt.RxAtrib.rssi, pkt.RxAtrib.snr);
    } else if (!pkt.RxAtrib.crc_err &&
               rc::is_foreign_rc_version(body, body_len)) {
      // Same crc gate as the EMAs, and for the same class of reason:
      // RC_MAGIC is two bytes, so ~1 in 65536 corrupt bodies matches it by
      // chance and must not print a version-mismatch scare. Log only —
      // the frame is still dropped exactly as it was before.
      log_foreign_rc_version(body[2]);
    }
  };

  std::thread msp_thread;
  if (cfg.msp.enable) {
    msp_thread = std::thread([&]() {
      // Robust control modulation, same tier as DISC_ACK; MSP is a third
      // producer on the mutex-guarded dev_sink.send() path (never the pool).
      std::vector<uint8_t> radiotap = devourer::build_stream_radiotap(control_tx_mode());
      uint16_t seq = 0;
      std::random_device rd;
      MspSource src(to_msp_source_cfg(cfg.msp),
        [&](const uint8_t* body, size_t n) {
          std::vector<uint8_t> frame;
          frame.reserve(radiotap.size() + kDot11HeaderLen + n);
          frame.insert(frame.end(), radiotap.begin(), radiotap.end());
          auto hdr = build_dot11_header(seq);
          seq = static_cast<uint16_t>((seq + 1) & 0xFFF);
          frame.insert(frame.end(), hdr.begin(), hdr.end());
          frame.insert(frame.end(), body, body + n);
          dev_sink.send(frame.data(), frame.size());
        },
        rd());  // random initial_seq (SwEncoder restart-safety contract)
      std::fprintf(stderr,
          "maburd msp: enabled symbol_size=%d window=%d block_payload=%d update_rate_hz=%.2g serial=%s baud=%d\n",
          cfg.msp.symbol_size, cfg.msp.window,
          cfg.msp.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen),
          cfg.msp.update_rate_hz, cfg.msp.serial.c_str(), cfg.msp.baud);
      MspSerial serial;
      uint8_t buf[512];
      while (!g_devourer_should_stop) {
        if (!serial.is_open()) {
          if (!serial.open(cfg.msp.serial, cfg.msp.baud)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
          }
          std::fprintf(stderr, "maburd msp: reading %s @ %d\n",
                       cfg.msp.serial.c_str(), cfg.msp.baud);
        }
        int n = serial.read(buf, sizeof buf);
        if (n > 0) src.on_serial_bytes(buf, static_cast<size_t>(n), now_steady_ms());
        else if (n < 0) serial.close();  // error -> reconnect
      }
    });
  }

  // TX body queue between the encode (hot) thread and the USB writer
  // thread: a bulk-OUT stall backs up HERE (bounded, drop-oldest =
  // FEC-recoverable erasures) instead of in the waybeam SHM ring, whose
  // overflow makes the RTP packetizer abort NALs mid-chain — sender-side
  // slice truncation no FEC can repair (bench 2026-07-13, the PixelPilot
  // glitch root cause). ~256 bodies ≈ 150 ms at 1700 bodies/s.
  constexpr size_t kTxQueueCap = 256;  // also feeds Telem.txq_cap
  TxQueue txq(kTxQueueCap);

  // Hot thread: pulls whole frames off the real SHM ring, runs them through
  // the UEP pipeline, queues bodies for the TX writer. Owns the UepEncoder
  // exclusively; never blocks on USB.
  std::thread hot_thread([&]() {
    // Ring name's single authority is now the compile-time VENC_RING_NAME
    // (drone/venc/venc_cfg.h "/mabur_f"); frame_ring_name config key
    // deleted (spec 2026-08-28 venc-foldin, Task B5 controller ruling).
    // venc_frame_ring_attach() normalises the leading '/' itself, so this
    // is behaviourally identical to the old default "mabur_f".
    FrameSource fsrc(VENC_RING_NAME);
    FramePipeline pipe;
    std::vector<uint8_t> fbuf(VENC_FRAME_META_SIZE + 512 * 1024);
    uint64_t last_reattach = 0;
    uint64_t last_ring_stats_ms = 0;
    bool vanish_boot_zeroed = false;  // first link-establish zeroes counters

    // Async FEC worker (spec 2026-07-17, promoted after hardware
    // acceptance): always on, unpinned (the worker sleeps when idle, so the
    // scheduler places it correctly). Declared before the UepEncoder so
    // engine dtors join their jobs first.
    FecWorker fec_worker;
    UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms, &fec_worker);
    // Hot-thread-owned, same exclusivity contract as uep above: writes the
    // shared air_feed_out, never read back by this thread.
    AirFeed feed(&air_feed_out);

    std::shared_ptr<const AppliedOp> last_applied_op;
    // Debug-HTTP per-layer overhead override transition tracking (fix
    // round 1, review finding): true while the last tick saw both
    // ovr_base_pct/ovr_enh_pct armed. Needed because clearing the override
    // has NO bounded re-assert to fall back on -- run_bitrate_policy's 5 s
    // reassert (kReassertMs) only re-sends the bitrate/roi_qp verbs, never
    // touches the UEP layers' overhead, so without this the encoder would
    // stay pinned at the last override value indefinitely (until the next
    // genuine op change) while AirFeed's on_frame fallback snapped back to
    // the stale op-pair anchor immediately -- exactly the encoder/anchor
    // disagreement the finding flagged.
    bool ov_override_was_armed = false;

    // dq_split gauge (dq-spike follow-up 2026-08-31): per-frame split of the
    // interval the wire's q_ms currently lumps together — venc-ring wait
    // (loop-top → read return) and FEC/SBI-pack CPU (read return → push).
    // Hot-thread-owned, reported on the 5 s ring-stats cadence.
    uint64_t split_n = 0;
    uint64_t split_ring_sum_us = 0, split_ring_max_us = 0;
    uint64_t split_cpu_sum_us = 0, split_cpu_max_us = 0;

    // fec_worker gauge (fec-compute handover 2026-09-01): per-layer split of
    // dq_split's cpu_us into wait-on-worker (join spins) vs the hot thread's
    // own work, plus the worker's per-repair build cost and queue depth.
    // take_fec_gauge's sums are cumulative — keep last-window copies and
    // diff here; the maxima reset inside the take.
    SwEncoder::SwFecGauge fec_gauge_prev[UepEncoder::kNumStreams]{};

    while (!g_devourer_should_stop) {
      uint64_t now = now_steady_ms();
      const uint64_t t0_us = now_steady_us();

      // Identity-compare, not generation-compare: see the matching comment
      // in run_dry_run's pump_op_change lambda above. reapply_with_shed()
      // (congestion) never bumps generation, so gating on generation here
      // would silently drop local shed changes that arrive between two
      // RCF-driven generation bumps — including the congestion-triggered
      // shed this loop is specifically responsible for applying to the
      // UepEncoder (op.shed) independent of the GS.
      auto op = shared_op.load();
      if (op && op != last_applied_op) {
        apply_op_to_uep(*op, uep);
        // Anchors AirFeed's excess_base/enh + telemetry-only ov_base/enh
        // against the pair actually applied above (Task 7: the fixed
        // per-rung pair, no solver left to re-derive it).
        feed.set_applied(op->fec_ov_base, op->fec_ov_enh);
        last_applied_op = op;
      }

      // Debug-HTTP per-layer overhead override (bench sweeps, :8301 POST
      // /venc/set?ov_base_pct=N&ov_enh_pct=N): armed (both >= 0) wins over
      // the op pair on the UEP layers every tick. Checked BEFORE this
      // tick's frame is read/encoded (not after, and not gated on a frame
      // having arrived) so that by the time pipe.encode() and feed.on_frame()
      // run below, the UEP layers and AirFeed's anchor already agree on
      // whichever value is in effect this tick -- an armed->cleared
      // transition re-applies the op pair to BOTH in one shot, exactly
      // once (not per-frame): the encoder stops flying the stale override
      // and AirFeed's excess_*/ov_* stop misreporting against it in the
      // same tick, instead of a one-tick-later correction.
      if (op) {
        const int ob = feed.out().ovr_base_pct.load(std::memory_order_relaxed);
        const int oe = feed.out().ovr_enh_pct.load(std::memory_order_relaxed);
        const bool armed = ob >= 0 && oe >= 0;
        if (armed) {
          uep.set_layer_overhead(0, ob / 100.0);
          uep.set_layer_overhead(1, oe / 100.0);
          ov_override_was_armed = true;
        } else if (ov_override_was_armed) {
          apply_op_to_uep(*op, uep);
          feed.set_applied(op->fec_ov_base, op->fec_ov_enh);
          ov_override_was_armed = false;
        }
      }

      // One whole Annex-B frame per iteration: a frame is already the atomic
      // thing waybeam's producer publishes, so there's no "drain more" knob
      // here; the ring's own depth (default a handful of slots) is the backlog
      // buffer.
      VencFrameMeta meta{};
      int n = fsrc.read(fbuf.data(), fbuf.size(), 5, &meta);
      const uint64_t t_read_us = now_steady_us();
      if (n > 0) {
        if (fsrc.reattach_count() != last_reattach) {
          last_reattach = fsrc.reattach_count();
          pipe.mark_discontinuity();  // joined a new ring mid-GOP
        }
        if (link_up_discont.exchange(false, std::memory_order_relaxed)) {
          pipe.mark_discontinuity();  // link just came up: pre-link frames died
          // FIRST establish only: drop the boot-window vanish counts
          // (encoder bring-up churn, ~8-9/boot — 2026-08-13 flight finding)
          // so telemetry reports in-flight vanishes. A mid-flight
          // re-establish must NOT erase in-flight counts.
          if (!vanish_boot_zeroed) {
            vanish_boot_zeroed = true;
            pipe.reset_vanish_counters();
          }
        }
        // Streaming push (dq-spike follow-up 2026-08-31): each body goes to
        // the TxQueue the moment its SBI group seals, so the radio drains
        // this frame's early bodies in parallel with the remaining GF256/SBI
        // packing — the old accumulate-then-push shape serialized ~3.4 ms of
        // that CPU in front of an idle radio (au_first queue wait measured
        // ~40 µs). enqueued_ms/pushed_us are per-body actual-push stamps
        // now, so the wire q_ms is the true TxQueue wait, and the enc_us
        // patch rides inside the sink (last_enc_us() latches before the
        // first sink call — see FramePipeline::encode's sink contract; the
        // SBI header sits outside the FEC envelope, so post-pack patching
        // stays CRC-safe).
        size_t emitted = 0;
        int emit_sid = -1;
        bool first = true;
        pipe.encode(uep, fbuf.data(), static_cast<size_t>(n), meta, now,
                    [&](UepBody&& b) {
                      mabur::sbi_set_enc_us(b.body.data(), b.body.size(),
                                            pipe.last_enc_us());
                      const uint64_t p_us = now_steady_us();
                      b.enqueued_ms = static_cast<uint32_t>(p_us / 1000);
                      b.pushed_us = p_us;
                      b.au_first = first;
                      first = false;
                      emitted += b.body.size();
                      emit_sid = b.stream_id;
                      txq.push(std::move(b));
                    });
        // dq_split accounting: ring wait is loop-top → read return (the
        // interval during which this frame did not yet exist for us), CPU is
        // read return → all bodies pushed (fragmentation + GF256 + SBI pack,
        // now overlapped with the radio drain rather than in front of it).
        if (!first) {
          const uint64_t t_done_us = now_steady_us();
          const uint64_t ring_us = t_read_us - t0_us;
          const uint64_t cpu_us = t_done_us - t_read_us;
          ++split_n;
          split_ring_sum_us += ring_us;
          if (ring_us > split_ring_max_us) split_ring_max_us = ring_us;
          split_cpu_sum_us += cpu_us;
          if (cpu_us > split_cpu_max_us) split_cpu_max_us = cpu_us;
        }
        // AirFeed (spec §2, Task 7 — the solver that used to redistribute
        // overhead here is gone; the per-rung pair is fixed and applied
        // directly by apply_op_to_uep above): feed actual emitted bytes,
        // excluding IDR outliers, for the EWMAs run_bitrate_policy's blend
        // consumes plus the excess_base/enh + ov_base/enh telemetry.
        if (op && !(meta.flags & VENC_FRAME_FLAG_IDR) && emit_sid >= 0)
          feed.on_frame(emit_sid, static_cast<size_t>(n), emitted);
        enc_frames_total.fetch_add(1, std::memory_order_relaxed);
        enc_bytes_total.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
        idr_disagree_total.store(pipe.idr_disagreements(),
                                 std::memory_order_relaxed);
        enhance_disagree_total.store(pipe.enhance_disagreements(),
                                     std::memory_order_relaxed);
        vanished_base_total.store(pipe.vanished_base(), std::memory_order_relaxed);
        vanished_enh_total.store(pipe.vanished_enhance(), std::memory_order_relaxed);
        self_idr_refused_total.store(pipe.self_idr_refused(),
                                     std::memory_order_relaxed);
      }
      // Ring-pressure observability (spec: the drain-feedback policy's
      // future input): one stderr line every 5 s.
      //
      // Only counters this process's CONSUMER handle can actually move.
      // venc_frame_ring_fill_t snapshots the local handle, and
      // writes/full_drops are incremented solely by the write path — they
      // are structurally 0 here, and the shm header carries
      // write_idx/read_idx but no counters, so the producer handle's copies
      // cannot cross into this one. fill_pct is the ring-wide signal
      // (write_idx - read_idx). Since the venc fold-in the producer is a
      // thread of THIS process, so its drop count is no longer unreachable:
      // it comes from venc_get_stats() (VencStats.full_drops), which reads
      // the encoder's own handle — see the T_TELEM collector below, which
      // publishes it as Telem.venc_full_drops. This line stays
      // consumer-side-only on purpose, so the two provenances never blur.
      // See tests/test_frame_source.cpp
      // (consumer_fill_reports_only_consumer_side_counters).
      if (now - last_ring_stats_ms >= 5000) {
        last_ring_stats_ms = now;
        venc_frame_ring_fill_t f{};
        if (fsrc.fill(&f)) {
          // Telem.ring_drops (spec 2026-07-26 drone-telemetry): the two
          // counters this process can actually move, per the comment above.
          ring_drops_total.store(
              static_cast<uint64_t>(f.oversize_drops) + static_cast<uint64_t>(f.bad_slot_drops),
              std::memory_order_relaxed);
          std::fprintf(stderr,
              "maburd frame_ring: fill=%u%% (%u/%u) reads=%llu oversize=%llu "
              "bad_slot=%llu idr_disagree=%llu enhance_disagree=%llu "
              "vanished=%llu/%llu self_idr_refused=%llu\n",
              f.fill_pct, f.used_slots, f.slot_count,
              (unsigned long long)f.reads,
              (unsigned long long)f.oversize_drops,
              (unsigned long long)f.bad_slot_drops,
              (unsigned long long)pipe.idr_disagreements(),
              (unsigned long long)pipe.enhance_disagreements(),
              (unsigned long long)pipe.vanished_base(),
              (unsigned long long)pipe.vanished_enhance(),
              (unsigned long long)pipe.self_idr_refused());
        }
        // dq_split window report (dq-spike follow-up): the pre-push half of
        // the interval the wire q_ms spans. The post-push half (true queue
        // wait) is the tx thread's dq_queue line on the same cadence.
        if (split_n > 0) {
          std::fprintf(stderr,
              "maburd dq_split: n=%llu ring_us mean=%llu max=%llu "
              "cpu_us mean=%llu max=%llu\n",
              (unsigned long long)split_n,
              (unsigned long long)(split_ring_sum_us / split_n),
              (unsigned long long)split_ring_max_us,
              (unsigned long long)(split_cpu_sum_us / split_n),
              (unsigned long long)split_cpu_max_us);
        }
        split_n = 0;
        split_ring_sum_us = split_ring_max_us = 0;
        split_cpu_sum_us = split_cpu_max_us = 0;
        for (int sid = 0; sid < UepEncoder::kNumStreams; ++sid) {
          const auto g = uep.take_fec_gauge(sid);
          const auto& p = fec_gauge_prev[sid];
          const uint64_t jobs = g.jobs - p.jobs;
          const uint64_t build_us = g.build_us - p.build_us;
          const uint64_t joins = g.join_waits - p.join_waits;
          const uint64_t wait_us = g.join_wait_us - p.join_wait_us;
          if (jobs > 0 || joins > 0) {
            std::fprintf(stderr,
                "maburd fec_worker sid=%d: jobs=%llu build_us/job=%llu "
                "inline=%llu joins=%llu join_wait_us mean=%llu max=%llu "
                "qdepth_max=%llu\n",
                sid,
                (unsigned long long)jobs,
                (unsigned long long)(jobs ? build_us / jobs : 0),
                (unsigned long long)(g.inline_full - p.inline_full),
                (unsigned long long)joins,
                (unsigned long long)(joins ? wait_us / joins : 0),
                (unsigned long long)g.join_wait_max_us,
                (unsigned long long)g.enq_depth_max);
          }
          fec_gauge_prev[sid] = g;
        }
      }

      // Idle-tail flush: no single source frame owns these bodies (poll can
      // combine leftovers across several frames' worth of idle time), so
      // they carry no patched enc_us — left at the packer's zero placeholder
      // (0 = unknown on the wire, per spec).
      auto polled = uep.poll(now);
      if (!polled.empty()) {
        // Fresh sample, not t0_us: by here the loop top is up to a full
        // read-timeout + encode stale, and these bodies never waited on it.
        const uint64_t t_poll_us = now_steady_us();
        for (auto& b : polled) {
          // Idle-tail bodies carry no per-frame enc_us, but the queue wait is
          // still real once they reach the tx thread — stamp it here too.
          b.enqueued_ms = static_cast<uint32_t>(t_poll_us / 1000);
          b.pushed_us = t_poll_us;
          txq.push(std::move(b));
        }
      }

      hot_beat.fetch_add(1, std::memory_order_relaxed);
    }
    txq.close();
  });

  // TX writer thread: sole caller of tx.send_bodies (RadioTx's
  // single-thread contract). Batches up to 3 bodies per call — devourer's
  // Jaguar3 send_packets packs them into one bulk-OUT URB (HalMAC parses at
  // most 3 descriptors per transfer), amortizing the per-URB tax that
  // capped the old inline path at ~2500 fps.
  std::thread tx_thread([&]() {
    std::vector<UepBody> batch;
    // dq_queue gauge (dq-spike follow-up 2026-08-31): TRUE push→pop queue
    // wait from the pre-push pushed_us stamp, per body and for the AU-first
    // body alone (the one whose q_ms the GS latches as dq). Thread-owned,
    // reported every 5 s.
    uint64_t qw_n = 0, qw_sum_us = 0, qw_max_us = 0;
    uint64_t qf_n = 0, qf_sum_us = 0, qf_max_us = 0;
    // tx_send gauge (burst-drain follow-up 2026-08-31): wall time of each
    // tx.send_bodies call and its body count, to split the measured
    // ~0.385 ms/body burst pace into USB round-trip vs airtime. A blocking
    // ~1.1 ms per 3-body batch here = the URB round-trip IS the pace.
    uint64_t sb_calls = 0, sb_bodies = 0, sb_sum_us = 0, sb_max_us = 0;
    uint32_t last_qw_report_ms = static_cast<uint32_t>(now_steady_ms());
    while (!g_devourer_should_stop) {
      batch.clear();
      if (txq.pop_batch(batch, 3, 5) == 0) continue;
      // Patch each body's SBI q_ms with its TxQueue wait (push→pop), and
      // fold the batch's worst case into the window-max gauge (spec
      // 2026-08-30 latency-accounting, Task 4). enqueued_ms == 0 means the
      // producer never stamped it (shouldn't happen post-Task-3/4, but
      // dry-run/test paths may still push bare bodies) -> leave q_ms at the
      // packer's zero placeholder rather than reporting a bogus wait.
      const uint32_t pop_ms = static_cast<uint32_t>(now_steady_ms());
      const uint64_t pop_us = now_steady_us();
      for (auto& b : batch) {
        uint32_t w = (b.enqueued_ms && pop_ms > b.enqueued_ms)
                         ? pop_ms - b.enqueued_ms : 0;
        if (w > 65535) w = 65535;
        mabur::sbi_set_q_ms(b.body.data(), b.body.size(),
                            static_cast<uint16_t>(w));
        uint32_t prev = txq_wait_max_ms.load(std::memory_order_relaxed);
        while (w > prev && !txq_wait_max_ms.compare_exchange_weak(prev, w)) {}
        if (b.pushed_us && pop_us > b.pushed_us) {
          const uint64_t tw = pop_us - b.pushed_us;
          ++qw_n; qw_sum_us += tw;
          if (tw > qw_max_us) qw_max_us = tw;
          if (b.au_first) {
            ++qf_n; qf_sum_us += tw;
            if (tw > qf_max_us) qf_max_us = tw;
          }
        }
      }
      if (pop_ms - last_qw_report_ms >= 5000) {
        last_qw_report_ms = pop_ms;
        if (qw_n > 0) {
          std::fprintf(stderr,
              "maburd dq_queue: bodies=%llu wait_us mean=%llu max=%llu "
              "au_first n=%llu mean=%llu max=%llu\n",
              (unsigned long long)qw_n,
              (unsigned long long)(qw_sum_us / qw_n),
              (unsigned long long)qw_max_us,
              (unsigned long long)qf_n,
              (unsigned long long)(qf_n ? qf_sum_us / qf_n : 0),
              (unsigned long long)qf_max_us);
        }
        qw_n = qw_sum_us = qw_max_us = 0;
        qf_n = qf_sum_us = qf_max_us = 0;
        if (sb_calls > 0) {
          std::fprintf(stderr,
              "maburd tx_send: calls=%llu bodies=%llu us/call mean=%llu "
              "max=%llu us/body=%llu\n",
              (unsigned long long)sb_calls,
              (unsigned long long)sb_bodies,
              (unsigned long long)(sb_sum_us / sb_calls),
              (unsigned long long)sb_max_us,
              (unsigned long long)(sb_bodies ? sb_sum_us / sb_bodies : 0));
        }
        sb_calls = sb_bodies = sb_sum_us = sb_max_us = 0;
      }
      const uint64_t sb_t0 = now_steady_us();
      tx.send_bodies(batch);
      const uint64_t sb_dt = now_steady_us() - sb_t0;
      ++sb_calls;
      sb_bodies += batch.size();
      sb_sum_us += sb_dt;
      if (sb_dt > sb_max_us) sb_max_us = sb_dt;
    }
  });

  // Agent thread: drains the RC queue every cfg.link.rc_drain_ms, ticks
  // RcAgent on cfg.link.tick_ms, runs the watchdog, and handles SIGUSR1
  // stats dumps.
  std::thread agent_thread([&]() {
    const uint64_t grace_ms = 10000;
    const uint64_t stale_ms = 3000;
    uint64_t start = now_steady_ms();

    uint64_t last_hot_beat = 0, last_rx_beat = 0;
    uint64_t last_hot_change_ms = start, last_rx_change_ms = start;
    uint64_t last_stats_ms = start;

    // T_TELEM (spec 2026-07-26 drone-telemetry): sent at ~1 Hz on this same
    // periodic path, on the mutex-guarded dev_sink.send() the MSP thread
    // also uses. Its own radiotap (control modulation, built once) and its
    // own dot11 seq counter — deliberately NOT the video path's tx.seq() or
    // RealActuator's DISC_ACK control_seq, so a telemetry-send bug can never
    // perturb either.
    uint64_t last_telem_ms = start;
    uint64_t rx_beat_at_last_telem = 0;
    uint16_t telem_wire_seq = 0;
    uint16_t telem_dot11_seq = 0;
    std::vector<uint8_t> telem_radiotap = devourer::build_stream_radiotap(control_tx_mode());

    mabur::TickGate tick_gate(now_steady_ms(), cfg.link.tick_ms);
    while (!g_devourer_should_stop) {
      uint64_t now = now_steady_ms();

      // Every wake (rc_drain_ms): drain and apply queued RCFs. This is the
      // whole point of the split — op actuation no longer waits for the
      // housekeeping tick (spec 2026-08-14 §3b; measured U(0, tick_ms=100)
      // before, close_ms median ~110 ms).
      std::vector<uint8_t> rc_body;
      while (rc_queue.pop(rc_body)) {
        agent.on_rc_frame(rc_body.data(), rc_body.size(), now);
      }

      if (tick_gate.due(now)) {
        devourer::ThermalStatus thermal = rtl_device->GetThermalStatus();
        devourer::TxStats txstats = rtl_device->GetTxStats();
        RadioHealth health;
        health.thermal_delta = thermal.valid ? thermal.delta : 0;
        health.tx_drops = txstats.failed;
        agent.tick(now, health);
        if (agent.take_link_established())
          link_up_discont.store(true, std::memory_order_relaxed);

        // Watchdog: after an initial grace period, a heartbeat going stale for
        // > stale_ms means the corresponding loop is wedged — EXCEPT rx_beat,
        // which only bumps when a frame is actually received off the air.
        // Silence on the RX side is the expected steady state whenever the
        // agent isn't LINKED (RENDEZVOUS: waiting for a DISC beacon; FAILSAFE:
        // GS has gone quiet, which is exactly the scenario failsafe exists
        // for) — gating rx-stale detection on agent.state() == LINKED tells
        // "the receive path is stuck" apart from "the ground station turned
        // off," which used to abort()/respawn-loop maburd forever on a merely
        // quiet channel. The hot-thread check stays unconditional: hot_beat
        // bumps every ring-read iteration regardless of RF activity, so its
        // staleness always means the pipeline thread itself is wedged. This
        // check runs on the agent thread, which already owns `agent`
        // (RcAgent::tick() above is called from here), so reading
        // agent.state() here is the same-thread access it already is
        // elsewhere in this loop — no cross-thread synchronization needed.
        uint64_t hb = hot_beat.load(std::memory_order_relaxed);
        uint64_t rb = rx_beat.load(std::memory_order_relaxed);
        if (hb != last_hot_beat) {
          last_hot_beat = hb;
          last_hot_change_ms = now;
        }
        if (rb != last_rx_beat) {
          last_rx_beat = rb;
          last_rx_change_ms = now;
        }
        if (now - start > grace_ms) {
          if (now - last_hot_change_ms > stale_ms) {
            std::fprintf(stderr, "watchdog: hot thread stalled (no beat for >%llums)\n",
                         static_cast<unsigned long long>(stale_ms));
            std::abort();
          }
          if (agent.state() == RcAgent::State::LINKED && now - last_rx_change_ms > stale_ms) {
            std::fprintf(stderr,
                         "watchdog: rx loop stalled while LINKED (no beat for >%llums)\n",
                         static_cast<unsigned long long>(stale_ms));
            std::abort();
          }
        }

        bool want_stats = g_sigusr1_flag.exchange(false);
        if (want_stats || now - last_stats_ms >= 1000) {
          last_stats_ms = now;
          std::fprintf(stderr,
                       "stats: state=%d hot_beat=%llu rx_beat=%llu seq=%u sent=%llu drops=%llu "
                       "txq=%zu txq_drop=%llu "
                       "thermal_delta=%d tx_failed=%llu venc_verb_fail=%llu\n",
                       static_cast<int>(agent.state()), static_cast<unsigned long long>(hb),
                       static_cast<unsigned long long>(rb), tx.seq(),
                       static_cast<unsigned long long>(tx.sent()),
                       static_cast<unsigned long long>(tx.drops()),
                       txq.depth(), static_cast<unsigned long long>(txq.dropped()),
                       health.thermal_delta,
                       static_cast<unsigned long long>(txstats.failed),
                       static_cast<unsigned long long>(actuator.venc_verb_failures));
        }

        if (now - last_telem_ms >= 1000) {
          last_telem_ms = now;

          TelemInputs ti;
          ti.state = static_cast<int>(agent.state());
          ti.failsafe_shed = agent.failsafe_shed();
          ti.probing = agent.probing();
          // "advanced in the last 2 s" (spec) approximated as "advanced over
          // the last telemetry tick" (~1 s here) — the collector runs on this
          // same 1 Hz cadence, so a stricter 2 s window would just double-count
          // the same beat across two ticks.
          ti.radio_rx_ok = rb > rx_beat_at_last_telem;
          rx_beat_at_last_telem = rb;
          ti.generation = agent.current().generation;
          // Telemetry rides the robust base rate (slot 0, mcs-1) to ensure
          // control packets are reliably delivered even at the edge of coverage.
          // The ladder is 2-slot: slot 0 (base, mcs-1) and slot 1 (enh, mcs).
          ti.mode = agent.current().ladder[0].mode;
          ti.mcs = agent.current().ladder[0].mcs;
          ti.bw = agent.current().ladder[0].bw;
          // applied_ov_base/enh report the commanded op PAIR (Task 6,
          // RC_VERSION 5 — the fixed per-rung values RcAgent applies
          // directly to the UEP layers), or the debug-HTTP per-layer
          // override when armed (the same two atomics run_bitrate_policy's
          // override check reads) — the AirFeed solver this used to report
          // instead is gone (Task 7).
          {
            const int ob = air_feed_out.ovr_base_pct.load(std::memory_order_relaxed);
            const int oe = air_feed_out.ovr_enh_pct.load(std::memory_order_relaxed);
            if (ob >= 0 && oe >= 0) {
              ti.applied_ov_base = ob / 100.0;
              ti.applied_ov_enh = oe / 100.0;
            } else {
              ti.applied_ov_base = agent.current().fec_ov_base;
              ti.applied_ov_enh = agent.current().fec_ov_enh;
            }
          }
          // have_feedback() false means no RCF has EVER been accepted (still
          // BOOT/RENDEZVOUS) — 0 would read as maximally fresh, the opposite of
          // the truth. Pass a value make_telem's saturate<uint16_t> clamps to
          // 65535 ("never"), matching the wire field's documented sentinel.
          ti.rcf_age_ms = agent.have_feedback()
                              ? (now - agent.last_feedback_ms())
                              : static_cast<uint64_t>(UINT16_MAX) + 1;
          ti.rcf_rx = agent.rcf_accepted();
          ti.enc_frames = enc_frames_total.load(std::memory_order_relaxed);
          ti.enc_bytes = enc_bytes_total.load(std::memory_order_relaxed);
          ti.cmd_kbps = actuator.last_bitrate_kbps;
          ti.qp = actuator.last_roi_qp;
          ti.ring_drops = ring_drops_total.load(std::memory_order_relaxed);
          ti.txq_depth = txq.depth();
          ti.txq_cap = kTxQueueCap;
          ti.txq_drops = txq.dropped();
          ti.txq_wait_max_ms = txq_wait_max_ms.exchange(0, std::memory_order_relaxed);
          ti.radio_sent = tx.sent();
          ti.radio_drops = tx.drops();
          ti.usb_fail = txstats.failed;
          ti.uplink = uplink_track.snap();
          ti.soc_temp_c = read_soc_temp_c();
          if (ti.soc_temp_c == -128)  // SigmaStar: no thermal_zone
            ti.soc_temp_c = read_soc_temp_c_sigmastar();
          ti.thermal_delta = health.thermal_delta;
          ti.load1 = read_load1();
          ti.idr_disagree = idr_disagree_total.load(std::memory_order_relaxed);
          ti.enhance_disagree = enhance_disagree_total.load(std::memory_order_relaxed);
          ti.vanished_base = vanished_base_total.load(std::memory_order_relaxed);
          ti.vanished_enh = vanished_enh_total.load(std::memory_order_relaxed);
          ti.self_idr_refused = self_idr_refused_total.load(std::memory_order_relaxed);
#ifdef MABUR_HAVE_VENC
          // Producer side of the frame ring, straight from the encoder
          // (venc_get_stats is thread-safe and reads the shm header, not a
          // cached copy). ring_drops above is the CONSUMER side — the two
          // count different losses and both are needed to tell "encoder
          // outran maburd" from "maburd rejected a slot". A silently
          // stalled encoder has neither: it shows as ti.enc_frames flat.
          VencStats vs{};
          venc_get_stats(&vs);
          ti.venc_full_drops = vs.full_drops;
          ti.venc_ring_fill_pct = static_cast<int>(vs.ring_fill_pct);
#endif

          auto telem = rc::pack_telem(make_telem(telem_wire_seq++, ti));

          std::vector<uint8_t> frame;
          frame.reserve(telem_radiotap.size() + kDot11HeaderLen + telem.size());
          frame.insert(frame.end(), telem_radiotap.begin(), telem_radiotap.end());
          auto hdr = build_dot11_header(telem_dot11_seq);
          telem_dot11_seq = static_cast<uint16_t>((telem_dot11_seq + 1) & 0xFFF);
          frame.insert(frame.end(), hdr.begin(), hdr.end());
          frame.insert(frame.end(), telem.begin(), telem.end());
          dev_sink.send(frame.data(), frame.size());
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(cfg.link.rc_drain_ms));
    }
  });

  // v1 only ever tunes the radio to 20 MHz — cfg.radio.width is parsed and
  // validated (config.cpp) but not otherwise consulted here. Rather than
  // silently ignoring a configured 40/80 and running at 20 MHz anyway, warn
  // once at startup so a mismatched config is visible in the log instead of
  // just quietly not doing what it says.
  if (cfg.radio.width != 20) {
    std::fprintf(stderr, "warning: radio.width=%d not supported in v1, using 20 MHz\n",
                 cfg.radio.width);
  }

  // Bring up TX FIRST and let it finish before anything transmits. InitWrite
  // runs the full power-on + firmware download + TX-path enable and returns
  // only once the chip is ready; StartRxLoop then runs the (blocking) RX worker
  // with TX+RX concurrent on the same handle. Opening device_ready between the
  // two is what keeps the hot/agent threads from clogging the bulk-OUT FIFO
  // mid-DLFW — see DevourerSink::ready.
  std::fprintf(stderr, "maburd bringing up TX on channel %d\n", cfg.radio.channel);
  rtl_device->InitWrite(
      SelectedChannel{static_cast<uint8_t>(cfg.radio.channel), 0, CHANNEL_WIDTH_20});
  // Bring-up record for the non-standard MAC state requested via
  // dev_cfg.tuning.disable_cca above. devourer logs its own carrier-sense line
  // at info, and the production cross-build compiles info out
  // (DEVOURER_LOG_MAX_LEVEL=WARN), so without this the deployed daemon leaves no
  // trace that it is transmitting without carrier sense. Unconditional: the flag
  // is hardcoded true, so there is nothing to branch on. Wording is deliberate --
  // this records what maburd REQUESTED of devourer, not a register readback.
  std::fprintf(stderr,
               "maburd radio: MAC carrier sense (CCA+EDCCA) requested OFF -- TX "
               "will not defer to co-channel traffic\n");

  // power_mode == "offset": program the wall-equalized per-rate diff table
  // once at bring-up, then zero the global offset once (see below) — power
  // is constant for the life of the process, no per-op trim.
  // SetTxPowerRateDiffs returns false on non-8822E boards (8822E-only in
  // v1, TxPower.h) — warn and continue rather than aborting bring-up, so
  // "offset" configured on an unsupported chip degrades to the untrimmed
  // efuse table instead of failing to fly.
  if (cfg.radio.power_mode == "offset") {
    auto plan = make_power_plan(cfg.radio.rate_walls_idx, cfg.radio.legacy_wall_idx,
                                 cfg.radio.base_ref_idx, cfg.radio.wall_margin_db);
    devourer::TxRateDiffsQdb diffs;
    diffs.cck = plan.cck;
    diffs.legacy = plan.legacy;
    for (int i = 0; i < 8; ++i) diffs.mcs[i] = plan.mcs[i];
    if (!rtl_device->SetTxPowerRateDiffs(diffs)) {
      std::fprintf(stderr,
                   "warning: SetTxPowerRateDiffs failed (non-8822E board?); "
                   "power_mode=offset will trim the untrimmed efuse table\n");
    }
    // Power is constant from here on. devourer documents the offset as
    // sticky across retunes, so zero it explicitly rather than assuming
    // whatever a prior process or bench tool left in the chip.
    rtl_device->SetTxPowerOffsetQdb(0);
  }

  // A-MPDU TX aggregation (spec 2026-09-01-ampdu-design.md): one devourer
  // call marks every data frame aggregatable (data QSEL 0 + AGG_EN +
  // MAX_AGG_NUM + density) with per-frame retry limit 0 (no_ack — no
  // BlockAck peer exists; FEC covers loss, and without retry0 the MAC
  // re-airs each aggregate to the retry limit: 92% wasted airtime), and
  // programs the 0x455 aggregate-fill timer. Frames are already QoS-Data
  // (radio_tx.cpp) whether or not this call runs — max_num 0 leaves
  // aggregation off with the wire unchanged (the measured-identical
  // singles path, dq-spike-findings §11). Must run after InitWrite (live
  // register write) and before device_ready opens the TX gate.
  if (cfg.ampdu.max_num > 0) {
    devourer::AmpduMode am;
    am.enabled = true;
    am.tid = 0;
    am.max_num = static_cast<uint8_t>(cfg.ampdu.max_num);
    am.density = 7;
    am.no_ack = true;
    am.max_time = static_cast<uint8_t>(cfg.ampdu.max_time);
    if (!rtl_device->SetAmpduMode(am)) {
      std::fprintf(stderr,
                   "warning: SetAmpduMode failed — running un-aggregated "
                   "(QoS-Data singles)\n");
    } else {
      std::fprintf(stderr,
                   "maburd radio: A-MPDU ON (max_num=%d density=7 no-ack "
                   "max_time=0x%02x)\n",
                   cfg.ampdu.max_num, cfg.ampdu.max_time);
    }
  } else {
    std::fprintf(stderr,
                 "maburd radio: A-MPDU OFF (ampdu.max_num=0) — QoS-Data "
                 "singles\n");
  }

  device_ready.store(true, std::memory_order_release);
  std::fprintf(stderr, "maburd entering RX loop on channel %d\n", cfg.radio.channel);
  rtl_device->StartRxLoop(rx_callback);

  // Init() returns once g_devourer_should_stop is set (SIGINT/SIGTERM) or the
  // device errors out internally. Ensure the flag is set on the error-out
  // path too, so hot_thread/agent_thread (which key off the same flag) are
  // guaranteed to exit and these joins complete.
  g_devourer_should_stop = true;
  if (hot_thread.joinable()) hot_thread.join();
  if (tx_thread.joinable()) tx_thread.join();
  if (agent_thread.joinable()) agent_thread.join();
  if (msp_thread.joinable()) msp_thread.join();
  tx_pool.stop();  // drain + join senders before device teardown

#ifdef MABUR_HAVE_VENC
  // After the thread joins above: the hot thread reads the frame ring that
  // stop() tears down.
  venc_core_stop();
#endif

  rtl_device->Stop();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(usb_ctx);
  return 0;
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

void print_usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s -c <config.json> [--dry-run --in <file> --out <file> [--rc-in <file>]]\n",
               argv0);
}

}  // namespace

int main(int argc, char** argv) {
  std::string cfg_path;
  bool dry_run = false;
  std::string in_path, out_path, rc_in_path;
  std::string msp_in_path, msp_out_path;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) {
      cfg_path = argv[++i];
    } else if (a == "--dry-run") {
      dry_run = true;
    } else if (a == "--in" && i + 1 < argc) {
      in_path = argv[++i];
    } else if (a == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (a == "--rc-in" && i + 1 < argc) {
      rc_in_path = argv[++i];
    } else if (a == "--msp-in" && i + 1 < argc) {
      msp_in_path = argv[++i];
    } else if (a == "--msp-out" && i + 1 < argc) {
      msp_out_path = argv[++i];
    } else if (a == "-h" || a == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      print_usage(argv[0]);
      return 1;
    }
  }

  if (cfg_path.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  Config cfg;
  try {
    cfg = load_config(cfg_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }

  std::fprintf(stderr,
               "fec: symbol_size=[%d,%d] bpb=[%d,%d] window=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.blocks_per_body[0], cfg.fec.blocks_per_body[1],
               cfg.fec.window);

  if (dry_run) {
    if (in_path.empty() || out_path.empty()) {
      std::fprintf(stderr, "error: --dry-run requires --in and --out\n");
      print_usage(argv[0]);
      return 1;
    }
    return run_dry_run(cfg, in_path, out_path, rc_in_path, msp_in_path, msp_out_path);
  }

  return run_real_mode(cfg);
}
