// maburd — drone-side daemon: reads RTP from the waybeam venc SHM ring (or,
// in --dry-run, a fixture file), runs it through the UEP/FEC pipeline, and
// hands radio-bound bodies to the adaptive-link-controlled RadioTx. A
// parallel agent thread runs RcAgent against inbound RC frames + periodic
// radio-health ticks, publishing AppliedOp changes the hot path picks up via
// a lock-free shared_ptr handoff.
//
// Two modes:
//   maburd -c /etc/mabur.json                     — real mode (devourer USB radio)
//   maburd -c cfg.json --dry-run --in F --out F [--rc-in F]  — file-driven, no radio
//
// Dry-run is the tested path (see tests/fixtures/rtp_stream.bin smoke test);
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

#include "config.h"
#include "mabur/msp_source.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sw_wire.h"
#include "mabur/fec_worker.h"
#include "mabur/uep_encoder.h"
#include "msp_serial.h"
#include "power_plan.h"
#include "radio_tx.h"
#include "rc_agent.h"
#include "ring_source.h"
#include "tx_queue.h"
#include "usb_tx_pool.h"
#include "waybeam_client.h"

#if defined(MABUR_DRY_RUN_ONLY)
// Not used; real mode is always compiled in, guarded at runtime by --dry-run
// so the same binary runs (in dry-run) on a machine with no dongle attached.
#endif

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
// RealActuator — bridges RcAgent to the radio/UEP/waybeam world.
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
// the waybeam VTX control API, and (real mode only) the devourer device's
// TX-power knob.
//
// Threading: apply_op()/send_control()/set_bitrate_kbps()/set_roi_qp()/
// request_idr() are all called from the agent thread only (RcAgent's
// contract). apply_op() publishes the new AppliedOp into shared_op via an
// atomic store of a fresh shared_ptr — the hot thread picks it up with an
// atomic load, so there is no lock and no torn read.
struct RealActuator : mabur::Actuator {
  mabur::RadioTx* tx = nullptr;
  mabur::WaybeamClient* wb = nullptr;
  mabur::FrameSink* sink = nullptr;
  std::atomic<std::shared_ptr<const mabur::AppliedOp>>* shared_op = nullptr;
  IRtlDevice* dev = nullptr;  // nullptr in dry-run
  bool dry_run = false;
  std::string power_mode = "override";  // radio.power_mode (see config.h)

  std::vector<uint8_t> control_radiotap;  // built once; control channel is fixed
  uint16_t control_seq = 0;

  void apply_op(const AppliedOp& op) override {
    tx->set_ladder(op.ladder);
    if (dev) {
      if (power_mode == "offset")
        dev->SetTxPowerOffsetQdb(op.pwr_offset_qdb);
      // "override": bench-diagnostic, ignores RCF-commanded power entirely
      // (see config.h's power_mode comment) — applies nothing per-op.
      // "none": leave the efuse per-rate table untouched.
    } else if (dry_run) {
      std::fprintf(stderr, "[dry-run] pwr_offset_qdb=%d fec_overhead=%.3f gen=%llu\n",
                   op.pwr_offset_qdb, op.fec_overhead,
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

  void set_bitrate_kbps(int k) override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_bitrate_kbps(%d)\n", k);
      return;
    }
    wb->set_param("video0.bitrate", std::to_string(k));
  }

  void set_roi_qp(int q) override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_roi_qp(%d)\n", q);
      return;
    }
    wb->set_param("fpv.roiQp", std::to_string(q));
  }

  void request_idr() override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] request_idr()\n");
      return;
    }
    wb->request_idr();
  }
};

uint64_t now_steady_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Applies a freshly-published AppliedOp (detected via shared_ptr identity,
// not generation — see the callers' pump-loop comments) to the
// hot-thread-owned UepEncoder (set_overhead_scale + per-layer shed). Called
// from the hot thread only.
void apply_op_to_uep(const AppliedOp& op, UepEncoder& uep) {
  uep.set_overhead_scale(op.fec_overhead);
  for (int i = 0; i < 4; ++i) uep.set_shed(i, op.shed[static_cast<size_t>(i)]);
}

// ---------------------------------------------------------------------------
// Dry-run mode
// ---------------------------------------------------------------------------

struct RcInRecord {
  uint32_t after_packet_index;
  std::vector<uint8_t> body;
};

std::vector<std::vector<uint8_t>> read_len_prefixed_u16(const std::string& path) {
  std::vector<std::vector<uint8_t>> pkts;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return pkts;
  while (true) {
    uint8_t lenb[2];
    if (std::fread(lenb, 1, 2, f) != 2) break;
    uint16_t len = static_cast<uint16_t>(lenb[0]) | (static_cast<uint16_t>(lenb[1]) << 8);
    std::vector<uint8_t> pkt(len);
    if (len > 0 && std::fread(pkt.data(), 1, len, f) != len) break;
    pkts.push_back(std::move(pkt));
  }
  std::fclose(f);
  return pkts;
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

  RadioTx tx(file_sink, cfg.radio.bw_set);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.wb = nullptr;
  actuator.sink = &file_sink;
  actuator.shared_op = &shared_op;
  actuator.dev = nullptr;
  actuator.dry_run = true;

  RcAgent agent(cfg, actuator);
  // Deterministic replay output: the async FEC worker is never attached in
  // dry-run mode (repair emission order would depend on thread timing).
  UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms);

  auto pkts = read_len_prefixed_u16(in_path);
  auto rc_recs = read_rc_in(rc_in_path);
  size_t rc_idx = 0;

  std::shared_ptr<const AppliedOp> last_applied_op;

  uint64_t sent_bodies = 0;
  uint64_t consumed_packets = 0;

  // First tick: BOOT -> RENDEZVOUS, applies MAX_RANGE op.
  agent.tick(now_steady_ms(), RadioHealth{});

  // Detects "a new AppliedOp was published" by shared_ptr IDENTITY, not by
  // generation: reapply_with_derate_and_shed() (thermal/congestion) publishes
  // a fresh AppliedOp via apply_op() WITHOUT bumping generation (by design —
  // generation tracks new operating points, not power/shed adjustments to
  // the current one), but every apply_op() call, including reapplies, always
  // stores a brand-new shared_ptr<const AppliedOp>. Comparing against
  // generation alone would silently miss local congestion/thermal
  // shed-and-derate changes whenever no new RCF/DISC/failsafe op happened in
  // between — the exact bug this identity-compare fixes.
  auto pump_op_change = [&]() {
    auto op = shared_op.load();
    if (op && op != last_applied_op) {
      apply_op_to_uep(*op, uep);
      last_applied_op = op;
    }
  };
  pump_op_change();

  // Drain semantics: deliver every RC record whose after_packet_index is
  // <= consumed_packets, in file order, regardless of whether that exact
  // count was ever hit as a distinct step. A strict `==` check (the
  // previous implementation) blocks forever on a record whose index is 0
  // (never equal to consumed_packets, which is incremented starting from
  // 1), duplicated, out-of-order, or >= pkts.size() — and because rc_idx
  // only ever advances past a match, one stuck record wedges every
  // subsequent record too. Draining on `<=` delivers exact matches at the
  // same point as before (e.g. after_packet_index=3 still lands right
  // after the 3rd packet is consumed) while guaranteeing every record is
  // eventually delivered — before the loop for index 0, inline as the
  // packet count catches up, and at EOF for anything left over.
  auto drain_rc_records = [&](uint64_t now, bool drain_all = false) {
    uint64_t gate = drain_all ? UINT64_MAX : consumed_packets;
    while (rc_idx < rc_recs.size() && rc_recs[rc_idx].after_packet_index <= gate) {
      agent.on_rc_frame(rc_recs[rc_idx].body.data(), rc_recs[rc_idx].body.size(), now);
      std::fprintf(stderr, "[dry-run] rc-in delivered record %zu (after_packet_index=%u) at consumed_packets=%llu\n",
                  rc_idx, rc_recs[rc_idx].after_packet_index,
                  static_cast<unsigned long long>(consumed_packets));
      ++rc_idx;
    }
  };

  // Deliver any records due before the first packet (after_packet_index=0).
  drain_rc_records(now_steady_ms());

  for (size_t i = 0; i < pkts.size(); ++i) {
    uint64_t now = now_steady_ms();

    pump_op_change();

    auto bodies = uep.add_rtp(pkts[i].data(), pkts[i].size(), now);
    for (auto& b : bodies) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    auto polled = uep.poll(now);
    for (auto& b : polled) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    ++consumed_packets;

    // Deliver any RC records due at or before this packet index, then tick
    // the agent (simulated radio health: empty/no drops).
    drain_rc_records(now);
    agent.tick(now, RadioHealth{});
    pump_op_change();
  }

  // EOF: deliver any records left over (duplicate/out-of-order/>=
  // pkts.size() indices), flush every layer, send whatever falls out, then
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
              "maburd dry-run stats: packets_in=%zu bodies_out=%llu seq=%u sent=%llu drops=%llu "
              "agent_state=%d rc_records=%zu/%zu\n",
              pkts.size(), static_cast<unsigned long long>(sent_bodies), tx.seq(),
              static_cast<unsigned long long>(tx.sent()), static_cast<unsigned long long>(tx.drops()),
              static_cast<int>(agent.state()), rc_idx, rc_recs.size());
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
  // Info-level events include one tx.agg line per aggregated URB (~600/s at
  // video rate) — that floods the RAM-backed /tmp/mabur.log. Warnings only.
  logger->set_level(Logger::Level::Warn);

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

  RadioTx tx(dev_sink, cfg.radio.bw_set);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  WaybeamClient waybeam(cfg.waybeam);

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.wb = &waybeam;
  actuator.sink = &dev_sink;
  actuator.shared_op = &shared_op;
  actuator.dev = rtl_device.get();
  actuator.dry_run = false;
  actuator.power_mode = cfg.radio.power_mode;

  RcAgent agent(cfg, actuator);

  RcQueue rc_queue;
  std::atomic<uint64_t> rx_beat{0};
  std::atomic<uint64_t> hot_beat{0};

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
  TxQueue txq(256);

  // Hot thread: pulls RTP off the real SHM ring, runs it through the UEP
  // pipeline, queues bodies for the TX writer. Owns the UepEncoder
  // exclusively; never blocks on USB.
  std::thread hot_thread([&]() {
    RingSource ring(cfg.ring_name);
    // Async FEC worker (spec 2026-07-17): repair envelopes build on the
    // second core. Declared BEFORE the UepEncoder so it is destroyed after
    // it — each layer's SwEncoder joins its outstanding jobs in its own
    // dtor first. A wedged worker shows up as flush()'s join spinning on
    // this thread, which the hot_beat watchdog already catches.
    std::unique_ptr<FecWorker> fec_worker;
    if (cfg.fec.async_worker)
      fec_worker = std::make_unique<FecWorker>(cfg.fec.worker_cpu);
    UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms, fec_worker.get());
    uint8_t buf[4096];

    std::shared_ptr<const AppliedOp> last_applied_op;

    while (!g_devourer_should_stop) {
      uint64_t now = now_steady_ms();

      // Identity-compare, not generation-compare: see the matching comment
      // in run_dry_run's pump_op_change lambda above. reapply_with_derate_
      // and_shed() (thermal/congestion) never bumps generation, so gating on
      // generation here would silently drop local shed/derate changes that
      // arrive between two RCF-driven generation bumps — including the
      // congestion-triggered shed this loop is specifically responsible for
      // applying to the UepEncoder (op.shed) independent of the GS.
      auto op = shared_op.load();
      if (op && op != last_applied_op) {
        apply_op_to_uep(*op, uep);
        last_applied_op = op;
      }

      // Drain a BURST per iteration, not one packet: the per-iteration
      // overhead (op check, poll, beat) capped the old 1-packet loop at
      // ~800 reads/s while waybeam produces 1000+ pkt/s at 60fps — the SHM
      // ring pinned at full (bench: fill 457-511/512) and waybeam's
      // packetizer aborted NALs mid-chain on the overflow. First read
      // blocks up to 5 ms; the rest are non-blocking.
      int burst = 0;
      int n;
      while (burst < 64 &&
             (n = ring.read(buf, sizeof buf, burst == 0 ? 5 : 0)) > 0) {
        auto bodies = uep.add_rtp(buf, static_cast<size_t>(n), now);
        for (auto& b : bodies) txq.push(std::move(b));
        ++burst;
      }

      auto polled = uep.poll(now);
      for (auto& b : polled) txq.push(std::move(b));

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
    while (!g_devourer_should_stop) {
      batch.clear();
      if (txq.pop_batch(batch, 3, 5) == 0) continue;
      tx.send_bodies(batch);
    }
  });

  // Agent thread: drains the RC queue, ticks RcAgent on cfg.link.tick_ms,
  // runs the watchdog, and handles SIGUSR1 stats dumps.
  std::thread agent_thread([&]() {
    const uint64_t grace_ms = 10000;
    const uint64_t stale_ms = 3000;
    uint64_t start = now_steady_ms();

    uint64_t last_hot_beat = 0, last_rx_beat = 0;
    uint64_t last_hot_change_ms = start, last_rx_change_ms = start;
    uint64_t last_stats_ms = start;

    while (!g_devourer_should_stop) {
      uint64_t now = now_steady_ms();

      std::vector<uint8_t> rc_body;
      while (rc_queue.pop(rc_body)) {
        agent.on_rc_frame(rc_body.data(), rc_body.size(), now);
      }

      devourer::ThermalStatus thermal = rtl_device->GetThermalStatus();
      devourer::TxStats txstats = rtl_device->GetTxStats();
      RadioHealth health;
      health.thermal_delta = thermal.valid ? thermal.delta : 0;
      health.tx_drops = txstats.failed;
      agent.tick(now, health);

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
                     "thermal_delta=%d tx_failed=%llu waybeam_failures=%llu\n",
                     static_cast<int>(agent.state()), static_cast<unsigned long long>(hb),
                     static_cast<unsigned long long>(rb), tx.seq(),
                     static_cast<unsigned long long>(tx.sent()),
                     static_cast<unsigned long long>(tx.drops()),
                     txq.depth(), static_cast<unsigned long long>(txq.dropped()),
                     health.thermal_delta,
                     static_cast<unsigned long long>(txstats.failed),
                     static_cast<unsigned long long>(waybeam.failures()));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(cfg.link.tick_ms));
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

  // power_mode == "offset": program the wall-equalized per-rate diff table
  // once at bring-up (RcAgent's per-op SetTxPowerOffsetQdb calls trim
  // *around* this table; they don't replace it). SetTxPowerRateDiffs
  // returns false on non-8822E boards (8822E-only in v1, TxPower.h) — warn
  // and continue rather than aborting bring-up, so "offset" configured on
  // an unsupported chip degrades to the untrimmed efuse table instead of
  // failing to fly.
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
    // Boot offset: the last commanded/config offset until the first
    // RCF/DISC op supersedes it, clamped the same way RcAgent clamps every
    // commanded offset.
    int boot_offset =
        std::clamp(cfg.radio.power_offset_qdb, cfg.radio.min_offset_qdb, 0);
    rtl_device->SetTxPowerOffsetQdb(boot_offset);
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
               "fec: symbol_size=[%d,%d,%d,%d] bpb=[%d,%d,%d,%d] window=%d async_worker=%s worker_cpu=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.symbol_size[2], cfg.fec.symbol_size[3],
               cfg.fec.blocks_per_body[0], cfg.fec.blocks_per_body[1],
               cfg.fec.blocks_per_body[2], cfg.fec.blocks_per_body[3],
               cfg.fec.window, cfg.fec.async_worker ? "on" : "off",
               cfg.fec.worker_cpu);

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
