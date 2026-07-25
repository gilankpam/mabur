// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> frame tail
// -> RTP out).
// Plan 2 scope: real-radio mode (N-card front-ends, control loop, card failover).
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include "aggregator.h"
#include "body_queue.h"
#include "config.h"
#include "frame_file_source.h"
#include "frame_stream.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "msp_font.h"
#include "msp_renderer.h"
#include "msp_sink.h"
#include "op_table.h"
#include "radio_frontend.h"
#include "rtp_packetizer.h"
#include "stats_exporter.h"
#include "tx_selector.h"
#include "udp_sink.h"
#include "vrx_controller.h"

namespace {

std::atomic<bool> g_stop{false};
std::atomic<bool> g_dump{false};
void on_signal(int) { g_stop.store(true); }
void on_usr1(int) { g_dump.store(true); }

uint64_t mono_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

void usage() {
  std::fprintf(stderr,
               "usage: maburgs -c <config.json> --dry-run --in <frames.bin>\n"
               "               [--cards N] [--drop-pct P] [--seed S] [--out-rtp <file>]\n");
}

struct RtpFileOut {
  FILE* f = nullptr;
  uint64_t written = 0;
  bool open(const char* path) { return (f = fopen(path, "wb")) != nullptr; }
  void write(const std::vector<uint8_t>& pkt) {
    const uint8_t len[2] = {static_cast<uint8_t>(pkt.size() & 0xFF),
                            static_cast<uint8_t>(pkt.size() >> 8)};
    fwrite(len, 1, 2, f);
    fwrite(pkt.data(), 1, pkt.size(), f);
    ++written;
  }
  ~RtpFileOut() { if (f) fclose(f); }
};

static int run_radio(const maburgs::Config& cfg) {
  std::fprintf(stderr,
               "fec: symbol_size=[%d,%d,%d,%d] decode_deadline_ms=%d seq_horizon=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.symbol_size[2], cfg.fec.symbol_size[3],
               cfg.fec.decode_deadline_ms, cfg.fec.seq_horizon);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGUSR1, on_usr1);

  const int n_cards = static_cast<int>(cfg.radio.cards.size());
  maburgs::BodyQueue queue;  // all cards share one queue; card_id tags origin
  std::vector<std::unique_ptr<maburgs::RadioFrontend>> fronts;
  for (int i = 0; i < n_cards; ++i) {
    maburgs::RadioFrontend::Cfg fc;
    fc.usb_vid = cfg.radio.cards[static_cast<size_t>(i)].usb_vid;
    fc.usb_pid = cfg.radio.cards[static_cast<size_t>(i)].usb_pid;
    fc.index = cfg.radio.cards[static_cast<size_t>(i)].index;
    fc.channel = cfg.radio.channel;
    fc.card_id = static_cast<uint8_t>(i);
    fronts.push_back(std::make_unique<maburgs::RadioFrontend>(fc, queue));
  }

  maburgs::Aggregator agg(cfg.uep_layers(),
                          static_cast<uint64_t>(cfg.fec.decode_deadline_ms),
                          static_cast<uint32_t>(cfg.fec.seq_horizon), n_cards);
  maburgs::UdpSink udp(cfg.video_out.host, cfg.video_out.port);
  // RTP order health of the emitted stream. The packetizer builds RTP from
  // frames FrameStream has already ordered by frame_id, so seq is monotonic by
  // construction — this counter is the canary on that construction (a gap or a
  // backward seq means the frame tail regressed), and it measures exactly what
  // a live decoder would choke on. seq16 from the RTP header; fwd_gap =
  // skipped-ahead seqs, back = packets emitted behind the highest seq seen.
  struct RtpOrder {
    bool has_last = false;
    uint16_t last = 0;
    uint64_t in_order = 0, fwd_gap = 0, back = 0, gap_seqs = 0;
  } rtp_order;

  // Stats sideport (spec: docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
  // Declared here (ahead of the RtpPacketizer/FrameStream construction below)
  // so the FrameStream end_frame lambda can capture `stats` by reference; both
  // must also outlive every lambda that captures them.
  // session id: nonzero random u32 so consumers detect restarts.
  std::optional<maburgs::UdpSink> stats_udp;
  std::optional<maburgs::StatsExporter> stats;
  if (cfg.stats.enable) {
    stats_udp.emplace(cfg.stats.host, cfg.stats.port);
    uint32_t session = 0;
    std::random_device rd;
    while (session == 0) session = rd();
    stats.emplace(session, cfg.stats.interval_ms,
                  [&](const std::string& s) {
                    return stats_udp->send(
                        reinterpret_cast<const uint8_t*>(s.data()), s.size());
                  });
    std::fprintf(stderr, "maburgs: stats sideport -> udp %s:%d every %d ms\n",
                 cfg.stats.host.c_str(), cfg.stats.port, cfg.stats.interval_ms);
  }

  // Video tail: FrameStream reassembles whole frames from the raw FRAG
  // fragments the decoder emits and streams Annex-B bytes into RtpPacketizer,
  // which builds RFC 7798 RTP for the udp sink.
  maburgs::RtpPacketizer pktz(
      {97, 0x4D414252u, 1400, 16667},
      [&](const std::vector<uint8_t>& pkt) {
        if (pkt.size() >= 4) {
          const uint16_t seq = static_cast<uint16_t>((pkt[2] << 8) | pkt[3]);
          if (rtp_order.has_last) {
            const uint16_t d = static_cast<uint16_t>(seq - rtp_order.last);
            if (d == 1) ++rtp_order.in_order;
            else if (d >= 1 && d <= 32767) { ++rtp_order.fwd_gap; rtp_order.gap_seqs += d - 1; }
            else ++rtp_order.back;
            if (d >= 1 && d <= 32767) rtp_order.last = seq;
          } else {
            rtp_order.has_last = true;
            rtp_order.last = seq;
          }
        }
        udp.send(pkt.data(), pkt.size());
      });
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video_out.frame_gap_timeout_ms),
       cfg.video_out.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h) { pktz.begin_frame(h); },
       [&](const uint8_t* d, size_t n) { pktz.data(d, n); },
       [&](bool c) {
         pktz.end_frame(c);
         if (stats) stats->on_frame(mono_ms());
       }});
  // Only fragments from a peer that advertised the frame wire format may reach
  // FrameStream: an older drone's bodies carry a mutually unparseable frag
  // header, and feeding them here would produce garbage video rather than an
  // obvious failure. Core-thread-owned, like everything else in this loop.
  bool frame_wire = false;
  bool refused_peer = false;  // one loud line per run, not per tick

  agg.set_frag_sink([&](const mabur::DecodedFrag& f) {
    if (frame_wire)
      fstream.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), mono_ms());
  });

  maburgs::LinkTable lt;
  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ctrl.src_bitrate_bps = cfg.link.src_bitrate_mbps * 1e6;
  vcfg.ctrl.margin_db = cfg.link.margin_db;
  vcfg.ctrl.min_offset_qdb = cfg.link.min_offset_qdb;
  vcfg.ctrl.max_offset_qdb = cfg.link.max_offset_qdb;
  vcfg.ctrl.base_ref_idx = cfg.link.base_ref_idx;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_overhead = cfg.link.static_overhead;
  vcfg.pin_offset_qdb = cfg.link.static_offset_qdb;
  maburgs::VrxController vrx(lt, vcfg);
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>& f, uint64_t us) {
    vrx.on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
  });

  std::unique_ptr<maburgs::UdpSink> msp_udp;
  std::unique_ptr<maburgs::MspRenderer> msp_renderer;
  std::unique_ptr<maburgs::MspSink> msp_sink;
  if (cfg.msp.enable) {
    const int bp = cfg.msp.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen);
    maburgs::MspSink::EmitFn emit;
    if (cfg.msp.render == "shm") {
      msp_renderer = std::make_unique<maburgs::MspRenderer>(
          maburgs::MspRenderCfg{cfg.msp.shm_name, cfg.msp.shm_x_offset, cfg.msp.shm_y_offset},
          maburgs::kMspFontBtfl);
      emit = [&](const uint8_t* d, size_t n) { msp_renderer->on_snapshot(d, n); };
      std::fprintf(stderr,
                   "maburgs: MSP OSD -> shm '%s' (PixelPilot) symbol_size=%d window=%d block_payload=%d\n",
                   cfg.msp.shm_name.c_str(), cfg.msp.symbol_size, cfg.msp.window, bp);
    } else {
      msp_udp = std::make_unique<maburgs::UdpSink>(cfg.msp.out_host, cfg.msp.out_port);
      emit = [&](const uint8_t* d, size_t n) { msp_udp->send(d, n); };
      std::fprintf(stderr,
                   "maburgs: MSP OSD -> udp %s:%d symbol_size=%d window=%d block_payload=%d\n",
                   cfg.msp.out_host.c_str(), cfg.msp.out_port, cfg.msp.symbol_size, cfg.msp.window, bp);
    }
    msp_sink = std::make_unique<maburgs::MspSink>(cfg.msp.symbol_size, cfg.msp.window, emit);
    agg.set_msp_sink([&](const uint8_t* b, size_t n, uint64_t us) {
      msp_sink->on_body(b, n, us / 1000);
    });
  }

  maburgs::TxSelector sel(
      maburgs::TxSelectorCfg{cfg.radio.tx_card, 3.0, 2000, 1500}, n_cards);

  std::vector<uint64_t> retry_at_ms(static_cast<size_t>(n_cards), 0);
  uint64_t last_stats_ms = 0;
  std::vector<mabur::node::RxBody> batch;

  while (!g_stop.load()) {
    const uint64_t now_ms_u = mono_ms();
    const double now_ms = static_cast<double>(now_ms_u);

    // Card lifecycle: (re)open dead front-ends with 2 s backoff.
    for (int i = 0; i < n_cards; ++i) {
      auto& fe = *fronts[static_cast<size_t>(i)];
      if (!fe.alive() && now_ms_u >= retry_at_ms[static_cast<size_t>(i)]) {
        fe.stop();
        if (!fe.open_and_start())
          std::fprintf(stderr, "card %d: open failed, retrying\n", i);
        retry_at_ms[static_cast<size_t>(i)] = now_ms_u + 2000;
      }
    }

    // Drain (blocks <=10 ms: this IS the control tick cadence).
    batch.clear();
    queue.drain(batch, 10);
    for (const auto& m : batch) {
      agg.on_rx_body(m);
      // Exclude RC frames AND MSP frames (SBI stream_id == kMspStreamId) from
      // the score window, mirroring Task 6's aggregator routing: MSP bodies
      // carry their own independent 802.11 seq, and letting them through here
      // would contaminate ScoreWindow::seq_gap_loss() with a gap sequence the
      // video decoder never sees. This keeps MSP traffic invisible to the
      // adaptive-link controller end-to-end (decoder residual + score window).
      if (m.crc_ok &&
          mabur::rc::frame_type(m.body.data(), m.body.size()) < 0 &&
          mabur::sbi_peek_stream_id(m.body.data(), m.body.size()) !=
              mabur::kMspStreamId) {
        // Video frame meta -> score window (max-chain values, Python parity).
        const double rssi = static_cast<double>(std::max(m.rssi[0], m.rssi[1]));
        const double snr = static_cast<double>(std::max(m.snr[0], m.snr[1]));
        vrx.on_video(rssi, snr, false, m.mac_seq,
                     static_cast<double>(m.mono_us) / 1000.0);
      }
    }
    // Re-read the clock: the drain above blocked up to 10 ms, and bodies
    // processed in it carry stamps newer than now_ms_u. Expiry/hold math
    // must run on a clock >= every stamp it has seen (the decoder and
    // reorder buffer also guard against stale clocks internally).
    const uint64_t drained_ms = mono_ms();
    agg.poll(drained_ms);

    // Session capability gate: the peer must be in an active SESSION (not
    // beaconing/pre-rendezvous) AND have advertised CAP_FRAME_WIRE in its
    // DiscAck before its video fragments are fed to the frame tail. A session
    // without the bit is a pre-frame-shm drone whose frag header this build
    // cannot parse: refuse its video loudly rather than render garbage. On any
    // change, drop FRAG-seq continuity and half-assembled frames — the new
    // session's seqs and frame_ids are unrelated to the old one's.
    const bool in_session = vrx.link_state() == maburgs::VrxState::SESSION;
    const bool fw = in_session && (vrx.peer_caps() & mabur::rc::CAP_FRAME_WIRE);
    if (fw != frame_wire) {
      frame_wire = fw;
      agg.decoder().reset_continuity();
      fstream.reset();
      std::fprintf(stderr, "maburgs: video tail -> %s\n",
                   fw ? "frame wire" : "off (no session)");
    }
    // Complain only about a peer we have actually heard a DiscAck from:
    // peer_caps() == 0 also reads as "no DiscAck yet", and the rendezvous
    // starts in SESSION, so gating on in_session alone printed this at every
    // startup — telling the operator to upgrade a maburd that was fine, seconds
    // before the tail came up anyway (caught on the rig 2026-07-25).
    if (vrx.peer_acked() && !fw && !refused_peer) {
      refused_peer = true;  // once per run: this cannot fix itself mid-session
      std::fprintf(stderr,
                   "maburgs: REFUSING video: peer session did not advertise "
                   "CAP_FRAME_WIRE (chip_caps=0x%04x). That drone predates the "
                   "frame wire format; upgrade maburd.\n",
                   vrx.peer_caps());
    }
    if (frame_wire) fstream.poll(drained_ms);

    // Control step: layer delivery + residual from the decode window.
    std::array<uint8_t, 4> ld{};
    for (int s = 0; s < 4; ++s)
      ld[static_cast<size_t>(s)] =
          static_cast<uint8_t>(agg.decoder().window_delivery_pct(s));
    auto [d0, e0] = agg.decoder().window_counts(0);
    auto [d1, e1] = agg.decoder().window_counts(1);
    std::optional<double> residual;
    if (e0 + e1 > 0)
      residual = 1.0 - static_cast<double>(d0 + d1) / static_cast<double>(e0 + e1);
    // Zero completed base-layer packets while video frames still arrive =
    // decode collapse; the SNR window is survivor-biased then (see
    // VrxController::step). Gate on having ever seen video so a pre-link
    // idle window doesn't count as starvation.
    const bool starved = (e0 + e1 == 0) && agg.last_video_us() != 0;
    if (auto out = vrx.step(now_ms, ld, residual, starved)) {
      if (!out->is_disc) agg.decoder().reset_window();  // window == RCF period
      std::vector<maburgs::CardSnapshot> snaps;
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        snaps.push_back(maburgs::CardSnapshot{
            fronts[static_cast<size_t>(i)]->alive(), t.snr_ema, t.rssi_b_ema,
            t.last_frame_us});
      }
      const int tx = sel.update(snaps, now_ms_u * 1000);
      fronts[static_cast<size_t>(tx)]->send_control(out->frame);
    }

    // 1 Hz stats line / SIGUSR1 dump.
    if (g_dump.exchange(false) || now_ms_u - last_stats_ms >= 1000) {
      last_stats_ms = now_ms_u;
      if (msp_sink) msp_sink->tick(now_ms_u);  // expire stale repair rows
      const auto& op = vrx.cur_op();
      std::fprintf(stderr,
                   "stats: state=%d tx_card=%d op=mcs%d/%d/ov%.2f/off%d "
                   "rtp=%llu udp_fail=%llu q_drop=%llu",
                   static_cast<int>(vrx.link_state()), sel.selected(), op.mcs,
                   op.bw, op.overhead, op.pwr_offset_qdb,
                   static_cast<unsigned long long>(udp.sent()),
                   static_cast<unsigned long long>(udp.failed()),
                   static_cast<unsigned long long>(queue.dropped()));
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        std::fprintf(stderr, " c%d[%s f=%llu cf=%llu snr=%.1f a=%.1f b=%.1f]",
                     i, fronts[static_cast<size_t>(i)]->alive() ? "up" : "DOWN",
                     static_cast<unsigned long long>(t.frames),
                     static_cast<unsigned long long>(t.crc_fail), t.snr_ema,
                     t.snr_a_ema, t.snr_b_ema);
      }
      for (int s = 0; s < 4; ++s) {
        const auto st = agg.decoder().stats(s);
        if (st.bodies == 0) continue;  // idle streams: keep the line short
        std::fprintf(stderr,
                     " s%d[p=%llu abn=%llu rec=%llu si=%llu st=%llu"
                     " bc=%llu sbf=%llu fl=%zu]",
                     s, static_cast<unsigned long long>(st.packets_out),
                     static_cast<unsigned long long>(st.syms_abandoned),
                     static_cast<unsigned long long>(st.syms_recovered),
                     static_cast<unsigned long long>(st.symbols_in),
                     static_cast<unsigned long long>(st.symbols_stale),
                     static_cast<unsigned long long>(st.symbols_bad_cfg),
                     static_cast<unsigned long long>(st.subblocks_failed),
                     st.rows_in_flight);
      }
      std::fprintf(stderr, " mis=%llu",
                   static_cast<unsigned long long>(agg.decoder().bodies_misrouted()));
      std::fprintf(stderr, " ord[ok=%llu gap=%llu(+%llu) back=%llu]",
                   static_cast<unsigned long long>(rtp_order.in_order),
                   static_cast<unsigned long long>(rtp_order.fwd_gap),
                   static_cast<unsigned long long>(rtp_order.gap_seqs),
                   static_cast<unsigned long long>(rtp_order.back));
      std::fprintf(stderr, " frames[clean/trunc/drop]=%llu/%llu/%llu badfrag=%llu stall=%llu",
                   static_cast<unsigned long long>(fstream.frames_clean()),
                   static_cast<unsigned long long>(fstream.frames_truncated()),
                   static_cast<unsigned long long>(fstream.frames_dropped()),
                   static_cast<unsigned long long>(fstream.bad_fragments()),
                   static_cast<unsigned long long>(fstream.stall_resets()));
      std::fprintf(stderr, "\n");
    }

    if (stats) {
      maburgs::StatsInput sin;
      sin.vtx_id = cfg.link.vtx_id;
      sin.in_session = in_session;
      sin.tx_card = sel.selected();
      sin.op = vrx.cur_op();
      sin.deadline_ms = cfg.fec.decode_deadline_ms;
      sin.residual_loss = residual;
      for (int s = 0; s < 4; ++s)
        sin.layer_delivery_pct[static_cast<size_t>(s)] = ld[static_cast<size_t>(s)];
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        maburgs::StatsCardIn ci;
        ci.up = fronts[static_cast<size_t>(i)]->alive();
        ci.frames = t.frames;
        ci.crc_fail = t.crc_fail;
        ci.seq_expected = t.seq_expected;
        ci.seq_received = t.seq_received;
        ci.rx_bytes = t.rx_bytes;
        ci.last_frame_us = t.last_frame_us;
        ci.self_frames = t.self_frames;
        ci.foreign = fronts[static_cast<size_t>(i)]->foreign();
        for (int k = 0; k < maburgs::kNumStatsClasses; ++k) {
          auto& cls = ci.classes[static_cast<size_t>(k)];
          const auto& tcls = t.cls[static_cast<size_t>(k)];
          cls.frames = tcls.frames;
          cls.has_ema = tcls.has_ema;
          cls.rssi_ema = tcls.rssi_ema;
          cls.rssi_a_ema = tcls.rssi_a_ema;
          cls.rssi_b_ema = tcls.rssi_b_ema;
          cls.snr_ema = tcls.snr_ema;
          cls.snr_a_ema = tcls.snr_a_ema;
          cls.snr_b_ema = tcls.snr_b_ema;
        }
        sin.cards.push_back(ci);
      }
      for (int s = 0; s < 4; ++s) {
        const auto st = agg.decoder().stats(s);
        auto& o = sin.streams[static_cast<size_t>(s)];
        o.bodies = st.bodies;
        o.subblocks_failed = st.subblocks_failed;
        o.syms_recovered = st.syms_recovered;
        o.syms_abandoned = st.syms_abandoned;
        o.symbols_in = st.symbols_in;
        o.symbols_stale = st.symbols_stale;
        o.symbols_bad_cfg = st.symbols_bad_cfg;
        o.rows_in_flight = st.rows_in_flight;
      }
      sin.frames_clean = fstream.frames_clean();
      sin.frames_truncated = fstream.frames_truncated();
      sin.frames_dropped = fstream.frames_dropped();
      sin.stall_resets = fstream.stall_resets();
      sin.rtp_ok = rtp_order.in_order;
      sin.rtp_gap = rtp_order.fwd_gap;
      sin.rtp_gap_seqs = rtp_order.gap_seqs;
      sin.rtp_back = rtp_order.back;
      sin.udp_sent = udp.sent();
      sin.udp_failed = udp.failed();
      sin.udp_bytes = udp.bytes();
      sin.q_drop = queue.dropped();
      stats->poll(drained_ms, sin);
    }
  }
  queue.close();
  for (auto& fe : fronts) fe->stop();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/maburgs.json";
  std::string in_path, out_rtp_path;
  bool dry_run = false;
  maburgs::FrameFileSource::Options src_opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) config_path = argv[++i];
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--in" && i + 1 < argc) in_path = argv[++i];
    else if (a == "--cards" && i + 1 < argc) src_opt.cards = std::atoi(argv[++i]);
    else if (a == "--drop-pct" && i + 1 < argc) src_opt.drop_pct = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) src_opt.seed = static_cast<uint32_t>(std::atol(argv[++i]));
    else if (a == "--out-rtp" && i + 1 < argc) out_rtp_path = argv[++i];
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "error: unknown arg %s\n", a.c_str()); usage(); return 2; }
  }

  if (!dry_run) {
    // real-radio mode: load config, then run. (Branches off BEFORE the
    // dry-run-only arg checks, exactly where the Plan-1 stub sat.)
    maburgs::Config cfg;
    try { cfg = maburgs::load_config(config_path); }
    catch (const std::exception& e) { std::fprintf(stderr, "error: %s\n", e.what()); return 2; }
    return run_radio(cfg);
  }

  // ---- dry-run path: MUST be byte-identical to Plan 1 ----
  if (in_path.empty()) { usage(); return 2; }
  if (src_opt.cards < 1) {
    std::fprintf(stderr, "error: --cards must be >= 1\n");
    usage();
    return 2;
  }

  maburgs::Config cfg;
  try {
    cfg = maburgs::load_config(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }

  maburgs::FrameFileSource src(in_path, src_opt);
  if (!src.ok()) {
    std::fprintf(stderr, "error: cannot read %s\n", in_path.c_str());
    return 2;
  }

  const int n_cards = src_opt.cards;
  maburgs::Aggregator agg(cfg.uep_layers(),
                          static_cast<uint64_t>(cfg.fec.decode_deadline_ms),
                          static_cast<uint32_t>(cfg.fec.seq_horizon), n_cards);
  RtpFileOut file_out;
  std::unique_ptr<maburgs::UdpSink> udp;
  maburgs::RtpPacketizer::Emit emit;
  if (!out_rtp_path.empty()) {
    if (!file_out.open(out_rtp_path.c_str())) {
      std::fprintf(stderr, "error: cannot write %s\n", out_rtp_path.c_str());
      return 2;
    }
    emit = [&](const std::vector<uint8_t>& pkt) { file_out.write(pkt); };
  } else {
    udp = std::make_unique<maburgs::UdpSink>(cfg.video_out.host, cfg.video_out.port);
    if (!udp->ok())
      std::fprintf(stderr, "warning: video_out %s:%d unusable; decoding anyway\n",
                   cfg.video_out.host.c_str(), cfg.video_out.port);
    emit = [&](const std::vector<uint8_t>& pkt) { udp->send(pkt.data(), pkt.size()); };
  }
  // Same video tail as run_radio (fragments -> FrameStream -> RtpPacketizer),
  // so a replay exercises the real assembly and packetization rather than a
  // dry-run-only shortcut. No session negotiation here: the input file IS the
  // drone's own output, so the format is known.
  maburgs::RtpPacketizer pktz({97, 0x4D414252u, 1400, 16667}, emit);
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video_out.frame_gap_timeout_ms),
       cfg.video_out.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h) { pktz.begin_frame(h); },
       [&](const uint8_t* d, size_t n) { pktz.data(d, n); },
       [&](bool c) { pktz.end_frame(c); }});
  uint64_t replay_ms = 0;  // clock of the body being fed, for gap timeouts
  agg.set_frag_sink([&](const mabur::DecodedFrag& f) {
    fstream.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), replay_ms);
  });
  uint64_t rc_frames = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_frames; });

  uint64_t last_ms = 0;
  while (auto m = src.next()) {
    replay_ms = m->mono_us / 1000;
    agg.on_rx_body(*m);
    const uint64_t now_ms = m->mono_us / 1000;
    fstream.poll(now_ms);
    if (now_ms >= last_ms + 1000) {
      agg.poll(now_ms);
      last_ms = now_ms;
    }
  }
  // Final expiry so abandoned symbols are accounted before the report, then
  // let FrameStream time out whatever is still half-assembled (its gap timeout
  // is what turns an unrecoverable hole into a truncated frame).
  agg.poll(last_ms + static_cast<uint64_t>(cfg.fec.decode_deadline_ms) + 1);
  fstream.poll(last_ms + static_cast<uint64_t>(cfg.fec.decode_deadline_ms) +
               static_cast<uint64_t>(cfg.video_out.frame_gap_timeout_ms) + 1);

  std::fprintf(stderr, "frames=%llu dropped=%llu malformed=%llu rc=%llu bad_card=%llu\n",
               static_cast<unsigned long long>(src.frames_read()),
               static_cast<unsigned long long>(src.dropped()),
               static_cast<unsigned long long>(src.malformed()),
               static_cast<unsigned long long>(rc_frames),
               static_cast<unsigned long long>(agg.bad_card_msgs()));
  for (int c = 0; c < n_cards; ++c) {
    const auto& t = agg.card(c);
    std::fprintf(stderr,
                 "card %d: frames=%llu crc_fail=%llu video=%llu seq %llu/%llu "
                 "rssiA=%.1f rssiB=%.1f snr=%.1f\n",
                 c, static_cast<unsigned long long>(t.frames),
                 static_cast<unsigned long long>(t.crc_fail),
                 static_cast<unsigned long long>(t.video_bodies),
                 static_cast<unsigned long long>(t.seq_received),
                 static_cast<unsigned long long>(t.seq_expected), t.rssi_a_ema,
                 t.rssi_b_ema, t.snr_ema);
  }
  for (int s = 0; s < 4; ++s) {
    const auto st = agg.decoder().stats(s);
    std::fprintf(stderr,
                 "stream %d: bodies=%llu sub_fail=%llu rec=%llu abn=%llu "
                 "pkts=%llu delivery=%d%%\n",
                 s, static_cast<unsigned long long>(st.bodies),
                 static_cast<unsigned long long>(st.subblocks_failed),
                 static_cast<unsigned long long>(st.syms_recovered),
                 static_cast<unsigned long long>(st.syms_abandoned),
                 static_cast<unsigned long long>(st.packets_out),
                 agg.decoder().window_delivery_pct(s));
  }
  std::fprintf(stderr,
               "frames_out: clean=%llu truncated=%llu dropped=%llu bad_frag=%llu\n",
               static_cast<unsigned long long>(fstream.frames_clean()),
               static_cast<unsigned long long>(fstream.frames_truncated()),
               static_cast<unsigned long long>(fstream.frames_dropped()),
               static_cast<unsigned long long>(fstream.bad_fragments()));
  if (!out_rtp_path.empty())
    std::fprintf(stderr, "rtp_out=%llu (file)\n",
                 static_cast<unsigned long long>(file_out.written));
  else
    std::fprintf(stderr, "rtp_out=%llu udp_failed=%llu\n",
                 static_cast<unsigned long long>(udp->sent()),
                 static_cast<unsigned long long>(udp->failed()));
  return 0;
}
