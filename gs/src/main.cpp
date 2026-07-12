// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> RTP out).
// Plan 2 scope: real-radio mode (N-card front-ends, control loop, card failover).
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "aggregator.h"
#include "body_queue.h"
#include "config.h"
#include "frame_file_source.h"
#include "mabur/rc_proto.h"
#include "op_table.h"
#include "radio_frontend.h"
#include "rtp_reorder.h"
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
                          static_cast<uint64_t>(cfg.fec.block_max_age_ms), n_cards);
  maburgs::UdpSink udp(cfg.video_out.host, cfg.video_out.port);
  // RTP order health of the emitted stream (bench 2026-07-13): packets
  // leave on FEC-block completion, so ordering is NOT guaranteed by
  // construction — a live decoder discards late/reordered RTP that the
  // transport counters happily count as delivered. seq16 from the RTP
  // header; fwd_gap = skipped-ahead seqs (missing-at-emit or reorder),
  // back = packets emitted behind the highest seq seen (late emissions).
  struct RtpOrder {
    bool has_last = false;
    uint16_t last = 0;
    uint64_t in_order = 0, fwd_gap = 0, back = 0, gap_seqs = 0;
  } rtp_order;
  // Reorder buffer between the FEC decoder and the UDP sink; the order
  // tracker sits AFTER it, so ord[] in the stats line reports the health of
  // the stream the decoder actually receives.
  maburgs::RtpReorder reorder(
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
      },
      // End-to-end latency budget: decoder block_max_age (device config,
      // ~250ms) < this hold, so a block that completes at its age limit
      // still beats the reorder deadline instead of landing in late_dropped.
      /*hold_ms=*/300);
  agg.set_rtp_sink([&](const mabur::DecodedRtp& r) {
    reorder.push(r.pkt, mono_ms());
  });

  maburgs::LinkTable lt;
  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ctrl.src_bitrate_bps = cfg.link.src_bitrate_mbps * 1e6;
  vcfg.ctrl.margin_db = cfg.link.margin_db;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_overhead = cfg.link.static_overhead;
  vcfg.pin_txagc = cfg.link.static_txagc;
  maburgs::VrxController vrx(lt, vcfg);
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>& f, uint64_t us) {
    vrx.on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
  });

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
      if (m.crc_ok &&
          mabur::rc::frame_type(m.body.data(), m.body.size()) < 0) {
        // Video frame meta -> score window (max-chain values, Python parity).
        const double rssi = static_cast<double>(std::max(m.rssi[0], m.rssi[1]));
        const double snr = static_cast<double>(std::max(m.snr[0], m.snr[1]));
        vrx.on_video(rssi, snr, false, m.mac_seq,
                     static_cast<double>(m.mono_us) / 1000.0);
      }
    }
    agg.poll(now_ms_u);
    reorder.poll(now_ms_u);

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
      const auto& op = vrx.cur_op();
      std::fprintf(stderr,
                   "stats: state=%d tx_card=%d op=mcs%d/%d/ov%.2f/agc%d "
                   "rtp=%llu udp_fail=%llu q_drop=%llu",
                   static_cast<int>(vrx.link_state()), sel.selected(), op.mcs,
                   op.bw, op.overhead, op.txagc,
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
        std::fprintf(stderr, " s%d[p=%llu u=%llu fe=%llu]", s,
                     static_cast<unsigned long long>(st.packets_out),
                     static_cast<unsigned long long>(st.blocks_unrecoverable),
                     static_cast<unsigned long long>(st.frag_evicted));
      }
      std::fprintf(stderr, " ord[ok=%llu gap=%llu(+%llu) back=%llu buf=%zu skip=%llu late=%llu]",
                   static_cast<unsigned long long>(rtp_order.in_order),
                   static_cast<unsigned long long>(rtp_order.fwd_gap),
                   static_cast<unsigned long long>(rtp_order.gap_seqs),
                   static_cast<unsigned long long>(rtp_order.back),
                   reorder.depth(),
                   static_cast<unsigned long long>(reorder.skipped()),
                   static_cast<unsigned long long>(reorder.late_dropped()));
      std::fprintf(stderr, "\n");
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
                          static_cast<uint64_t>(cfg.fec.block_max_age_ms), n_cards);
  RtpFileOut file_out;
  std::unique_ptr<maburgs::UdpSink> udp;
  if (!out_rtp_path.empty()) {
    if (!file_out.open(out_rtp_path.c_str())) {
      std::fprintf(stderr, "error: cannot write %s\n", out_rtp_path.c_str());
      return 2;
    }
    agg.set_rtp_sink([&](const mabur::DecodedRtp& r) { file_out.write(r.pkt); });
  } else {
    udp = std::make_unique<maburgs::UdpSink>(cfg.video_out.host, cfg.video_out.port);
    if (!udp->ok())
      std::fprintf(stderr, "warning: video_out %s:%d unusable; decoding anyway\n",
                   cfg.video_out.host.c_str(), cfg.video_out.port);
    agg.set_rtp_sink([&](const mabur::DecodedRtp& r) {
      udp->send(r.pkt.data(), r.pkt.size());
    });
  }
  uint64_t rc_frames = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_frames; });

  uint64_t last_ms = 0;
  while (auto m = src.next()) {
    agg.on_rx_body(*m);
    const uint64_t now_ms = m->mono_us / 1000;
    if (now_ms >= last_ms + 1000) {
      agg.poll(now_ms);
      last_ms = now_ms;
    }
  }
  // Final expiry so unrecoverable blocks are accounted before the report.
  agg.poll(last_ms + static_cast<uint64_t>(cfg.fec.block_max_age_ms) + 1);

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
                 "rssiB=%.1f snr=%.1f\n",
                 c, static_cast<unsigned long long>(t.frames),
                 static_cast<unsigned long long>(t.crc_fail),
                 static_cast<unsigned long long>(t.video_bodies),
                 static_cast<unsigned long long>(t.seq_received),
                 static_cast<unsigned long long>(t.seq_expected), t.rssi_b_ema,
                 t.snr_ema);
  }
  for (int s = 0; s < 4; ++s) {
    const auto st = agg.decoder().stats(s);
    std::fprintf(stderr,
                 "stream %d: bodies=%llu sub_fail=%llu blocks=%llu unrec=%llu "
                 "pkts=%llu delivery=%d%%\n",
                 s, static_cast<unsigned long long>(st.bodies),
                 static_cast<unsigned long long>(st.subblocks_failed),
                 static_cast<unsigned long long>(st.blocks_decoded),
                 static_cast<unsigned long long>(st.blocks_unrecoverable),
                 static_cast<unsigned long long>(st.packets_out),
                 agg.decoder().window_delivery_pct(s));
  }
  if (!out_rtp_path.empty())
    std::fprintf(stderr, "rtp_out=%llu (file)\n",
                 static_cast<unsigned long long>(file_out.written));
  else
    std::fprintf(stderr, "rtp_out=%llu udp_failed=%llu\n",
                 static_cast<unsigned long long>(udp->sent()),
                 static_cast<unsigned long long>(udp->failed()));
  return 0;
}
