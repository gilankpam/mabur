// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> frame tail
// -> AU records; the original RTP output was deleted in PR C).
// Plan 2 scope: real-radio mode (N-card front-ends, control loop, card failover).
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "aggregator.h"
#include "au_doorbell.h"
#include "au_ring.h"
#include "body_queue.h"
#include "config.h"
#include "ctl_log.h"
#include "frame_file_source.h"
#include "frame_stream.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/uep_encoder.h"
#include "msp_sink.h"
#include "radio_frontend.h"
#include "s1_labels.h"
#include "s1_loss.h"
#include "snr_units.h"
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
               "               [--cards N] [--drop-pct P] [--seed S] [--out-aus <file>]\n");
}

// Dry-run AU capture for the e2e: one LP record per reassembled AU --
// u32 total_len | u8 sid | u8 flags (framewire idr|discont, bit 0x04 =
// complete here (NOT the ring's 0x80 -- this LP format is local to the
// dry-run/e2e pair)) | u32 pts_us | Annex-B bytes. Parsed by
// tests/integration/verify_aus.py; keep the two in sync.
struct AuFileOut {
  FILE* f = nullptr;
  uint64_t written = 0;
  std::vector<uint8_t> au;
  mabur::framewire::FrameHdr hdr{};
  uint8_t sid = 0;
  bool in_au = false;
  bool open(const char* path) { f = fopen(path, "wb"); return f != nullptr; }
  ~AuFileOut() { if (f) fclose(f); }
  void begin(const mabur::framewire::FrameHdr& h, uint8_t s) {
    if (!f) return;
    hdr = h; sid = s; au.clear(); in_au = true;
  }
  void append(const uint8_t* d, size_t n) {
    if (f && in_au) au.insert(au.end(), d, d + n);
  }
  void finish(bool complete) {
    // Host-endian fwrite of the u32 fields; verify_aus.py unpacks "<I".
    // Fine on every LE host in play (the dry-run only runs on the x86-64
    // dev box); explicit LE serialization needed if that ever changes.
    if (!f || !in_au) return;
    in_au = false;
    const uint32_t len = static_cast<uint32_t>(au.size());
    const uint8_t flags = static_cast<uint8_t>(hdr.flags | (complete ? 0x04 : 0));
    fwrite(&len, 4, 1, f);
    fwrite(&sid, 1, 1, f);
    fwrite(&flags, 1, 1, f);
    fwrite(&hdr.pts_us, 4, 1, f);
    if (len) fwrite(au.data(), 1, len, f);
    ++written;
  }
};

static int run_radio(const maburgs::Config& cfg) {
  std::fprintf(stderr,
               "fec: symbol_size=[%d,%d,%d,%d] decode_deadline_ms=%d seq_horizon=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.symbol_size[2], cfg.fec.symbol_size[3],
               cfg.fec.decode_deadline_ms, cfg.fec.seq_horizon);

  // Ladder feasibility log: one line per effective (post-max_mcs-filter)
  // rung, so a boot log alone tells you whether the configured ladder can
  // physically carry the video the encoder is about to be told to produce.
  for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
    const maburgs::Rung& rung = cfg.link.ladder_cfg.ladder[i];
    const auto spec = mabur::rc::ladder_from(mabur::rc::PhyMode::HT,
                                              static_cast<uint8_t>(rung.mcs), 20)[1];
    const double eff1 = mabur::uep_layer_overhead(1, rung.overhead);
    const double budget = eff1 / (1 + eff1);
    const double src_mbps =
        mabur::rc::phy_rate_mbps(spec) * 0.65 / (1 + eff1);
    std::fprintf(stderr,
                 "ladder[%zu]: mcs%d ov%.2f eff1=%.2f budget=%.0f%% ~%.1f Mbps src\n",
                 i, rung.mcs, rung.overhead, eff1, budget * 100.0, src_mbps);
  }

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
  // Stats sideport (spec: docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
  // Declared here (ahead of the FrameStream construction below)
  // so the FrameStream end_frame lambda can capture `stats` by reference; both
  // must also outlive every lambda that captures them.
  // session id: nonzero random u32 so consumers detect restarts.
  // UdpSink has a user-declared destructor and a deleted copy constructor
  // with no declared move constructor, so it has neither -- std::vector
  // growth (even reserve() on an empty vector) instantiates a copy/move
  // path unconditionally and fails to compile against it. unique_ptr sidesteps
  // that: the vector moves pointers, never UdpSink objects.
  std::vector<std::unique_ptr<maburgs::UdpSink>> stats_udp;
  std::optional<maburgs::StatsExporter> stats;
  if (cfg.stats.enable) {
    stats_udp.reserve(cfg.stats.out.size());
    for (const auto& o : cfg.stats.out)
      stats_udp.push_back(std::make_unique<maburgs::UdpSink>(o.host, o.port));
    uint32_t session = 0;
    std::random_device rd;
    while (session == 0) session = rd();
    stats.emplace(session, cfg.stats.interval_ms,
                  [&stats_udp](const std::string& s) {
                    // Every destination gets the same buffer. One failing
                    // sink must not stop the others -- a dead consumer is
                    // not a reason to blind the live ones.
                    bool any = false;
                    for (auto& u : stats_udp)
                      if (u->send(reinterpret_cast<const uint8_t*>(s.data()), s.size()))
                        any = true;
                    return any;
                  });
    for (const auto& o : cfg.stats.out)
      std::fprintf(stderr, "maburgs: stats sideport -> udp %s:%d every %d ms\n",
                   o.host.c_str(), o.port, cfg.stats.interval_ms);
  }

  // Drone telemetry (T_TELEM): display-only, not rendezvous traffic — held
  // here for the DRONE display region rather than forwarded to the vrx
  // controller. Core-thread-owned, like everything else in this loop.
  // rx_ms is the GS-side mono stamp the aggregator carries on every rc frame.
  // Spec 2026-07-26 drone-telemetry.
  struct { std::optional<mabur::rc::Telem> t; uint64_t rx_ms = 0; } latest_telem;

  maburgs::AuRingWriter au_ring;
  maburgs::AuDoorbell au_bell;
  bool au_on = false;
  if (cfg.au_ring.enable) {
    const maburgs::AuRingGeom geom{
        static_cast<uint32_t>(cfg.au_ring.slot_kb) * 1024u,
        static_cast<uint32_t>(cfg.au_ring.slot_count)};
    au_on = au_ring.open(cfg.au_ring.path, geom);
    if (au_on && !au_bell.open(cfg.au_ring.socket, au_ring.geom()))
      std::fprintf(stderr, "warning: au_ring doorbell %s unusable\n",
                   cfg.au_ring.socket.c_str());
    if (!au_on)
      std::fprintf(stderr, "warning: au_ring %s unusable; disabled\n",
                   cfg.au_ring.path.c_str());
  } else {
    // PR C: the ring IS the video output. A disabled ring means every
    // reassembled frame is decoded and thrown away -- legal for FEC-only
    // bench work, but never silently.
    std::fprintf(stderr,
                 "warning: au_ring disabled -- NO video output (frames are "
                 "reassembled and discarded)\n");
  }

  // Video tail: FrameStream reassembles whole frames from the raw FRAG
  // fragments the decoder emits; whole access units leave maburgs through
  // the shm AU ring (PR C: the RTP packetizer/UDP path is gone — maburplay
  // is the consumer, ausniff the external gate).
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms),
       cfg.video.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h, uint8_t sid) {
         if (au_on) au_ring.begin(h, sid);
       },
       [&](const uint8_t* d, size_t n) {
         if (au_on) au_ring.append(d, n);
       },
       [&](bool c) {
         if (au_on) {
           const uint64_t rec = au_ring.finish(c);
           if (rec != UINT64_MAX) au_bell.notify(rec);
         }
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

  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ladder = cfg.link.ladder_cfg;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_overhead = cfg.link.static_overhead;
  maburgs::VrxController vrx(vcfg);

  // Dedicated adaptive-link log (spec 2026-08-05-s3-probe-promote-design.md
  // section 5): maburgs' own compact S/E/P/N record of every rung decision,
  // independent of the stats sideport so the learning dataset survives a
  // dead/absent consumer (2026-08-04: statsrec wasn't running and the
  // flight jsonl froze hours before the session). static_mcs >= 0 bypasses
  // the adaptive controller entirely (LinkCfg::static_mcs) -- nothing to
  // log in that mode.
  std::optional<maburgs::CtlLog> ctl_log;
  if (cfg.link.ctl_log && cfg.link.static_mcs < 0) {
    std::string header = "ladder=";
    for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
      const maburgs::Rung& r = cfg.link.ladder_cfg.ladder[i];
      if (i) header += ",";
      header += std::to_string(r.mcs) + "/" +
                std::to_string(static_cast<int>(std::lround(r.overhead * 100)));
    }
    char tail[64];
    std::snprintf(tail, sizeof(tail), " down_util=%.2f up_util=%.2f",
                  cfg.link.ladder_cfg.down_util, cfg.link.ladder_cfg.up_util);
    header += tail;
    ctl_log.emplace(cfg.link.ctl_log_dir, header);
    if (ctl_log->ok())
      std::fprintf(stderr, "ctl-log: %s\n", ctl_log->path().c_str());
    // else: CtlLog's constructor already printed the ok()=false reason to
    // stderr (opendir/fopen failure) -- non-fatal, logging just stays off.
  }

  // Measured-loss ladder feedback: stream 1 (base layer)'s cumulative
  // (expected, arrived) symbol totals, pre-FEC-repair. expected = source
  // symbols ever seen by seq framing (delivered directly + recovered by FEC
  // + abandoned as unrecoverable); arrived = delivered directly PLUS
  // recovered symbols whose direct copy landed afterwards
  // (syms_recovered_arrived) — a repair that merely wins an arrival race is
  // not channel loss. Without that term, rung 0's 2x parity read a clean
  // bench as 19-26% pre-FEC loss and pinned u above up_util forever (stuck
  // at mcs0, 2026-07-27). Not the same window as window_counts()'s post-FEC
  // residual below: this one is symbol-level and time-windowed
  // (S1LossWindow), that one is packet-level and RCF-period-windowed.
  maburgs::S1LossWindow s1_loss;
  // s3 probe-before-promote feedback (same windowing machinery as s1_loss,
  // stream 3): pre-FEC loss for probe/s3-demote decisions, plus a separate
  // residual (abandoned/expected) window for the steady-state demote path.
  maburgs::S1LossWindow s3_loss, s3_resid;
  // Current-rung-only siblings (transition attribution, spec 2026-08-14):
  // same machinery, fed from the attributed counters. The originals stay
  // as the observability totals and the link.attrib=false decision path.
  maburgs::S1LossWindow s1_loss_cur, s3_loss_cur, s3_resid_cur;
  // Fade-trigger staleness gate (spec 2026-08-14 §3): per-card s1-class
  // frame counts snapshotted once per feedback window (same cadence as
  // reset_window()), so "zero s1 frames this window" can NaN the RF labels
  // before they reach the controller. A frozen EMA must never read as a
  // live signal. Snapshotted for EVERY card, not just the chosen one: the
  // choice itself is freshness-gated, so which card is chosen can change
  // between windows and each needs its own baseline.
  std::vector<uint64_t> prev_s1_cls_frames(static_cast<size_t>(n_cards), 0);
  // Scratch for select_s1_label_card(), hoisted so the per-window refill
  // allocates nothing after the first pass.
  std::vector<maburgs::S1CardLabelInput> s1_label_cards(
      static_cast<size_t>(n_cards));
  // Windows where total residual was positive but attributed residual was
  // zero at a rung > 0 — i.e. windows where the legacy instant demote
  // would have fired on pure debris. THE headline artifact-rate number.
  // Counted per contiguous suppressed episode, edge-detected — the loop
  // runs per drained batch, far faster than the RCF window cadence, so a
  // level-triggered increment here would count one episode tens of times.
  uint64_t attrib_suppressed = 0;
  bool attrib_suppressed_latched = false;
  // Commanded-MCS edge detect for boundary marking. -1 forces a first mark
  // (which finds cur_mcs unknown -> closed plain-fallback boundary).
  int last_marked_op_mcs = -1;
  double last_marked_op_ov = -1.0;
  int last_marked_s3_mcs = -1;
  // Change-detect on ctl().last_event(): initialize to the pre-any-event
  // default (t_ms 0) so boot doesn't print a phantom transition line.
  double last_ctl_event_ms = vrx.ctl().last_event().t_ms;
  // Same change-detect pattern for the ctl log's probe/penalty records
  // (initialized to the pre-any-event default so boot doesn't print one).
  double last_probe_t_ms = vrx.ctl().last_probe().t_ms;
  double last_penalty_t_ms = vrx.ctl().last_penalty().t_ms;
  double last_rung_log_ms = 0;
  // R-line snapshot of every rung with any data (spec 2026-08-13).
  auto emit_rung_lines = [&](double t_ms) {
    if (!ctl_log) return;
    const maburgs::RungStore& st = vrx.ctl().rungs();
    for (int i = 0; i < static_cast<int>(st.size()); ++i) {
      const maburgs::RungStat& rs = st.stat(i);
      if (rs.u.n == 0 && rs.probe_u.n == 0) continue;
      const double age_s =
          rs.last_sample_ms < 0 ? -1.0 : (t_ms - rs.last_sample_ms) / 1000.0;
      const double sd = rs.evm_n ? std::sqrt(rs.evm_var_db2)
                                 : std::numeric_limits<double>::quiet_NaN();
      ctl_log->rung(t_ms, i, rs.u.v, rs.resid.v, rs.u3.v, rs.s3_resid.v,
                     rs.evm_db, sd, rs.u.n, age_s, rs.probe_u.v,
                     rs.probe_u.n);
    }
  };
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>& f, uint64_t us) {
    if (mabur::rc::frame_type(f.data(), f.size()) == mabur::rc::T_TELEM) {
      // A CRC-clean frame can still fail to parse as a valid Telem (e.g. a
      // corrupted T_TELEM whose CRC happens to pass this layer but whose
      // internal fields don't parse) — only overwrite the holder on success,
      // so a bad frame leaves the last good telemetry (and its rx_ms stamp)
      // untouched rather than clobbering it with nullopt.
      if (auto t = mabur::rc::parse_telem(f.data(), f.size())) {
        latest_telem.t = t;
        latest_telem.rx_ms = us / 1000;
      }
      return;
    }
    vrx.on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
  });

  std::unique_ptr<maburgs::UdpSink> msp_udp;
  std::unique_ptr<maburgs::MspSink> msp_sink;
  if (cfg.msp.enable) {
    const int bp = cfg.msp.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen);
    msp_udp = std::make_unique<maburgs::UdpSink>(cfg.msp.out_host, cfg.msp.out_port);
    maburgs::MspSink::EmitFn emit = [&](const uint8_t* d, size_t n) { msp_udp->send(d, n); };
    std::fprintf(stderr,
                 "maburgs: MSP OSD -> udp %s:%d symbol_size=%d window=%d block_payload=%d\n",
                 cfg.msp.out_host.c_str(), cfg.msp.out_port, cfg.msp.symbol_size,
                 cfg.msp.window, bp);
    msp_sink = std::make_unique<maburgs::MspSink>(cfg.msp.symbol_size, cfg.msp.window, emit);
    agg.set_msp_sink([&](const uint8_t* b, size_t n, uint64_t us) {
      msp_sink->on_body(b, n, us / 1000);
    });
  }

  maburgs::TxSelector sel(
      maburgs::TxSelectorCfg{cfg.radio.tx_card, 3.0, 2000, 1500}, n_cards);

  std::vector<uint64_t> retry_at_ms(static_cast<size_t>(n_cards), 0);
  uint64_t last_stats_ms = 0;
  // Foreign-RC_VERSION warning throttle. Separate `logged` flag rather than a
  // 0 sentinel: mono_ms() is small but not guaranteed non-zero at startup.
  uint64_t last_foreign_rc_ms = 0;
  bool foreign_rc_logged = false;
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
      // A drone at a different RC_VERSION is invisible to every RC path here:
      // frame_type() returns -1 for it, which is this loop's affirmative "this
      // is video" signal, so its T_TELEM/DISC_ACK is counted into the card
      // totals AND fed to vrx.on_video() below, contaminating
      // ScoreWindow::seq_gap_loss() with seqs the decoder never sees. The
      // visible end state is no video at all, indistinguishable from the
      // stale-caps restart deadlock. Log it (rate-limited to 1/5s: a
      // mismatched peer transmits continuously and /tmp is tmpfs) and change
      // nothing else -- the frame is still handled exactly as before.
      //
      // Gated on crc_ok because RC_MAGIC is only two bytes: roughly 1 in 65536
      // corrupt video bodies matches it by chance, and a corrupt frame must
      // not raise a version-mismatch alarm.
      if (m.crc_ok &&
          mabur::rc::is_foreign_rc_version(m.body.data(), m.body.size()) &&
          (!foreign_rc_logged || now_ms_u - last_foreign_rc_ms >= 5000)) {
        foreign_rc_logged = true;
        last_foreign_rc_ms = now_ms_u;
        std::fprintf(stderr,
                     "maburgs: heard an RC frame at RC_VERSION %u but this "
                     "build speaks %u, so no RC path can read it and it is "
                     "being miscounted as video (rate-limited to 1/5s). "
                     "The pair is half-deployed: no control link and, because "
                     "DISC_ACK carries CAP_FRAME_WIRE, no video either. "
                     "Finish the deploy on BOTH ends; restarting maburd will "
                     "not help.\n",
                     static_cast<unsigned>(m.body[2]),
                     static_cast<unsigned>(mabur::rc::RC_VERSION));
      }
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
    if (au_on) au_bell.poll();

    // Transition boundaries for loss attribution: edge-detect the COMMANDED
    // per-stream MCS. Streams 0-2 follow the op; s3 runs the probe
    // candidate while a probe is up. Overhead-only steps mark too (FEC
    // re-key debris exists without a PHY change; the decoder then uses the
    // plain same-MCS fallback). Static-pin mode: the op never changes, so
    // nothing ever arms.
    {
      const auto& op_now = vrx.cur_op();
      if (op_now.mcs != last_marked_op_mcs ||
          op_now.overhead != last_marked_op_ov) {
        for (int s = 0; s <= 2; ++s)
          agg.decoder().mark_transition(s, static_cast<uint8_t>(op_now.mcs),
                                        now_ms_u);
        last_marked_op_mcs = op_now.mcs;
        last_marked_op_ov = op_now.overhead;
      }
      const int s3_mcs_now =
          vrx.ctl().probing() ? vrx.ctl().probe_mcs() : op_now.mcs;
      if (s3_mcs_now != last_marked_s3_mcs) {
        agg.decoder().mark_transition(3, static_cast<uint8_t>(s3_mcs_now),
                                      now_ms_u);
        last_marked_s3_mcs = s3_mcs_now;
      }
    }

    // Control step: layer delivery + residual from the decode window, plus
    // stream 1's cumulative symbol totals feeding the measured-loss window.
    std::array<uint8_t, 4> ld{};
    for (int s = 0; s < 4; ++s)
      ld[static_cast<size_t>(s)] =
          static_cast<uint8_t>(agg.decoder().window_delivery_pct(s));
    auto [d0, e0] = agg.decoder().window_counts(0);
    auto [d1, e1] = agg.decoder().window_counts(1);
    std::optional<double> residual;
    if (e0 + e1 > 0)
      residual = 1.0 - static_cast<double>(d0 + d1) / static_cast<double>(e0 + e1);
    auto [d0c, e0c] = agg.decoder().window_counts_cur(0);
    auto [d1c, e1c] = agg.decoder().window_counts_cur(1);
    std::optional<double> residual_cur;
    if (e0c + e1c > 0)
      residual_cur =
          1.0 - static_cast<double>(d0c + d1c) / static_cast<double>(e0c + e1c);
    // Zero completed base-layer packets while video frames still arrive =
    // decode collapse; the ladder's video_starved path forces the failsafe
    // rung then rather than trusting this window's (survivor-biased) loss
    // sample. Gate on having ever seen video so a pre-link idle window
    // doesn't count as starvation.
    const bool starved = (e0 + e1 == 0) && agg.last_video_us() != 0;

    const auto s1 = agg.decoder().stats(1);
    s1_loss.add(s1.syms_delivered + s1.syms_recovered + s1.syms_abandoned,
                s1.syms_delivered + s1.syms_recovered_arrived, now_ms);
    const auto s1_sample = s1_loss.sample(now_ms);

    const uint64_t s1_ab_cur = s1.syms_abandoned - s1.syms_abandoned_stale;
    s1_loss_cur.add(s1.syms_delivered + s1.syms_recovered + s1_ab_cur,
                    s1.syms_delivered + s1.syms_recovered_arrived, now_ms);
    const auto s1_cur_sample = s1_loss_cur.sample(now_ms);

    // s3 feedback for the probe-before-promote / s3-demote logic: pre-FEC
    // loss (same shape as s1's window) plus a residual (abandoned/expected)
    // window scored against the CURRENT rung's s3 budget.
    const auto s3 = agg.decoder().stats(3);
    const uint64_t s3_expected =
        s3.syms_delivered + s3.syms_recovered + s3.syms_abandoned;
    s3_loss.add(s3_expected, s3.syms_delivered + s3.syms_recovered_arrived, now_ms);
    s3_resid.add(s3_expected, s3_expected - s3.syms_abandoned, now_ms);
    const auto s3_sample = s3_loss.sample(now_ms);
    const auto s3_rsample = s3_resid.sample(now_ms);

    const uint64_t s3_ab_cur = s3.syms_abandoned - s3.syms_abandoned_stale;
    const uint64_t s3_exp_cur = s3.syms_delivered + s3.syms_recovered + s3_ab_cur;
    s3_loss_cur.add(s3_exp_cur, s3.syms_delivered + s3.syms_recovered_arrived,
                    now_ms);
    s3_resid_cur.add(s3_exp_cur, s3_exp_cur - s3_ab_cur, now_ms);
    const auto s3_cur_sample = s3_loss_cur.sample(now_ms);
    const auto s3_rcur_sample = s3_resid_cur.sample(now_ms);

    // The strongest card that ACTUALLY RECEIVED s1 this feedback window
    // supplies all three s1 RF labels. Freshness is part of the argmax, not a
    // filter after it (select_s1_label_card, s1_labels.h): a card whose
    // front-end wedged keeps a frozen-high EMA and would otherwise outrank a
    // live sibling forever. -1 = nothing measured s1 this window, so all
    // three labels stay NaN — inert for the fade trigger, null on the wire.
    for (int i = 0; i < n_cards; ++i) {
      const auto& ct = agg.card(i).cls[static_cast<size_t>(maburgs::RfClass::S1)];
      s1_label_cards[static_cast<size_t>(i)] = maburgs::S1CardLabelInput{
          ct.has_ema, ct.frames, prev_s1_cls_frames[static_cast<size_t>(i)],
          ct.snr_ema};
    }
    const int s1_best_card = maburgs::select_s1_label_card(s1_label_cards);
    // SNR (label + fade input) and EVM (label only) come from that ONE card,
    // never independently-best across cards, so the three are a coherent
    // single-card snapshot of the same radio at the same instant.
    double s1_snr_db = std::numeric_limits<double>::quiet_NaN();
    double s1_evm_db = std::numeric_limits<double>::quiet_NaN();
    double s1_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
    if (s1_best_card >= 0) {
      const auto& ct =
          agg.card(s1_best_card).cls[static_cast<size_t>(maburgs::RfClass::S1)];
      // Raw units are devourer's half-dB (snr_units.h); raw - 110 is the
      // exporter's own dBm conversion (stats_exporter.cpp rssi keys).
      s1_snr_db = ct.snr_ema * maburgs::kSnrRawToDb;
      if (ct.evm_has) s1_evm_db = ct.evm_ema * maburgs::kEvmRawToDb;
      s1_rssi_dbm = ct.rssi_ema - 110.0;
    }

    // link.attrib=true: the controller judges the rung it is on — every
    // demote input reads the CURRENT-rung (attributed) value; stale
    // transition debris can no longer fire any demote. false: the legacy
    // totals, verbatim. sample_valid / s3_valid / s3_expected_syms stay
    // total-based on purpose: they gate "was there traffic at all", and an
    // all-stale window must still count as feedback (a cur-based valid
    // would un-stamp last_feedback_ms_ and could walk into the blind-side
    // timeout during a long boundary).
    const bool attrib = cfg.link.ladder_cfg.attrib;
    maburgs::LinkHealth health{
        s1_sample.valid,
        attrib ? (s1_cur_sample.valid ? s1_cur_sample.loss : 0.0)
               : s1_sample.loss,
        attrib ? residual_cur.value_or(0.0) : residual.value_or(0.0),
        starved};
    health.s3_valid = s3_sample.valid;
    health.s3_pre_fec_loss = attrib
                                 ? (s3_cur_sample.valid ? s3_cur_sample.loss : 0.0)
                                 : s3_sample.loss;
    health.s3_residual_loss =
        attrib ? (s3_rcur_sample.valid ? s3_rcur_sample.loss : 0.0)
               : (s3_rsample.valid ? s3_rsample.loss : 0.0);
    health.s3_expected_syms = s3_loss.expected_in_window(now_ms);
    health.s1_snr_db = s1_snr_db;
    health.s1_evm_db = s1_evm_db;
    health.s1_rssi_dbm = s1_rssi_dbm;
    // WHICH card those three came from: the argmax above re-runs every window
    // and does not stick, so the source can hop to a weaker sibling and step
    // both labels down together — the fade trigger's joint condition exactly.
    // The controller re-baselines its EWMAs on a change (review finding
    // 2026-08-14). -1 (no card measured s1) rides along with the NaN labels
    // and is deliberately NOT treated as a hop.
    health.s1_label_card = s1_best_card;
    // Artifact-rate meter: a window the legacy instant demote would have
    // acted on (total residual > 0, rung > 0) that the attributed view
    // reads clean. Counted regardless of the switch so an attrib=false
    // run still measures what attribution WOULD have suppressed. Edge-
    // detected on the latch above: one increment per contiguous suppressed
    // episode, not once per main-loop pass.
    const bool attrib_suppressed_now = vrx.ctl().rung() > 0 &&
                                        residual.value_or(0.0) > 0.0 &&
                                        residual_cur.value_or(0.0) <= 0.0;
    if (attrib_suppressed_now && !attrib_suppressed_latched) ++attrib_suppressed;
    attrib_suppressed_latched = attrib_suppressed_now;

    if (auto out = vrx.step(now_ms, ld, health)) {
      if (!out->is_disc) {
        agg.decoder().reset_window();  // window == RCF period
        // The RF staleness window and the loss window MUST share this
        // boundary: both are "since the last health the controller acted on".
        for (int i = 0; i < n_cards; ++i)
          prev_s1_cls_frames[static_cast<size_t>(i)] =
              agg.card(i).cls[static_cast<size_t>(maburgs::RfClass::S1)].frames;
      }
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
    // ctl: rung transition line — load-bearing for post-mortems (Task 6
    // adds the sideport link.ctl block; this stderr line is independent of
    // it and persists in /tmp/maburgs.log even when no sideport consumer is
    // listening).
    if (const auto& e = vrx.ctl().last_event(); e.t_ms != last_ctl_event_ms) {
      last_ctl_event_ms = e.t_ms;
      std::fprintf(stderr, "ctl: rung %d->%d reason=%s u=%.2f pre=%.3f\n",
                   e.from, e.to, maburgs::to_string(e.reason), e.u,
                   health.pre_fec_loss);
      if (ctl_log)
        ctl_log->event(e.t_ms, e.from, e.to, maburgs::to_string(e.reason),
                        e.u, e.snr_db, e.evm_db);
      if (ctl_log) emit_rung_lines(now_ms);  // store state at the decision
    }
    // s3 probe-before-promote records: same t_ms-change detect pattern as
    // the rung-transition line above.
    if (const auto& p = vrx.ctl().last_probe(); p.t_ms != last_probe_t_ms) {
      last_probe_t_ms = p.t_ms;
      if (ctl_log)
        ctl_log->probe(p.t_ms, p.rung, maburgs::to_string(p.outcome),
                        p.snr_db, p.u_pred, p.dur_ms, p.evm_db);
    }
    if (const auto& n = vrx.ctl().last_penalty(); n.t_ms != last_penalty_t_ms) {
      last_penalty_t_ms = n.t_ms;
      if (ctl_log) ctl_log->penalty(n.t_ms, n.rung, n.k, n.until_ms);
    }

    // 1 Hz stats line / SIGUSR1 dump.
    if (g_dump.exchange(false) || now_ms_u - last_stats_ms >= 1000) {
      last_stats_ms = now_ms_u;
      if (msp_sink) msp_sink->tick(now_ms_u);  // expire stale repair rows
      if (ctl_log) {
        const auto& c = vrx.ctl();
        ctl_log->sample(now_ms, c.rung(), c.util(), health.s1_snr_db,
                         residual.value_or(0.0), c.util3(),
                         health.s3_residual_loss, health.s1_evm_db,
                         residual_cur.value_or(0.0), c.fade_drssi(),
                         c.fade_dsnr());
        if (now_ms - last_rung_log_ms >= cfg.link.rung_log_period_s * 1000.0) {
          last_rung_log_ms = now_ms;
          emit_rung_lines(now_ms);
        }
      }
      const auto& op = vrx.cur_op();
      std::fprintf(stderr,
                   "stats: state=%d tx_card=%d op=mcs%d/%d/ov%.2f "
                   "ring=%llu ring_drop=%llu q_drop=%llu",
                   static_cast<int>(vrx.link_state()), sel.selected(), op.mcs,
                   op.bw, op.overhead,
                   static_cast<unsigned long long>(au_ring.published()),
                   static_cast<unsigned long long>(au_ring.dropped_oversize()),
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
                     " s%d[p=%llu abn=%llu rec=%llu ra=%llu si=%llu st=%llu"
                     " bc=%llu sbf=%llu fl=%zu]",
                     s, static_cast<unsigned long long>(st.packets_out),
                     static_cast<unsigned long long>(st.syms_abandoned),
                     static_cast<unsigned long long>(st.syms_recovered),
                     static_cast<unsigned long long>(st.syms_recovered_arrived),
                     static_cast<unsigned long long>(st.symbols_in),
                     static_cast<unsigned long long>(st.symbols_stale),
                     static_cast<unsigned long long>(st.symbols_bad_cfg),
                     static_cast<unsigned long long>(st.subblocks_failed),
                     st.rows_in_flight);
      }
      std::fprintf(stderr, " mis=%llu",
                   static_cast<unsigned long long>(agg.decoder().bodies_misrouted()));
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
      sin.attrib_on = cfg.link.ladder_cfg.attrib;
      sin.attrib_suppressed = attrib_suppressed;
      sin.residual_cur = residual_cur;
      if (const double cms = agg.decoder().last_boundary_close_ms(1); cms >= 0)
        sin.attrib_close_ms = cms;
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
        ci.tx_frames = fronts[static_cast<size_t>(i)]->tx_frames();
        ci.tx_fail = fronts[static_cast<size_t>(i)]->tx_fail();
        static_assert(maburgs::kNumStatsClasses == maburgs::kNumRfClasses,
                      "class arrays must stay in lockstep");
        for (int k = 0; k < maburgs::kNumStatsClasses; ++k) {
          auto& cls = ci.classes[static_cast<size_t>(k)];
          const auto& tcls = t.cls[static_cast<size_t>(k)];
          cls.frames = tcls.frames;
          cls.bytes = tcls.bytes;
          cls.has_ema = tcls.has_ema;
          cls.rssi_ema = tcls.rssi_ema;
          cls.rssi_a_ema = tcls.rssi_a_ema;
          cls.rssi_b_ema = tcls.rssi_b_ema;
          cls.snr_ema = tcls.snr_ema;
          cls.snr_a_ema = tcls.snr_a_ema;
          cls.snr_b_ema = tcls.snr_b_ema;
          cls.evm_ema = tcls.evm_ema;
          cls.evm_a_ema = tcls.evm_a_ema;
          cls.evm_b_ema = tcls.evm_b_ema;
          cls.evm_has = tcls.evm_has;
          cls.evm_a_has = tcls.evm_a_has;
          cls.evm_b_has = tcls.evm_b_has;
        }
        sin.cards.push_back(ci);
      }
      for (int s = 0; s < 4; ++s) {
        const auto st = agg.decoder().stats(s);
        auto& o = sin.streams[static_cast<size_t>(s)];
        o.bodies = st.bodies;
        o.subblocks_failed = st.subblocks_failed;
        o.syms_recovered = st.syms_recovered;
        o.syms_recovered_arrived = st.syms_recovered_arrived;
        o.syms_abandoned = st.syms_abandoned;
        o.syms_abandoned_stale = st.syms_abandoned_stale;
        o.symbols_in = st.symbols_in;
        o.symbols_stale = st.symbols_stale;
        o.symbols_bad_cfg = st.symbols_bad_cfg;
        o.rows_in_flight = st.rows_in_flight;
      }
      sin.frames_clean = fstream.frames_clean();
      sin.frames_truncated = fstream.frames_truncated();
      sin.frames_dropped = fstream.frames_dropped();
      sin.stall_resets = fstream.stall_resets();
      sin.ring_published = au_ring.published();
      sin.ring_dropped_oversize = au_ring.dropped_oversize();
      sin.ring_bytes = au_ring.bytes_published();
      sin.q_drop = queue.dropped();
      sin.telem = latest_telem.t;
      sin.telem_rx_ms = latest_telem.rx_ms;
      // Ladder controller snapshot: absent in static-pin mode, where the
      // controller exists but is never ticked (see VrxController::ctl()).
      if (cfg.link.static_mcs < 0) {
        const auto& c = vrx.ctl();
        maburgs::StatsCtlIn ci;
        ci.rung_idx = c.rung();
        ci.rung_mcs = c.op().mcs;
        ci.rung_ov = c.op().overhead;
        ci.util = c.util();
        ci.pre_fec_loss = c.pre_fec_loss();
        ci.budget = c.budget();
        ci.probation_ms_left = c.probation_ms_left(now_ms);
        for (const auto& p : c.penalized(now_ms)) ci.penalized.push_back(p);
        for (const auto& r : cfg.link.ladder_cfg.ladder)
          ci.ladder.emplace_back(r.mcs, r.overhead);
        ci.down_util = cfg.link.ladder_cfg.down_util;
        ci.up_util = cfg.link.ladder_cfg.up_util;
        const auto& cnt = c.counters();
        ci.demotes_residual = cnt.demotes_residual;
        ci.demotes_util = cnt.demotes_util;
        ci.promotes = cnt.promotes;
        ci.probation_fails = cnt.probation_fails;
        ci.starved_drops = cnt.starved_drops;
        ci.timeout_drops = cnt.timeout_drops;
        const auto& e = c.last_event();
        ci.last_event_t_ms = e.t_ms;
        ci.last_event_from = e.from;
        ci.last_event_to = e.to;
        ci.last_event_reason = maburgs::to_string(e.reason);
        ci.last_event_u = e.u;
        ci.last_event_snr_db = e.snr_db;
        ci.last_event_evm_db = e.evm_db;
        ci.util3 = c.util3();
        ci.probes_started = cnt.probes_started;
        ci.probes_ok = cnt.probes_ok;
        ci.probe_fails = cnt.probe_fails;
        ci.probe_aborts = cnt.probe_aborts;
        ci.demotes_s3_residual = cnt.demotes_s3_residual;
        ci.demotes_s3_util = cnt.demotes_s3_util;
        ci.demotes_fade = cnt.demotes_fade;
        ci.fade_active = c.fade_active(now_ms);
        ci.fade_drssi = c.fade_drssi();
        ci.fade_dsnr = c.fade_dsnr();
        const auto& pr = c.last_probe();
        ci.last_probe_t_ms = pr.t_ms;
        ci.last_probe_rung = pr.rung;
        ci.last_probe_outcome = maburgs::to_string(pr.outcome);
        ci.last_probe_snr_db = pr.snr_db;
        ci.last_probe_evm_db = pr.evm_db;
        ci.last_probe_u_pred = pr.u_pred;
        ci.last_probe_dur_ms = pr.dur_ms;
        const maburgs::RungStore& rstore = c.rungs();
        for (std::size_t ri = 0; ri < rstore.size(); ++ri) {
          const maburgs::RungStat& rs = rstore.stat(static_cast<int>(ri));
          maburgs::StatsRungIn rg;
          rg.mcs = cfg.link.ladder_cfg.ladder[ri].mcs;
          rg.ov = cfg.link.ladder_cfg.ladder[ri].overhead;
          rg.u = rs.u.v;
          rg.resid = rs.resid.v;
          rg.u3 = rs.u3.v;
          rg.resid3 = rs.s3_resid.v;
          rg.evm_db = rs.evm_db;
          rg.evm_sd_db = rs.evm_n
                              ? std::sqrt(rs.evm_var_db2)
                              : std::numeric_limits<double>::quiet_NaN();
          rg.n = rs.u.n;
          rg.probe_n = rs.probe_u.n;
          rg.age_s = rs.last_sample_ms < 0
                          ? -1.0
                          : (now_ms - rs.last_sample_ms) / 1000.0;
          rg.probe_age_s = rs.last_probe_ms < 0
                                ? -1.0
                                : (now_ms - rs.last_probe_ms) / 1000.0;
          rg.dwell_s = rstore.dwell_ms(static_cast<int>(ri), now_ms) / 1000.0;
          rg.visits = rs.visits;
          rg.exits_bad = rs.exits_bad;
          rg.probe_u = rs.probe_u.v;
          ci.rungs.push_back(rg);
        }
        sin.ctl = std::move(ci);
      }
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
  std::string in_path, out_aus_path;
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
    else if (a == "--out-aus" && i + 1 < argc) out_aus_path = argv[++i];
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
  AuFileOut file_out;
  if (!out_aus_path.empty() && !file_out.open(out_aus_path.c_str())) {
    std::fprintf(stderr, "error: cannot write %s\n", out_aus_path.c_str());
    return 2;
  }
  maburgs::AuRingWriter au_ring;
  maburgs::AuDoorbell au_bell;
  bool au_on = false;
  if (cfg.au_ring.enable) {
    const maburgs::AuRingGeom geom{
        static_cast<uint32_t>(cfg.au_ring.slot_kb) * 1024u,
        static_cast<uint32_t>(cfg.au_ring.slot_count)};
    au_on = au_ring.open(cfg.au_ring.path, geom);
    if (au_on && !au_bell.open(cfg.au_ring.socket, au_ring.geom()))
      std::fprintf(stderr, "warning: au_ring doorbell %s unusable\n",
                   cfg.au_ring.socket.c_str());
    if (!au_on)
      std::fprintf(stderr, "warning: au_ring %s unusable; disabled\n",
                   cfg.au_ring.path.c_str());
  }

  // Same video tail as run_radio (fragments -> FrameStream -> AU records),
  // so a replay exercises the real assembly rather than a dry-run-only
  // shortcut. --out-aus captures each reassembled AU as an LP record for
  // the e2e's NAL-exact comparison (tests/integration/verify_aus.py). No
  // session negotiation here: the input file IS the drone's own output.
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms),
       cfg.video.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h, uint8_t sid) {
         if (au_on) au_ring.begin(h, sid);
         file_out.begin(h, sid);
       },
       [&](const uint8_t* d, size_t n) {
         if (au_on) au_ring.append(d, n);
         file_out.append(d, n);
       },
       [&](bool c) {
         if (au_on) {
           const uint64_t rec = au_ring.finish(c);
           if (rec != UINT64_MAX) au_bell.notify(rec);
         }
         file_out.finish(c);
       }});
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
    if (au_on) au_bell.poll();
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
               static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms) + 1);

  if (au_on)
    std::fprintf(stderr, "au_ring: published=%llu dropped_oversize=%llu\n",
                 static_cast<unsigned long long>(au_ring.published()),
                 static_cast<unsigned long long>(au_ring.dropped_oversize()));

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
  if (!out_aus_path.empty())
    std::fprintf(stderr, "aus_out=%llu (file)\n",
                 static_cast<unsigned long long>(file_out.written));
  return 0;
}
