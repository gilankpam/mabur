// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> frame tail
// -> AU records; the original RTP output was deleted in PR C).
// Plan 2 scope: real-radio mode (N-card front-ends, control loop, card failover).
#include <algorithm>
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
#include "gap_timeout_policy.h"
#include "ladder_residual.h"
#include "lat_window.h"
#ifdef MABUR_LOSS_SIM
#include "loss_control.h"
#endif
#include "mabur/probe_wire.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/uep_encoder.h"
#include "msp_sink.h"
#include "pts_anchor.h"
#include "probe_log.h"
#include "probe_track.h"
#include "rcf_slot.h"
#include "rtt_estimator.h"
#include "radio_frontend.h"
#include "rf_labels.h"
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

// Same clock the au ring writer stamps t_first_us/t_complete_us with
// (clock_gettime CLOCK_MONOTONIC) -- steady_clock == CLOCK_MONOTONIC on
// this glibc/Linux target, so this µs value and the ring's µs stamps share
// one timebase and are directly subtractable (see the fec-segment comment
// in run_radio's end_frame lambda).
uint64_t mono_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

void usage() {
  std::fprintf(stderr,
               "usage: maburgs -c <config.json> --dry-run --in <frames.bin>\n"
               "               [--cards N] [--drop-pct P] [--seed S] [--out-aus <file>]\n"
#ifdef MABUR_LOSS_SIM
               "       maburgs -c <config.json> [--loss-sim [port]]\n"
               "\n"
               "  --loss-sim [port]  BENCH ONLY: bind a loopback UDP command\n"
               "                     socket (default port 8302) for injecting\n"
               "                     per-stream loss. Starts at zero; see\n"
               "                     tools/bench/losssim.py.\n"
               "                     Injection is INDEPENDENT PER CARD, so a\n"
               "                     body only reaches the decoder as lost when\n"
               "                     every card drops it: `sN loss=X` sets the\n"
               "                     per-card rate, `sN eff=X` sets the nominal\n"
               "                     union rate (percard^ncards) and solves for\n"
               "                     per-card. Replies always state both.\n"
               "                     `eff` is NOMINAL -- it assumes every card\n"
               "                     heard every body, which only holds on a\n"
               "                     clean link; on a link already losing\n"
               "                     bodies the true injected loss is higher.\n"
               "                     Record real loss from the stats sideport's\n"
               "                     per-stream counters, never from the dial.\n"
#endif
               );
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

#ifdef MABUR_LOSS_SIM
static int run_radio(const maburgs::Config& cfg, int loss_sim_port) {
#else
static int run_radio(const maburgs::Config& cfg) {
#endif
  std::fprintf(stderr, "fec: symbol_size=[%d,%d] seq_horizon=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.seq_horizon);

  // Ladder feasibility log: one line per effective (post-max_mcs-filter)
  // rung, so a boot log alone tells you whether the configured ladder can
  // physically carry the video the encoder is about to be told to produce.
  for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
    const maburgs::Rung& rung = cfg.link.ladder_cfg.ladder[i];
    const auto spec = mabur::rc::ladder_from(mabur::rc::PhyMode::HT,
                                              static_cast<uint8_t>(rung.mcs), 20);
    // Same-rate-fixed-pairs (Task 4): both sids run the same PHY rate now,
    // so one `rate` covers the whole rung; only the per-sid overhead
    // (hence per-sid budget) still differs.
    const double rate = mabur::rc::phy_rate_mbps(spec[0]);
    const double denom = 0.5 * (1 + rung.overhead_base) / rate +
                         0.5 * (1 + rung.overhead_enh) / rate;
    const double src_mbps = 0.65 / denom;
    std::fprintf(stderr,
                 "ladder[%zu]: mcs%d ov %.2f/%.2f budgets=%.0f%%/%.0f%% ~%.1f Mbps src\n",
                 i, rung.mcs, rung.overhead_base, rung.overhead_enh,
                 100.0 * rung.overhead_base / (1 + rung.overhead_base),
                 100.0 * rung.overhead_enh / (1 + rung.overhead_enh), src_mbps);
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
                          static_cast<uint32_t>(cfg.fec.seq_horizon), n_cards);

#ifdef MABUR_LOSS_SIM
  // BENCH RIG (MABUR_LOSS_SIM). Off unless --loss-sim was given; rates
  // always start at zero and are set live, so no config file can carry an
  // injection rate across a reboot.
  maburgs::LossControl loss_ctl;
  if (loss_sim_port > 0) {
    // n_cards is what converts the per-card injection rate into the effective
    // (union) rate the decoder actually sees, so the control socket needs it.
    if (loss_ctl.open(loss_sim_port, agg.n_cards())) {
      std::fprintf(stderr,
                   "maburgs: LOSS-SIM control on udp 127.0.0.1:%d "
                   "(all streams zero, ncards=%d; `loss=` is PER-CARD, `eff=` "
                   "is the NOMINAL union rate = percard^ncards -- nominal "
                   "because it assumes every card heard every body, so record "
                   "real loss from the stats sideport, not from the dial)\n",
                   loss_sim_port, agg.n_cards());
    } else {
      std::fprintf(stderr, "warning: loss-sim port %d unusable; disabled\n",
                   loss_sim_port);
    }
  }
#endif
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

  // Control-path RTT + pts-offset estimator (link-rtt, 2026-09-02). Fed
  // from the same core thread as latest_telem: RCF send stamps below,
  // telem echoes in the rc sink.
  maburgs::RttEstimator rtt_est;
  // RCF slotting (gs-uplink-self-blanking findings 2026-09-02): control
  // frames wait for the end-of-AU callback (FrameStream sink below) so the
  // send lands in the drone's inter-AU idle. See rcf_slot.h.
  maburgs::RcfSlotter rcf_slot(
      maburgs::RcfSlotCfg{cfg.link.rcf_slot_hold_ms, 100, 2, 3, 1});
  // The one place a control frame leaves the GS (direct or via the RCF
  // slotter): card + RTT stamp travel with the frame (SlotFrame).
  auto send_control_frame = [&](const maburgs::SlotFrame& f) {
    static const bool gaplog = std::getenv("MABUR_GAPLOG") != nullptr;
    if (gaplog) {
      const uint64_t now_ms = mono_ms();
      std::fprintf(stderr,
                   "gstx card=%d mono=%llu reason=%d hold=%llu since_au=%llu\n",
                   f.card, static_cast<unsigned long long>(mono_us()),
                   static_cast<int>(f.reason),
                   static_cast<unsigned long long>(now_ms - f.offered_ms),
                   static_cast<unsigned long long>(now_ms - rcf_slot.last_au_ms()));
    }
    fronts[static_cast<size_t>(f.card)]->send_control(f.frame);
    if (f.stamp_rtt) rtt_est.on_rcf_sent(f.seq, mono_us());
  };

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

  // Head-segment latency aggregates (sideport link.video.lat, spec
  // 2026-08-30-latency-accounting Task 10): pts->mono anchor + rolling
  // percentile window, both core-loop-owned like everything else here.
  // cur_au_pts is set in begin_frame and read back in end_frame -- legal
  // because FrameStream's contract pairs begin/end for the SAME AU with no
  // other AU's begin in between (single-threaded core loop).
  maburgs::PtsAnchor lat_anchor;
  maburgs::LatWindow lat_win;
  uint32_t cur_au_pts = 0;

  // Probe stream (spec 2026-09-04 section 3): scored by ProbeTrack against
  // the enh AU count; the ENH layer's geometry gives bpb/block_payload, so
  // the same array the Aggregator was built from decides how a probe body
  // is parsed. Core-thread-owned like every other window in this loop.
  const auto probe_layer = cfg.uep_layers()[1];  // ENH layer geometry
  const int probe_bpb = probe_layer.blocks_per_body;
  const int probe_block_payload =
      static_cast<int>(mabur::sw::kSwHeaderLen) + probe_layer.fec.symbol_size;
  maburgs::ProbeTrack probe_track(maburgs::ProbeTrackCfg{probe_bpb, 100, n_cards});
  maburgs::S1LossWindow probe_loss;  // union, commanded profile
  std::vector<maburgs::S1LossWindow> probe_card_loss(static_cast<size_t>(n_cards));
  uint8_t probe_cmd_last = mabur::rc::kNoProbeProfile;
  // A commanded-profile change blanks the windows: bodies already in flight
  // carry the OLD profile and ProbeTrack stops scoring them, so the window
  // must refill from the new profile's bodies only (RCF lag + finalize).
  constexpr double kProbeSwitchBlankMs = 150.0;
  // Per-body probe log, indexed to the same NNNN as the ctl log so a
  // probe-NNNN/ctl-NNNN pair from one boot lines up.
  std::optional<maburgs::ProbeLog> probe_log;

  // Video tail: FrameStream reassembles whole frames from the raw FRAG
  // fragments the decoder emits; whole access units leave maburgs through
  // the shm AU ring (PR C: the RTP packetizer/UDP path is gone — maburplay
  // is the consumer, ausniff the external gate).
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms),
       cfg.video.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h, uint8_t sid) {
         cur_au_pts = h.pts_us;
         if (au_on) au_ring.begin(h, sid);
         rcf_slot.on_au_first(mono_ms());
         // One probe expectation per ENH access unit: the probe body rides
         // the enh AU's send opportunity, so the AU count is what "expected"
         // means (probe_track.h explains why seq gaps cannot be).
         if (sid == 1) probe_track.on_enh_au(h.frame_id, static_cast<double>(mono_ms()));
       },
       [&](const uint8_t* d, size_t n) {
         if (au_on) au_ring.append(d, n);
       },
       [&](bool c, const maburgs::AuLatMeta& lat) {
         if (au_on) {
           const uint64_t rec = au_ring.finish(c, lat);
           if (rec != UINT64_MAX) au_bell.notify(rec);
         }
         rcf_slot.on_au_complete(mono_ms());
         {
           static const bool gaplog_au = std::getenv("MABUR_GAPLOG") != nullptr;
           if (gaplog_au)
             std::fprintf(stderr, "auc mono=%llu complete=%d\n",
                          static_cast<unsigned long long>(mono_us()), c ? 1 : 0);
         }
         if (stats) stats->on_frame(mono_ms());
         // au_tail gauge (usb-feed probe 2026-09-01): fec = arrival span
         // (last body mono - first body mono) + publish tail (now - last
         // body: repair/decode/assembly/ring write). Core-thread-owned,
         // 5 s stderr windows. Names where the fec residual lives when the
         // air span is known from rx_pace.
         if (lat.t_first_us && lat.t_last_arr_us >= lat.t_first_us) {
           static uint64_t at_n = 0, at_span_sum = 0, at_span_max = 0;
           static uint64_t at_tail_sum = 0, at_tail_max = 0, at_last_rep = 0;
           const uint64_t now = mono_us();
           const uint64_t span = lat.t_last_arr_us - lat.t_first_us;
           const uint64_t tail =
               now > lat.t_last_arr_us ? now - lat.t_last_arr_us : 0;
           ++at_n;
           at_span_sum += span;
           at_tail_sum += tail;
           if (span > at_span_max) at_span_max = span;
           if (tail > at_tail_max) at_tail_max = tail;
           if (at_last_rep == 0) at_last_rep = now;
           if (now - at_last_rep >= 5000000 && at_n > 0) {
             std::fprintf(stderr,
                          "maburgs au_tail: n=%llu span_us mean=%llu max=%llu "
                          "tail_us mean=%llu max=%llu\n",
                          (unsigned long long)at_n,
                          (unsigned long long)(at_span_sum / at_n),
                          (unsigned long long)at_span_max,
                          (unsigned long long)(at_tail_sum / at_n),
                          (unsigned long long)at_tail_max);
             at_n = at_span_sum = at_span_max = 0;
             at_tail_sum = at_tail_max = 0;
             at_last_rep = now;
           }
         }
         // Head-segment latency: t_first_us is the radio's mono stamp on
         // the AU's first body and t_complete_us (via mono_us() below,
         // the "now" at this closure) shares its timebase -- steady_clock
         // == CLOCK_MONOTONIC on this glibc/Linux target, so the two are
         // directly subtractable with no clock-domain conversion.
         if (lat.t_first_us) {
           // Enc-excess air fix (2026-08-31), mirrored in the player's
           // LatTracker::on_submit: the anchor consumes the encode/queue-
           // corrected arrival so its floor means "best post-encoder,
           // post-queue transit"; enc/dq report their full wire values and
           // air is the transit excess. enc + dq + air == t_first - map(pts)
           // exactly (additive invariant unchanged). Corrections come only
           // from FCS-clean bodies and are plausibility-capped; an
           // implausible one withholds the sample rather than dragging the
           // snap-down floor.
           const int64_t adjust = static_cast<int64_t>(lat.enc_us) +
                                  static_cast<int64_t>(lat.drone_q_ms) * 1000;
           if (adjust <= maburgs::PtsAnchor::kMaxAnchorAdjustUs &&
               static_cast<int64_t>(lat.t_first_us) > adjust) {
             const uint64_t adj_arrival =
                 lat.t_first_us - static_cast<uint64_t>(adjust);
             const auto obs = lat_anchor.observe(cur_au_pts, adj_arrival);
             if (!obs.discont && lat_anchor.usable()) {
               const int64_t excess =
                   static_cast<int64_t>(adj_arrival) -
                   static_cast<int64_t>(lat_anchor.map_us(obs.pts64));
               const uint64_t t_done = mono_us();
               const uint32_t fec = static_cast<uint32_t>(
                   t_done > lat.t_first_us ? t_done - lat.t_first_us : 0);
               lat_win.add(lat.enc_us,
                           static_cast<uint32_t>(lat.drone_q_ms) * 1000,
                           static_cast<uint32_t>(excess > 0 ? excess : 0), fec);
             }
           }
         }
       }});
  // Only fragments from a peer that advertised the frame wire format may reach
  // FrameStream: an older drone's bodies carry a mutually unparseable frag
  // header, and feeding them here would produce garbage video rather than an
  // obvious failure. Core-thread-owned, like everything else in this loop.
  bool frame_wire = false;
  bool refused_peer = false;  // one loud line per run, not per tick

  agg.set_frag_sink([&](const mabur::DecodedFrag& f) {
    if (frame_wire)
      fstream.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), mono_ms(),
                            {f.body_mono_us, f.q_ms, f.enc_us});
  });

  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ladder = cfg.link.ladder_cfg;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_overhead_base = cfg.link.static_overhead_base;
  vcfg.pin_overhead_enh = cfg.link.static_overhead_enh;
  vcfg.rcf_repeat_copies = cfg.link.rcf_repeat_copies;
  vcfg.rcf_repeat_ms = cfg.link.rcf_repeat_ms;
  vcfg.probe_pin_mcs = cfg.link.ladder_cfg.probe.pin_mcs;
  maburgs::VrxController vrx(vcfg);

  // Dedicated adaptive-link log (spec 2026-08-05-s3-probe-promote-design.md
  // section 5): maburgs' own compact S/E/P/N record of every rung decision,
  // independent of the stats sideport so the learning dataset survives a
  // dead/absent consumer (2026-08-04: statsrec wasn't running and the
  // flight jsonl froze hours before the session).
  //
  // Opened whenever link.ctl_log is set, INCLUDING static-pin mode
  // (static_mcs >= 0). It used to be skipped there -- a pinned link never
  // ticks the adaptive controller, so there were no rung decisions to
  // record -- but the probe stream changed that (spec 2026-09-04 section
  // 8.3 steps 1-2): the pinned bench runs are exactly the ones whose
  // per-body probe-NNNN.log matters, and ProbeLog takes its NNNN from this
  // CtlLog so the two files pair up per boot.
  //
  // What a pinned S line actually contains: the controller is never
  // updated, so u/util read 0 and E/P/N/R records never fire at all. The
  // three probe columns are `<rung> nan <probe_n>`, and only the last is a
  // measurement. `rung` is probe_rung() off a frozen idx_ == 0 -- pinning
  // does not disable link.probe.enable -- so it prints
  // min(probe.rung_offset, top), i.e. 1 on a normal multi-rung ladder, NOT
  // -1. `u` is nan because the gate never leaves Off. `probe_n` is the real
  // count of expected blocks in the window: with link.probe.pin_mcs >= 0
  // the RCF carries a probe profile, so ProbeTrack books bpb per enh AU and
  // this is nonzero -- exactly the number the pinned bench runs want. It is
  // 0 only when pin_mcs < 0, where nothing commands a probe profile.
  std::optional<maburgs::CtlLog> ctl_log;
  if (cfg.link.ctl_log) {
    std::string header = "ladder=";
    for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
      const maburgs::Rung& r = cfg.link.ladder_cfg.ladder[i];
      if (i) header += ",";
      header += std::to_string(r.mcs) + "/" +
                std::to_string(static_cast<int>(std::lround(r.overhead_base * 100))) +
                ":" +
                std::to_string(static_cast<int>(std::lround(r.overhead_enh * 100)));
    }
    char tail[96];
    std::snprintf(tail, sizeof(tail), " down_util=%.2f up_util=%.2f probe_offset=%d",
                  cfg.link.ladder_cfg.down_util, cfg.link.ladder_cfg.up_util,
                  cfg.link.ladder_cfg.probe.rung_offset);
    header += tail;
    ctl_log.emplace(cfg.link.ctl_log_dir, header);
    if (ctl_log->ok())
      std::fprintf(stderr, "ctl-log: %s\n", ctl_log->path().c_str());
    // else: CtlLog's constructor already printed the ok()=false reason to
    // stderr (opendir/fopen failure) -- non-fatal, logging just stays off.
  }
  if (ctl_log && ctl_log->ok()) {
    probe_log.emplace(cfg.link.ctl_log_dir, ctl_log->index(), probe_bpb);
    if (probe_log->ok())
      std::fprintf(stderr, "probe-log: %s\n", probe_log->path().c_str());
  }

  // Measured-loss ladder feedback: stream 1 (base layer)'s cumulative
  // (expected, arrived) symbol totals, pre-FEC-repair. expected = source
  // symbols ever seen by seq framing (delivered directly + recovered by FEC
  // + abandoned as unrecoverable); arrived = delivered directly PLUS
  // recovered symbols whose direct copy landed afterwards
  // (syms_recovered_arrived) — a repair that merely wins an arrival race is
  // not channel loss. Without that term, rung 0's 2x parity read a clean
  // bench as 19-26% pre-FEC loss and pinned u above up_util forever (stuck
  // at mcs0, 2026-07-27). This is the PRE-FEC window; the post-FEC residual
  // below is a different question over the same symbol counters (what FEC
  // could not repair at all) -- see gs/src/ladder_residual.cpp.
  maburgs::S1LossWindow s1_loss;
  // s3 probe-before-promote feedback (same windowing machinery as s1_loss,
  // stream 3): pre-FEC loss for probe/s3-demote decisions. The steady-state
  // demote path's residual (abandoned/expected) window is s3_resid_cur
  // below -- attribution is unconditional, so there is no total-based
  // sibling left to keep here.
  maburgs::S1LossWindow s3_loss;
  // Current-rung-only siblings (transition attribution, spec 2026-08-14):
  // same machinery, fed from the attributed counters. s1_loss/s3_loss stay
  // as the observability totals (residual_loss, the artifact-rate meter).
  maburgs::S1LossWindow s1_loss_cur, s1_resid_cur, s3_loss_cur, s3_resid_cur;
  // Post-transition settle for s1_resid_cur (the block-4 instant-demote
  // input; see the blank_until call at the sid-0 transition edge-detect).
  // Budget: one 50 ms edge-detect tick + the ~80 ms abandonment-horizon
  // booking lag + margin. Deliberately half of s3_settle_ms: a genuine
  // continuing fade then steps ~200 ms/rung, inside the ~410-440 ms/rung
  // cadence flight-validated 2026-08-14, and nowhere near the broken
  // 50 ms/rung debris cascade this exists to stop.
  const double kResidSettleMs = 150.0;
  // Observability siblings of s1_resid_cur, pooled over both layers: the
  // sideport's link.residual_loss / link.attrib.residual_cur and the ctl
  // log's S-line resid/resid_cur. These MUST be windowed, not cumulative --
  // the decoder's abandonment counters are monotonic since boot, and a
  // lifetime average never returns to zero, which would break
  // flightreport.py's residual-episode detection and turn the player OSD's
  // post-loss row into a number that barely moves during a real burst.
  maburgs::S1LossWindow pool_resid, pool_resid_cur;
  // Per-layer delivery percent (sideport link.layer_delivery_pct), windowed
  // for the same reason.
  std::array<maburgs::S1LossWindow, 2> layer_resid;
  // Fade-trigger staleness gate (spec 2026-08-14 §3, source repointed to
  // the s1+s3 pool 2026-08-15): per-card pooled frame counts snapshotted
  // once per feedback window (same cadence as the prev_pkts_out snapshot
  // below), so "zero
  // s1+s3 frames this window" can NaN the RF labels before they reach the
  // controller. A frozen EMA must never read as a live signal. Snapshotted
  // for EVERY card, not just the chosen one: the choice itself is
  // freshness-gated, so which card is chosen can change between windows
  // and each needs its own baseline.
  std::vector<uint64_t> prev_pool_frames(static_cast<size_t>(n_cards), 0);
  // Scratch for select_label_card(), hoisted so the per-window refill
  // allocates nothing after the first pass.
  std::vector<maburgs::CardLabelInput> label_card_inputs(
      static_cast<size_t>(n_cards));
  // Completed-packet high-water mark at the controller's last non-disc step;
  // the starvation gate diffs against it (see the control step below).
  uint64_t prev_pkts_out = 0;
  // Commanded-MCS edge detect for boundary marking. -1 forces a first mark
  // (which finds cur_mcs unknown -> closed plain-fallback boundary).
  int last_marked_op_mcs = -1;
  double last_marked_op_ov = -1.0;
  int last_marked_enh_mcs = -1;
  // Card the last real RCF/DISC emission went out on; repeat-burst copies
  // reuse it instead of re-running the selector between emissions.
  int last_tx_card = 0;
  // Change-detect on ctl().last_event(): initialize to the pre-any-event
  // default (t_ms 0) so boot doesn't print a phantom transition line.
  double last_ctl_event_ms = vrx.ctl().last_event().t_ms;
  // Same change-detect pattern for the ctl log's probe/penalty records
  // (initialized to the pre-any-event default so boot doesn't print one).
  double last_probe_t_ms = vrx.ctl().last_probe_edge().t_ms;
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
        // link-rtt: every telem is a sync sample. `us` is the radio
        // frontend's steady_clock stamp — same base as the mono_us() send
        // stamps below, so the subtraction is one clock throughout.
        rtt_est.on_telem(t->rcf_seq_echo, (t->flags & 0x08) != 0,
                         t->rcf_age_ms, t->pts_at_build, us);
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

  // Probe stream bodies (SBI stream 5). Same core-thread contract as the MSP
  // sink: the aggregator calls this from the drain loop below. Every body is
  // scored per-card; ProbeTrack merges the cards into the union the ladder
  // reads. The RF labels are this body's own, not the card's EMA -- a probe
  // sample must carry the radio conditions it actually flew through.
  agg.set_probe_sink([&](uint8_t card, const mabur::node::RxBody& m) {
    mabur::probe::ProbeRx rx;
    if (!mabur::probe::parse_probe_body(m.body.data(), m.body.size(),
                                        probe_block_payload, &rx))
      return;
    const double snr = m.phy_valid
                            ? std::max(m.snr[0], m.snr[1]) * maburgs::kSnrRawToDb
                            : std::nan("");
    // EVM: raw half-dB, negative = clean, 0 = not sampled (node.h) -- pick
    // the better sampled chain, NaN when neither chain reported.
    const int8_t evm_raw =
        (m.evm[0] != 0 && (m.evm[1] == 0 || m.evm[0] < m.evm[1])) ? m.evm[0]
                                                                  : m.evm[1];
    const double evm = evm_raw != 0 ? evm_raw * maburgs::kEvmRawToDb : std::nan("");
    probe_track.on_body(card, rx, snr, evm,
                        static_cast<double>(m.mono_us) / 1000.0);
  });

  maburgs::TxSelector sel(
      maburgs::TxSelectorCfg{cfg.radio.tx_card, 3.0, 2000, 1500}, n_cards);

  std::vector<uint64_t> retry_at_ms(static_cast<size_t>(n_cards), 0);
  uint64_t last_stats_ms = 0;
  // Separate from last_stats_ms since 2026-08-15: the ctl log runs on
  // link.ctl_log_period_ms, the stderr line stays at 1 Hz.
  uint64_t last_ctl_sample_ms = 0;
  // Foreign-RC_VERSION warning throttle. Separate `logged` flag rather than a
  // 0 sentinel: mono_ms() is small but not guaranteed non-zero at startup.
  uint64_t last_foreign_rc_ms = 0;
  bool foreign_rc_logged = false;
  std::vector<mabur::node::RxBody> batch;

  // Rate-aware gap timeout: updated 1 Hz from the per-stream seq-advance
  // rate, pushed into FrameStream (gap_timeout_policy.h).
  maburgs::GapTimeoutPolicy gap_policy(cfg.video.frame_gap_timeout_ms,
                                       cfg.video.frame_gap_timeout_max_ms);
  uint64_t gap_update_ms = 0;

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
      // totals AND fed to vrx.on_video() below, holding off the rendezvous
      // video-silence fallback on traffic the decoder never sees. The
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
      // Exclude RC frames, MSP frames (SBI stream_id == kMspStreamId) AND
      // probe frames (stream_id == kProbeStreamId), mirroring the
      // aggregator's routing: only real video may refresh the rendezvous
      // video-silence timer, or a link carrying nothing but MSP telemetry
      // or probe canaries would never fall back to BEACONING. This keeps
      // MSP and probe traffic invisible to the adaptive-link controller
      // end-to-end.
      const int sid_peek =
          mabur::sbi_peek_stream_id(m.body.data(), m.body.size());
      if (m.crc_ok &&
          mabur::rc::frame_type(m.body.data(), m.body.size()) < 0 &&
          sid_peek != mabur::kMspStreamId && sid_peek != mabur::kProbeStreamId) {
        vrx.on_video(static_cast<double>(m.mono_us) / 1000.0);
      }
    }
    // Re-read the clock: the drain above blocked up to 10 ms, and bodies
    // processed in it carry stamps newer than now_ms_u. Timeout/hold math
    // must run on a clock >= every stamp it has seen (the decoder and
    // reorder buffer also guard against stale clocks internally).
    const uint64_t drained_ms = mono_ms();
    if (drained_ms >= gap_update_ms + 1000) {
      gap_update_ms = drained_ms;
      for (int s = 0; s < 2; ++s) {
        gap_policy.update(s, agg.decoder().newest_seq(s),
                          agg.decoder().repair_window(s), drained_ms);
        fstream.set_gap_timeout(
            s, static_cast<uint64_t>(gap_policy.timeout_ms(s)));
      }
    }
#ifdef MABUR_LOSS_SIM
    if (loss_ctl.ok()) loss_ctl.poll(agg.loss_sim());
#endif

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
      lat_anchor.reset();  // new session's pts space is unrelated to the old one's
      // Drop any pre-reset samples too: without this, the anchor re-warms
      // (kWarmFrames) before the next flush, but the window itself still
      // holds up to kCap stale pre-reset samples that would silently mix
      // into that first post-reset flush -- worst right after a reconnect.
      lat_win.clear();
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
    // per-sid MCS. sid 0 (base) mirrors the drone's mcs-1 rule
    // (rc::ladder_from(...)[0].mcs) and always tracks the op; sid 1 (enh)
    // runs the op MCS too -- since the continuous probe stream replaced the
    // discrete probe attempt (2026-09-04), the enh layer is never diverted
    // to a candidate rate; the probe rides its own stream instead.
    // Overhead-only steps mark sid 0 too (FEC re-key debris exists without a
    // PHY change; the decoder then uses the plain same-MCS fallback).
    // Static-pin mode: the op never changes, so nothing ever arms.
    {
      const auto& op_now = vrx.cur_op();
      if (op_now.mcs != last_marked_op_mcs ||
          op_now.overhead_base != last_marked_op_ov) {
        const auto base_spec = mabur::rc::ladder_from(
            op_now.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
            static_cast<uint8_t>(op_now.mcs), static_cast<uint8_t>(op_now.bw))[0];
        agg.decoder().mark_transition(0, static_cast<uint8_t>(base_spec.mcs),
                                      now_ms_u);
        // Block 4's instant-demote window must not outlive the rung it
        // measured: the demote fires on anything > 0, has no settle blank,
        // and is exempt from min_between_changes_ms, so debris left in the
        // 500 ms window re-fires every 50 ms tick until rung 0 (flight
        // ctl-0160: every s1 residual event cascaded 4-5 rungs to the
        // floor, one at 26-32 dB SNR). The settle also swallows the
        // abandonment horizon's ~80 ms late booking of old-rung loss the
        // watermark cannot see (in-flight symbols above the highest
        // old-rate seq). Observability windows (pool_resid*, s1_loss*)
        // stay untouched -- they report, they don't decide.
        s1_resid_cur.blank_until(now_ms + kResidSettleMs);
        last_marked_op_mcs = op_now.mcs;
        last_marked_op_ov = op_now.overhead_base;
      }
      const int enh_mcs_now = op_now.mcs;
      if (enh_mcs_now != last_marked_enh_mcs) {
        agg.decoder().mark_transition(1, static_cast<uint8_t>(enh_mcs_now),
                                      now_ms_u);
        last_marked_enh_mcs = enh_mcs_now;
      }
    }

    // Control step: post-FEC residual from the FEC decoder's own abandonment
    // counters — ONE formula for every consumer since 2026-09-02, see
    // gs/src/ladder_residual.cpp. Per-layer delivery is operator-facing only
    // — it rides the stats sideport below, never the RCF (RC_VERSION 3
    // dropped it; maburd never read it).
    std::array<uint8_t, 2> ld{};
    for (int s = 0; s < 2; ++s) {
      const auto lc =
          maburgs::residual_counts(agg.decoder(), s, /*cur=*/false);
      auto& w = layer_resid[static_cast<size_t>(s)];
      w.add(lc.expected, lc.arrived, now_ms);
      const auto ls = w.sample(now_ms);
      // Idle layer reads 100, matching the deleted window_delivery_pct (the
      // 2026-07 controller-wedge finding in docs/bench-validation.md turns
      // on that convention). Truncating like the old integer ratio did.
      const int pct =
          ls.valid ? static_cast<int>(100.0 * (1.0 - ls.loss)) : 100;
      ld[static_cast<size_t>(s)] =
          static_cast<uint8_t>(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    }
    const auto rc_tot =
        maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/false);
    const auto rc_cur =
        maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/true);
    pool_resid.add(rc_tot.expected, rc_tot.arrived, now_ms);
    pool_resid_cur.add(rc_cur.expected, rc_cur.arrived, now_ms);
    const auto pr = pool_resid.sample(now_ms);
    const auto prc = pool_resid_cur.sample(now_ms);
    std::optional<double> residual;
    if (pr.valid) residual = pr.loss;
    std::optional<double> residual_cur;
    if (prc.valid) residual_cur = prc.loss;
    // Zero completed packets while video frames still arrive = decode
    // collapse; the ladder's video_starved path forces the failsafe rung
    // then rather than trusting this window's (survivor-biased) loss sample.
    // Gate on having ever seen video so a pre-link idle window doesn't count
    // as starvation.
    //
    // packets_out is CUMULATIVE, so this diffs against a snapshot taken at
    // the same boundary the deleted reset_window() used (the controller's
    // last non-disc step) — the residual counters are cumulative too now, so
    // "expected == 0 this window" is no longer a thing the decoder can say.
    const uint64_t pkts_now = agg.decoder().stats(0).packets_out +
                              agg.decoder().stats(1).packets_out;
    const bool starved = (pkts_now == prev_pkts_out) && agg.last_video_us() != 0;

    // s1_* names are historical (predate the 2-stream flatten): this is the
    // BASE sid's (0) window, the critical always-decode layer that drives
    // the ladder's ordinary demote/promote decisions.
    const auto s1 = agg.decoder().stats(0);
    s1_loss.add(s1.syms_delivered + s1.syms_recovered + s1.syms_abandoned,
                s1.syms_delivered + s1.syms_recovered_arrived, now_ms);
    const auto s1_sample = s1_loss.sample(now_ms);

    const uint64_t s1_ab_cur = s1.syms_abandoned - s1.syms_abandoned_stale;
    s1_loss_cur.add(s1.syms_delivered + s1.syms_recovered + s1_ab_cur,
                    s1.syms_delivered + s1.syms_recovered_arrived, now_ms);
    const auto s1_cur_sample = s1_loss_cur.sample(now_ms);

    // Block 4's instant-demote input: BASE post-FEC loss from the FEC
    // decoder's own abandonment count, mirroring s3_resid_cur below. See
    // gs/src/ladder_residual.cpp for why this replaced the packet-level
    // delivery window on 2026-09-02.
    const auto s1_rc = maburgs::ladder_residual_counts(agg.decoder());
    s1_resid_cur.add(s1_rc.expected, s1_rc.arrived, now_ms);
    const auto s1_rcur_sample = s1_resid_cur.sample(now_ms);

    // s3 feedback for the probe-before-promote / s3-demote logic: pre-FEC
    // loss (same shape as s1's window), scored against the CURRENT rung's
    // budget. s3_* names are historical too: this is the ENH sid's (1)
    // window — the shed-able layer the probe candidate rides.
    const auto s3 = agg.decoder().stats(1);
    const uint64_t s3_expected =
        s3.syms_delivered + s3.syms_recovered + s3.syms_abandoned;
    s3_loss.add(s3_expected, s3.syms_delivered + s3.syms_recovered_arrived, now_ms);
    const auto s3_sample = s3_loss.sample(now_ms);

    const uint64_t s3_ab_cur = s3.syms_abandoned - s3.syms_abandoned_stale;
    const uint64_t s3_exp_cur = s3.syms_delivered + s3.syms_recovered + s3_ab_cur;
    s3_loss_cur.add(s3_exp_cur, s3.syms_delivered + s3.syms_recovered_arrived,
                    now_ms);
    s3_resid_cur.add(s3_exp_cur, s3_exp_cur - s3_ab_cur, now_ms);
    const auto s3_cur_sample = s3_loss_cur.sample(now_ms);
    const auto s3_rcur_sample = s3_resid_cur.sample(now_ms);

    // Probe window (spec 2026-09-04 section 3.3): union block counters for
    // the profile currently commanded, windowed by the same machinery as the
    // s1/s3 windows above. Blanked on a commanded-profile change so it
    // refills only from bodies carrying the new profile (RCF lag + the
    // finalize window). Per-card siblings are diagnostics only -- the ladder
    // reads the union, because that is the path production video takes.
    probe_track.tick(now_ms);
    if (const uint8_t pc = vrx.probe_profile(); pc != probe_cmd_last) {
      probe_cmd_last = pc;
      probe_track.set_commanded(pc, now_ms);
      probe_loss.blank_until(now_ms + kProbeSwitchBlankMs);
      for (auto& w : probe_card_loss) w.blank_until(now_ms + kProbeSwitchBlankMs);
    }
    const auto& pu = probe_track.union_counts();
    probe_loss.add(pu.expected_blocks, pu.arrived_blocks, now_ms);
    for (int i = 0; i < n_cards; ++i) {
      const auto& pcnt = probe_track.card_counts(i);
      probe_card_loss[static_cast<size_t>(i)].add(pcnt.expected_blocks,
                                                  pcnt.arrived_blocks, now_ms);
    }
    const auto probe_sample = probe_loss.sample(now_ms);
    // Drain every iteration even with no log open: ProbeTrack's bounded
    // structure is its pending ring, NOT the finalized list -- that is a
    // plain std::vector it appends to and only take_finalized() clears, so
    // skipping the drain would grow it without limit for the life of the
    // process.
    if (probe_log)
      for (const auto& f : probe_track.take_finalized())
        probe_log->row(f.t_ms, f.seq, f.profile & 0x0F, f.enh_fid, f.blocks_ok,
                       f.card_mask, f.snr_db[0], f.snr_db[1], f.evm_db[0],
                       f.evm_db[1]);
    else
      probe_track.take_finalized();

    // The strongest card that ACTUALLY RECEIVED s1-or-s3 this feedback
    // window supplies all three RF labels. Freshness is part of the argmax,
    // not a filter after it (select_label_card, rf_labels.h): a card whose
    // front-end wedged keeps a frozen-high EMA and would otherwise outrank
    // a live sibling forever. -1 = nothing measured this window, so all
    // three labels stay NaN -- inert for the fade trigger, null on the wire.
    for (int i = 0; i < n_cards; ++i) {
      const auto& ct = agg.card(i).rf_pool;
      label_card_inputs[static_cast<size_t>(i)] = maburgs::CardLabelInput{
          ct.has_ema, ct.frames, prev_pool_frames[static_cast<size_t>(i)],
          ct.snr_ema};
    }
    const int best_card = maburgs::select_label_card(label_card_inputs);
    // SNR (label + fade input) and EVM (label only) come from that ONE card,
    // never independently-best across cards, so the three are a coherent
    // single-card snapshot of the same radio at the same instant.
    double rf_snr_db = std::numeric_limits<double>::quiet_NaN();
    double rf_evm_db = std::numeric_limits<double>::quiet_NaN();
    double rf_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
    if (best_card >= 0) {
      const auto& ct = agg.card(best_card).rf_pool;
      // Raw units are devourer's half-dB (snr_units.h); raw - 110 is the
      // exporter's own dBm conversion (stats_exporter.cpp rssi keys).
      rf_snr_db = ct.snr_ema * maburgs::kSnrRawToDb;
      if (ct.evm_has) rf_evm_db = ct.evm_ema * maburgs::kEvmRawToDb;
      rf_rssi_dbm = ct.rssi_ema - 110.0;
    }

    // Every demote input reads the CURRENT-rung (attributed) value; stale
    // transition debris can no longer fire any demote. Unconditional since
    // 2026-08-15. sample_valid / s3_valid / s3_expected_syms stay
    // total-based on purpose: they gate "was there traffic at all", and an
    // all-stale window must still count as feedback (a cur-based valid
    // would un-stamp last_feedback_ms_ and could walk into the blind-side
    // timeout during a long boundary).
    maburgs::LinkHealth health{
        s1_sample.valid,
        s1_cur_sample.valid ? s1_cur_sample.loss : 0.0,
        s1_rcur_sample.valid ? s1_rcur_sample.loss : 0.0,
        starved};
    health.s3_valid = s3_sample.valid;
    health.s3_pre_fec_loss = s3_cur_sample.valid ? s3_cur_sample.loss : 0.0;
    health.s3_residual_loss =
        s3_rcur_sample.valid ? s3_rcur_sample.loss : 0.0;
    health.s3_expected_syms = s3_loss.expected_in_window(now_ms);
    // Probe gate inputs (spec 2026-09-04 section 3.3). probe_rung is the rung
    // the sample was commanded at. The h.probe_rung == pr equality the
    // controller checks against is always true in practice -- both sides
    // read probe_rung() on the same tick -- and is kept only as a cheap
    // invariant check, not the staleness guard: the real guard against a
    // sample surviving a profile switch is the 150 ms
    // probe_loss.blank_until() set at the switch edge plus mark_transition's
    // streak reset.
    health.probe_valid = probe_sample.valid;
    health.probe_loss = probe_sample.valid ? probe_sample.loss : 0.0;
    health.probe_expected_syms = probe_loss.expected_in_window(now_ms);
    health.probe_rung = vrx.ctl().probe_rung();
    health.rf_snr_db = rf_snr_db;
    health.rf_evm_db = rf_evm_db;
    health.rf_rssi_dbm = rf_rssi_dbm;
    if (auto out = vrx.step(now_ms, health)) {
      if (!out->is_disc) {
        prev_pkts_out = pkts_now;  // window == RCF period
        // The RF staleness window and the loss window MUST share this
        // boundary: both are "since the last health the controller acted on".
        for (int i = 0; i < n_cards; ++i)
          prev_pool_frames[static_cast<size_t>(i)] = agg.card(i).rf_pool.frames;
      }
      std::vector<maburgs::CardSnapshot> snaps;
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        snaps.push_back(maburgs::CardSnapshot{
            fronts[static_cast<size_t>(i)]->alive(), t.snr_ema, t.rssi_b_ema,
            t.last_frame_us});
      }
      const int tx = sel.update(snaps, now_ms_u * 1000);
      last_tx_card = tx;
      // link-rtt: build_rcf bumped seq_, so rcf_seq() IS this frame's seq;
      // captured now because the slotter may send it later. DISCs are
      // rendezvous traffic, not RCFs — the drone never ages against them,
      // so they are not matchable sends. The 1 Hz DISC keepalive is slotted
      // like any other send (it killed a PPDU per second when it bypassed).
      maburgs::SlotFrame sf{std::move(out->frame), vrx.rcf_seq(), tx,
                            !out->is_disc};
      if (!rcf_slot.offer(sf, drained_ms, false)) send_control_frame(sf);
    }
    // RCF repeat burst drain (rcf-uplink-loss findings 2026-08-14): extra
    // copies of an op-CHANGING RCF, spaced rcf_repeat_ms, so the drone's
    // 30-50% half-duplex uplink loss doesn't cost a feedback_ms quantum
    // per lost command. Deliberately OUTSIDE the step() block above:
    // repeats are re-sends, not feedback boundaries, so they must not
    // reset the decoder window ("window == RCF period" holds for step()
    // emissions only) and they reuse the card the last real emission
    // selected rather than re-running the selector.
    while (auto rf = vrx.poll_repeat(now_ms)) {
      // Repeats carry fresh seqs (poll_repeat contract), so each is an
      // independently matchable send for the RTT estimator.
      maburgs::SlotFrame sf{std::move(*rf), vrx.rcf_seq(), last_tx_card, true};
      if (!rcf_slot.offer(sf, drained_ms, false)) send_control_frame(sf);
    }
    // Slotted sends whose hold ended (an AU completed in this iteration's
    // drain, or the hold timed out).
    for (const auto& f : rcf_slot.take_due(drained_ms)) send_control_frame(f);
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
    // Probe gate EDGE records (ctllog 10): same t_ms-change detect pattern as
    // the rung-transition line above. Off edges are not logged -- the gate
    // leaving/entering Off just tracks whether a candidate rung exists at
    // all (top rung, feature disabled), which the S line's probe_rung column
    // already carries once per dwell sample. Also skip rung < 0: promoting
    // onto the top rung has no candidate rung to probe, and logging it would
    // read on flightreport as a phantom rung -1.
    if (const auto& pe = vrx.ctl().last_probe_edge(); pe.t_ms != last_probe_t_ms) {
      last_probe_t_ms = pe.t_ms;
      if (ctl_log && pe.state != maburgs::ProbeGateState::Off && pe.rung >= 0)
        ctl_log->probe(pe.t_ms, pe.rung, maburgs::to_string(pe.state), pe.snr_db,
                        pe.u, static_cast<int>(pe.prev_dur_ms), pe.evm_db);
    }
    if (const auto& n = vrx.ctl().last_penalty(); n.t_ms != last_penalty_t_ms) {
      last_penalty_t_ms = n.t_ms;
      if (ctl_log) ctl_log->penalty(n.t_ms, n.rung, n.k, n.until_ms);
    }

    // SIGUSR1 is consumed ONCE and shared: both blocks below honour it, so a
    // dump still produces an off-cadence S line AND a stderr line the way it
    // did when the two were one block.
    const bool dump_now = g_dump.exchange(false);

    // Adaptive-link log, on its OWN cadence (link.ctl_log_period_ms, default
    // 1000). Split from the stderr block below on 2026-08-15: the ctl log is
    // the instrument the ladder is tuned from and wants to run as fast as
    // 50 ms, while the stderr line is human-readable and lands in /tmp, which
    // is tmpfs — running that at 20 Hz would fill RAM for no one's benefit.
    if (ctl_log &&
        (dump_now || now_ms_u - last_ctl_sample_ms >= static_cast<uint64_t>(
                                                          cfg.link.ctl_log_period_ms))) {
      last_ctl_sample_ms = now_ms_u;
      const auto& c = vrx.ctl();
      // measured_rung(), not rung(): every other field in this row is a
      // window measurement, and a demote has already stepped the live rung
      // down by the time we get here (see LadderController::measured_rung()).
      // Probe columns: the gate's candidate rung, its scored utilization
      // (NaN unless the gate actually has a verdict) and the window's block
      // count, so a post-mortem can tell "clean probe" from "no probe data".
      const auto pg = vrx.ctl().probe_gate(now_ms);
      ctl_log->sample(now_ms, c.measured_rung(), c.util(), health.rf_snr_db,
                       residual.value_or(0.0), c.util3(),
                       health.s3_residual_loss, health.rf_evm_db,
                       residual_cur.value_or(0.0), c.fade_drssi(),
                       c.fade_dsnr(), health.rf_rssi_dbm, pg.rung,
                       (pg.state == maburgs::ProbeGateState::Clean ||
                        pg.state == maburgs::ProbeGateState::Lossy)
                           ? pg.u
                           : std::numeric_limits<double>::quiet_NaN(),
                       probe_loss.expected_in_window(now_ms));
      // R lines keep their own, much slower period — they are a store
      // snapshot, not a dwell sample, and must not follow the S cadence.
      if (now_ms - last_rung_log_ms >= cfg.link.rung_log_period_s * 1000.0) {
        last_rung_log_ms = now_ms;
        emit_rung_lines(now_ms);
      }
    }

    // 1 Hz stats line / SIGUSR1 dump.
    if (dump_now || now_ms_u - last_stats_ms >= 1000) {
      last_stats_ms = now_ms_u;
      if (msp_sink) msp_sink->tick(now_ms_u);  // expire stale repair rows
      const auto& op = vrx.cur_op();
      std::fprintf(stderr,
                   "stats: state=%d tx_card=%d op=mcs%d/%d/ov%.2f "
                   "ring=%llu ring_drop=%llu q_drop=%llu",
                   static_cast<int>(vrx.link_state()), sel.selected(), op.mcs,
                   op.bw, op.overhead_base,
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
      for (int s = 0; s < 2; ++s) {
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
#ifdef MABUR_LOSS_SIM
      if (agg.loss_sim().enabled())
        std::fprintf(stderr, " LOSS-SIM[s0/s1/s2/s3/s4/s5]=%llu/%llu/%llu/%llu/%llu/%llu",
                     static_cast<unsigned long long>(agg.loss_sim().dropped(0)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(1)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(2)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(3)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(4)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(5)));
#endif
      std::fprintf(stderr, "\n");
    }

    if (stats) {
      maburgs::StatsInput sin;
      sin.vtx_id = cfg.link.vtx_id;
      sin.in_session = in_session;
      sin.tx_card = sel.selected();
      sin.op = vrx.cur_op();
      for (int s = 0; s < 2; ++s)
        sin.gap_timeout_ms[s] = gap_policy.timeout_ms(s);
      sin.residual_loss = residual;
      sin.residual_cur = residual_cur;
      // The same s1 window the ladder's LinkHealth reads, exported
      // unconditionally: static-pin mode never fills sin.ctl below, and the
      // OSD's pre-FEC LOSS figure has to come from somewhere. Left empty on
      // an invalid window rather than defaulted to 0.0 the way LinkHealth
      // does it -- a controller needs a number every tick, a gauge does not,
      // and "no sample" must not render as a real zero-loss link.
      if (s1_cur_sample.valid) sin.pre_fec_loss = s1_cur_sample.loss;
      if (const double cms = agg.decoder().last_boundary_close_ms(0); cms >= 0)
        sin.attrib_close_ms = cms;
      for (int s = 0; s < 2; ++s)
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
      for (int s = 0; s < 2; ++s) {
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
      sin.rcf_slot = {rcf_slot.released_au(), rcf_slot.released_timeout(),
                      rcf_slot.passthru()};
      // link-rtt block. floor via floor_us_from (pts_anchor.h), which owns
      // the 32-bit-seed vs 64-bit-MI-domain wrap rule.
      if (rtt_est.has_rtt()) {
        maburgs::StatsRttIn ri;
        ri.rtt_ms = rtt_est.rtt_ms();
        ri.rtt_min_ms = rtt_est.rtt_min_ms();
        ri.n = rtt_est.samples();
        if (rtt_est.has_offset()) {
          ri.pts_off_us = rtt_est.pts_off_us();
          if (lat_anchor.usable())
            ri.floor_ms = static_cast<double>(maburgs::floor_us_from(
                              lat_anchor.base_us(), rtt_est.pts_off_us())) /
                          1000.0;
        }
        sin.rtt = ri;
      }
      // lat_win.flush() is destructive (reads AND clears): only pay for it
      // on a poll that stats->due() says will actually emit. This loop
      // iterates roughly every 10ms (the drain() cadence above) while
      // interval_ms is >= 100ms, so an unconditional flush() here would
      // clear the window ~10x more often than it is ever read, reporting
      // only the last loop tick's handful of samples instead of a real
      // rolling window.
      if (lat_anchor.usable() && stats->due(drained_ms))
        sin.video_lat = lat_win.flush();
      // Ladder controller snapshot: absent in static-pin mode, where the
      // controller exists but is never ticked (see VrxController::ctl()).
      if (cfg.link.static_mcs < 0) {
        const auto& c = vrx.ctl();
        maburgs::StatsCtlIn ci;
        ci.rung_idx = c.rung();
        ci.rung_mcs = c.op().mcs;
        ci.rung_ov_base = c.op().overhead_base;
        ci.rung_ov_enh = c.op().overhead_enh;
        ci.util = c.util();
        ci.pre_fec_loss = c.pre_fec_loss();
        ci.budget = c.budget_base();
        ci.probation_ms_left = c.probation_ms_left(now_ms);
        for (const auto& p : c.penalized(now_ms)) ci.penalized.push_back(p);
        for (const auto& r : cfg.link.ladder_cfg.ladder)
          ci.ladder.emplace_back(r.mcs, r.overhead_base, r.overhead_enh);
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
        ci.promotes_probed = cnt.promotes_probed;
        ci.probe_holds = cnt.probe_holds;
        ci.demotes_s3_residual = cnt.demotes_s3_residual;
        ci.demotes_s3_util = cnt.demotes_s3_util;
        ci.demotes_fade = cnt.demotes_fade;
        ci.fade_active = c.fade_active(now_ms);
        ci.fade_drssi = c.fade_drssi();
        ci.fade_dsnr = c.fade_dsnr();
        const maburgs::RungStore& rstore = c.rungs();
        for (std::size_t ri = 0; ri < rstore.size(); ++ri) {
          const maburgs::RungStat& rs = rstore.stat(static_cast<int>(ri));
          maburgs::StatsRungIn rg;
          rg.mcs = cfg.link.ladder_cfg.ladder[ri].mcs;
          rg.ov_base = cfg.link.ladder_cfg.ladder[ri].overhead_base;
          rg.ov_enh = cfg.link.ladder_cfg.ladder[ri].overhead_enh;
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
      // Probe snapshot: OUTSIDE the ladder block on purpose, so static-pin
      // mode (link.probe.pin_mcs on the bench) still exports what the probe
      // stream is doing. In that mode the controller is never ticked, so
      // `state` stays "off" and `u`/`loss` export as JSON null (have_sample
      // needs a non-Off gate). `rung` is NOT -1 there: pinning does not
      // disable link.probe.enable, so probe_rung() off a frozen idx_ == 0
      // reports min(probe.rung_offset, top). The informative fields in pin
      // mode are `on`, `mcs`, `n`, `exp`, `rx`, `off_profile` and the
      // per-card rows -- cards[].loss has its own validity flag and does
      // not depend on the gate.
      {
        maburgs::StatsProbeIn pin;
        const uint8_t pc = vrx.probe_profile();
        pin.on = pc != mabur::rc::kNoProbeProfile;
        pin.mcs = pin.on ? (pc & 0x0F) : -1;
        const auto g = vrx.ctl().probe_gate(now_ms);
        pin.rung = g.rung;
        pin.state = maburgs::to_string(g.state);
        pin.have_sample =
            probe_sample.valid && g.state != maburgs::ProbeGateState::Off;
        pin.u = g.u;
        pin.loss = probe_sample.valid ? probe_sample.loss : 0.0;
        pin.streak_ms = static_cast<int>(g.streak_ms);
        pin.n = probe_loss.expected_in_window(now_ms);
        pin.exp = pu.expected_blocks;
        pin.rx = pu.bodies_rx;
        pin.off_profile = probe_track.off_profile();
        for (int i = 0; i < n_cards; ++i) {
          const auto cs = probe_card_loss[static_cast<size_t>(i)].sample(now_ms);
          pin.cards.push_back({cs.valid, cs.valid ? cs.loss : 0.0,
                               probe_track.card_counts(i).bodies_rx});
        }
        sin.probe = std::move(pin);
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
#ifdef MABUR_LOSS_SIM
  int loss_sim_port = 0;
#endif
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
#ifdef MABUR_LOSS_SIM
    else if (a == "--loss-sim") {
      loss_sim_port = 8302;
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        const int p = std::atoi(argv[i + 1]);
        if (p > 0 && p < 65536) { loss_sim_port = p; ++i; }
      }
    }
#endif
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "error: unknown arg %s\n", a.c_str()); usage(); return 2; }
  }

  if (!dry_run) {
    // real-radio mode: load config, then run. (Branches off BEFORE the
    // dry-run-only arg checks, exactly where the Plan-1 stub sat.)
    maburgs::Config cfg;
    try { cfg = maburgs::load_config(config_path); }
    catch (const std::exception& e) { std::fprintf(stderr, "error: %s\n", e.what()); return 2; }
#ifdef MABUR_LOSS_SIM
    return run_radio(cfg, loss_sim_port);
#else
    return run_radio(cfg);
#endif
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
       [&](bool c, const maburgs::AuLatMeta& lat) {
         if (au_on) {
           const uint64_t rec = au_ring.finish(c, lat);
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
    last_ms = now_ms;
  }
  // Let FrameStream time out whatever is still half-assembled (its gap timeout
  // is what turns an unrecoverable hole into a truncated frame).
  fstream.poll(last_ms + static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms) +
               1);

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
  for (int s = 0; s < 2; ++s) {
    const auto st = agg.decoder().stats(s);
    std::fprintf(stderr,
                 "stream %d: bodies=%llu sub_fail=%llu rec=%llu abn=%llu "
                 "pkts=%llu delivery=%d%%\n",
                 s, static_cast<unsigned long long>(st.bodies),
                 static_cast<unsigned long long>(st.subblocks_failed),
                 static_cast<unsigned long long>(st.syms_recovered),
                 static_cast<unsigned long long>(st.syms_abandoned),
                 static_cast<unsigned long long>(st.packets_out),
                 maburgs::delivery_pct(
                     maburgs::residual_counts(agg.decoder(), s, false)));
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
