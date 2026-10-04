#pragma once
// WebGs — the devourer-free core of the browser ground station (spec
// 2026-09-27-web-gs). Composes the SAME units maburgs's core loop runs
// (Aggregator with one card, FrameStream, GapTimeoutPolicy,
// LinkHealthAssembler, VrxController, RcfSlotter, RttEstimator) so the web
// GS decodes and drives the ladder exactly like maburgs.
//
// Two modes:
//  - Gs: drives the drone link (rendezvous DISC, ladder, slotted RCFs) via
//    Io::send (required: the ctor throws std::invalid_argument without it).
//    Video is decoded only while in SESSION with a peer that advertised
//    CAP_FRAME_WIRE (maburgs main.cpp's `frame_wire`); either edge resets the
//    decoder's continuity and the FrameStream.
//  - Spotter: passive. There is no transmit path by construction: the send
//    callback is dropped in the constructor and no VrxController/RcfSlotter
//    exists. Its op point is fixed at the configured width (no MCS).
//    Video is always decoded; a drone restart (Telem tlm_seq stepping back,
//    DroneRestartDetector) resets the decoder continuity + FrameStream.
// Both modes decode the MSP OSD stream (stream_id 4) into Io::on_osd; it is
// receive-only and independent of the video gate.
//
// Gs with a roster also runs maburgs's `ChannelCore` (gs/src/channel_core.h)
// at the same six seams run_radio() does: card tick + core tick per tick(),
// on_rc_body/on_session_opened in the rc sink, is_link_video/note_video in
// on_rx, note_au_end at every AU end, disc_targets/may_send/disc_for_card/
// note_sent around every send, snapshot() for stats. The scout thread is the
// core's own (a pthread in the browser, spec §5.5/§8). Without a roster
// (replay/tests) Gs sends through Io::send and runs no core.
//
// Single-threaded API: on_rx/tick/stats from one thread, one clock (core
// mono us) for both the RX stamps and tick(). The core's scout thread is the
// only other thread; it reads the clock through the core's clock fns.
#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aggregator.h"
#include "channel_core.h"
#include "config.h"
#include "drone_restart.h"
#include "frame_stream.h"
#include "gap_timeout_policy.h"
#include "link_card.h"
#include "link_health.h"
#include "mabur/node.h"
#include "mabur/rc_proto.h"
#include "msp_sink.h"
#include "osd_screen.h"
#include "rcf_slot.h"
#include "relay_stats.h"
#include "rtt_estimator.h"
#include "stats_exporter.h"
#include "vrx_controller.h"

namespace webgs {

enum class Mode { Gs, Spotter };

struct Au {
  std::vector<uint8_t> data;       // Annex-B, FrameHdr stripped
  uint32_t pts_us = 0;             // drone capture stamp (u32, wraps)
  uint8_t sid = 0;                 // 0 base, 1 enh
  uint8_t flags = 0;               // framewire: 0x01 IDR, 0x02 DISCONT
  bool complete = false;
  uint64_t t_first_us = 0;         // core clock
  uint64_t t_complete_us = 0;      // core clock
  std::optional<int64_t> cap_to_complete_us;  // GS mode once RTT has an offset
};

struct ChanStats {              // GS mode with a roster; mirrors ChannelSnapshot / the sideport
  std::string scan_state;       // off | scouting | moving | frozen
  uint64_t scan_rounds = 0;
  std::optional<int> scan_pick;
  maburgs::StatsHopIn hop;
};

struct Stats {
  Mode mode = Mode::Gs;
  bool session = false;            // rendezvous SESSION (Gs only)
  bool peer_acked = false;
  int rung = -1;                   // commanded (Gs); -1 in Spotter
  int mcs = -1, bw = 0;            // commanded (Gs) / base-stream RX mcs + configured width (Spotter)
  std::string probe_state;         // to_string(ProbeGateState), "" in Spotter
  std::optional<double> pre_fec_loss, residual;
  double snr_db = std::numeric_limits<double>::quiet_NaN();
  double rssi_dbm = std::numeric_limits<double>::quiet_NaN();
  std::optional<double> rtt_ms, rtt_min_ms;
  std::optional<int64_t> pts_off_us;
  uint64_t bodies = 0, aus_complete = 0, aus_truncated = 0, sends = 0;
  // RCFs only (sends minus DISC beacons/keep-alives): the denominator for
  // "RCF heard %" against the drone's Telem.rcf_rx, which counts RCFs only.
  uint64_t rcf_sent = 0;
  std::optional<uint32_t> drone_rcf_rx;   // Telem, cumulative
  std::optional<uint8_t> drone_state;
  std::optional<uint8_t> rec_status;      // Telem.rec_status raw (VTX recorder)
  uint32_t idr_req = 0;                   // page IDR requests so far (set_idr_requests)
  std::optional<int> drone_temp_c;        // Telem.soc_temp_c; unset when -128 (unavailable)
  uint64_t osd_snaps = 0;     // MSP snapshots out of the FEC sink
  uint64_t osd_screens = 0;   // OSD screens published (Io::on_osd)
  // Pairing key (spec 2026-10-01 link-pairing §8): the drone's rendezvous
  // answer told us our key doesn't match its, and the fingerprint of the
  // key this core is configured with (for the page to show beside it).
  bool key_mismatch = false;
  std::string key_fp;
  int channel = 0;              // the card's live channel; start_ch with no roster (replay)
  std::optional<ChanStats> chan;   // Gs with a roster only
};
// One line, no trailing newline. Channel fields: "channel": int;
// "scan_state": str|null; "scan_rounds": int|null; "scan_pick": int|null;
// "hop": {enable, verdict, evidence, ref_rung|null, epoch, state,
// target|null, hops, holds, last_ms|null} | null (null without a core).
std::string stats_json(const Stats& s);

// Validates the page's channel set/start/width with the shared rules: width
// 20|40; the set 1-8 unique members in [1,177], at width 40 every member an
// HT40 pair primary on one offset (mabur::channel_set_issue); start_ch a
// member; in pinned mode start_ch == the pin; GS mode additionally no 40 MHz
// rung while tuned 20 (maburgs::link_width_issue). Spotter only listens, so
// a 20 MHz spotter under a 40-capable ladder is fine (it sees the 20 MHz
// rungs). std::nullopt = OK, else the reason.
std::optional<std::string> channel_width_error(const maburgs::Config& cfg, Mode mode,
                                               int start_ch, int width);

// The page's relay stats fields, appended to each STATS line (keys as the
// 2026-09-29 web relay client emitted them; web/ui/src/lib/view.js
// reads them).
std::string relay_stats_fields(const maburgs::RelayStatsIn& r);

// Capture -> AU complete, core clock, from the drone's u32 pts and the
// RTT estimator's pts offset (pts - GS-mono). Modular in 32 bits: valid
// while the true latency is < 2^31 us.
int64_t cap_to_complete_us(uint32_t pts32, uint64_t t_complete_us, int64_t pts_off_us);

struct Io {
  std::function<void(Au&&)> on_au;
  // Gs only: one RC body (VrxController output) to wrap + transmit. Never
  // stored in Spotter mode.
  std::function<void(const std::vector<uint8_t>& rc_body)> send;
  // Optional, both modes: once per tick() after the control step.
  // `sent` is the LAST frame sent this tick (a tick can send more than one:
  // a direct send plus slotter releases), nullptr when none went out.
  std::function<void(double now_ms, const maburgs::LinkHealth&, int rung,
                     const std::vector<uint8_t>* sent)> on_control_tick;
  // Optional, both modes: one MSP OSD screen (spec 2026-09-27-web-msp-osd),
  // at most every 30 ms. rows x cols cells, row-major, char | page << 8.
  std::function<void(int rows, int cols, const uint16_t* cells)> on_osd;
  // Gs with a roster: the core's stderr-style lines ("maburgs channel: ...",
  // "maburgs hop: ..."), exact text as maburgs prints them. The glue prints them.
  // Called from the core thread or the scout thread.
  std::function<void(const std::string& line)> on_log;
  // Gs with a roster: the remembered channel changed (ChannelCore's store);
  // the glue prints `CHANNEL <n>`, the page stores it and passes it back as --ch.
  std::function<void(uint8_t ch)> on_channel_store;
};

struct Opts {
  bool adaptive_gap = true;  // false = fixed cfg.video.frame_gap_timeout_ms
                             // (AU parity with maburgs --dry-run, which has no policy)
  // false = the ChannelCore runs without threads (tests): tick() runs one scout
  // step itself, the core's clock is WebGs's tick clock, and a scout sleep
  // ADVANCES that clock. true (default) = the scout thread, real clock, real sleeps.
  bool core_threads = true;
  // Tests: called from the core's SleepFn after the clock advance (or the
  // real sleep), with the slept ms -- keeps a test's own clock (a fake
  // card's energy clock) in step with the core's.
  std::function<void(int ms)> sleep_hook;
};

class WebGs {
 public:
  // cards: the roster (USB first, then relays), n_usb of them USB. They
  // outlive this object: the core's threads are joined in the destructor,
  // before the caller stops the cards. Empty = no radio (replay/tests) -> Gs
  // mode sends through Io::send and runs no core. Spotter never sends; its
  // cards only report the live channel.
  WebGs(const maburgs::Config& cfg, Mode mode, uint8_t start_ch, int width,
        std::vector<maburgs::LinkCard*> cards, int n_usb, Io io, Opts opts = {});
  ~WebGs();
  WebGs(const WebGs&) = delete;
  WebGs& operator=(const WebGs&) = delete;
  void on_rx(const mabur::node::RxBody& m);   // m.mono_us = core clock
  void tick(uint64_t now_us);                 // same clock as on_rx stamps
  Stats stats() const;
  // VTX onboard recorder wish (spec 2026-09-27-web-ui §3.3), RecControl
  // semantics: the RCF byte stays 0 until the first call, then
  // kRecKnown | (on ? kRecOn : 0). Spotter: no-op (no VrxController).
  void set_vtx_rec(bool on);
  // GS-requested IDR (spec 2026-09-28): the page's cumulative request count.
  // Its low byte is the RCF idr_epoch; the drone serves one IDR per change.
  // Spotter: kept for stats only (no VrxController, nothing sent).
  void set_idr_requests(uint32_t n);
  Mode mode() const { return mode_; }
  // Test seams.
  const maburgs::VrxController* vrx() const { return vrx_.get(); }  // nullptr in Spotter
  const maburgs::ChannelCore* channel_core() const { return chan_.get(); }   // nullptr without a core
  const maburgs::LinkHealthAssembler& health() const { return lha_; }
  uint64_t sends() const { return sends_; }
  // Video-tail resets so far (session edges in Gs, drone restarts in Spotter).
  uint64_t resets() const { return resets_; }
  // replay only: synthesize the drone's DISC_ACK (as run_hop_inject_test does)
  void inject_disc_ack_for_replay(uint64_t now_us);

 private:
  void send_(const maburgs::SlotFrame& f);
  void reset_video_();
  const Mode mode_;
  Io io_;
  const Opts opts_;
  // Newest clock seen (on_rx stamp or tick). Atomic: the core's clock fns
  // read it (core_threads = false) and may be called off the API thread.
  std::atomic<uint64_t> now_us_{0};
  maburgs::Aggregator agg_;
  maburgs::FrameStream fs_;
  maburgs::GapTimeoutPolicy gap_;
  uint64_t gap_update_ms_ = 0;
  maburgs::LinkHealthAssembler lha_;
  OsdScreen osd_;                                  // MSP OSD, both modes
  std::unique_ptr<maburgs::MspSink> msp_;          // null when [msp] enable = false
  uint64_t msp_tick_ms_ = 0;
  std::unique_ptr<maburgs::VrxController> vrx_;   // Gs only
  std::unique_ptr<maburgs::RcfSlotter> slot_;     // Gs only
  maburgs::RttEstimator rtt_;
  // Gs: in SESSION && peer CAP_FRAME_WIRE (edge-reset). Spotter: always true.
  bool frame_wire_ = false;
  maburgs::DroneRestartDetector restart_;         // Spotter only
  maburgs::LinkHealth last_health_;
  maburgs::OpPoint spotter_op_;                   // fixed: configured width only
  int air_mcs_ = -1;                              // last base-stream RX mcs (Spotter readout)
  std::optional<mabur::rc::Telem> telem_;
  Au cur_;
  uint64_t bodies_ = 0, aus_complete_ = 0, aus_truncated_ = 0, sends_ = 0, rcf_sent_ = 0;
  uint64_t resets_ = 0;
  uint32_t idr_req_ = 0;
  std::string key_fp_;   // mabur::key_fingerprint(cfg.link.key), set at construction

  // ---- roster + channel core (Gs with cards) ----
  std::vector<maburgs::LinkCard*> cards_;
  int n_usb_ = 0;
  const uint8_t start_ch_;
  struct Sink;                                      // ChannelSink -> Io::on_log; records ignored (no scan.log here)
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<maburgs::ChannelCore> chan_;     // last member: destroyed (threads joined) first
  void sleep_(int ms);                              // the core's SleepFn (see Opts::core_threads)
  uint64_t now_us_for_core_() const;               // the core's clock (see Opts::core_threads)
  uint64_t now_ms_for_core_() const;
};

}  // namespace webgs
