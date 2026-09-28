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
//    exists. The op point shown is the drone's own applied one (Telem).
//    Video is always decoded; a drone restart (Telem tlm_seq stepping back,
//    DroneRestartDetector) resets the decoder continuity + FrameStream.
// Both modes decode the MSP OSD stream (stream_id 4) into Io::on_osd; it is
// receive-only and independent of the video gate.
//
// Single-threaded: on_rx/tick/stats from one thread, one clock (core mono
// us) for both the RX stamps and tick().
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aggregator.h"
#include "config.h"
#include "drone_restart.h"
#include "frame_stream.h"
#include "gap_timeout_policy.h"
#include "link_health.h"
#include "mabur/node.h"
#include "mabur/rc_proto.h"
#include "msp_sink.h"
#include "osd_screen.h"
#include "rcf_slot.h"
#include "rtt_estimator.h"
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

struct Stats {
  Mode mode = Mode::Gs;
  bool session = false;            // rendezvous SESSION (Gs only)
  bool peer_acked = false;
  int rung = -1;                   // commanded (Gs); -1 in Spotter
  int mcs = -1, bw = 0;            // commanded (Gs) / drone-applied (Spotter)
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
  std::optional<uint16_t> drone_idr_gs;   // Telem.idr_gs: requests the drone served
  uint64_t osd_snaps = 0;     // MSP snapshots out of the FEC sink
  uint64_t osd_screens = 0;   // OSD screens published (Io::on_osd)
};
std::string stats_json(const Stats& s);   // one line, no trailing newline

// Validates the page's channel/width override (spec: ch/w come from the
// page, not the config) with maburgs's own loader checks
// (maburgs::radio_width_issue / link_width_issue): channel in the loader's
// [1,200], width 20|40, 40 only on an HT40 pair, and -- GS mode, which
// commands the ladder -- no 40 MHz rung / static pin while tuned 20.
// Spotter only listens, so a 20 MHz spotter under a 40-capable ladder is
// fine (it sees the 20 MHz rungs). std::nullopt = OK, else the reason.
std::optional<std::string> channel_width_error(const maburgs::Config& cfg, Mode mode,
                                               int channel, int width);

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
};

struct Opts {
  bool adaptive_gap = true;  // false = fixed cfg.video.frame_gap_timeout_ms
                             // (AU parity with maburgs --dry-run, which has no policy)
};

class WebGs {
 public:
  WebGs(const maburgs::Config& cfg, Mode mode, uint8_t channel, int width, Io io,
        Opts opts = {});
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
  const uint32_t vtx_id_;
  uint64_t now_us_ = 0;          // newest clock seen (on_rx stamp or tick)
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
  maburgs::OpPoint spotter_op_;                   // from drone Telem
  std::optional<mabur::rc::Telem> telem_;
  Au cur_;
  uint64_t bodies_ = 0, aus_complete_ = 0, aus_truncated_ = 0, sends_ = 0, rcf_sent_ = 0;
  uint64_t resets_ = 0;
  uint32_t idr_req_ = 0;
};

}  // namespace webgs
