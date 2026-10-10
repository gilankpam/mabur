#include "web_gs.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

#include "json.hpp"
#include "mabur/channel_set.h"
#include "mabur/profile.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "vrx_cfg.h"

namespace webgs {

int64_t cap_to_complete_us(uint32_t pts32, uint64_t t_complete_us, int64_t pts_off_us) {
  const uint32_t cap_in_pts =
      static_cast<uint32_t>(t_complete_us + static_cast<uint64_t>(pts_off_us));
  return static_cast<int32_t>(cap_in_pts - pts32);
}

std::optional<std::string> channel_width_error(const maburgs::Config& cfg, Mode mode,
                                               int start_ch, int width) {
  if (width != 20 && width != 40) return "radio.width: must be 20 or 40";
  if (auto e = mabur::channel_set_issue(cfg.radio.channels, static_cast<uint8_t>(width),
                                        "radio.channels"))
    return e->field + ": " + e->why;
  if (start_ch < 1 || start_ch > 177 ||
      !mabur::channel_set_member(cfg.radio.channels, static_cast<uint8_t>(start_ch)))
    return "channel " + std::to_string(start_ch) + " is not a member of radio.channels";
  if (cfg.radio.pin && *cfg.radio.pin != start_ch)
    return "radio.channel: pinned to " + std::to_string(*cfg.radio.pin) +
           " but the start channel is " + std::to_string(start_ch);
  if (auto e = maburgs::radio_width_issue(static_cast<uint8_t>(start_ch), width))
    return e->field + ": " + e->why;
  if (mode == Mode::Gs)
    if (auto e = maburgs::link_width_issue(cfg.link, width)) return e->field + ": " + e->why;
  return std::nullopt;
}

std::string relay_stats_fields(const maburgs::RelayStatsIn& r) {
  char b[640];
  std::snprintf(b, sizeof b,
                ",\"radio\":\"relay\",\"relay_state\":%u,\"relay_ch\":%u,\"relay_sec\":%u,"
                "\"relay_owned\":%d,\"relay_you_own\":%d,\"relay_frames\":%llu,\"relay_gaps\":%llu,"
                "\"relay_rx_drops\":%llu,\"relay_tx_ring_drops\":%llu,\"relay_tx\":%u,\"relay_tx_fail\":%u,"
                "\"relay_tx_refused\":%u,\"relay_your_drops\":%u,\"relay_tx_scan_drop\":%u,\"relay_sweeps\":%llu",
                r.state, r.ch, r.sec, r.owned ? 1 : 0, r.you_own ? 1 : 0,
                static_cast<unsigned long long>(r.frames), static_cast<unsigned long long>(r.gaps),
                static_cast<unsigned long long>(r.rx_drops), static_cast<unsigned long long>(r.tx_drops),
                r.tx, r.tx_fail, r.tx_refused, r.your_drops,
                r.tx_scan_drop, static_cast<unsigned long long>(r.sweeps));
  return b;
}

namespace {
maburgs::LinkHealthCfg lh_cfg(const maburgs::Config& cfg) {
  const auto enh = cfg.uep_layers()[1];
  return {1, enh.blocks_per_body,
          static_cast<int>(mabur::sw::kSwHeaderLen) + enh.fec.symbol_size};
}
}  // namespace

// ChannelSink for the page: the stderr-style lines go to Io::on_log; the
// scan.log records have no home here and are dropped.
struct WebGs::Sink final : maburgs::ChannelSink {
  WebGs* w;
  explicit Sink(WebGs* o) : w(o) {}
  void dwell(double, int, const maburgs::ScoutDwell&) override {}
  void pick(double, std::optional<uint8_t>, uint64_t, const std::vector<maburgs::RankEntry>&,
            int) override {}
  void move(const maburgs::MoveEvent&) override {}
  void verdict(double, const maburgs::VerdictOut&, const std::vector<maburgs::VerdictCardIn>&,
               const maburgs::VerdictLinkIn&) override {}
  void hop(const maburgs::HopEvent&) override {}
  void log(const std::string& line) override {
    if (w->io_.on_log) w->io_.on_log(line);
  }
};

WebGs::WebGs(const maburgs::Config& cfg, Mode mode, uint8_t start_ch, int width,
             std::vector<maburgs::LinkCard*> cards, int n_usb, Io io, Opts opts)
    : mode_(mode),
      io_(std::move(io)),
      opts_(opts),
      agg_(cfg.uep_layers(), static_cast<uint32_t>(cfg.fec.seq_horizon), 1,
           static_cast<uint32_t>(cfg.link.arrival_guard_syms)),
      fs_({static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms), cfg.video.frame_lookahead},
          {[this](const mabur::framewire::FrameHdr& h, uint8_t sid, uint8_t /*nslices: no streamed decode in the browser*/) {
             cur_ = Au{};
             cur_.pts_us = h.pts_us;
             cur_.sid = sid;
             cur_.flags = h.flags;
             if (slot_) slot_->on_au_first(now_us_ / 1000);
             // One probe expectation per video AU (main.cpp begin_frame).
             lha_.on_au_begin(sid, h.frame_id, static_cast<double>(now_us_ / 1000));
           },
           [this](const uint8_t* p, size_t n) { cur_.data.insert(cur_.data.end(), p, p + n); },
           [this](bool complete, const maburgs::AuLatMeta& lat) {
             // main.cpp: every AU end, clean or truncated (in-flight scout alignment).
             if (chan_) chan_->note_au_end();
             cur_.complete = complete;
             cur_.salvaged = !complete && lat.slice.salvaged;
             if (cur_.salvaged) ++aus_salvaged_;
             cur_.t_first_us = lat.t_first_us;
             cur_.t_complete_us = now_us_;
             if (slot_)
               slot_->on_au_complete(now_us_ / 1000,
                                     cur_.sid < 2 &&
                                         lha_.probe_commanded() != mabur::rc::kNoProbeProfile);
             if (rtt_.has_offset())
               cur_.cap_to_complete_us =
                   cap_to_complete_us(cur_.pts_us, cur_.t_complete_us, rtt_.pts_off_us());
             ++(complete ? aus_complete_ : aus_truncated_);
             if (!complete && cur_.sid == 0) ++aus_truncated_base_;
             if (io_.on_au) io_.on_au(std::move(cur_));
             cur_ = Au{};
           }}),
      gap_(cfg.video.frame_gap_timeout_ms, cfg.video.frame_gap_timeout_max_ms),
      lha_(lh_cfg(cfg)),
      osd_([this](int rows, int cols, const uint16_t* cells) {
        if (io_.on_osd) io_.on_osd(rows, cols, cells);
      }),
      cards_(std::move(cards)),
      n_usb_(n_usb),
      start_ch_(start_ch) {
  // The radio is tuned by the glue. A spotter's link setting is just the
  // configured width: it drives no ladder, so it needs no MCS (the readout
  // comes off the air, air_mcs_), and the drone's applied-op echo left
  // Telem 2026-09-30.
  spotter_op_.bw = width;
  key_fp_ = mabur::key_fingerprint(cfg.link.key);
  if (mode_ == Mode::Gs) {
    if (!io_.send && cards_.empty())
      throw std::invalid_argument("webgs: Gs mode needs Io::send or a card");
    // VrxCfg::op_channel seeds the proposal; with a roster the core owns it
    // from here on.
    maburgs::VrxCfg vc = maburgs::vrx_cfg_from(cfg, start_ch);
    vc.rz_nonce = opts_.rz_nonce;
    vrx_ = std::make_unique<maburgs::VrxController>(vc);
    slot_ = std::make_unique<maburgs::RcfSlotter>(
        maburgs::RcfSlotCfg{cfg.link.rcf_slot_hold_ms, 100, 2, 3, 1});
    if (cfg.link.nack.enable) {
      nack_ = std::make_unique<mabur::NackTracker>(cfg.link.nack);
      nack_lookback_ = static_cast<uint32_t>(cfg.link.nack.lookback);
      nack_down_util_ = cfg.link.ladder_cfg.down_util;
      key_ = cfg.link.key;
    }
    if (!cards_.empty()) {
      sink_ = std::make_unique<Sink>(this);
      maburgs::ChannelCoreCfg cc;
      cc.radio = cfg.radio;
      cc.hop = cfg.hop;
      cc.key = cfg.link.key;
      cc.start_ch = start_ch;
      cc.n_usb = n_usb_;
      cc.leak_per_frame = 1.0;   // as maburgs (bench row 7, docs/channel-select.md)
      cc.threaded = opts_.core_threads;
      cc.store_name = "CHANNEL line";
      chan_ = std::make_unique<maburgs::ChannelCore>(
          cc, cards_, *vrx_, *sink_,
          [this](uint8_t ch) {
            if (io_.on_channel_store) io_.on_channel_store(ch);
            return true;
          },
          [this] { return now_ms_for_core_(); }, [this] { return now_us_for_core_(); },
          [this](int ms) { sleep_(ms); });
    }
  } else {
    io_.send = nullptr;   // spotter: no transmit path exists
    frame_wire_ = true;   // no session to gate on: always decode
    if (!cards_.empty()) {
      SpotterFollowCfg fc;
      fc.channels = cfg.radio.channels;
      fc.start = start_ch;
      follow_ = std::make_unique<SpotterFollow>(fc);
    }
  }
  agg_.set_frag_sink([this](const mabur::DecodedFrag& f) {
    if (!frame_wire_) return;
    fs_.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), now_us_ / 1000,
                      {f.body_mono_us, f.q_ms, f.enc_us, f.air_ms, f.sw_seq, true, f.retx});
  });
  agg_.set_rc_sink([this](uint8_t, const std::vector<uint8_t>& f, uint64_t us) {
    if (mabur::rc::frame_type(f.data(), f.size()) == mabur::rc::T_TELEM) {
      if (auto t = mabur::rc::parse_telem(f.data(), f.size())) {
        telem_ = t;
        // Spotter has no session edge: a maburd restart (new seqs, frame_ids
        // from 0) is the one discontinuity it can see. Safe mid-drain: an RC
        // body never reaches the decoder.
        if (!vrx_ && restart_.on_telem(t->tlm_seq, static_cast<double>(us) / 1000.0))
          reset_video_();
        // RTT pairs telem echoes with our own sends: Gs only.
        if (vrx_)
          rtt_.on_telem(t->rcf_seq_echo, (t->flags & 0x08) != 0, t->rcf_age_ms,
                        t->pts_at_build, us);
        // Drone NACK counters are per Telem period: sum each period once.
        if (nack_ && (!nack_tlm_seen_ || *nack_tlm_seen_ != t->tlm_seq)) {
          nack_tlm_seen_ = t->tlm_seq;
          drone_nack_rx_ += t->nack_rx;
          drone_retx_syms_ += t->retx_syms;
          drone_retx_refused_ += t->retx_refused;
        }
      }
      return;
    }
    if (!vrx_) {
      // Spotter: the real GS's RCF carries the standing hop order in the
      // clear; follow it without verifying the tag (spec §6.2). on_rx already
      // fed this body to on_frame (every CRC-good body, DISC included).
      if (follow_ && mabur::rc::frame_type(f.data(), f.size()) == mabur::rc::T_RCF)
        if (const auto r = mabur::rc::parse_rcf(f.data(), f.size()))
          follow_->on_rcf(r->hop_ch, r->hop_epoch, rx_ch_cur_, static_cast<double>(us) / 1000.0);
      return;
    }
    const bool was_session = vrx_->link_state() == maburgs::VrxState::SESSION;
    vrx_->on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
    // main.cpp (final review C1): the ack that OPENED the session links
    // where it was heard (on_rc_body() handed the core its rx channel).
    if (chan_ && !was_session && vrx_->link_state() == maburgs::VrxState::SESSION &&
        mabur::rc::frame_type(f.data(), f.size()) == mabur::rc::T_DISC_ACK)
      if (const auto ack = mabur::rc::parse_disc_ack(f.data(), f.size()))
        chan_->on_session_opened(*ack, static_cast<double>(us) / 1000.0);
  });
  agg_.set_probe_sink([this](uint8_t card, const mabur::node::RxBody& m) {
    if (slot_) slot_->on_probe_tail(now_us_ / 1000);
    lha_.on_probe_body(card, m);
  });
  // MSP OSD (maburgs main.cpp's msp sink): receive-only, both modes.
  if (cfg.msp.enable) {
    msp_ = std::make_unique<maburgs::MspSink>(
        cfg.msp.symbol_size, cfg.msp.window,
        [this](const uint8_t* d, size_t n) { osd_.feed(d, n, now_us_ / 1000); });
    agg_.set_msp_sink([this](const uint8_t* b, size_t n, uint64_t us) {
      msp_->on_body(b, n, us / 1000);
    });
  }
}

WebGs::~WebGs() {
  // Join the core's threads while every member they touch is still alive
  // (the cards outlive this object).
  if (chan_) chan_->shutdown();
}

uint64_t WebGs::now_us_for_core_() const {
  if (!opts_.core_threads) return now_us_.load(std::memory_order_relaxed);
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

uint64_t WebGs::now_ms_for_core_() const { return now_us_for_core_() / 1000; }

void WebGs::sleep_(int ms) {
  if (ms <= 0) return;
  if (!opts_.core_threads)
    now_us_.fetch_add(static_cast<uint64_t>(ms) * 1000, std::memory_order_relaxed);
  else
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  if (opts_.sleep_hook) opts_.sleep_hook(ms);
}

// Spotter: the follower decides, the one card follows. A RadioFrontend
// retune is a synchronous FastRetune at the link width (members are HT40
// primaries on one offset); a RemoteCard retune is a TUNE whose channel()
// reads the target at once and whose ready() reads false until the relay's
// STATUS confirms (~50 ms) -- so the retune is re-issued while channel()
// disagrees (a pre-ready USB card refuses it: next tick), and the follower
// hears about the member only once the card is ready() on it.
void WebGs::step_follow_(double now_ms) {
  follow_->tick(now_ms);
  auto& c = *cards_[0];
  const uint8_t want = follow_->desired();
  if (c.channel() != want) c.retune(want);
  if (c.ready() && c.channel() == want && follow_reported_ != want) {
    follow_reported_ = want;
    follow_->on_card_channel(want, now_ms);
  }
}

void WebGs::on_rx(const mabur::node::RxBody& m) {
  if (m.mono_us > now_us_) now_us_ = m.mono_us;
  ++bodies_;
  rx_ch_cur_ = m.rx_channel;
  // Before the aggregator routes it (the rc sink runs inside on_rx_body):
  // a sweep hearing the GS on the member locks before the RCF is read.
  if (follow_ && m.crc_ok) follow_->on_frame(m.rx_channel, static_cast<double>(now_us_) / 1000.0);
  if (chan_) chan_->on_rc_body(m.rx_channel);   // read by the rc sink inside on_rx_body()
  agg_.on_rx_body(m);
  if (!m.crc_ok) return;
  const int sid = mabur::sbi_peek_stream_id(m.body.data(), m.body.size());
  // Spotter MCS readout, off the air: base-stream bodies only (enh can
  // differ, the probe rides the next rung's candidate MCS); 255 = no rate
  // on this body (legacy/VHT, or a relay frame without one), keep the last.
  if (sid == 0 && m.mcs != 255) air_mcs_ = m.mcs;
  if (!vrx_) return;
  if (mabur::rc::frame_type(m.body.data(), m.body.size()) < 0 && sid != mabur::kMspStreamId &&
      sid != mabur::kProbeStreamId) {
    // Only real video heard where the link lives refreshes the rendezvous
    // silence timer (main.cpp, ChannelPlan::is_link_video).
    if (!chan_ || chan_->is_link_video(m.rx_channel))
      vrx_->on_video(static_cast<double>(m.mono_us) / 1000.0);
    if (chan_) chan_->note_video(m.rx_channel);   // hop confirmation reads the stamp
  }
}

void WebGs::reset_video_() {
  agg_.decoder().reset_continuity();
  fs_.reset();   // closes in-flight frames as truncated AUs (end_frame)
  if (nack_) nack_->clear();   // tracked seqs dropped; the counter is kept (nack_vtx_seen_)
  cur_ = Au{};
  ++resets_;
}

// maburgs main.cpp's NACK block (spec 2026-10-05 fec-nack §3): base-layer
// selective repeat. Direct send, mid-burst -- never the RcfSlotter (bench:
// slotted fill p50 44 ms, past the gap timeout). Option A: the retransmit
// fixes the video only; every ladder input still books the loss. No
// calibration radio-silence gate here: the page has no calibration.
void WebGs::poll_nack_(uint64_t now_ms) {
  const auto sctx = vrx_->session_ctx();
  // != 0: session_ctx() keeps reporting the held nonce in BEACONING.
  if (sctx.vtx_nonce != 0 && sctx.vtx_nonce != nack_vtx_seen_) {
    nack_vtx_seen_ = sctx.vtx_nonce;
    nack_->restart_counter();
  }
  if (sctx.vtx_nonce == 0 || !frame_wire_) {
    nack_->clear();
    return;
  }
  mabur::NackInputs ni;
  ni.missing = [&] { return agg_.decoder().missing_sources(0, nack_lookback_); };
  ni.state = [&](uint32_t s) { return agg_.decoder().source_state(0, s); };
  ni.tail = [&]() -> std::optional<mabur::NackTailView> {
    auto tv = fs_.tail_view(0);
    if (!tv) return std::nullopt;
    return mabur::NackTailView{tv->count, tv->max_idx, tv->seq_at_max, tv->last_progress_ms};
  };
  // The stop rule's util is the ladder's own demote input (LadderController::u_).
  ni.util = [&] { return vrx_->ctl().util(); };
  ni.down_util = nack_down_util_;
  ni.gap_timeout_ms = fs_.gap_ms(0);
  if (auto n = nack_->poll(now_ms, ni)) {
    mabur::rc::TagCtx ctx = sctx;
    ctx.seq32 = n->counter;
    maburgs::SlotFrame sf{mabur::rc::pack_nack(*n, key_, ctx), 0, 0, false};
    sf.offered_ms = now_ms;
    if (send_(sf)) ++nack_sent_;
  }
}

// Drain the tracker's window once a second into the stats readout
// (nearest-rank percentiles, as the sideport exporter computes them).
void WebGs::drain_nack_window_(uint64_t now_ms) {
  if (nack_win_t0_ms_ == 0) { nack_win_t0_ms_ = now_ms; return; }
  if (now_ms < nack_win_t0_ms_ + 1000) return;
  auto w = nack_->take_window();
  const double secs = static_cast<double>(now_ms - nack_win_t0_ms_) / 1000.0;
  nack_win_t0_ms_ = now_ms;
  nack_fill_pps_ = static_cast<double>(w.filled) / secs;
  nack_late_max_ = w.late_ms_max;
  std::sort(w.fill_ms.begin(), w.fill_ms.end());
  auto pct = [&w](double p) -> std::optional<uint32_t> {
    if (w.fill_ms.empty()) return std::nullopt;
    const double rank = std::ceil(p * static_cast<double>(w.fill_ms.size()));
    const size_t i = rank < 1.0 ? 0 : static_cast<size_t>(rank) - 1;
    return w.fill_ms[std::min(w.fill_ms.size() - 1, i)];
  };
  nack_fill_p50_ = pct(0.5);
  nack_fill_p90_ = pct(0.9);
  nack_fill_max_ = w.fill_ms.empty() ? std::nullopt : std::optional<uint32_t>(w.fill_ms.back());
}

bool WebGs::send_(const maburgs::SlotFrame& f) {
  if (chan_ && !chan_->may_send(f.card)) return false;   // the scout gate: dropped, counted by the core
  bool ok = true;
  if (!cards_.empty())
    ok = cards_[static_cast<size_t>(f.card)]->send_control(f.frame);
  else
    io_.send(f.frame);
  ++sends_;
  // stamp_rtt is set exactly for RCFs (tick(): `!out->is_disc`).
  if (f.stamp_rtt) {
    ++rcf_sent_;
    rtt_.on_rcf_sent(f.seq, now_us_);
  }
  if (chan_) chan_->note_sent(ok, f.stamp_rtt);
  return true;
}

void WebGs::tick(uint64_t now_us) {
  if (now_us > now_us_) now_us_ = now_us;
  const uint64_t now_ms_u = now_us_ / 1000;
  const double now_ms = static_cast<double>(now_ms_u);
  for (auto* c : cards_) c->tick(now_ms_u);
  if (follow_) step_follow_(now_ms);
  if (opts_.adaptive_gap && now_ms_u >= gap_update_ms_ + 1000) {
    gap_update_ms_ = now_ms_u;
    for (int s = 0; s < 2; ++s) {
      gap_.update(s, agg_.decoder().newest_seq(s), agg_.decoder().repair_window(s), now_ms_u);
      fs_.set_gap_timeout(s, static_cast<uint64_t>(gap_.timeout_ms(s)));
    }
  }
  if (msp_ && now_ms_u >= msp_tick_ms_ + 1000) {
    msp_tick_ms_ = now_ms_u;
    msp_->tick(now_ms_u);   // expire stale repair rows (~1 Hz, as maburgs)
  }
  osd_.tick(now_ms_u);      // publishes a held screen once its 30 ms is up
  if (chan_) {
    // maburgs main.cpp's per-tick block (gs/src/channel_core.h). One TX
    // card: card 0 (the page has no TxSelector and no calibration).
    maburgs::ChannelTickIn cin;
    cin.now_ms = now_ms;
    cin.in_session = vrx_->link_state() == maburgs::VrxState::SESSION;
    cin.cal_running = false;
    cin.tx_card = 0;
    cin.agg = &agg_;
    chan_->tick(cin);
    chan_->note_tx_card(0);
  }
  if (vrx_) {
    // maburgs main.cpp: video tail only while in SESSION with a
    // CAP_FRAME_WIRE peer; any change drops FRAG-seq continuity and
    // half-assembled frames -- the new session's seqs/frame_ids are
    // unrelated to the old one's.
    const bool fw = vrx_->link_state() == maburgs::VrxState::SESSION &&
                    (vrx_->peer_caps() & mabur::rc::CAP_FRAME_WIRE);
    if (fw != frame_wire_) {
      frame_wire_ = fw;
      reset_video_();
    }
  }
  if (frame_wire_) fs_.poll(now_ms_u);
  if (nack_) {
    poll_nack_(now_ms_u);
    drain_nack_window_(now_ms_u);
  }
  maburgs::LinkHealthInputs in;
  if (vrx_) {
    in.op = vrx_->cur_op();
    in.probe_profile = vrx_->probe_profile();
    in.probe_rung = vrx_->ctl().probe_rung();
  } else {
    in.op = spotter_op_;
  }
  const auto lh = lha_.tick(now_ms, agg_, in);
  last_health_ = lh.health;
  const std::vector<uint8_t>* sent = nullptr;
  std::vector<uint8_t> sent_copy;
  if (vrx_) {
    if (lh.probe_tail_ms) slot_->set_probe_tail_ms(*lh.probe_tail_ms);
    if (auto out = vrx_->step(now_ms, lh.health)) {
      if (!out->is_disc) lha_.on_step_sent(agg_);
      // A DISC follows the scout (scan_disc_targets via the core); an RCF
      // goes to card 0. rcf_seq() IS this frame's seq (build_rcf bumped
      // it); DISCs are not matchable RTT sends.
      const std::vector<int> targets =
          (out->is_disc && chan_) ? chan_->disc_targets(0) : std::vector<int>{0};
      for (size_t k = 0; k < targets.size(); ++k) {
        maburgs::SlotFrame sf{k + 1 == targets.size() ? std::move(out->frame) : out->frame,
                              vrx_->rcf_seq(), targets[k], !out->is_disc};
        // A DISC proposes the channel it is sent on (main.cpp, C1 addendum A).
        if (out->is_disc && chan_) sf.frame = chan_->disc_for_card(sf.frame, targets[k]);
        if (!slot_->offer(sf, now_ms_u, false) && send_(sf)) {
          sent_copy = sf.frame;
          sent = &sent_copy;
        }
      }
    }
    for (const auto& f : slot_->take_due(now_ms_u)) {
      if (!send_(f)) continue;
      sent_copy = f.frame;
      sent = &sent_copy;
    }
  }
  if (io_.on_control_tick)
    io_.on_control_tick(now_ms, lh.health, vrx_ ? vrx_->ctl().rung() : -1, sent);
}

void WebGs::inject_disc_ack_for_replay(uint64_t now_us) {
  if (!vrx_) return;
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx_->rz_nonce();
  ack.vtx_nonce = 1;  // replay has no drone: any held vtx_nonce opens SESSION
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
  ack.agreed_channel = vrx_->proposal();
  ack.seq = 1;
  const auto wire = mabur::rc::pack_disc_ack(ack);
  vrx_->on_rc_frame(wire.data(), wire.size(), static_cast<double>(now_us) / 1000.0);
}

void WebGs::set_vtx_rec(bool on) {
  if (!vrx_) return;
  vrx_->set_rec_wish(static_cast<uint8_t>(mabur::rc::kRecKnown | (on ? mabur::rc::kRecOn : 0)));
}

void WebGs::set_idr_requests(uint32_t n) {
  idr_req_ = n;
  if (vrx_) vrx_->set_idr_epoch(static_cast<uint8_t>(n));
}

Stats WebGs::stats() const {
  Stats s;
  s.mode = mode_;
  const double now_ms = static_cast<double>(now_us_ / 1000);
  if (vrx_) {
    s.session = vrx_->link_state() == maburgs::VrxState::SESSION;
    s.peer_acked = vrx_->peer_acked();
    s.rung = vrx_->ctl().rung();
    s.mcs = vrx_->cur_op().mcs;
    s.bw = vrx_->cur_op().bw;
    s.probe_state = maburgs::to_string(vrx_->ctl().probe_gate(now_ms).state);
    s.key_mismatch = vrx_->key_mismatch();
    if (nack_) {
      NackStatsOut n;
      n.cum = nack_->stats();
      n.sent = nack_sent_;
      n.settle_ms = nack_->settle_ms();
      n.fill_pps = nack_fill_pps_;
      n.fill_p50_ms = nack_fill_p50_;
      n.fill_p90_ms = nack_fill_p90_;
      n.fill_max_ms = nack_fill_max_;
      n.late_ms_max = nack_late_max_;
      n.drone_rx = drone_nack_rx_;
      n.drone_retx_syms = drone_retx_syms_;
      n.drone_retx_refused = drone_retx_refused_;
      s.nack = n;
    }
  } else {
    s.bw = spotter_op_.bw;   // configured width
    s.mcs = air_mcs_;        // base-stream RX MCS, -1 until one is heard
  }
  s.key_fp = key_fp_;
  const auto pre = lha_.pre_all();
  if (pre.valid) s.pre_fec_loss = pre.loss;
  s.residual = lha_.residual();
  s.snr_db = last_health_.rf_snr_db;
  s.rssi_dbm = last_health_.rf_rssi_dbm;
  if (rtt_.has_rtt()) {
    s.rtt_ms = rtt_.rtt_ms();
    s.rtt_min_ms = rtt_.rtt_min_ms();
  }
  if (rtt_.has_offset()) s.pts_off_us = rtt_.pts_off_us();
  s.bodies = bodies_;
  s.aus_complete = aus_complete_;
  s.aus_truncated = aus_truncated_;
  s.aus_truncated_base = aus_truncated_base_;
  s.aus_salvaged = aus_salvaged_;
  s.sends = sends_;
  s.rcf_sent = rcf_sent_;
  s.idr_req = idr_req_;
  if (telem_) {
    s.drone_rcf_rx = telem_->rcf_rx;
    s.drone_state = telem_->state;
    s.rec_status = telem_->rec_status;
    if (telem_->soc_temp_c != -128) s.drone_temp_c = telem_->soc_temp_c;
  }
  s.osd_snaps = msp_ ? msp_->snapshots_out() : 0;
  s.osd_screens = osd_.screens();
  s.channel = cards_.empty() ? start_ch_ : cards_[0]->channel();
  if (chan_) {
    const auto cs = chan_->snapshot(0);
    ChanStats c;
    c.scan_state = cs.scan_state;
    c.scan_rounds = cs.scan_rounds;
    c.scan_pick = cs.scan_pick;
    c.hop = cs.hop;
    s.chan = c;
  }
  if (follow_) {
    s.follow_state = to_string(follow_->state());
    s.follows = follow_->follows();
  }
  return s;
}

std::string stats_json(const Stats& s) {
  nlohmann::json j;
  j["mode"] = s.mode == Mode::Gs ? "gs" : "spotter";
  j["session"] = s.session;
  j["peer_acked"] = s.peer_acked;
  j["rung"] = s.rung;
  j["mcs"] = s.mcs;
  j["bw"] = s.bw;
  j["probe"] = s.probe_state;
  j["key_mismatch"] = s.key_mismatch;
  j["key_fp"] = s.key_fp;
  auto opt = [&](const char* k, const auto& v) {
    if (v)
      j[k] = *v;
    else
      j[k] = nullptr;
  };
  j["channel"] = s.channel;
  if (s.chan) {
    j["scan_state"] = s.chan->scan_state;
    j["scan_rounds"] = s.chan->scan_rounds;
    opt("scan_pick", s.chan->scan_pick);
    const auto& hp = s.chan->hop;
    auto optj = [](const auto& v) { return v ? nlohmann::json(*v) : nlohmann::json(nullptr); };
    nlohmann::json h;
    h["verdict"] = hp.verdict;
    h["evidence"] = hp.evidence;
    h["ref_rung"] = optj(hp.ref_rung);
    h["epoch"] = hp.epoch;
    h["state"] = hp.state;
    h["target"] = optj(hp.target);
    h["hops"] = hp.hops;
    h["holds"] = hp.holds;
    h["last_ms"] = optj(hp.last_ms);
    h["sweep_timeouts"] = hp.sweep_timeouts;
    j["hop"] = h;
  } else {
    j["scan_state"] = nullptr;
    j["scan_rounds"] = nullptr;
    j["scan_pick"] = nullptr;
    j["hop"] = nullptr;
  }
  opt("follow_state", s.follow_state);
  opt("follows", s.follows);
  opt("pre_fec_loss", s.pre_fec_loss);
  opt("residual", s.residual);
  opt("rtt_ms", s.rtt_ms);
  opt("rtt_min_ms", s.rtt_min_ms);
  opt("pts_off_us", s.pts_off_us);
  opt("drone_rcf_rx", s.drone_rcf_rx);
  opt("drone_state", s.drone_state);
  opt("drone_temp_c", s.drone_temp_c);
  if (s.rec_status) {
    j["rec_state"] = *s.rec_status & 0x03;
    j["rec_err"] = *s.rec_status >> 2;
  } else {
    j["rec_state"] = nullptr;
    j["rec_err"] = nullptr;
  }
  j["snr_db"] = std::isnan(s.snr_db) ? nlohmann::json(nullptr) : nlohmann::json(s.snr_db);
  j["rssi_dbm"] = std::isnan(s.rssi_dbm) ? nlohmann::json(nullptr) : nlohmann::json(s.rssi_dbm);
  j["bodies"] = s.bodies;
  j["aus"] = s.aus_complete;
  j["trunc"] = s.aus_truncated;
  j["trunc_base"] = s.aus_truncated_base;
  j["salvaged"] = s.aus_salvaged;
  j["sends"] = s.sends;
  j["rcf_sent"] = s.rcf_sent;
  j["idr_req"] = s.idr_req;
  j["osd_snaps"] = s.osd_snaps;
  j["osd_screens"] = s.osd_screens;
  if (s.nack) {
    nlohmann::json n;
    const auto& c = s.nack->cum;
    n["req"] = c.requests;
    n["rep"] = c.repeats;
    n["syms"] = c.syms_requested;
    n["tail"] = c.tail_requests;
    n["fill"] = c.filled;
    n["late"] = c.late_fill;
    n["waste"] = c.wasted;
    n["drop"] = c.dropped_deadline;
    n["sup"] = c.suppressed;
    n["lead"] = c.lead_skipped;
    n["sent"] = s.nack->sent;
    n["settle_ms"] = s.nack->settle_ms;
    auto o = [&n](const char* k, const auto& v) {
      if (v) n[k] = *v; else n[k] = nullptr;
    };
    o("fill_pps", s.nack->fill_pps);
    o("fill_p50", s.nack->fill_p50_ms);
    o("fill_p90", s.nack->fill_p90_ms);
    o("fill_max", s.nack->fill_max_ms);
    n["late_max"] = s.nack->late_ms_max;
    n["drone_rx"] = s.nack->drone_rx;
    n["drone_syms"] = s.nack->drone_retx_syms;
    n["drone_refused"] = s.nack->drone_retx_refused;
    j["nack"] = n;
  } else {
    j["nack"] = nullptr;
  }
  return j.dump();
}

}  // namespace webgs
