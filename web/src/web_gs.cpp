#include "web_gs.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "json.hpp"
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
                                               int channel, int width) {
  // Same range as load_config's radio.channel.
  if (channel < 1 || channel > 200)
    return "channel " + std::to_string(channel) + " out of range [1,200]";
  if (auto e = maburgs::radio_width_issue(static_cast<uint8_t>(channel), width))
    return e->field + ": " + e->why;
  if (mode == Mode::Gs)
    if (auto e = maburgs::link_width_issue(cfg.link, width)) return e->field + ": " + e->why;
  return std::nullopt;
}

namespace {
maburgs::LinkHealthCfg lh_cfg(const maburgs::Config& cfg) {
  const auto enh = cfg.uep_layers()[1];
  return {1, enh.blocks_per_body,
          static_cast<int>(mabur::sw::kSwHeaderLen) + enh.fec.symbol_size};
}
}  // namespace

WebGs::WebGs(const maburgs::Config& cfg, Mode mode, uint8_t channel, int width, Io io,
             Opts opts)
    : mode_(mode),
      io_(std::move(io)),
      opts_(opts),
      vtx_id_(cfg.link.vtx_id),
      agg_(cfg.uep_layers(), static_cast<uint32_t>(cfg.fec.seq_horizon), 1,
           static_cast<uint32_t>(cfg.link.arrival_guard_syms)),
      fs_({static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms), cfg.video.frame_lookahead},
          {[this](const mabur::framewire::FrameHdr& h, uint8_t sid) {
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
             cur_.complete = complete;
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
             if (io_.on_au) io_.on_au(std::move(cur_));
             cur_ = Au{};
           }}),
      gap_(cfg.video.frame_gap_timeout_ms, cfg.video.frame_gap_timeout_max_ms),
      lha_(lh_cfg(cfg)),
      osd_([this](int rows, int cols, const uint16_t* cells) {
        if (io_.on_osd) io_.on_osd(rows, cols, cells);
      }) {
  (void)width;  // the radio is tuned by the glue; the rung width comes from the ladder
  if (mode_ == Mode::Gs) {
    if (!io_.send) throw std::invalid_argument("webgs: Gs mode needs Io::send");
    vrx_ = std::make_unique<maburgs::VrxController>(maburgs::vrx_cfg_from(cfg, channel));
    vrx_->set_proposal(channel);   // never move the drone: we cannot follow
    slot_ = std::make_unique<maburgs::RcfSlotter>(
        maburgs::RcfSlotCfg{cfg.link.rcf_slot_hold_ms, 100, 2, 3, 1});
  } else {
    io_.send = nullptr;   // spotter: no transmit path exists
    frame_wire_ = true;   // no session to gate on: always decode
  }
  agg_.set_frag_sink([this](const mabur::DecodedFrag& f) {
    if (!frame_wire_) return;
    fs_.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), now_us_ / 1000,
                      {f.body_mono_us, f.q_ms, f.enc_us, f.air_ms});
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
        mabur::rc::PhyMode pm;
        uint8_t mcs = 0, bw = 20;
        mabur::rc::decode_profile(t->applied_profile, pm, mcs, bw);
        spotter_op_.vht = pm == mabur::rc::PhyMode::VHT;
        spotter_op_.mcs = mcs;
        spotter_op_.bw = bw;
        spotter_op_.overhead_base = t->applied_ov_base;
        spotter_op_.overhead_enh = t->applied_ov_enh;
      }
      return;
    }
    if (vrx_) vrx_->on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
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

void WebGs::on_rx(const mabur::node::RxBody& m) {
  if (m.mono_us > now_us_) now_us_ = m.mono_us;
  ++bodies_;
  agg_.on_rx_body(m);
  if (!vrx_ || !m.crc_ok) return;
  // Only real video refreshes the rendezvous silence timer (main.cpp).
  const int sid = mabur::sbi_peek_stream_id(m.body.data(), m.body.size());
  if (mabur::rc::frame_type(m.body.data(), m.body.size()) < 0 && sid != mabur::kMspStreamId &&
      sid != mabur::kProbeStreamId)
    vrx_->on_video(static_cast<double>(m.mono_us) / 1000.0);
}

void WebGs::reset_video_() {
  agg_.decoder().reset_continuity();
  fs_.reset();   // closes in-flight frames as truncated AUs (end_frame)
  cur_ = Au{};
  ++resets_;
}

void WebGs::send_(const maburgs::SlotFrame& f) {
  io_.send(f.frame);
  ++sends_;
  // stamp_rtt is set exactly for RCFs (tick(): `!out->is_disc`).
  if (f.stamp_rtt) {
    ++rcf_sent_;
    rtt_.on_rcf_sent(f.seq, now_us_);
  }
}

void WebGs::tick(uint64_t now_us) {
  if (now_us > now_us_) now_us_ = now_us;
  const uint64_t now_ms_u = now_us_ / 1000;
  const double now_ms = static_cast<double>(now_ms_u);
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
      // rcf_seq() IS this frame's seq (build_rcf bumped it); DISCs are not
      // matchable RTT sends.
      maburgs::SlotFrame sf{std::move(out->frame), vrx_->rcf_seq(), 0, !out->is_disc};
      if (!slot_->offer(sf, now_ms_u, false)) {
        send_(sf);
        sent_copy = sf.frame;
        sent = &sent_copy;
      }
    }
    for (const auto& f : slot_->take_due(now_ms_u)) {
      send_(f);
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
  ack.vtx_id = vtx_id_;
  ack.vrx_nonce = vrx_->rz_nonce();
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
  } else if (telem_) {
    s.mcs = spotter_op_.mcs;
    s.bw = spotter_op_.bw;
  }
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
  s.sends = sends_;
  s.rcf_sent = rcf_sent_;
  s.idr_req = idr_req_;
  if (telem_) {
    s.drone_rcf_rx = telem_->rcf_rx;
    s.drone_state = telem_->state;
    s.rec_status = telem_->rec_status;
    s.drone_idr_gs = telem_->idr_gs;
  }
  s.osd_snaps = msp_ ? msp_->snapshots_out() : 0;
  s.osd_screens = osd_.screens();
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
  auto opt = [&](const char* k, const auto& v) {
    if (v)
      j[k] = *v;
    else
      j[k] = nullptr;
  };
  opt("pre_fec_loss", s.pre_fec_loss);
  opt("residual", s.residual);
  opt("rtt_ms", s.rtt_ms);
  opt("rtt_min_ms", s.rtt_min_ms);
  opt("pts_off_us", s.pts_off_us);
  opt("drone_rcf_rx", s.drone_rcf_rx);
  opt("drone_state", s.drone_state);
  opt("drone_idr_gs", s.drone_idr_gs);
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
  j["sends"] = s.sends;
  j["rcf_sent"] = s.rcf_sent;
  j["idr_req"] = s.idr_req;
  j["osd_snaps"] = s.osd_snaps;
  j["osd_screens"] = s.osd_screens;
  return j.dump();
}

}  // namespace webgs
