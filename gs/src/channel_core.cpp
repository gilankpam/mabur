#include "channel_core.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

#include "hop_blank.h"
#include "hop_burst_gate.h"
#include "scout_pick.h"
#include "width_resync.h"

namespace maburgs {

std::string ChannelCore::logf_(const char* fmt, ...) {
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  return std::string(b);
}

ChannelCore::ChannelCore(ChannelCoreCfg cfg, std::vector<LinkCard*> cards, VrxController& vrx,
                         ChannelSink& sink, StoreFn store, NowMsFn now_ms, NowUsFn now_us,
                         SleepFn sleep_ms)
    : cfg_(std::move(cfg)), cards_(std::move(cards)), vrx_(vrx), sink_(sink),
      store_(std::move(store)), now_ms_(std::move(now_ms)), now_us_(std::move(now_us)),
      sleep_(std::move(sleep_ms)),
      n_cards_(static_cast<int>(cards_.size())),
      pinned_(cfg_.radio.pin.has_value()),
      saved_op_(cfg_.start_ch),
      plan_(ChannelPlanCfg{cfg_.start_ch, cfg_.radio.channels, n_cards_,
                           cfg_.radio.scan.search_after_ms}),
      boot_pick_(false),
      verdict_(cfg_.hop, n_cards_),
      ranker_(cfg_.hop, cfg_.radio.channels, 0),
      hopc_(cfg_.hop),
      cur_ch_(static_cast<size_t>(n_cards_), cfg_.start_ch),
      width_tried_(static_cast<size_t>(n_cards_), false),
      energy_last_(static_cast<size_t>(n_cards_)),
      dwell_stats_(static_cast<size_t>(n_cards_)),
      window_prev_(static_cast<size_t>(n_cards_)),
      window_prev_crc_(static_cast<size_t>(n_cards_), 0),
      window_prev_ok_(static_cast<size_t>(n_cards_), false),
      window_prev_ms_(static_cast<size_t>(n_cards_), 0),
      nhm_win_(static_cast<size_t>(n_cards_)),
      dwell_gen_(static_cast<size_t>(n_cards_)) {
  gs_start_ms_ = now_ms_();
  nhm_op_period_ = nhm_period_4us(std::max(cfg_.hop.window_ms - 10, 1));
  can_scout_.assign(static_cast<size_t>(n_cards_), false);
  snr_ok_.assign(static_cast<size_t>(n_cards_), true);
  for (int i = 0; i < n_cards_; ++i) {
    can_scout_[static_cast<size_t>(i)] = cards_[static_cast<size_t>(i)]->can_scout();
    snr_ok_[static_cast<size_t>(i)] = cards_[static_cast<size_t>(i)]->caps().snr_ok;
  }
  one_card_ = cfg_.n_usb == 1;
  scout_card_ = pick_boot_scout(can_scout_);
  if (scout_card_ >= 0) {
    ScoutCfg sc;
    sc.channels = cfg_.radio.channels;
    sc.measure = !pinned_;
    sc.dwell_ms = cfg_.radio.scan.dwell_ms;
    sc.settle_ms = cfg_.radio.scan.settle_ms;
    sc.min_rounds = cfg_.radio.scan.min_rounds;
    sc.search_ms = cfg_.radio.scan.search_ms;
    sc.op_window_ms = cfg_.radio.scan.op_window_ms;
    sc.beacon_period_ms = 20;
    sc.one_card_ms = cfg_.radio.scan.one_card_ms;
    sc.pick_margin = static_cast<uint32_t>(cfg_.radio.scan.pick_margin);
    sc.one_card = one_card_;
    sc.link_width_mhz = cfg_.radio.width;
    sc.busy_dbm = cfg_.hop.verdict.busy_dbm;
    sc.blocked_pct = cfg_.hop.verdict.blocked_pct;
    sc.leak_per_frame = cfg_.leak_per_frame;
    scout_ = std::make_unique<ChannelScout>(
        sc, *cards_[static_cast<size_t>(scout_card_)],
        [this] { return static_cast<int64_t>(now_ms_()); },
        [this](int ms) { sleep_(ms); });
    scout_->set_op(cfg_.start_ch);
    scout_->set_search(true);
    scout_search_req_ = true;
    inflight_ = std::make_unique<InflightScout>(
        InflightScoutCfg{cfg_.hop.dwell_observe_ms, cfg_.hop.dwell_period_ms,
                         cfg_.radio.channels, cfg_.radio.width, cfg_.hop.verdict.busy_dbm},
        *cards_[static_cast<size_t>(scout_card_)],
        [this] { return static_cast<int64_t>(now_us_()); },
        [this](int ms) { sleep_(ms); });
  }
  boot_pick_ = BootPick(scout_ != nullptr && !pinned_);
  if (pinned_) frozen_pick_ = cfg_.start_ch;
  tx_card_now_.store(0);
}

ChannelCore::~ChannelCore() { shutdown(); }

bool ChannelCore::scout_owns_() const {
  return scout_ && (scout_->working() || scout_search_req_ || (!pinned_ && scout_->pick_open()));
}

ChannelSnapshot ChannelCore::snapshot() const {
  ChannelSnapshot s;
  const int tx = tx_card_now_.load();
  s.channel = cards_[static_cast<size_t>(tx)]->channel();
  s.scan_state = (!scout_ || pinned_)        ? "off"
                 : !boot_pick_.open()        ? "frozen"
                 : boot_pick_.relocating()   ? "moving"
                                             : "scouting";
  s.scan_rounds = scout_ ? scout_->rounds() : 0;
  s.scan_pick = frozen_pick_;
  s.hop.enable = cfg_.hop.enable;
  s.hop.verdict = to_string(last_verdict_out_.v);
  s.hop.evidence = last_verdict_out_.evidence;
  if (last_verdict_out_.ref_rung >= 0) s.hop.ref_rung = last_verdict_out_.ref_rung;
  s.hop.epoch = hopc_.epoch();
  s.hop.state = hop_state_name(hopc_.state());
  if (const uint8_t hc = hopc_.hop_ch(); hc != 0) s.hop.target = hc;
  s.hop.hops = hopc_.hops();
  s.hop.holds = hopc_.holds();
  s.hop.last_ms = last_hop_event_ms_;
  s.energy = energy_last_;
  s.dwell = dwell_stats_;
  s.scout_gated_sends = scout_gated_sends_;
  return s;
}

ChannelTickOut ChannelCore::tick(const ChannelTickIn& in) {
  now_ms_cur_ = in.now_ms;
  now_ms_u_cur_ = static_cast<uint64_t>(in.now_ms);
  tx_card_now_.store(in.tx_card, std::memory_order_relaxed);
  in_session_atomic_.store(in.in_session, std::memory_order_relaxed);
  cal_running_atomic_.store(in.cal_running, std::memory_order_relaxed);
  // The scout thread starts once its card is up (threaded), or one step
  // runs here (tests).
  if (scout_ && !scout_started_ && cards_[static_cast<size_t>(scout_card_)]->ready()) {
    scout_started_ = true;
    if (cfg_.threaded) scout_thread_ = std::thread([this] { scout_->run(); });
  }
  if (!cfg_.threaded && scout_started_) run_scout_step();
  plan_.tick(in.now_ms, in.in_session || in.cal_running);
  step_scout_inputs_();
  step_boot_pick_(in);
  step_store_();
  step_hop_edge_and_window_(in);
  step_controller_(in);
  step_move_edge_(in);
  step_drains_();
  step_width_resync_();
  for (const auto& ev : plan_.take_events()) {
    sink_.move(ev);
    sink_.log(logf_("maburgs channel: %s card %d %u -> %u", to_string(ev.reason), ev.card,
                    static_cast<unsigned>(ev.from), static_cast<unsigned>(ev.to)));
  }
  step_mechanical_retune_();
  // s1_hop_loss is fed from the same base-sid arrival counters as the
  // assembler's s1_loss, on the same now_ms.
  if (in.agg) {
    const auto s1 = in.agg->decoder().stats(0);
    s1_hop_loss_.add(s1.arr_expected, s1.arr_arrived, in.now_ms);
  }
  ChannelTickOut out;
  out.dwell_busy = dwell_busy_.load();
  out.tx_frozen = tx_selection_frozen(out.dwell_busy, plan_.hopping());
  return out;
}

void ChannelCore::run_scout_step() { if (scout_) scout_->run_once(); }
void ChannelCore::run_inflight_step() { inflight_body_(); }

// stubs filled by Tasks 3-5
void ChannelCore::on_card_died(int) {}
void ChannelCore::on_card_reopened(int) {}
void ChannelCore::shutdown() {}
void ChannelCore::on_rc_body(uint8_t rx_ch) { rc_body_rx_ch_ = rx_ch; }

void ChannelCore::on_session_opened(const mabur::rc::DiscAck& ack, double now_ms) {
  // Final review C1: the link forms where the drone is found. Only the ack
  // that OPENED the session -- our nonce, not key-mismatch flagged, so a
  // stranger's drone cannot drag the cards -- and only on the rx_channel of
  // this body. plan.link_found() moves op to it for every card; the
  // relocation to plan.want(), if any, is an ordinary hop order (BootPick).
  const uint8_t x = rc_body_rx_ch_;
  if (ack.vrx_nonce == vrx_.rz_nonce() && !(ack.flags & mabur::rc::kAckKeyMismatch) &&
      x != 0 && x != plan_.op() && plan_.member(x) && !plan_.hopping()) {
    sink_.log(logf_("maburgs channel: drone found on %u (op %u): the link forms there",
                    static_cast<unsigned>(x), static_cast<unsigned>(plan_.op())));
    plan_.link_found(now_ms, x);
    vrx_.set_proposal(plan_.op());   // a DISC proposes the channel it is sent on: stay
  }
}

std::vector<int> ChannelCore::disc_targets(int tx) const {
  if (!scout_owns_()) return {tx};
  std::vector<bool> ready(static_cast<size_t>(n_cards_));
  for (int i = 0; i < n_cards_; ++i) ready[static_cast<size_t>(i)] = cards_[static_cast<size_t>(i)]->ready();
  return scan_disc_targets(cfg_.n_usb, n_cards_, scout_card_, scout_->beaconing(), ready);
}

bool ChannelCore::may_send(int card) const {
  if (scout_owns_() && ((card == scout_card_ && !scout_->beaconing()) || scout_->quiet())) {
    ++scout_gated_sends_;   // mutable: the gate is a query the caller makes before sending
    return false;
  }
  return true;
}

std::vector<uint8_t> ChannelCore::disc_for_card(const std::vector<uint8_t>& frame, int card) const {
  // A DISC proposes the channel it is sent on (final review C1 addendum A).
  const uint8_t ch = cards_[static_cast<size_t>(card)]->channel();
  if (plan_.member(ch)) return disc_for_channel(frame, ch, cfg_.key);
  return frame;
}

void ChannelCore::note_sent(bool sent_ok, bool is_rcf) {
  if (sent_ok) ++ctrl_sent_total_;
  if (is_rcf) ++rcf_sent_total_;
}
void ChannelCore::freeze_pick_(double, const char*) {}
void ChannelCore::dispatch_hop_action_(const HopAction&, bool, double) {}
void ChannelCore::apply_hop_action_(const HopAction&, double) {}
void ChannelCore::step_scout_inputs_() {}
void ChannelCore::step_boot_pick_(const ChannelTickIn&) {}
void ChannelCore::step_store_() {}
void ChannelCore::step_hop_edge_and_window_(const ChannelTickIn&) {}
void ChannelCore::step_controller_(const ChannelTickIn&) {}
void ChannelCore::step_move_edge_(const ChannelTickIn&) {}
void ChannelCore::step_drains_() {}
void ChannelCore::step_width_resync_() {}
void ChannelCore::step_mechanical_retune_() {}
void ChannelCore::inflight_body_() {}
void ChannelCore::scout_loop_() {}

}  // namespace maburgs
