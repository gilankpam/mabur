#include "channel_core.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "hop_blank.h"
#include "hop_burst_gate.h"
#include "mabur/channel_file.h"
#include "pair_pick.h"
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

void ChannelCore::on_card_died(int card) {
  if (scout_ && card == scout_card_ && !scout_card_down_) {
    scout_card_down_ = true;
    if (scout_->working()) {
      scout_card_died_seen_ = true;
      sink_.log(logf_("maburgs channel: scout card %d died at %llu rounds; search held "
                      "until it reopens, card rejoins at %u MHz",
                      card, static_cast<unsigned long long>(scout_->rounds()),
                      static_cast<unsigned>(cfg_.radio.width)));
    }
    scout_search_req_ = false;
    scout_->set_search(false);
  }
}

void ChannelCore::on_card_reopened(int card) {
  cur_ch_[static_cast<size_t>(card)] = cards_[static_cast<size_t>(card)]->channel();
  width_tried_[static_cast<size_t>(card)] = false;
  if (card == scout_card_) scout_card_down_ = false;
}

void ChannelCore::shutdown() {
  if (shut_) return;
  shut_ = true;
  scout_run_.store(false);
  if (inflight_started_ && scout_thread2_.joinable()) scout_thread2_.join();
  if (scout_) scout_->stop();
  if (scout_started_ && scout_thread_.joinable()) scout_thread_.join();
}
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
void ChannelCore::freeze_pick_(double t, const char* why) {
  if (!scout_) return;
  scout_->freeze();
  const int mr = (one_card_ && scout_->prelude_done() &&
                  scout_->rounds() < static_cast<uint64_t>(cfg_.radio.scan.min_rounds))
                     ? 1
                     : cfg_.radio.scan.min_rounds;
  const auto all = scout_->ranking();
  bool any = false;
  if (cfg_.radio.width == 40) {
    any = any_pair_ranked(all, cfg_.radio.channels, mr);
  } else {
    for (const auto& e : all) any = any || e.visits >= static_cast<uint32_t>(mr);
  }
  const uint8_t pick = plan_.want();
  if (any) ranker_.set_boot_pick(pick);
  sink_.pick(t, any ? std::optional<uint8_t>(pick) : std::nullopt, scout_->rounds(), all, mr);
  sink_.log(logf_("maburgs channel: pick frozen on %u (%s) after %llu rounds",
                  static_cast<unsigned>(pick), why,
                  static_cast<unsigned long long>(scout_->rounds())));
}

void ChannelCore::apply_hop_action_(const HopAction& act, double now_ms) {
  switch (act.kind) {
    case HopAction::Order:
      vrx_.set_hop(act.target, act.epoch);
      vrx_.restore_rung(act.restore_rung, now_ms);
      vrx_.blank_store(now_ms + cfg_.hop.confirm_ms + 150.0);
      // Two-card: retune the lead now. One-card (lead < 0): the sole radio
      // stays on the OLD channel while the order rides one_card_repeats
      // RCFs; OneCardRetune below moves it.
      if (act.lead_card >= 0) plan_.hop_order(now_ms, act.target, act.lead_card);
      break;
    case HopAction::OneCardRetune:
      plan_.hop_order(now_ms, act.target, -1);
      break;
    case HopAction::Confirm:
      plan_.hop_confirmed(now_ms);
      break;
    case HopAction::Withdraw:
      vrx_.set_hop(act.target, act.epoch);
      plan_.hop_withdraw(now_ms);
      break;
    case HopAction::VerifyPass:
      verdict_.reset();   // thaw the frozen references (spec section 2)
      break;
    case HopAction::Hold:
    case HopAction::None:
      break;
  }
}

void ChannelCore::dispatch_hop_action_(const HopAction& act, bool relocate_tick, double now_ms) {
  apply_hop_action_(act, now_ms);
  boot_pick_.note_hop_action(act.kind, relocate_tick);
  if (auto b = hop_verdict_loss_blank_until(act, now_ms)) s1_hop_loss_.blank_until(*b);
  switch (act.kind) {
    case HopAction::Order:
      hopping_atomic_.store(true);
      rcf_sent_at_order_ = rcf_sent_total_;
      break;
    case HopAction::Confirm:
    case HopAction::Withdraw:
      hopping_atomic_.store(false);
      break;
    default:
      break;
  }
  for (const auto& e : hopc_.take_events()) {
    sink_.hop(e);
    sink_.log(logf_("maburgs hop: %s epoch %u target %u score %u +%.0f ms", e.kind.c_str(),
                    e.epoch, e.target, e.score, e.elapsed_ms));
    last_hop_event_ms_ = static_cast<uint64_t>(e.elapsed_ms >= 0 ? e.elapsed_ms : 0.0);
  }
}

void ChannelCore::step_scout_inputs_() {
  if (!scout_) { scout_working_atomic_.store(false, std::memory_order_relaxed); return; }
  scout_->set_op(plan_.op());
  scout_search_req_ = plan_.release_scout() && !scout_card_down_ &&
                      !(dwell_busy_.load() && dwell_card_.load() == scout_card_);
  scout_->set_search(scout_search_req_);
  scout_->set_tx_frames(ctrl_sent_total_);
  const bool w = scout_owns_();
  if (scout_was_working_ && !w) {
    cur_ch_[static_cast<size_t>(scout_card_)] = cards_[static_cast<size_t>(scout_card_)]->channel();
    width_tried_[static_cast<size_t>(scout_card_)] = false;
  }
  scout_was_working_ = w;
  scout_working_atomic_.store(scout_owns_(), std::memory_order_relaxed);
}

void ChannelCore::step_boot_pick_(const ChannelTickIn& in) {
  const double now_ms = in.now_ms;
  const bool linked = in.in_session || in.cal_running;
  BootPickIn bi;
  bi.now_ms = now_ms;
  bi.since_start_ms = now_ms_u_cur_ - gs_start_ms_;
  bi.max_ms = cfg_.radio.scan.max_ms;
  bi.in_session = in.in_session;
  bi.cal_running = in.cal_running;
  bi.one_card = one_card_;
  if (scout_) {
    bi.scout_mature = scout_->mature();
    bi.scout_op_ranked = scout_->op_ranked();
    bi.scout_prelude_done = scout_->prelude_done();
    bi.proposal = scout_->proposal();
  }
  bi.scout_owns = scout_owns_();
  bi.op = plan_.op();
  bi.want = plan_.want();
  bi.relocate_due = plan_.relocate_due();
  bi.plan_hopping = plan_.hopping();
  bi.hop_active = hop_active(in.in_session, in.cal_running);
  bi.hop_idle_or_hold = hopc_.state() == HopState::Idle || hopc_.state() == HopState::Hold;
  bi.link_edge = link_edge_seen_;
  bi.scout_card_died = scout_card_died_seen_;
  link_edge_seen_ = false;
  scout_card_died_seen_ = false;
  pending_relocate_.reset();
  last_bi_ = bi;
  const BootPickOut bo = boot_pick_.tick(bi);
  switch (bo.kind) {
    case BootPickOut::Commit:
      if (bo.ch != plan_.op())
        sink_.log(logf_("maburgs channel: commit %u -> %u (no link)",
                        static_cast<unsigned>(plan_.op()), static_cast<unsigned>(bo.ch)));
      plan_.commit(now_ms, bo.ch);
      freeze_pick_(now_ms, bo.reason);
      break;
    case BootPickOut::AckPrelude:
      sink_.log(logf_("maburgs channel: one-card prelude ranking picks %u (op %u)%s",
                      static_cast<unsigned>(bi.proposal), static_cast<unsigned>(plan_.op()),
                      linked ? ", linked: not committed" : ""));
      if (bo.ch != 0) plan_.commit(now_ms, bo.ch);
      scout_->ack_prelude(plan_.op());
      break;
    case BootPickOut::WantPick:
      plan_.set_want(now_ms, bo.ch);
      scout_->freeze();
      sink_.log(logf_("maburgs channel: boot pick wants %u, link on %u: relocating",
                      static_cast<unsigned>(bo.ch), static_cast<unsigned>(plan_.op())));
      break;
    case BootPickOut::Relocate:
      pending_relocate_ = bo.ch;
      break;
    case BootPickOut::Freeze:
      if (bo.accept_op) plan_.set_want(now_ms, plan_.op());
      freeze_pick_(now_ms, bo.reason);
      frozen_pick_ = plan_.want();
      break;
    case BootPickOut::AcceptOp:
      sink_.log(logf_("maburgs channel: relocation to %u did not land; staying on %u",
                      static_cast<unsigned>(plan_.want()), static_cast<unsigned>(plan_.op())));
      plan_.set_want(now_ms, plan_.op());
      break;
    case BootPickOut::None:
      break;
  }
  if (bo.kind == BootPickOut::Commit) frozen_pick_ = plan_.want();
}

void ChannelCore::step_store_() {
  if (plan_.op() != saved_op_) {
    saved_op_ = plan_.op();
    vrx_.set_proposal(plan_.op());
    if (!store_(saved_op_))
      sink_.log(logf_("maburgs channel: could not write %s", mabur::kGsChannelFile));
  }
}

void ChannelCore::step_hop_edge_and_window_(const ChannelTickIn& in) {
  const double now_ms = in.now_ms;
  const uint64_t now_ms_u = now_ms_u_cur_;
  const bool active = hop_active(in.in_session, in.cal_running);
  if (active != hop_was_active_) {
    hop_was_active_ = active;
    if (!active) dispatch_hop_action_(hopc_.on_session_lost(now_ms, plan_.op()), false, now_ms);
    verdict_.reset();
    last_verdict_ = Verdict::Healthy;
    last_verdict_out_ = VerdictOut{};
    std::fill(window_prev_ok_.begin(), window_prev_ok_.end(), false);
    recovered_prev_window_ = in.agg ? in.agg->decoder().stats(0).syms_recovered +
                                          in.agg->decoder().stats(1).syms_recovered
                                    : 0;
    au_seq_prev_ = au_seq_.load(std::memory_order_relaxed);
    verdict_.new_session();
    last_window_ms_ = now_ms_u;
  }
  if (!(active && now_ms_u - last_window_ms_ >= static_cast<uint64_t>(cfg_.hop.window_ms))) return;
  last_window_ms_ = now_ms_u;
  std::vector<VerdictCardIn> vc(static_cast<size_t>(n_cards_));
  int starved_valid = 0;
  bool starved_all_zero = true;
  for (int i = 0; i < n_cards_; ++i) {
    auto& fe = *cards_[static_cast<size_t>(i)];
    const size_t si = static_cast<size_t>(i);
    const bool busy = (dwell_busy_.load() && dwell_card_.load() == i) ||
                      (scout_owns_() && i == scout_card_);
    if (!verdict_card_usable(fe.ready(), busy, fe.channel(), plan_.op())) {
      window_prev_ok_[si] = false;
      nhm_win_[si].invalidate();
      continue;
    }
    const NhmBusyRead nb = fe.read_nhm_busy();
    const bool nhm_ok = nhm_win_[si].usable(nb, fe.channel(), dwell_gen_[si].load(std::memory_order_acquire));
    const ScoutEnergy e = fe.read_energy_scout();
    const ScoutFrames f = fe.frames();
    const uint64_t crc_fail = in.agg ? in.agg->card(i).crc_fail : 0;
    const double rssi_raw = in.agg ? in.agg->card(i).rssi_a_ema : 0.0;
    const double snr_raw = in.agg ? in.agg->card(i).snr_ema : 0.0;
    const uint8_t arm_ch = fe.channel();
    const uint32_t arm_gen = dwell_gen_[si].load(std::memory_order_acquire);
    if (fe.arm_nhm_busy(nhm_op_period_)) nhm_win_[si].armed(arm_ch, nhm_op_period_, arm_gen);
    else nhm_win_[si].invalidate();
    if (window_prev_ok_[si]) {
      vc[si].valid = true;
      ++starved_valid;
      if (f.own - window_prev_[si].own != 0) starved_all_zero = false;
      vc[si].fa = e.fa_ofdm;
      vc[si].cca = e.cca_ofdm;
      vc[si].foreign = static_cast<uint32_t>(f.foreign - window_prev_[si].foreign);
      vc[si].crc_fail = static_cast<uint32_t>(crc_fail - window_prev_crc_[si]);
      vc[si].rssi_dbm = rssi_raw_to_dbm(rssi_raw);
      vc[si].snr_db = snr_ok_[si] ? snr_raw_to_db(snr_raw) : std::nan("");
      vc[si].snr_valid = snr_ok_[si];
      const double win_us = static_cast<double>(now_ms_u - window_prev_ms_[si]) * 1000.0;
      const double own_pct = win_us > 0
          ? std::min(100.0, 100.0 * static_cast<double>(f.own_air_us - window_prev_[si].own_air_us) / win_us)
          : 0.0;
      const auto busy_pct = nhm_ok ? nhm_busy_pct(nb, cfg_.hop.verdict.busy_dbm) : std::nullopt;
      vc[si].busy_valid = busy_pct.has_value();
      vc[si].nhm_busy_pct = busy_pct.value_or(0.0);
      vc[si].own_air_pct = own_pct;
      energy_last_[si] = StatsEnergyIn{e.cca_ofdm, e.fa_ofdm, f.own - window_prev_[si].own,
                                       f.foreign - window_prev_[si].foreign, std::nullopt, busy_pct, own_pct};
    }
    window_prev_[si] = f;
    window_prev_crc_[si] = crc_fail;
    window_prev_ms_[si] = now_ms_u;
    window_prev_ok_[si] = true;
  }
  VerdictLinkIn vl;
  const auto s1 = s1_hop_loss_.sample(now_ms);
  vl.pre_fec_loss = s1.valid ? s1.loss : 0.0;
  const uint64_t recovered_now = in.agg ? in.agg->decoder().stats(0).syms_recovered +
                                              in.agg->decoder().stats(1).syms_recovered
                                        : 0;
  vl.recovered = static_cast<uint32_t>(recovered_now - recovered_prev_window_);
  recovered_prev_window_ = recovered_now;
  vl.starved = starved_valid > 0 && starved_all_zero;
  const uint64_t au_now = au_seq_.load(std::memory_order_relaxed);
  vl.au_count = static_cast<uint32_t>(au_now - au_seq_prev_);
  au_seq_prev_ = au_now;
  const auto vo = verdict_.window(now_ms, vc, vl, vrx_.ctl().rung());
  if (vo.v != Verdict::Healthy || vo.v != last_verdict_) sink_.verdict(now_ms, vo, vc, vl);
  last_verdict_ = vo.v;
  last_verdict_out_ = vo;
  if (const auto blank = hop_store_blank_until(vo, cfg_.hop.enable, cfg_.hop.confirm_ms))
    vrx_.blank_store(*blank);
}

void ChannelCore::step_controller_(const ChannelTickIn& in) {
  const double now_ms = in.now_ms;
  if (!hop_active(in.in_session, in.cal_running)) { pending_relocate_.reset(); return; }
  auto fill_hop_targets = [&](HopTick& k) {
    k.best = ranker_.best(now_ms, plan_.op(), hopc_.backed_off(now_ms), /*require_unblocked=*/true);
    k.escape = ranker_.best(now_ms, plan_.op(), hopc_.backed_off_failed(now_ms), /*require_unblocked=*/true);
    k.best_score = 0;
    k.escape_score = 0;
    for (const auto& e : ranker_.ranking(now_ms)) {
      if (k.best && e.ch == *k.best) k.best_score = e.score;
      if (k.escape && e.ch == *k.escape) k.escape_score = e.score;
    }
  };
  HopTick ht;
  ht.now_ms = now_ms;
  ht.verdict = last_verdict_out_;
  ht.cur_op = plan_.op();
  if (hopc_.state() != HopState::Ordered) {
    std::vector<bool> ready(static_cast<size_t>(n_cards_));
    for (int i = 0; i < n_cards_; ++i) ready[static_cast<size_t>(i)] = cards_[static_cast<size_t>(i)]->ready();
    hop_lead_latched_ = pick_hop_lead(ready, in.tx_card);
  }
  ht.lead_card = hop_lead_latched_;
  ht.n_cards = hop_lead_latched_ >= 0 ? n_cards_ : 1;
  fill_hop_targets(ht);
  ht.video_on_target = plan_.hopping() && last_video_ch_ == plan_.hop_target();
  ht.rcf_sent_since_order = static_cast<int>(rcf_sent_total_ - rcf_sent_at_order_);
  const bool relocation_owns = boot_pick_.relocating() || pending_relocate_.has_value() ||
                               boot_pick_.relocation_pending(last_bi_);
  if (!scout_owns_() && !relocation_owns &&
      hop_burst_due(hopc_.state(), last_verdict_out_.trigger, now_ms, last_burst_ms_,
                    cfg_.hop.dwell_period_ms)) {
    last_burst_ms_ = now_ms;
    const int burst_card = pick_burst_card(can_scout_, in.tx_card);
    if (burst_card < 0 || !inflight_) {
      fill_hop_targets(ht);
    } else {
      std::lock_guard<std::mutex> ilk(inflight_mu_);
      auto& fe = *cards_[static_cast<size_t>(burst_card)];
      inflight_->set_radio(fe);
      std::vector<ScoutDwell> recs;
      for (const auto& v : inflight_->burst(plan_.op(), recs)) ranker_.add(v);
      for (const auto& d : recs) sink_.dwell(now_ms, burst_card, d);
      cur_ch_[static_cast<size_t>(burst_card)] = fe.channel();
      nhm_win_[static_cast<size_t>(burst_card)].invalidate();
      fill_hop_targets(ht);
    }
  }
  if (scout_owns_()) {
    ht.best.reset();
    ht.escape.reset();
  } else if (pending_relocate_) {
    ht.verdict.trigger = true;
    ht.relocate = true;
    ht.best = *pending_relocate_;
    ht.best_score = 0;
    ht.escape.reset();
    ht.escape_score = 0;
  } else if (relocation_owns) {
    ht.best.reset();
    ht.escape.reset();
  }
  const HopAction act = hopc_.tick(ht);
  dispatch_hop_action_(act, ht.relocate, now_ms);
  if (ht.relocate)
    sink_.log(logf_("maburgs channel: relocate %u -> %u %s", static_cast<unsigned>(plan_.op()),
                    static_cast<unsigned>(*pending_relocate_),
                    act.kind == HopAction::Order ? "placed" : "refused (hop cap)"));
  pending_relocate_.reset();
}

void ChannelCore::step_move_edge_(const ChannelTickIn& in) {
  const double now_ms = in.now_ms;
  vrx_.set_proposal(plan_.op());
  vrx_.set_keepalive_hold(hopc_.state() == HopState::Ordered);
  if (cal_move_hold_.take(vrx_.take_move_edge(), in.cal_running) && !plan_.hopping()) {
    const uint8_t proposed = vrx_.proposal();
    const uint8_t agreed = vrx_.agreed_channel();
    if (!plan_.member(agreed))
      sink_.log(logf_("maburgs channel: drone acked %u, not in our set; ignored",
                      static_cast<unsigned>(agreed)));
    plan_.on_ack(now_ms, agreed, proposed);
    vrx_.set_proposal(plan_.op());
    link_edge_seen_ = true;
  }
}

void ChannelCore::step_drains_() {
  if (scout_)
    for (const auto& d : scout_->take_dwells()) sink_.dwell(now_ms_cur_, scout_card_, d);
  // In-flight scout thread: started once the boot pick is closed and the
  // boot scout owns no card. Two-card only.
  if (!inflight_started_ && !scout_owns_() && !boot_pick_.open() && n_cards_ >= 2 &&
      (cfg_.hop.enable || cfg_.hop.scout_when_disabled)) {
    inflight_started_ = true;
    if (cfg_.threaded) {
      scout_run_.store(true);
      scout_thread2_ = std::thread([this] { scout_loop_(); });
    }
  }
  std::vector<std::pair<int, ScoutDwell>> drained;
  std::vector<HopVisit> visits;
  {
    std::lock_guard<std::mutex> lk(dwell_mu_);
    drained.swap(dwell_recs_);
    visits.swap(dwell_visits_);
  }
  size_t vi = 0;
  for (auto& rec : drained) {
    const int card = rec.first;
    cur_ch_[static_cast<size_t>(card)] = cards_[static_cast<size_t>(card)]->channel();
    sink_.dwell(now_ms_cur_, card, rec.second);
    const bool ok = !(rec.second.survey.flags & devourer::chanmig::kFlagRetuneFailed);
    StatsDwellIn ds = dwell_stats_[static_cast<size_t>(card)].value_or(StatsDwellIn{});
    ++ds.visits;
    ds.cost_us = static_cast<uint32_t>(rec.second.to_us + rec.second.read_us + rec.second.back_us);
    if (ok && vi < visits.size()) ds.score = HopRanker::score(visits[vi]);
    dwell_stats_[static_cast<size_t>(card)] = ds;
    if (ok) ++vi;
  }
  for (const auto& v : visits) ranker_.add(v);
}

void ChannelCore::step_width_resync_() {
  if (!width_resync_open(!scout_owns_(), scout_ != nullptr, /*scout_frozen=*/!boot_pick_.open())) return;
  for (int i = 0; i < n_cards_; ++i) {
    auto& fe = *cards_[static_cast<size_t>(i)];
    const WidthCard wc{fe.ready(), fe.width(), width_tried_[static_cast<size_t>(i)],
                       dwell_busy_.load() && dwell_card_.load() == i};
    if (!needs_width_fix(wc, cfg_.radio.width)) continue;
    width_tried_[static_cast<size_t>(i)] = true;
    std::lock_guard<std::mutex> ilk(inflight_mu_);
    const uint8_t ch = fe.channel();
    if (fe.set_width(ch, cfg_.radio.width)) {
      cur_ch_[static_cast<size_t>(i)] = fe.channel();
      nhm_win_[static_cast<size_t>(i)].invalidate();
    } else {
      sink_.log(logf_("maburgs radio: card %d width resync to %u MHz on ch %u failed", i,
                      static_cast<unsigned>(cfg_.radio.width), static_cast<unsigned>(ch)));
    }
  }
}

void ChannelCore::step_mechanical_retune_() {
  for (int i = 0; i < n_cards_; ++i) {
    const bool scouting = scout_owns_() && i == scout_card_;
    const bool inflight_dwelling = dwell_busy_.load() && dwell_card_.load() == i;
    auto& fe = *cards_[static_cast<size_t>(i)];
    if (scouting || inflight_dwelling || !fe.ready()) continue;
    const uint8_t want = plan_.desired(i);
    if (cur_ch_[static_cast<size_t>(i)] != want && fe.retune(want)) {
      cur_ch_[static_cast<size_t>(i)] = want;
      nhm_win_[static_cast<size_t>(i)].invalidate();
    }
  }
}

void ChannelCore::inflight_body_() {
  if (!inflight_) return;
  if (!in_session_atomic_.load() || cal_running_atomic_.load() || hopping_atomic_.load() ||
      scout_working_atomic_.load() || n_cards_ < 2)
    return;
  const int card = pick_inflight_scout(can_scout_, tx_card_now_.load());
  if (card < 0) return;
  auto& fe = *cards_[static_cast<size_t>(card)];
  if (!fe.ready()) return;
  std::optional<uint8_t> dwell_ch;
  {
    std::lock_guard<std::mutex> ilk(inflight_mu_);
    dwell_ch = inflight_->next_candidate(fe.channel());
  }
  if (!dwell_ch) return;
  if (cfg_.threaded) {
    const uint64_t s0 = au_seq_.load();   // align to the next AU boundary (<= 17 ms wait)
    for (int i = 0; i < 20 && au_seq_.load() == s0; ++i) sleep_(1);
  }
  std::lock_guard<std::mutex> ilk(inflight_mu_);
  inflight_->set_radio(fe);
  dwell_card_.store(card);
  dwell_busy_.store(true);
  ScoutDwell d;
  HopVisit v;
  const bool ok = inflight_->dwell(*dwell_ch, fe.channel(), d, v);
  dwell_gen_[static_cast<size_t>(card)].fetch_add(1, std::memory_order_release);
  dwell_busy_.store(false);
  dwell_card_.store(-1);
  std::lock_guard<std::mutex> lk(dwell_mu_);
  dwell_recs_.emplace_back(card, d);
  if (ok) dwell_visits_.push_back(v);
}

void ChannelCore::scout_loop_() {
  while (scout_run_.load()) {
    sleep_(cfg_.hop.dwell_period_ms);
    if (!scout_run_.load()) break;
    inflight_body_();
  }
}

}  // namespace maburgs
