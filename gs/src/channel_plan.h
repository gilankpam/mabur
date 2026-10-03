#pragma once
#include <cstdint>
#include <vector>

#include "mabur/channel_set.h"

namespace maburgs {

enum class MoveReason {
  Commit, AckOverride, HopLead, HopFollow, HopWithdraw, HopOneCard, LinkFound
};
const char* to_string(MoveReason r);

struct MoveEvent {
  double t_ms = 0;
  int card = -1;  // -1 = all cards
  uint8_t from = 0, to = 0;
  MoveReason reason = MoveReason::Commit;
};

struct ChannelPlanCfg {
  uint8_t start = 0;
  std::vector<uint8_t> channels;
  int n_cards = 2;
  int search_after_ms = 5000;
};

// Where the link lives (spec 2026-10-03-auto-channel-set §4.1, GS side).
// Pure: the caller passes the clock and the rendezvous state; the plan
// answers "which channel should card i be on", and records the moves that
// change where the link lives. There is no home channel any more -- the
// drone picks its own channel from the set and the GS scout sweeps the set
// looking for it; this plan only has to say when the spare card is free to
// go scout (release_scout()) and track op_, the channel the link (and the
// non-scouting card) sits on.
//
// want_ (final review C1, 2026-10-04): the channel the link SHOULD live on
// -- the pin, else the start channel, later the committed pick. A DISC only
// finds the drone: when the ack that opens a session was heard on a member
// X != op, link_found() moves op_ to X for every card (the link forms
// where the drone is), and the caller relocates it to want_ with an
// ordinary hop order (relocate_due()). Every move of a linked link is a
// hop order.
class ChannelPlan {
 public:
  explicit ChannelPlan(ChannelPlanCfg cfg);
  void tick(double now_ms, bool in_session);
  // agreed == op_, or agreed not a set member: no-op (nothing changed, or
  // the drone named a channel outside the agreed set -- ignore it rather
  // than move op_ off the set). Otherwise op_ follows agreed; Commit if the
  // drone agreed with what the GS proposed, AckOverride if it insisted on
  // something else.
  void on_ack(double now_ms, uint8_t agreed, uint8_t proposed);
  // The GS's own pick. Not a set member: no-op. Sets want_; with no session
  // (as of the last tick()) also op_ -- the cards go there and beacon, M
  // "commit" -- while in session op_ stays and the caller relocates the
  // link with a hop order (relocate_due()).
  void commit(double now_ms, uint8_t to);
  // Where the link should live, set directly (members only): the boot pick
  // accepting the channel the link is on, or wanting a pick while linked.
  void set_want(double now_ms, uint8_t ch);
  // The ack that opened the session was heard on x: the link forms there.
  // op_ = x for every card, M all <op> <x> link_found. x == op_, not a
  // member, or a hop in flight: no-op.
  void link_found(double now_ms, uint8_t x);
  // In session, no hop in flight, and the link is not where it should be.
  bool relocate_due() const { return in_session_ && !hopping_ && op_ != want_; }
  uint8_t want() const { return want_; }
  uint8_t op() const { return op_; }
  bool member(uint8_t ch) const;
  // The spare card may leave op_ and go scout the rest of the set: not in
  // session, past search_after_ms since loss (or never linked at all, in
  // which case there is nothing on op_ worth protecting and it may search
  // immediately), and no hop in flight.
  bool release_scout() const;
  uint8_t desired(int card) const;
  std::vector<MoveEvent> take_events();

  // In-flight channel hop: one card leads onto target, the other keeps the
  // link alive on op_ until hop_confirmed() (video seen on target) or
  // hop_withdraw() (no video, lead card returns). op_ only moves on confirm,
  // and want_ moves with it: a confirmed hop -- a relocation or a reactive
  // one -- is where the link should live from then on.
  void hop_order(double now_ms, uint8_t target, int lead_card);
  // Both are no-ops when no hop is in flight: the caller's hop state
  // machine and this plan do not enter hopping_ at the same instant (a
  // one-card Order defers hop_order() until OneCardRetune), so a withdraw
  // can legitimately arrive with nothing to withdraw.
  void hop_confirmed(double now_ms);
  void hop_withdraw(double now_ms);
  // Does a video body received on rx_ch keep the session alive? Only where
  // the link lives: op, or the hop target while a hop is in flight (the lead
  // card's confirming frames). Scout dwells elsewhere must not -- bench
  // 2026-09-26: home-dwell frames held a split GS in SESSION for 60 s.
  // rx_ch 0 = mid-retune or a replay source: unknown, counts.
  bool is_link_video(uint8_t rx_ch) const {
    return rx_ch == 0 || rx_ch == op_ || (hopping_ && rx_ch == hop_target_);
  }
  bool hopping() const { return hopping_; }
  uint8_t hop_target() const { return hop_target_; }
  int hop_lead() const { return hop_lead_; }

 private:
  ChannelPlanCfg cfg_;
  uint8_t op_;
  uint8_t want_;
  bool in_session_ = false;
  bool ever_linked_ = false;
  bool have_lost_since_ = false;
  double lost_since_ms_ = 0;
  double now_ms_ = 0;
  std::vector<MoveEvent> events_;

  bool hopping_ = false;
  uint8_t hop_target_ = 0;
  int hop_lead_ = -1;
  uint8_t hop_from_ = 0;
  double hop_start_ms_ = 0;
};

}  // namespace maburgs
