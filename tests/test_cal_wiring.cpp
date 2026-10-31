// cal_wiring: the drone's calibration replies out of the tunnel into the
// session, the sideport's link.cal input, and the run's stderr narration.
#include "mtest.h"
#include "cal_wiring.h"
#include "cal_control.h"
#include "ladder_controller.h"
#include "cal_session.h"
#include "mabur/cal_mav.h"
#include "mabur/mav.h"

#include <string>
#include <variant>
#include <vector>

using namespace maburgs;
namespace cal = mabur::cal;
using State = CalSession::State;

namespace {
const std::vector<cal::SweepSpec> kRun = {{cal::Kind::Power, 20}, {cal::Kind::Capacity, 20}};

std::vector<uint8_t> drone_packet(const cal::CalMsg& m) {
  return cal::encode(m, mabur::mav::kSysId, 191, mabur::mav::kChanGsLocal);
}

CalSnapshot snap(State st, size_t sweep_i, cal::SweepSpec cur = {cal::Kind::Power, 20}) {
  CalSnapshot s;
  s.state = st;
  s.sweep_i = sweep_i;
  s.sweep_n = kRun.size();
  s.cur = cur;
  s.channel = 149;
  s.pick_note = "auto: 149 is the best member";
  return s;
}
}  // namespace

// --- feed_cal_packet ----------------------------------------------------

TEST(air_bytes_count_the_qos_data_header_and_the_fcs) {
  // The drone's RadioTx sends every calibration frame as QoS-Data: a
  // 24-byte dot11 header plus 2 bytes of QoS control, then the body, then
  // the 4-byte FCS.
  CHECK(kCalDot11HeaderLen == 26);
  CHECK(cal_air_bytes(mabur::cal::kCalPayloadLen) == 26 + 64 + 4);
  CHECK(cal_air_bytes(1405) == 1435);
}

TEST(a_sweep_ack_from_the_tunnel_moves_the_session_on) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(77, kRun, 1000, &err));
  auto cmd = s.due_mav(1000);
  REQUIRE(cmd.has_value());
  const auto* sc = std::get_if<cal::SweepCmd>(&*cmd);
  REQUIRE(sc != nullptr);
  cal::SweepAck ack;
  ack.nonce = sc->nonce;
  ack.seq = sc->seq;
  ack.start_in_ms = 0;
  const auto pkt = drone_packet(cal::CalMsg{ack});
  feed_cal_packet(s, pkt.data(), pkt.size(), 1010);
  CHECK(s.state() == State::Sweep);
}

TEST(garbage_and_out_of_place_messages_change_nothing) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(77, kRun, 1000, &err));
  REQUIRE(s.due_mav(1000).has_value());
  const std::vector<uint8_t> junk = {0xFD, 1, 2, 3, 4, 5};
  feed_cal_packet(s, junk.data(), junk.size(), 1010);
  cal::SweepDone d;   // a SweepDone before the ack: not this state's
  d.nonce = 77;
  const auto pkt = drone_packet(cal::CalMsg{d});
  feed_cal_packet(s, pkt.data(), pkt.size(), 1010);
  CHECK(s.state() == State::AwaitAck);
}

// --- cal_sideport --------------------------------------------------------

TEST(sideport_block_is_absent_while_no_run_owns_the_air) {
  CalSession s(CalSessionCfg{});
  CHECK(!cal_sideport(s).present);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(77, kRun, 1000, &err));
  const StatsCalIn in = cal_sideport(s);
  CHECK(in.present);
  CHECK(in.state == "await_ack");
  CHECK(in.sweep_k == 1 && in.sweep_n == 2);
  CHECK(in.kind == "power");
  CHECK(in.bw == 20);
  s.abort("operator abort");
  CHECK(!cal_sideport(s).present);
}

// --- naming helpers ------------------------------------------------------

TEST(sweep_tokens_and_kind_names) {
  CHECK(cal_sweep_token({cal::Kind::Power, 40}) == "power40");
  CHECK(cal_sweep_token({cal::Kind::Capacity, 20}) == "cap20");
  CalSnapshot v = snap(State::Verify, 0);
  CHECK(std::string(cal_kind_name(v)) == "verify");
  CHECK(std::string(cal_kind_name(snap(State::Sweep, 1, {cal::Kind::Capacity, 20}))) ==
        "capacity");
  CHECK(std::string(cal_state_name(State::AwaitDone)) == "await_done");
}

// --- CalNarrator ---------------------------------------------------------

TEST(narrator_names_the_run_once_at_the_first_command) {
  CalNarrator n("/x/cal.log");
  CHECK(n.observe(snap(State::Idle, 0), kRun).empty());
  CHECK(n.observe(snap(State::Picking, 0), kRun).empty());
  auto l = n.observe(snap(State::AwaitAck, 0), kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] ==
        "calibration started -> /x/cal.log sweeps=[power20,cap20] ch=149 "
        "(auto: 149 is the best member)");
  CHECK(n.observe(snap(State::Sweep, 0), kRun).empty());
  CHECK(n.observe(snap(State::AwaitDone, 0), kRun).empty());
}

TEST(narrator_reports_each_apply_each_sweep_and_the_end) {
  CalNarrator n("/x/cal.log");
  n.observe(snap(State::Idle, 0), kRun);
  n.observe(snap(State::AwaitAck, 0), kRun);
  n.observe(snap(State::Apply, 0), kRun);
  auto l = n.observe(snap(State::Verify, 0), kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] == "calibration apply 1/2 power20: ok");
  l = n.observe(snap(State::AwaitAck, 1, {cal::Kind::Capacity, 20}), kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] == "calibration sweep 1/2 power20 done");
  n.observe(snap(State::Apply, 1, {cal::Kind::Capacity, 20}), kRun);
  l = n.observe(snap(State::Done, 1, {cal::Kind::Capacity, 20}), kRun);
  REQUIRE(l.size() == 3);
  CHECK(l[0] == "calibration apply 2/2 cap20: ok");
  CHECK(l[1] == "calibration sweep 2/2 cap20 done");
  CHECK(l[2] == "calibration done");
  CHECK(n.observe(snap(State::Done, 1, {cal::Kind::Capacity, 20}), kRun).empty());
}

// Revert (always "ok"): the drone's note never reaches /tmp/maburgs.log.
TEST(narrator_carries_the_drones_note_on_an_adopted_apply) {
  CalNarrator n("/x/cal.log");
  n.observe(snap(State::AwaitAck, 0, {cal::Kind::Power, 40}), kRun);
  n.observe(snap(State::Apply, 0, {cal::Kind::Power, 40}), kRun);
  CalSnapshot v = snap(State::Verify, 0, {cal::Kind::Power, 40});
  v.width[1].applied = true;
  v.width[1].apply_reason = "ok; mcs6 kept from the 20 MHz table";
  auto l = n.observe(v, kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] == "calibration apply 1/2 power40: ok; mcs6 kept from the 20 MHz table");
}

TEST(narrator_reports_a_refused_apply_with_its_reason) {
  CalNarrator n("/x/cal.log");
  n.observe(snap(State::AwaitAck, 0), kRun);
  n.observe(snap(State::Apply, 0), kRun);
  CalSnapshot f = snap(State::Failed, 0);
  f.fail_reason = "apply refused by the drone: no offset mode";
  auto l = n.observe(f, kRun);
  REQUIRE(l.size() == 2);
  CHECK(l[0] == "calibration apply 1/2 power20: apply refused by the drone: no offset mode");
  CHECK(l[1] == "calibration failed: apply refused by the drone: no offset mode");
}

TEST(narrator_reports_an_abort) {
  CalNarrator n("/x/cal.log");
  n.observe(snap(State::AwaitAck, 0), kRun);
  n.observe(snap(State::Sweep, 0), kRun);
  CalSnapshot a = snap(State::Idle, 0);
  a.fail_reason = "operator abort";
  auto l = n.observe(a, kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] == "calibration aborted: operator abort");
  // A sweep with no apply goes straight on: one done line, no apply line.
  CalNarrator m("/x/cal.log");
  m.observe(snap(State::AwaitDone, 0), kRun);
  l = m.observe(snap(State::AwaitAck, 1, {cal::Kind::Capacity, 20}), kRun);
  REQUIRE(l.size() == 1);
  CHECK(l[0] == "calibration sweep 1/2 power20 done");
}

// --- effective width -----------------------------------------------------

TEST(effective_width_is_the_narrower_of_the_gs_and_the_linked_drone) {
  CHECK(cal_effective_width(40, /*linked=*/true, /*peer_width=*/20) == 20);
  CHECK(cal_effective_width(40, true, 40) == 40);
  CHECK(cal_effective_width(20, true, 40) == 20);
  // No DiscAck width yet, or no link: the GS's own width.
  CHECK(cal_effective_width(40, true, 0) == 40);
  CHECK(cal_effective_width(40, false, 20) == 40);
}

TEST(a_bare_start_on_a_40_gs_linked_to_a_20_drone_runs_two_sweeps) {
  LadderCfg ladder;
  ladder.ladder.push_back({4, 0.5, 0.5, 40});
  const uint8_t w = cal_effective_width(40, true, 20);
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  REQUIRE(CalControl::apply("start", s, &reply, default_sweeps(w, ladder), w));
  REQUIRE(s.sweeps().size() == 2);
  CHECK(s.sweeps()[0].bw == 20 && s.sweeps()[1].bw == 20);
  // And a 40 token is refused against that drone.
  s.abort("x");
  CHECK(!CalControl::apply("start power40", s, &reply, default_sweeps(w, ladder), w));
  CHECK(reply == "err refused: GS radio.width is 20");
}

// --- CalSession::is_running ----------------------------------------------

TEST(is_running_is_await_ack_through_verify) {
  CHECK(!CalSession::is_running(State::Idle));
  CHECK(!CalSession::is_running(State::Picking));
  CHECK(CalSession::is_running(State::AwaitAck));
  CHECK(CalSession::is_running(State::Analyze));
  CHECK(CalSession::is_running(State::Verify));
  CHECK(!CalSession::is_running(State::Done));
  CHECK(!CalSession::is_running(State::Failed));
}

MTEST_MAIN
