// RcfSlotter: hold GS control frames until the drone's inter-AU idle
// (gs-uplink-self-blanking findings 2026-09-02). Time is ms.
#include "mtest.h"
#include "rcf_slot.h"

using namespace maburgs;

static SlotFrame sf(uint16_t seq) {
  return SlotFrame{{0x40, static_cast<uint8_t>(seq)}, seq, 0, true};
}
static bool offer(RcfSlotter& s, uint16_t seq, uint64_t now, bool bypass) {
  SlotFrame f = sf(seq);
  return s.offer(f, now, bypass);
}

TEST(disabled_passes_everything_through) {
  RcfSlotter s(RcfSlotCfg{0, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(!offer(s, 1, 1005, false));
  CHECK(s.take_due(1005).empty());
  CHECK(s.passthru() == 1);
}

TEST(no_recent_video_passes_through) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  CHECK(!offer(s, 1, 500, false));          // never saw an AU
  s.on_au_complete(1000, false);
  CHECK(!offer(s, 2, 1200, false));         // 200 ms > video_recent_ms
  CHECK(s.passthru() == 2);
  CHECK(s.take_due(1200).empty());
}

TEST(bypass_flag_passes_through) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(!offer(s, 1, 1008, true));
  CHECK(s.passthru() == 1);
}

// Burst-cadence rule: first bodies every ~16.7 ms; a completion too close
// to the next predicted burst must NOT release (the send would land in it).
static RcfSlotter cadenced() {
  RcfSlotter s(RcfSlotCfg{40, 100, 2, 3, 1});
  for (uint64_t t = 0; t <= 10; ++t) s.on_au_first(1000 + t * 17);  // last first at 1170
  return s;
}

TEST(period_is_learned_from_first_bodies) {
  RcfSlotter s = cadenced();
  CHECK(s.period_ms() > 16.5 && s.period_ms() < 17.1);
}

TEST(completion_close_to_next_burst_is_skipped_then_next_releases) {
  RcfSlotter s = cadenced();                    // next burst due ~1187
  s.on_au_complete(1178, false);
  CHECK(offer(s, 1, 1182, false));              // held (1182+3 !< 1187-1)
  s.on_au_complete(1184, false);                // 1184+3 !< 1186 -> skipped
  CHECK(s.take_due(1184).empty());
  s.on_au_first(1187);                          // next burst starts; due ~1204
  s.on_au_complete(1195, false);                // 1195+3 < 1203 -> release
  auto due = s.take_due(1195);
  CHECK(due.size() == 1 && due[0].reason == SlotReason::Au);
}

TEST(grace_only_when_idle_ahead) {
  RcfSlotter s = cadenced();                    // next burst due ~1187
  s.on_au_complete(1184, false);
  CHECK(offer(s, 1, 1185, false));              // in grace but no idle ahead -> held
  s.on_au_first(1187);
  s.on_au_complete(1195, false);
  auto due = s.take_due(1195);
  CHECK(due.size() == 1);
}

TEST(no_first_body_history_means_every_completion_releases) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(offer(s, 1, 1008, false));
  s.on_au_complete(1017, false);
  CHECK(s.take_due(1017).size() == 1);
}


TEST(held_until_next_au_completion) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(offer(s, 7, 1008, false));          // 8 ms after the AU: mid-burst risk
  CHECK(s.take_due(1010).empty());
  CHECK(s.take_due(1016).empty());
  s.on_au_complete(1017, false);
  auto due = s.take_due(1017);
  CHECK(due.size() == 1);
  CHECK(due[0].seq == 7);
  CHECK(due[0].reason == SlotReason::Au);
  CHECK(due[0].offered_ms == 1008);
  CHECK(s.released_au() == 1);
  CHECK(s.released_timeout() == 0);
  CHECK(s.take_due(1018).empty());             // nothing pending now
}

TEST(within_grace_after_au_goes_now_and_counts_as_au) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  SlotFrame f = sf(1);
  CHECK(!s.offer(f, 1001, false));             // 1 ms after the AU: still idle
  CHECK(f.reason == SlotReason::Grace);
  CHECK(s.released_au() == 1);
  CHECK(s.passthru() == 0);
}

TEST(hold_expires_after_hold_max) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(offer(s, 3, 1010, false));
  CHECK(s.take_due(1029).empty());
  auto due = s.take_due(1030);                 // 20 ms hold reached
  CHECK(due.size() == 1);
  CHECK(due[0].seq == 3);
  CHECK(due[0].reason == SlotReason::Timeout);
  CHECK(s.released_timeout() == 1);
}

TEST(multiple_pending_release_together_in_order) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(offer(s, 1, 1005, false));
  CHECK(offer(s, 2, 1010, false));
  CHECK(offer(s, 3, 1015, false));
  s.on_au_complete(1016, false);
  auto due = s.take_due(1016);
  CHECK(due.size() == 3);
  CHECK(due[0].seq == 1 && due[1].seq == 2 && due[2].seq == 3);
  CHECK(s.released_au() == 3);
}

TEST(hold_timer_counts_from_oldest_pending) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000, false);
  CHECK(offer(s, 1, 1005, false));
  CHECK(offer(s, 2, 1020, false));
  CHECK(s.take_due(1024).empty());
  CHECK(s.take_due(1025).size() == 2);         // 1005 + 20
}

// --- probe-tail (spec 2026-09-04 slotter-tail): the probe body trails every
// enh AU, so a release armed at completion must wait out its airtime.
// (No on_au_first() history here except where the test is specifically about
// cadence -- with no first-body history idle_ahead() is trivially true,
// which is what isolates the tail mechanic under test, same as the existing
// no_first_body_history_means_every_completion_releases test above.)

TEST(enh_completion_does_not_release_until_the_probe_is_seen) {
  // The probe body is the last PPDU of every ENH burst, and a send issued
  // at the completion goes on air right where it lands (bench 2026-09-05:
  // the probe arrives 0.9 ms p50 / 4 ms p99 after the completion stamp,
  // the send's USB+chip latency is 1-1.5 ms). So an ENH completion arms
  // nothing by itself; the probe's own arrival is the release.
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  RcfSlotter s(cfg);
  s.on_au_complete(0, false);          // establish "video flowing"
  CHECK(offer(s, 1, 10, false));       // outside grace -> held
  s.on_au_complete(20, true);
  CHECK(s.take_due(20).empty());
  CHECK(s.take_due(21).empty());
  s.on_probe_tail(21);
  auto due = s.take_due(21);
  REQUIRE(due.size() == 1);
  CHECK(due[0].reason == SlotReason::Probe);
  CHECK(s.released_probe() == 1);
  CHECK(s.released_au() == 0);
}

TEST(lost_probe_falls_back_to_the_learned_deadline) {
  // No probe ever arrives (lost on air): the hold must still end at
  // completion + tail_ub, not at hold_max. tail_ub starts from
  // probe_tail_ms (the body's own airtime, a physical floor) + 1 ms.
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  RcfSlotter s(cfg);
  s.on_au_complete(0, false);
  CHECK(offer(s, 1, 10, false));
  s.on_au_complete(20, true);
  CHECK(s.take_due(22).empty());
  auto due = s.take_due(23);           // 20 + (2 + 1)
  REQUIRE(due.size() == 1);
  CHECK(due[0].reason == SlotReason::Au);
  CHECK(s.released_au() == 1);
}

TEST(deadline_learns_from_observed_completion_to_probe_offsets) {
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 1;
  RcfSlotter s(cfg);
  s.on_au_complete(0, true);
  s.on_probe_tail(5);                  // observed offset 5 ms > floor 1
  CHECK(s.tail_ub_ms() == 6);          // ceil(5) + 1
  CHECK(offer(s, 1, 90, false));
  s.on_au_complete(100, true);         // this one's probe is lost
  CHECK(s.take_due(105).empty());
  CHECK(s.take_due(106).size() == 1);
  // The estimate decays back toward the floor when offsets shrink.
  for (int i = 0; i < 400; ++i) { s.on_au_complete(200 + i * 10, true); s.on_probe_tail(201 + i * 10); }
  CHECK(s.tail_ub_ms() == 2);          // floor 1 + 1
}

TEST(spent_deadline_does_not_release_a_later_offer) {
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  RcfSlotter s(cfg);
  s.on_au_complete(20, true);           // nothing pending; probe then lost
  CHECK(s.take_due(30).empty());        // deadline (23) passes unused
  CHECK(offer(s, 1, 40, false));        // outside grace -> held
  CHECK(s.take_due(40).empty());        // the stale deadline must not fire
  CHECK(s.take_due(45).empty());
  s.on_au_complete(50, false);          // the next completion is the slot
  auto due = s.take_due(50);
  REQUIRE(due.size() == 1);
  CHECK(due[0].reason == SlotReason::Au);
}

TEST(probe_arrival_too_close_to_the_next_burst_does_not_release) {
  RcfSlotCfg cfg{40, 100, 2, 3, 1};
  cfg.probe_tail_ms = 1;
  RcfSlotter s(cfg);
  s.on_au_complete(999, false);
  s.on_au_first(1000);
  s.on_au_first(1016);                  // period ~16.6 ms; next due ~1032.6
  CHECK(offer(s, 1, 1020, false));
  s.on_au_complete(1027, true);
  s.on_probe_tail(1029);                // 1029+3+1 = 1033 !< 1032.6 -> not idle ahead
  CHECK(s.take_due(1029).empty());
  CHECK(s.released_probe() == 0);
  // ... and the deadline path (1027 + 2 = 1029, already past) must not
  // release it either.
  CHECK(s.take_due(1030).empty());
  s.on_au_first(1033);
  s.on_au_complete(1040, false);        // next (base) completion is the slot
  auto due = s.take_due(1040);
  REQUIRE(due.size() == 1);
  CHECK(due[0].reason == SlotReason::Au);
}

TEST(probe_arrival_opens_the_grace_window) {
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  cfg.grace_ms = 2;
  RcfSlotter s(cfg);
  s.on_au_complete(20, true);
  CHECK(offer(s, 1, 21, false));        // before the probe: the burst is still on air
  s.on_probe_tail(24);                  // later than the 2 ms estimate
  CHECK(s.take_due(24).size() == 1);    // the held frame goes out on the probe
  SlotFrame f2 = sf(2);
  CHECK(!s.offer(f2, 26, false));       // 2 ms after the probe: grace
  CHECK(f2.reason == SlotReason::Grace);
  SlotFrame f3 = sf(3);
  CHECK(s.offer(f3, 27, false));        // 3 ms after: outside grace, held
}

TEST(probe_tail_not_applied_to_a_base_au) {
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  RcfSlotter s(cfg);
  s.on_au_complete(0, false);
  CHECK(offer(s, 1, 10, false));
  s.on_au_complete(20, false);
  auto due = s.take_due(20);
  CHECK(due.size() == 1 && due[0].reason == SlotReason::Au);
}

TEST(probe_tail_blocks_the_grace_window) {
  RcfSlotCfg cfg{30, 100, 2, 3, 1};
  cfg.probe_tail_ms = 2;
  cfg.grace_ms = 3;
  RcfSlotter s(cfg);
  s.on_au_complete(20, true);                  // establishes have_au_/last_au_ms_
  CHECK(offer(s, 1, 21, false));                // since(1) < tail(2) -> held
  SlotFrame f2 = sf(2);
  CHECK(!s.offer(f2, 23, false));               // since(3) in [tail,grace] -> now
  CHECK(f2.reason == SlotReason::Grace);
  CHECK(s.released_au() == 1);
}

TEST(lost_probe_deadline_checks_idle_ahead_at_the_deadline) {
  // The tail is NOT charged at the completion any more (the release does
  // not happen there); it is the deadline instant that must be idle-ahead.
  RcfSlotCfg cfg{40, 100, 2, 3, 1};
  cfg.probe_tail_ms = 5;
  RcfSlotter s(cfg);
  s.on_au_complete(999, false);
  s.on_au_first(1000);
  s.on_au_first(1016);                  // period learned ~16.6 ms; next due ~1032.6
  CHECK(offer(s, 1, 1020, false));      // held
  s.on_au_complete(1025, true);
  CHECK(s.take_due(1025).empty());
  // deadline 1025 + (5+1) = 1031: 1031+3+1 = 1035 !< 1032.6 -> skipped
  CHECK(s.take_due(1031).empty());
  CHECK(s.take_due(1059).empty());      // still not due; hold hasn't expired
  auto due = s.take_due(1060);          // hold_max (40) reached from hold_start=1020
  CHECK(due.size() == 1 && due[0].reason == SlotReason::Timeout);
}

TEST(probe_tail_not_charged_against_a_base_au_completion) {
  RcfSlotCfg cfg{40, 100, 2, 3, 1};
  cfg.probe_tail_ms = 5;
  RcfSlotter s(cfg);
  s.on_au_complete(999, false);
  s.on_au_first(1000);
  s.on_au_first(1016);
  CHECK(offer(s, 1, 1020, false));
  s.on_au_complete(1025, false);
  auto due = s.take_due(1025);
  CHECK(due.size() == 1 && due[0].reason == SlotReason::Au);
}

TEST(set_probe_tail_ms_clamps_negative_to_zero) {
  RcfSlotCfg cfg{20, 100, 2, 3, 1};
  RcfSlotter s(cfg);
  s.set_probe_tail_ms(-5);
  s.on_au_complete(1000, true);         // pending empty: just sets last_au_ms_
  SlotFrame f = sf(1);
  CHECK(!s.offer(f, 1001, false));      // grace still works => tail clamped to 0
  CHECK(f.reason == SlotReason::Grace);
}

MTEST_MAIN
