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
  s.on_au_complete(1000);
  CHECK(!offer(s, 1, 1005, false));
  CHECK(s.take_due(1005).empty());
  CHECK(s.passthru() == 1);
}

TEST(no_recent_video_passes_through) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  CHECK(!offer(s, 1, 500, false));          // never saw an AU
  s.on_au_complete(1000);
  CHECK(!offer(s, 2, 1200, false));         // 200 ms > video_recent_ms
  CHECK(s.passthru() == 2);
  CHECK(s.take_due(1200).empty());
}

TEST(bypass_flag_passes_through) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000);
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
  s.on_au_complete(1178);
  CHECK(offer(s, 1, 1182, false));              // held (1182+3 !< 1187-1)
  s.on_au_complete(1184);                       // 1184+3 !< 1186 -> skipped
  CHECK(s.take_due(1184).empty());
  s.on_au_first(1187);                          // next burst starts; due ~1204
  s.on_au_complete(1195);                       // 1195+3 < 1203 -> release
  auto due = s.take_due(1195);
  CHECK(due.size() == 1 && due[0].reason == SlotReason::Au);
}

TEST(grace_only_when_idle_ahead) {
  RcfSlotter s = cadenced();                    // next burst due ~1187
  s.on_au_complete(1184);
  CHECK(offer(s, 1, 1185, false));              // in grace but no idle ahead -> held
  s.on_au_first(1187);
  s.on_au_complete(1195);
  auto due = s.take_due(1195);
  CHECK(due.size() == 1);
}

TEST(no_first_body_history_means_every_completion_releases) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000);
  CHECK(offer(s, 1, 1008, false));
  s.on_au_complete(1017);
  CHECK(s.take_due(1017).size() == 1);
}


TEST(held_until_next_au_completion) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000);
  CHECK(offer(s, 7, 1008, false));          // 8 ms after the AU: mid-burst risk
  CHECK(s.take_due(1010).empty());
  CHECK(s.take_due(1016).empty());
  s.on_au_complete(1017);
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
  s.on_au_complete(1000);
  SlotFrame f = sf(1);
  CHECK(!s.offer(f, 1001, false));             // 1 ms after the AU: still idle
  CHECK(f.reason == SlotReason::Grace);
  CHECK(s.released_au() == 1);
  CHECK(s.passthru() == 0);
}

TEST(hold_expires_after_hold_max) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000);
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
  s.on_au_complete(1000);
  CHECK(offer(s, 1, 1005, false));
  CHECK(offer(s, 2, 1010, false));
  CHECK(offer(s, 3, 1015, false));
  s.on_au_complete(1016);
  auto due = s.take_due(1016);
  CHECK(due.size() == 3);
  CHECK(due[0].seq == 1 && due[1].seq == 2 && due[2].seq == 3);
  CHECK(s.released_au() == 3);
}

TEST(hold_timer_counts_from_oldest_pending) {
  RcfSlotter s(RcfSlotCfg{20, 100, 2, 3, 1});
  s.on_au_complete(1000);
  CHECK(offer(s, 1, 1005, false));
  CHECK(offer(s, 2, 1020, false));
  CHECK(s.take_due(1024).empty());
  CHECK(s.take_due(1025).size() == 2);         // 1005 + 20
}

MTEST_MAIN
