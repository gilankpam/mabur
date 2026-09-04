#include <cmath>
#include "mtest.h"
#include "probe_track.h"
using namespace maburgs;
using mabur::probe::ProbeRx;

static ProbeRx rx(uint32_t seq, uint8_t profile, uint32_t bits) {
  ProbeRx r; r.hdr.seq = seq; r.hdr.profile = profile; r.hdr.enh_fid = static_cast<uint16_t>(seq);
  r.n_blocks = 4; r.survivors = bits; r.n_ok = __builtin_popcount(bits);
  return r;
}
static ProbeTrack make() { return ProbeTrack(ProbeTrackCfg{4, 100, 2}); }

TEST(nothing_booked_while_no_probe_commanded) {
  auto t = make();
  t.on_enh_au(1, 0); t.tick(200);
  CHECK(t.union_counts().expected_blocks == 0);
}

TEST(union_is_or_of_card_bitmaps_and_expected_comes_from_au_count) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.on_body(0, rx(1, 0x06, 0b0011), 30.0, -24.0, 5);
  t.on_body(1, rx(1, 0x06, 0b1100), 28.0, -22.0, 6);
  t.tick(50);
  CHECK(t.union_counts().expected_blocks == 0);  // not finalized yet
  t.tick(120);
  CHECK(t.union_counts().expected_blocks == 4);
  CHECK(t.union_counts().arrived_blocks == 4);
  CHECK(t.union_counts().bodies_rx == 1);
  CHECK(t.card_counts(0).arrived_blocks == 2);
  CHECK(t.card_counts(1).arrived_blocks == 2);
  CHECK(t.card_counts(0).expected_blocks == 4);
  auto fin = t.take_finalized();
  REQUIRE(fin.size() == 1);
  CHECK(fin[0].seq == 1); CHECK(fin[0].blocks_ok == 4); CHECK(fin[0].card_mask == 0b11);
  CHECK(std::fabs(fin[0].snr_db[1] - 28.0) < 1e-9);
  CHECK(t.take_finalized().empty());
}

TEST(delivered_body_finalizes_with_its_au) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.on_body(0, rx(1, 0x06, 0b1111), 30.0, -24.0, 25);
  t.tick(105);
  // The AU's own timer (t=0+100=100) has elapsed, but its matched body
  // hasn't finalized yet (first_ms=25, +100=125): neither counter may
  // finalize ahead of the other.
  CHECK(t.union_counts().expected_blocks == 0);
  CHECK(t.union_counts().arrived_blocks == 0);
  t.tick(130);
  CHECK(t.union_counts().expected_blocks == 4);
  CHECK(t.union_counts().arrived_blocks == 4);
}

TEST(lost_probe_still_books_expected_on_the_au_timer) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.tick(105);
  CHECK(t.union_counts().expected_blocks == 4);
  CHECK(t.union_counts().arrived_blocks == 0);
}

TEST(late_body_after_au_timer_books_arrived_only) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.tick(105);
  CHECK(t.union_counts().expected_blocks == 4);
  t.on_body(0, rx(1, 0x06, 0b1111), 30.0, -24.0, 200);
  t.tick(310);
  CHECK(t.union_counts().expected_blocks == 4);  // no double-booking
  CHECK(t.union_counts().arrived_blocks == 4);
}

TEST(total_loss_reads_as_loss_not_silence) {
  auto t = make();
  t.set_commanded(0x06, 0);
  for (int i = 0; i < 15; ++i) t.on_enh_au(static_cast<uint16_t>(i), i * 33.0);
  t.tick(15 * 33.0 + 150);
  CHECK(t.union_counts().expected_blocks == 60);
  CHECK(t.union_counts().arrived_blocks == 0);
}

TEST(off_profile_body_is_not_scored_not_lost) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0); t.on_enh_au(2, 33);
  t.on_body(0, rx(1, 0x05, 0b1111), 30.0, -24.0, 5);   // stale profile, cancels AU fid=1
  t.on_body(0, rx(2, 0x06, 0b1111), 30.0, -24.0, 40);
  t.tick(300);
  CHECK(t.off_profile() == 1);
  CHECK(t.union_counts().expected_blocks == 4);  // AU 1 cancelled, AU 2 booked
  CHECK(t.union_counts().arrived_blocks == 4);
}

TEST(off_profile_body_without_a_pending_au_books_nothing) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_body(0, rx(1, 0x05, 0b1111), 30.0, -24.0, 5);  // off-profile, no AU to cancel
  t.tick(300);
  CHECK(t.union_counts().expected_blocks == 0);
  CHECK(t.off_profile() == 1);
  CHECK(t.union_counts().bodies_rx == 0);
}

TEST(off_profile_body_still_logs_a_finalized_row) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_body(0, rx(1, 0x05, 0b1111), 30.0, -24.0, 5);  // off-profile, RCF-lag row
  t.tick(300);
  CHECK(t.union_counts().bodies_rx == 0);  // counters stay gated on-profile
  auto fin = t.take_finalized();
  REQUIRE(fin.size() == 1);
  CHECK(fin[0].seq == 1);
  CHECK(fin[0].profile == 0x05);  // its own profile, not the commanded one
  CHECK(fin[0].blocks_ok == 4);
}

TEST(duplicate_seq_on_same_card_counts_once) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.on_body(0, rx(1, 0x06, 0b1111), 30.0, -24.0, 5);
  t.on_body(0, rx(1, 0x06, 0b1111), 30.0, -24.0, 6);
  t.tick(300);
  CHECK(t.union_counts().bodies_rx == 1);
  CHECK(t.union_counts().arrived_blocks == 4);
}

TEST(ring_overflow_finalizes_oldest) {
  auto t = make();
  t.set_commanded(0x06, 0);
  for (uint32_t s = 0; s < 70; ++s) t.on_body(0, rx(s, 0x06, 0b1111), 30.0, -24.0, 1.0);
  CHECK(t.union_counts().bodies_rx == 6);       // 70 - 64 forced out early
  CHECK(t.union_counts().arrived_blocks == 24); // 6 x 4 blocks each
}

TEST(counters_are_monotonic_across_a_profile_switch) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(1, 0);
  t.on_body(0, rx(1, 0x06, 0b1111), 30.0, -24.0, 5);
  t.tick(200);
  const uint64_t sample_a_expected = t.union_counts().expected_blocks;
  const uint64_t sample_a_arrived = t.union_counts().arrived_blocks;
  CHECK(sample_a_expected == 4);
  CHECK(sample_a_arrived == 4);

  t.set_commanded(0x07, 200);
  t.on_enh_au(2, 233);
  t.on_body(0, rx(2, 0x06, 0b1111), 30.0, -24.0, 240);  // stale profile, cancels AU fid=2
  t.on_enh_au(3, 266);
  t.on_body(0, rx(3, 0x07, 0b1111), 30.0, -24.0, 275);

  t.tick(300);  // neither the new AUs nor the new bodies have hit finalize_ms yet
  CHECK(t.union_counts().expected_blocks >= sample_a_expected);
  CHECK(t.union_counts().arrived_blocks >= sample_a_arrived);

  t.tick(500);
  CHECK(t.union_counts().expected_blocks == 8);  // AUs 1 and 3; AU 2 cancelled
  CHECK(t.union_counts().arrived_blocks == 8);
  CHECK(t.off_profile() == 1);
  CHECK(t.union_counts().expected_blocks >= sample_a_expected);
  CHECK(t.union_counts().arrived_blocks >= sample_a_arrived);
}

MTEST_MAIN
