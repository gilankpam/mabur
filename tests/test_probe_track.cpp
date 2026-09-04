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
  t.on_enh_au(0); t.tick(200);
  CHECK(t.union_counts().expected_blocks == 0);
}

TEST(union_is_or_of_card_bitmaps_and_expected_comes_from_au_count) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(0);
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

TEST(total_loss_reads_as_loss_not_silence) {
  auto t = make();
  t.set_commanded(0x06, 0);
  for (int i = 0; i < 15; ++i) t.on_enh_au(i * 33.0);
  t.tick(15 * 33.0 + 150);
  CHECK(t.union_counts().expected_blocks == 60);
  CHECK(t.union_counts().arrived_blocks == 0);
}

TEST(off_profile_body_is_not_scored_not_lost) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(0); t.on_enh_au(33);
  t.on_body(0, rx(1, 0x05, 0b1111), 30.0, -24.0, 5);   // stale profile
  t.on_body(0, rx(2, 0x06, 0b1111), 30.0, -24.0, 40);
  t.tick(300);
  CHECK(t.off_profile() == 1);
  CHECK(t.union_counts().expected_blocks == 4);  // 2 AUs - 1 off-profile
  CHECK(t.union_counts().arrived_blocks == 4);
}

TEST(expected_never_drops_below_arrived) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_body(0, rx(1, 0x05, 0b1111), 30.0, -24.0, 5);  // off-profile, no AU booked
  t.tick(300);
  CHECK(t.union_counts().expected_blocks == 0);
}

TEST(duplicate_seq_on_same_card_counts_once) {
  auto t = make();
  t.set_commanded(0x06, 0);
  t.on_enh_au(0);
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
  CHECK(t.union_counts().bodies_rx >= 6);  // 70 - 64 forced out early
}

MTEST_MAIN
