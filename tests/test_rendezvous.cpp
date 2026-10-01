#include "mtest.h"
#include "rendezvous.h"
using namespace maburgs;
using mabur::rc::DiscAck;
using mabur::rc::kAckKeyMismatch;

static VrxRzConfig cfg(uint32_t nonce = 0) { return VrxRzConfig{1000, 20, 149, nonce}; }
static DiscAck ack_for(const VrxRendezvous& rz, uint32_t vtx, uint8_t flags = 0) {
  DiscAck a; a.vrx_nonce = rz.nonce(); a.vtx_nonce = vtx; a.flags = flags; return a;
}

TEST(starts_beaconing_and_paces) {
  VrxRendezvous rz(cfg());
  CHECK(rz.state() == VrxState::BEACONING);
  CHECK(rz.tick(0) == VrxAction::Beacon);
  CHECK(rz.tick(10) == VrxAction::Idle);
  CHECK(rz.tick(21) == VrxAction::Beacon);
}

TEST(nonce_is_random_per_instance_unless_pinned) {
  VrxRendezvous a(cfg()), b(cfg());
  CHECK(a.nonce() != b.nonce());
  CHECK(a.nonce() != 0);
  VrxRendezvous p(cfg(0xCAFEF00D));
  CHECK(p.nonce() == 0xCAFEF00D);
  CHECK(p.beacon().vrx_nonce == 0xCAFEF00D);
}

TEST(video_alone_never_enters_session) {
  VrxRendezvous rz(cfg());
  rz.feed_video(100);
  CHECK(rz.state() == VrxState::BEACONING);
  CHECK(!rz.vtx_nonce().has_value());
}

TEST(unflagged_ack_adopts_vtx_nonce_and_enters_session) {
  VrxRendezvous rz(cfg());
  bool adopted = false;
  DiscAck wrong; wrong.vrx_nonce = rz.nonce() ^ 1; wrong.vtx_nonce = 5;
  CHECK(!rz.feed_disc_ack(wrong, 100, &adopted));
  CHECK(!adopted);
  CHECK(rz.feed_disc_ack(ack_for(rz, 0xBEEF0001), 100, &adopted));
  CHECK(adopted);
  CHECK(rz.state() == VrxState::SESSION);
  CHECK(rz.vtx_nonce() == std::optional<uint32_t>(0xBEEF0001));
  CHECK(rz.tick(150) == VrxAction::TxFeedback);
  // Retry of the same ack: not a new adoption.
  CHECK(rz.feed_disc_ack(ack_for(rz, 0xBEEF0001), 200, &adopted));
  CHECK(!adopted);
  // A restarted drone issues a new nonce: adopted, seq must reset (caller).
  CHECK(rz.feed_disc_ack(ack_for(rz, 0xBEEF0002), 300, &adopted));
  CHECK(adopted);
  CHECK(rz.vtx_nonce() == std::optional<uint32_t>(0xBEEF0002));
}

TEST(link_loss_keeps_vtx_nonce_and_video_resumes_session) {
  VrxRendezvous rz(cfg());
  bool adopted = false;
  rz.feed_disc_ack(ack_for(rz, 7), 100, &adopted);
  CHECK(rz.tick(1200) == VrxAction::Beacon);        // lost at 100 + 1000
  CHECK(rz.state() == VrxState::BEACONING);
  CHECK(rz.vtx_nonce().has_value());
  rz.feed_video(1300);
  CHECK(rz.state() == VrxState::SESSION);
}

TEST(flagged_acks_enter_key_mismatch_after_a_second_unflagged_leaves) {
  VrxRendezvous rz(cfg());
  bool adopted = false;
  CHECK(rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), 0, &adopted));
  CHECK(!adopted);
  CHECK(rz.state() == VrxState::BEACONING);
  rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), 500, &adopted);
  CHECK(rz.state() == VrxState::BEACONING);
  rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), 1001, &adopted);
  CHECK(rz.state() == VrxState::KEY_MISMATCH);
  CHECK(!rz.vtx_nonce().has_value());
  CHECK(rz.tick(1021) == VrxAction::Beacon);          // keeps beaconing
  CHECK(rz.tick(1030) == VrxAction::Idle);
  rz.feed_disc_ack(ack_for(rz, 9), 1100, &adopted);   // corrected key on one end
  CHECK(adopted);
  CHECK(rz.state() == VrxState::SESSION);
}

// A stranger's flagged acks plus two lost acks from our own (still LINKED)
// drone put us in KEY_MISMATCH while we hold its vtx_nonce. When our drone's
// next ack carries the SAME nonce it is not a new session: reporting it as
// adopted would make the controller zero seq32, and the drone (seq32 must be
// strictly greater) would reject every RCF until its failsafe. Revert = clear
// vtx_nonce_ on entering KEY_MISMATCH: adopted reads true here.
TEST(key_mismatch_keeps_the_held_vtx_nonce_and_same_nonce_is_not_new) {
  VrxRendezvous rz(cfg());
  bool adopted = false;
  rz.feed_disc_ack(ack_for(rz, 0xBEEF0001), 0, &adopted);
  CHECK(adopted);
  rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), 100, &adopted);
  rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), 1100, &adopted);
  CHECK(rz.state() == VrxState::KEY_MISMATCH);
  CHECK(rz.vtx_nonce() == std::optional<uint32_t>(0xBEEF0001));
  CHECK(rz.tick(1110) != VrxAction::TxFeedback);      // held nonce sends nothing
  rz.feed_video(1120);
  CHECK(rz.state() == VrxState::KEY_MISMATCH);         // video does not leave it
  CHECK(rz.feed_disc_ack(ack_for(rz, 0xBEEF0001), 1200, &adopted));
  CHECK(!adopted);
  CHECK(rz.state() == VrxState::SESSION);
  CHECK(rz.vtx_nonce() == std::optional<uint32_t>(0xBEEF0001));
}

TEST(stranger_drone_flagged_acks_never_trigger_while_ours_answers) {
  VrxRendezvous rz(cfg());
  bool adopted = false;
  for (double t = 0; t < 5000; t += 20) {
    rz.feed_disc_ack(ack_for(rz, 0, kAckKeyMismatch), t, &adopted);      // stranger
    rz.feed_disc_ack(ack_for(rz, 42), t + 1, &adopted);                   // ours
    CHECK(rz.state() == VrxState::SESSION);
  }
}

TEST(beacon_carries_settable_proposal) {
  VrxRendezvous rz(VrxRzConfig{1000, 20, 136});
  CHECK(rz.beacon().op_channel == 136);
  rz.set_proposal(149);
  CHECK(rz.beacon().op_channel == 149);
}
MTEST_MAIN
