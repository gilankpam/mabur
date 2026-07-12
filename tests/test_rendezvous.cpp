#include "mtest.h"
#include "rendezvous.h"
using namespace maburgs;

TEST(session_to_beaconing_and_pacing) {
  VrxRendezvous rz(VrxRzConfig{1, 1000, 20, 149});
  CHECK(rz.state() == VrxState::SESSION);
  CHECK(rz.tick(500) == VrxAction::TxFeedback);      // video "seen" at t=0
  CHECK(rz.tick(1001) == VrxAction::Beacon);         // lost -> first beacon fires
  CHECK(rz.state() == VrxState::BEACONING);
  CHECK(rz.tick(1010) == VrxAction::Idle);           // inside the 20 ms period
  CHECK(rz.tick(1021) == VrxAction::Beacon);
}

TEST(beacon_frame_fields) {
  VrxRendezvous rz(VrxRzConfig{7, 1000, 20, 149});
  auto d1 = rz.beacon();
  auto d2 = rz.beacon();
  CHECK(d1.vtx_id == 7);
  CHECK(d1.op_channel == 149);
  CHECK(d1.vrx_nonce == ((7ull * 2654435761ull) & 0xFFFFFFFFull));
  CHECK(d2.seq == d1.seq + 1);
}

TEST(disc_ack_completes_rendezvous) {
  VrxRendezvous rz(VrxRzConfig{1, 1000, 20, 149});
  rz.tick(1001);                                     // -> BEACONING
  mabur::rc::DiscAck bad; bad.vtx_id = 1; bad.vrx_nonce = 0xDEAD;
  CHECK(!rz.feed_disc_ack(bad, 1100));
  CHECK(rz.state() == VrxState::BEACONING);
  mabur::rc::DiscAck ok; ok.vtx_id = 1; ok.vrx_nonce = rz.nonce();
  CHECK(rz.feed_disc_ack(ok, 1100));
  CHECK(rz.state() == VrxState::SESSION);
  CHECK(rz.tick(1150) == VrxAction::TxFeedback);     // video expected imminently
}

TEST(video_returns_to_session) {
  VrxRendezvous rz(VrxRzConfig{1, 1000, 20, 149});
  rz.tick(1001);
  CHECK(rz.state() == VrxState::BEACONING);
  rz.feed_video(1200);
  CHECK(rz.state() == VrxState::SESSION);
  CHECK(rz.tick(1250) == VrxAction::TxFeedback);
}
MTEST_MAIN
