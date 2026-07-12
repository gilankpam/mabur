#include "mtest.h"
#include "vrx_controller.h"
#include "mabur/rc_proto.h"
using namespace maburgs;

static VrxController make() {
  static LinkTable lt;
  VrxCfg cfg;
  cfg.vtx_id = 1;
  return VrxController(lt, cfg);
}

// Drive video at 1 kHz and step at 10 ms; classify emissions per second.
TEST(rcf_pacing_and_keepalive_disc) {
  auto vrx = make();
  int rcf = 0, disc = 0;
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  for (int t = 0; t < 5000; t += 10) {
    const double now = t;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(t / 10), now);
    if (auto out = vrx.step(now, ld, 0.0)) {
      const int ft = mabur::rc::frame_type(out->frame.data(), out->frame.size());
      if (ft == mabur::rc::T_RCF) { CHECK(!out->is_disc); ++rcf; }
      else if (ft == mabur::rc::T_DISC) { CHECK(out->is_disc); ++disc; }
    }
  }
  CHECK(rcf >= 40 && rcf <= 50);   // ~10 Hz for 5 s, minus keepalive slots
  CHECK(disc >= 4 && disc <= 6);   // keep-alive ~1 Hz (fix a)
}

TEST(rcf_fields_are_correct) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 98, 80, 10};
  vrx.on_video(-55.0, 25.0, false, 500, 0.0);
  std::optional<VrxController::Out> out;
  double now = 0;
  while (!out || out->is_disc) {         // skip a leading keepalive DISC
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(500 + now / 10), now);
    out = vrx.step(now, ld, 0.05);
  }
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->vtx_id == 1);
  CHECK(r->ack_seq >= 500);
  REQUIRE(r->layer_delivery.size() == 4);
  CHECK(r->layer_delivery[2] == 80);
  CHECK(r->pwr_idx == vrx.cur_op().txagc);
  CHECK(r->fec_overhead_16ths ==
        mabur::rc::overhead_to_16ths(vrx.cur_op().overhead));
}

TEST(silence_beacons_fast_and_recovers) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.on_video(-55.0, 25.0, false, 1, 0.0);
  // 2 s of silence: BEACONING at the 20 ms cadence.
  int discs = 0;
  for (double now = 1200; now < 2200; now += 10)
    if (auto out = vrx.step(now, ld, std::nullopt)) {
      CHECK(out->is_disc);
      ++discs;
    }
  CHECK(vrx.link_state() == VrxState::BEACONING);
  CHECK(discs >= 45);                       // ~50 in 1 s at 20 ms pacing
  // Failsafe op point while blind:
  CHECK(vrx.cur_op().mcs == 0);
  CHECK(vrx.cur_op().txagc == 63);
  // Video returns -> SESSION and RCFs resume.
  vrx.on_video(-55.0, 25.0, false, 900, 2500.0);
  CHECK(vrx.link_state() == VrxState::SESSION);
}

TEST(disc_ack_feeds_rendezvous) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, std::nullopt);          // silence -> BEACONING
  CHECK(vrx.link_state() == VrxState::BEACONING);
  mabur::rc::DiscAck ack;
  ack.vtx_id = 1;
  ack.vrx_nonce = static_cast<uint32_t>((1ull * 2654435761ull) & 0xFFFFFFFFull);
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.link_state() == VrxState::SESSION);
}
MTEST_MAIN
