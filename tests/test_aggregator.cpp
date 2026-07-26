#include "mtest.h"
#include "frame_fixture.h"
#include "vectors.h"
#include "aggregator.h"
#include "mabur/uep_encoder.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include "mabur/sbi.h"

using namespace maburgs;

static std::array<mabur::UepLayerCfg, 4> vec_layers() {
  std::array<mabur::UepLayerCfg, 4> L{};
  const double ov[4] = {1.00, 0.75, 0.50, 0.25};
  for (int s = 0; s < 4; ++s) L[s] = mabur::UepLayerCfg{mabur::SwConfig{64, 128, ov[s]}, 4};
  return L;
}

static mabur::node::RxBody msg(uint8_t card, uint16_t seq, bool crc_ok,
                               std::vector<uint8_t> body) {
  mabur::node::RxBody m;
  m.card_id = card; m.mac_seq = seq; m.crc_ok = crc_ok;
  m.rssi[0] = 38; m.rssi[1] = 40;   // both chains sane (see docs/chain-a-rssi-validation-handoff.md)
  m.snr[0] = 10; m.snr[1] = 25;
  m.mono_us = 1000u * seq;
  m.body = std::move(body);
  return m;
}

// Video bodies come from a real UepEncoder over frame_stream.bin — the
// sliding-window scheme has no golden-vector wire format to pin against
// (see test_uep.cpp), so the aggregator is exercised against its paired
// encoder's output, same as the drone/GS pairing on air.
static std::vector<std::vector<uint8_t>> encode_fixture_bodies() {
  mabur::UepEncoder enc(vec_layers(), /*flush_ms=*/1'000'000'000ULL);
  std::vector<std::vector<uint8_t>> bodies;
  auto frames = mtest::load_frame_fixture(std::string(MABUR_FIXTURE_DIR) +
                                          "/frame_stream.bin");
  for (size_t i = 0; i < frames.size(); ++i) {
    auto unit = mtest::frame_unit(frames[i], static_cast<uint16_t>(i));
    for (auto& b : enc.add_frame(frames[i].stream_id(), unit.data(), unit.size(), 0))
      bodies.push_back(std::move(b.body));
  }
  for (auto& b : enc.flush_all()) bodies.push_back(std::move(b.body));
  return bodies;
}

TEST(routes_video_to_decoder_and_rc_to_control) {
  auto rc = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto bodies = encode_fixture_bodies();
  Aggregator agg(vec_layers(), 200, 512, 2);
  int rcs = 0;
  mtest::FragCollector frames;
  agg.set_frag_sink([&](const mabur::DecodedFrag& f) { frames.add(f); });
  agg.set_rc_sink([&](uint8_t card, const std::vector<uint8_t>&, uint64_t) {
    ++rcs; CHECK(card == 1);
  });
  uint16_t seq = 0;
  for (auto& b : bodies) agg.on_rx_body(msg(0, seq++, true, b));
  auto ack = mtest::unhex(rc["disc_ack"][0]["wire"].get<std::string>());
  agg.on_rx_body(msg(1, seq++, true, ack));
  // Every fixture frame reassembles from the fragments the sink emitted.
  CHECK(frames.completed().size() == 13);
  CHECK(frames.pending() == 0);
  CHECK(rcs == 1);
  CHECK(agg.card(0).video_bodies > 0);
  CHECK(agg.card(1).rc_frames == 1);
  // The drone's hw seq counter numbers ALL its injected frames, DISC_ACKs
  // included — so the ack (the last crc-ok frame fed) is the latest seq.
  CHECK(agg.last_video_seq() == seq - 1);
}

TEST(seq_gap_tracking_crc_ok_only) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);   // not SBI, not RC -> misroutes, still counted
  agg.on_rx_body(msg(0, 10, true, junk));
  agg.on_rx_body(msg(0, 13, true, junk));      // gap of 3: 2 lost
  agg.on_rx_body(msg(0, 500, false, junk));    // crc fail: no seq accounting
  agg.on_rx_body(msg(0, 14, true, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 4);
  CHECK(c.crc_fail == 1);
  CHECK(c.seq_received == 3);
  CHECK(c.seq_expected == 5);                  // 1 + 3 + 1
  CHECK(agg.last_video_seq() == 14);
}

// maburd's parallel USB feed can swap whole ≤3-frame URB batches on air.
// A swap is reordering, not loss: late frames must credit delivery, not
// book an outage AND re-count the gap (the old last_seq walk did both —
// one swap inflated seq_expected by ~6).
TEST(seq_urb_batch_swap_is_not_loss) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);
  const uint16_t order[] = {0, 1, 2, 6, 7, 8, 3, 4, 5, 9};
  for (uint16_t s : order) agg.on_rx_body(msg(0, s, true, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.seq_received == 10);
  CHECK(c.seq_expected == 10);   // 0..9 all arrived — 100% delivery
}

TEST(ema_tracks_both_rssi_chains_and_max_snr) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);
  agg.on_rx_body(msg(0, 1, true, junk));
  CHECK(agg.card(0).rssi_a_ema == 38.0);       // seeded from first frame
  CHECK(agg.card(0).rssi_b_ema == 40.0);
  CHECK(agg.card(0).snr_ema == 25.0);          // max(10, 25)
  agg.on_rx_body(msg(0, 2, true, junk));
  CHECK(agg.card(0).rssi_a_ema == 38.0);       // constant input -> constant EMA
  CHECK(agg.card(0).rssi_b_ema == 40.0);
}

TEST(unknown_card_dropped) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);
  agg.on_rx_body(msg(7, 1, true, junk));
  CHECK(agg.bad_card_msgs() == 1);
  CHECK(agg.card(0).frames == 0);
}

TEST(msp_stream_body_routes_to_msp_sink_not_video) {
  // Build a valid MSP body via MspSource so it carries stream_id == 4.
  mabur::MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  mabur::MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  std::vector<uint8_t> blob;
  std::vector<uint8_t> clear = {2};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, clear.data(), clear.size());
  std::vector<uint8_t> ds = {3, 0, 0, 0, 'X'};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, ds.data(), ds.size());
  std::vector<uint8_t> draw = {4};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, draw.data(), draw.size());
  src.on_serial_bytes(blob.data(), blob.size(), 1000);
  REQUIRE(!bodies.empty());

  Aggregator agg(vec_layers(), 200, 512, 1);
  int msp_hits = 0, video_hits = 0;
  agg.set_msp_sink([&](const uint8_t*, size_t, uint64_t){ ++msp_hits; });
  agg.set_frag_sink([&](const mabur::DecodedFrag&){ ++video_hits; });

  agg.on_rx_body(msg(0, 1, true, bodies[0]));

  CHECK(msp_hits == 1);
  CHECK(video_hits == 0);
}

TEST(card_rx_bytes_counts_all_frames) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  agg.on_rx_body(msg(0, 1, true, {1, 2, 3}));         // 3 bytes, crc ok
  agg.on_rx_body(msg(0, 2, false, {1, 2, 3, 4, 5}));  // 5 bytes, crc fail
  CHECK(agg.card(0).rx_bytes == 8);  // air bytes: CRC-fail bodies count too
}

TEST(card_rssi_ema_tracks_per_frame_chain_max) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  auto m1 = msg(0, 1, true, {1});
  m1.rssi[0] = 60; m1.rssi[1] = 40;  // chain A wins this frame
  agg.on_rx_body(m1);
  CHECK(agg.card(0).rssi_ema == 60.0);  // first clean frame seeds the EMA
  auto m2 = msg(0, 2, true, {1});
  m2.rssi[0] = 40; m2.rssi[1] = 50;  // chain B wins this frame
  agg.on_rx_body(m2);
  // kEmaAlpha = 0.1: 0.9*60 + 0.1*50 = 59.0 (per-frame max, NOT max of EMAs)
  CHECK(agg.card(0).rssi_ema > 58.9 && agg.card(0).rssi_ema < 59.1);
  auto m3 = msg(0, 3, false, {1});  // crc fail: EMAs must not move
  m3.rssi[0] = 0; m3.rssi[1] = 0;
  agg.on_rx_body(m3);
  CHECK(agg.card(0).rssi_ema > 58.9 && agg.card(0).rssi_ema < 59.1);
}

TEST(class_split_video_msp_ctrl) {
  auto bodies = encode_fixture_bodies();     // stream-tagged video bodies
  auto rc = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  Aggregator agg(vec_layers(), 200, 512, 1);
  uint16_t seq = 0;
  for (auto& b : bodies) agg.on_rx_body(msg(0, seq++, true, b));
  auto ack = mtest::unhex(rc["disc_ack"][0]["wire"].get<std::string>());
  agg.on_rx_body(msg(0, seq++, true, ack));
  const CardTrack& c = agg.card(0);
  uint64_t stream_frames = 0;
  for (int s = 0; s < 4; ++s) stream_frames += c.cls[s].frames;
  CHECK(stream_frames > 0);                        // video bodies classified
  CHECK(c.cls[int(RfClass::Ctrl)].frames == 1);    // the DISC_ACK
  CHECK(c.cls[int(RfClass::Ctrl)].has_ema);
  CHECK(c.cls[int(RfClass::Ctrl)].snr_ema == 25.0);  // msg() snr max
  CHECK(c.self_frames == 0);
  // Per-class byte accounting: class bytes partition the classified share
  // of the card's rx_bytes (ctrl bytes = the ack body's size).
  CHECK(c.cls[int(RfClass::Ctrl)].bytes == ack.size());
  uint64_t class_bytes = 0;
  for (int k = 0; k < kNumRfClasses; ++k) class_bytes += c.cls[k].bytes;
  CHECK(class_bytes == c.rx_bytes);   // fixture bodies all classify cleanly
}

TEST(self_rc_frames_counted_but_never_tracked) {
  auto rc = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  Aggregator agg(vec_layers(), 200, 512, 1);
  int rc_routed = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_routed; });
  auto rcf = mtest::unhex(rc["rcf"][0]["wire"].get<std::string>());
  agg.on_rx_body(msg(0, 100, true, rcf));          // GS-originated type
  const CardTrack& c = agg.card(0);
  CHECK(c.self_frames == 1);
  CHECK(c.frames == 0);                            // excluded from totals
  CHECK(c.rx_bytes == 0);
  CHECK(!c.has_seq);                               // GS seq counter kept out
  CHECK(!c.cls[int(RfClass::Ctrl)].has_ema);
  CHECK(rc_routed == 1);                           // still routed to the sink
}

TEST(crc_fail_stays_out_of_classes) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);
  agg.on_rx_body(msg(0, 1, false, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 1 && c.crc_fail == 1);
  for (int i = 0; i < kNumRfClasses; ++i) CHECK(c.cls[i].frames == 0);
}

// A corrupt frame whose bytes happen to byte-match an RCF wire must NOT be
// diverted as a self frame — the self-diversion is gated on crc_ok because
// real self frames are point-blank captures and always CRC-clean. It still
// owes ordinary frame/crc_fail/rx_bytes accounting, and — being CRC-fail —
// no class movement.
TEST(crc_fail_rcf_lookalike_not_diverted_as_self) {
  auto rc = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  Aggregator agg(vec_layers(), 200, 512, 1);
  int rc_routed = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_routed; });
  auto rcf = mtest::unhex(rc["rcf"][0]["wire"].get<std::string>());
  agg.on_rx_body(msg(0, 100, false, rcf));   // crc fail but byte-matches RCF wire
  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 1);
  CHECK(c.crc_fail == 1);
  CHECK(c.self_frames == 0);
  for (int i = 0; i < kNumRfClasses; ++i) CHECK(c.cls[i].frames == 0);
}

TEST(msp_body_classified_into_msp_class) {
  // Build a valid MSP body via MspSource so it carries stream_id == 4, same
  // as msp_stream_body_routes_to_msp_sink_not_video above.
  mabur::MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  mabur::MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  std::vector<uint8_t> blob;
  std::vector<uint8_t> clear = {2};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, clear.data(), clear.size());
  std::vector<uint8_t> ds = {3, 0, 0, 0, 'X'};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, ds.data(), ds.size());
  std::vector<uint8_t> draw = {4};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, draw.data(), draw.size());
  src.on_serial_bytes(blob.data(), blob.size(), 1000);
  REQUIRE(!bodies.empty());

  Aggregator agg(vec_layers(), 200, 512, 1);
  agg.on_rx_body(msg(0, 1, true, bodies[0]));
  const CardTrack& c = agg.card(0);
  CHECK(c.cls[int(RfClass::Msp)].frames == 1);
  CHECK(c.cls[int(RfClass::Msp)].has_ema);
}

TEST(crc_clean_unparseable_body_gets_no_class) {
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);   // not SBI, not RC -> misroutes, still counted
  agg.on_rx_body(msg(0, 1, true, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 1);
  for (int i = 0; i < kNumRfClasses; ++i) CHECK(c.cls[i].frames == 0);
}
MTEST_MAIN
