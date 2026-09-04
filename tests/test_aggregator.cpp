#include "mtest.h"
#include "frame_fixture.h"
#include "aggregator.h"
#include "mabur/uep_encoder.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/probe_wire.h"
#include "mabur/sw_wire.h"

using namespace maburgs;

static std::array<mabur::UepLayerCfg, 2> vec_layers() {
  std::array<mabur::UepLayerCfg, 2> L{};
  const double ov[2] = {1.00, 0.75};
  for (int s = 0; s < 2; ++s) L[s] = mabur::UepLayerCfg{mabur::SwConfig{64, 128, ov[s]}, 4};
  return L;
}

static mabur::node::RxBody msg(uint8_t card, uint16_t seq, bool crc_ok,
                               std::vector<uint8_t> body) {
  mabur::node::RxBody m;
  m.card_id = card; m.mac_seq = seq; m.crc_ok = crc_ok;
  m.rssi[0] = 38; m.rssi[1] = 40;   // both chains sane (see docs/chain-a-rssi-validation-handoff.md)
  m.snr[0] = 10; m.snr[1] = 25;
  m.evm[0] = -48; m.evm[1] = -44;  // raw half-dB, negative = clean, 0 = not sampled
  m.mono_us = 1000u * seq;
  m.body = std::move(body);
  return m;
}

// RC frame wires built directly in C++ from the same values as
// gen_vectors.py's rcfs[0]/acks[0] -- mabur owns the RC wire bytes (see
// test_rc.cpp), so this file packs a real frame instead of reading a
// "wire" key out of rc.json.
static std::vector<uint8_t> rcf_fixture_wire() {
  mabur::rc::Rcf r;
  r.vtx_id = 0xDEADBEEF; r.seq = 7; r.profile = 0x24;
  r.fec_overhead_base = 0.5;
  r.fec_overhead_enh = 0.5;
  return mabur::rc::pack_rcf(r);
}

static std::vector<uint8_t> disc_ack_fixture_wire() {
  mabur::rc::DiscAck a;
  a.vtx_id = 1; a.vrx_nonce = 0xCAFE0001; a.chip_caps = 0x0003;
  a.agreed_channel = 149; a.agreed_width = 20; a.seq = 1;
  return mabur::rc::pack_disc_ack(a);
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

static std::vector<uint8_t> video_body() {
  static auto bodies = encode_fixture_bodies();
  return bodies.empty() ? std::vector<uint8_t>() : bodies[0];
}

TEST(routes_video_to_decoder_and_rc_to_control) {
  auto bodies = encode_fixture_bodies();
  Aggregator agg(vec_layers(), 512, 2);
  int rcs = 0;
  mtest::FragCollector frames;
  agg.set_frag_sink([&](const mabur::DecodedFrag& f) { frames.add(f); });
  agg.set_rc_sink([&](uint8_t card, const std::vector<uint8_t>&, uint64_t) {
    ++rcs; CHECK(card == 1);
  });
  uint16_t seq = 0;
  for (auto& b : bodies) agg.on_rx_body(msg(0, seq++, true, b));
  auto ack = disc_ack_fixture_wire();
  agg.on_rx_body(msg(1, seq++, true, ack));
  // Every fixture frame reassembles from the fragments the sink emitted.
  CHECK(frames.completed().size() == 13);
  CHECK(frames.pending() == 0);
  CHECK(rcs == 1);
  CHECK(agg.card(0).video_bodies > 0);
  CHECK(agg.card(1).rc_frames == 1);
  // The drone's chip stamps one hardware seq on every frame it injects
  // (EN_HWSEQ), RC frames included, so the DISC_ACK — the last crc-ok frame
  // fed here — advances the mark like any video body. (Pre-2026-09-03 the
  // walk excluded RC on the belief it had its own counter: phantom loss.)
  CHECK(agg.last_video_seq() == seq - 1);
}

TEST(seq_gap_tracking_crc_ok_only) {
  Aggregator agg(vec_layers(), 512, 1);
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
  Aggregator agg(vec_layers(), 512, 1);
  std::vector<uint8_t> junk(20, 0);
  const uint16_t order[] = {0, 1, 2, 6, 7, 8, 3, 4, 5, 9};
  for (uint16_t s : order) agg.on_rx_body(msg(0, s, true, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.seq_received == 10);
  CHECK(c.seq_expected == 10);   // 0..9 all arrived — 100% delivery
}

TEST(ema_tracks_both_rssi_chains_and_max_snr) {
  Aggregator agg(vec_layers(), 512, 1);
  std::vector<uint8_t> junk(20, 0);
  agg.on_rx_body(msg(0, 1, true, junk));
  CHECK(agg.card(0).rssi_a_ema == 38.0);       // seeded from first frame
  CHECK(agg.card(0).rssi_b_ema == 40.0);
  CHECK(agg.card(0).snr_ema == 25.0);          // max(10, 25)
  agg.on_rx_body(msg(0, 2, true, junk));
  CHECK(agg.card(0).rssi_a_ema == 38.0);       // constant input -> constant EMA
  CHECK(agg.card(0).rssi_b_ema == 40.0);
}

// A-MPDU aggregated subframes (all but the first) carry no PHY status:
// devourer zeroes/garbles their rssi/snr and clears rx_pkt_attrib.physt.
// phy_valid == false must gate them out of the rssi/snr EMA fold entirely
// (fold_rf), whether or not they are the frame that would have seeded it.
TEST(phy_invalid_frames_do_not_fold_rssi_snr) {
  Aggregator agg(vec_layers(), 512, 1);
  std::vector<uint8_t> junk(20, 0);
  agg.on_rx_body(msg(0, 1, true, junk));  // valid: seeds the EMAs
  const auto& c = agg.card(0);
  const double rssi_a = c.rssi_a_ema, rssi_b = c.rssi_b_ema;
  const double snr_a = c.snr_a_ema, snr_b = c.snr_b_ema, snr = c.snr_ema;
  const double rssi = c.rssi_ema;
  for (uint16_t seq = 2; seq <= 5; ++seq) {
    auto m = msg(0, seq, true, junk);
    m.phy_valid = false;
    m.rssi[0] = 120; m.rssi[1] = 127;   // wild values that would blow up the EMA
    m.snr[0] = 100; m.snr[1] = 100;
    agg.on_rx_body(m);
  }
  CHECK(c.rssi_a_ema == rssi_a);
  CHECK(c.rssi_b_ema == rssi_b);
  CHECK(c.rssi_ema == rssi);
  CHECK(c.snr_a_ema == snr_a);
  CHECK(c.snr_b_ema == snr_b);
  CHECK(c.snr_ema == snr);
  // A phy_valid frame after the invalid run must still fold as before.
  auto m2 = msg(0, 6, true, junk);
  m2.rssi[0] = 50;  // deliberately different from msg()'s constant 38
  agg.on_rx_body(m2);
  CHECK(c.rssi_a_ema != rssi_a);  // the valid frame did move the EMA
}

// phy_valid == false as the very FIRST frame for a card must not seed the
// EMAs either -- has_ema must stay false until a valid sample arrives.
TEST(phy_invalid_first_frame_does_not_seed_rssi_snr_ema) {
  Aggregator agg(vec_layers(), 512, 1);
  std::vector<uint8_t> junk(20, 0);
  auto m = msg(0, 1, true, junk);
  m.phy_valid = false;
  agg.on_rx_body(m);
  CHECK(!agg.card(0).has_ema);
  agg.on_rx_body(msg(0, 2, true, junk));  // first valid frame seeds it
  CHECK(agg.card(0).has_ema);
}

TEST(unknown_card_dropped) {
  Aggregator agg(vec_layers(), 512, 1);
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

  Aggregator agg(vec_layers(), 512, 1);
  int msp_hits = 0, video_hits = 0;
  agg.set_msp_sink([&](const uint8_t*, size_t, uint64_t){ ++msp_hits; });
  agg.set_frag_sink([&](const mabur::DecodedFrag&){ ++video_hits; });

  agg.on_rx_body(msg(0, 1, true, bodies[0]));

  CHECK(msp_hits == 1);
  CHECK(video_hits == 0);
}

static std::vector<uint8_t> probe_body_fixture(uint32_t seq) {
  return mabur::probe::build_probe_body(mabur::probe::ProbeHdr{seq, 0x06, 1}, 4,
                                        static_cast<int>(mabur::sw::kSwHeaderLen) + 64);
}

TEST(probe_body_routes_to_probe_sink_not_video) {
  Aggregator agg(vec_layers(), 512, 2);
  int frags = 0, probes = 0; uint8_t probe_card = 99;
  agg.set_frag_sink([&](const mabur::DecodedFrag&) { ++frags; });
  agg.set_probe_sink([&](uint8_t card, const mabur::node::RxBody&) { ++probes; probe_card = card; });
  agg.on_rx_body(msg(1, 10, true, probe_body_fixture(5)));
  CHECK(probes == 1); CHECK(probe_card == 1); CHECK(frags == 0);
  CHECK(agg.card(1).video_bodies == 0);
  CHECK(agg.card(1).cls[static_cast<size_t>(RfClass::Probe)].frames == 1);
  CHECK(agg.card(1).rf_pool.frames == 0);  // probe stays out of the RF label pool
}

TEST(crc_failed_probe_body_is_not_delivered) {
  Aggregator agg(vec_layers(), 512, 2);
  int probes = 0;
  agg.set_probe_sink([&](uint8_t, const mabur::node::RxBody&) { ++probes; });
  agg.on_rx_body(msg(0, 10, false, probe_body_fixture(5)));
  CHECK(probes == 0);
}

// The drone's chip numbers every injected frame from ONE hardware seq
// counter (EN_HWSEQ): video, MSP and its RC frames alike. So an MSP body
// or a DISC_ACK interleaved with video must advance the per-card seq walk
// like any video body. Skipping them booked one phantom lost frame per
// such frame — ~0.3 % of loss_pct on the bench
// (docs/gs-uplink-self-blanking-findings-2026-09-02.md).
TEST(msp_and_rc_frames_advance_card_seq_walk_no_phantom_loss) {
  mabur::MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  mabur::MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  std::vector<uint8_t> blob;
  std::vector<uint8_t> clear = {2};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, clear.data(), clear.size());
  std::vector<uint8_t> draw = {4};
  mabur::msp_append_message(blob, mabur::MSP_CMD_DISPLAYPORT, draw.data(), draw.size());
  src.on_serial_bytes(blob.data(), blob.size(), 1000);
  REQUIRE(!bodies.empty());

  Aggregator agg(vec_layers(), 512, 1);
  agg.set_msp_sink([&](const uint8_t*, size_t, uint64_t){});
  std::vector<uint8_t> junk(20, 0);
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t){});
  agg.on_rx_body(msg(0, 10, true, junk));                     // video
  agg.on_rx_body(msg(0, 11, true, bodies[0]));                // MSP, same counter
  agg.on_rx_body(msg(0, 12, true, junk));                     // video
  agg.on_rx_body(msg(0, 13, true, disc_ack_fixture_wire()));  // drone RC, same counter
  agg.on_rx_body(msg(0, 14, true, junk));                     // video
  const CardTrack& c = agg.card(0);
  CHECK(c.seq_received == 5);
  CHECK(c.seq_expected == 5);                   // nothing lost
  CHECK(agg.last_video_seq() == 14);
}

TEST(card_rx_bytes_counts_all_frames) {
  Aggregator agg(vec_layers(), 512, 1);
  agg.on_rx_body(msg(0, 1, true, {1, 2, 3}));         // 3 bytes, crc ok
  agg.on_rx_body(msg(0, 2, false, {1, 2, 3, 4, 5}));  // 5 bytes, crc fail
  CHECK(agg.card(0).rx_bytes == 8);  // air bytes: CRC-fail bodies count too
}

TEST(card_rssi_ema_tracks_per_frame_chain_max) {
  Aggregator agg(vec_layers(), 512, 1);
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

TEST(evm_ema_best_chain_min_and_per_chain) {
  Aggregator agg(vec_layers(), 1024, 1);
  agg.on_rx_body(msg(0, 1, true, video_body()));
  const auto& c = agg.card(0);
  CHECK(c.evm_has); CHECK(c.evm_a_has); CHECK(c.evm_b_has);
  CHECK(c.evm_ema == -48.0);    // min(-48, -44): more negative = better chain
  CHECK(c.evm_a_ema == -48.0);
  CHECK(c.evm_b_ema == -44.0);
}

TEST(evm_zero_samples_are_skipped_not_folded) {
  Aggregator agg(vec_layers(), 1024, 1);
  agg.on_rx_body(msg(0, 1, true, video_body()));
  auto m2 = msg(0, 2, true, video_body());
  m2.evm[0] = 0; m2.evm[1] = 0;    // no phy status: must not drag EMAs to 0
  agg.on_rx_body(m2);
  const auto& c = agg.card(0);
  CHECK(c.evm_ema == -48.0);
  CHECK(c.evm_a_ema == -48.0);
  CHECK(c.evm_b_ema == -44.0);
  auto m3 = msg(0, 3, true, video_body());
  m3.evm[0] = 0; m3.evm[1] = -40;  // single-chain sample: only B and combined fold
  agg.on_rx_body(m3);
  CHECK(c.evm_a_ema == -48.0);                       // A untouched
  CHECK(c.evm_b_ema == 0.9 * -44.0 + 0.1 * -40.0);   // kEmaAlpha = 0.1
  CHECK(c.evm_ema == 0.9 * -48.0 + 0.1 * -40.0);     // best = the only nonzero chain
}

// -128 (int8 min) is the chip's "not measured" sentinel for an absent
// spatial stream — measured on the bench 2026-08-10: every 1SS non-STBC
// frame carries evm[1] = -128 (txagcbench sweep, all three MCS). Folding it
// would peg the stream-B EMA at -64 dB and, via best-chain min(), the
// combined EVM too.
TEST(evm_int8_min_sentinel_is_skipped_like_zero) {
  Aggregator agg(vec_layers(), 1024, 1);
  auto m = msg(0, 1, true, video_body());
  m.evm[0] = -48; m.evm[1] = -128;   // 1SS frame: stream B not measured
  agg.on_rx_body(m);
  const auto& c = agg.card(0);
  CHECK(c.evm_a_ema == -48.0);
  CHECK(!c.evm_b_has);               // sentinel never folds
  CHECK(c.evm_ema == -48.0);         // combined = the real stream, not min(-48,-128)
  auto m2 = msg(0, 2, true, video_body());
  m2.evm[0] = -128; m2.evm[1] = -128;
  agg.on_rx_body(m2);
  CHECK(c.evm_a_ema == -48.0);       // all-sentinel frame folds nothing
  CHECK(c.evm_ema == -48.0);
}

TEST(evm_absent_until_first_sample_and_class_scoped) {
  Aggregator agg(vec_layers(), 1024, 1);
  auto m = msg(0, 1, true, video_body());
  m.evm[0] = 0; m.evm[1] = 0;
  agg.on_rx_body(m);
  CHECK(!agg.card(0).evm_has);   // snr/rssi EMAs exist, EVM stays absent
  agg.on_rx_body(msg(0, 2, true, video_body()));
  const auto& ct = agg.card(0).cls[int(RfClass::S0)];  // video_body() is s0
  CHECK(ct.evm_has);
  CHECK(ct.evm_ema == -48.0);
}

TEST(class_split_video_msp_ctrl) {
  auto bodies = encode_fixture_bodies();     // stream-tagged video bodies
  Aggregator agg(vec_layers(), 512, 1);
  uint16_t seq = 0;
  for (auto& b : bodies) agg.on_rx_body(msg(0, seq++, true, b));
  auto ack = disc_ack_fixture_wire();
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
  Aggregator agg(vec_layers(), 512, 1);
  int rc_routed = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_routed; });
  auto rcf = rcf_fixture_wire();
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
  Aggregator agg(vec_layers(), 512, 1);
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
  Aggregator agg(vec_layers(), 512, 1);
  int rc_routed = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_routed; });
  auto rcf = rcf_fixture_wire();
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

  Aggregator agg(vec_layers(), 512, 1);
  agg.on_rx_body(msg(0, 1, true, bodies[0]));
  const CardTrack& c = agg.card(0);
  CHECK(c.cls[int(RfClass::Msp)].frames == 1);
  CHECK(c.cls[int(RfClass::Msp)].has_ema);
}

// --- base+enh pooled RF track (spec 2026-08-15, re-scoped for the 2-stream
// split-rate ladder by task-10-airtime-balance-uep) ---
// The RF label source and the predictive fade trigger read this instead of
// cls[S0]/cls[S1] alone: base and enh no longer share a PHY rate (base
// mirrors mcs-1, enh runs the profile mcs), but RSSI/SNR/EVM are channel
// properties and TX power is constant across MCS, so the two stay
// statistically homogeneous and pooling both beats either stream alone,
// while msp/ctrl are excluded (their mix ratio drifts with rung/shed state).

// Minimal SBI body carrying a chosen stream_id. blocks_per_body = 1 so one
// add() yields exactly one body.
static std::vector<uint8_t> sbi_body(uint8_t stream_id) {
  const int block_payload = 32;
  mabur::SbiPacker packer(block_payload, 1, stream_id);
  std::vector<uint8_t> env(static_cast<size_t>(block_payload), 0xAB);
  auto out = packer.add(env.data(), env.size());
  REQUIRE(out.size() == 1);
  return out[0];
}

TEST(rf_pool_folds_base_and_enh) {
  Aggregator agg(vec_layers(), 4096, 1);
  agg.on_rx_body(msg(0, 1, true, sbi_body(0)));
  agg.on_rx_body(msg(0, 2, true, sbi_body(1)));
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == 2);
  CHECK(c.cls[int(RfClass::S0)].frames == 1);
  CHECK(c.cls[int(RfClass::S1)].frames == 1);
  CHECK(c.rf_pool.has_ema);
  // msg() feeds snr {10,25} and rssi {38,40} on every frame; the tracks fold
  // the per-frame best chain, so a constant input leaves the EMA exact.
  CHECK(c.rf_pool.snr_ema == 25.0);
  CHECK(c.rf_pool.rssi_ema == 40.0);
  // EVM keeps ClassTrack's per-chain validity semantics: msg() feeds
  // {-48,-44}, both sampled, best-of = most negative.
  CHECK(c.rf_pool.evm_has);
  CHECK(c.rf_pool.evm_a_has);
  CHECK(c.rf_pool.evm_b_has);
  CHECK(c.rf_pool.evm_ema == -48.0);
}

TEST(rf_pool_evm_skips_unsampled_chains) {
  // 0 and INT8_MIN are "not a sample" (fold_evm); folding them would peg the
  // EMA at an impossible value. Pool must inherit that, not average zeros.
  Aggregator agg(vec_layers(), 4096, 1);
  auto m = msg(0, 1, true, sbi_body(1));
  m.evm[0] = -30;
  m.evm[1] = 0;  // not sampled
  agg.on_rx_body(m);
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.evm_a_has);
  CHECK(!c.rf_pool.evm_b_has);
  CHECK(c.rf_pool.evm_ema == -30.0);
}

TEST(rf_pool_excludes_msp_and_ctrl) {
  Aggregator agg(vec_layers(), 4096, 1);
  agg.on_rx_body(msg(0, 1, true, sbi_body(0)));                  // base: counts
  agg.on_rx_body(msg(0, 2, true, sbi_body(mabur::kMspStreamId)));  // msp
  agg.on_rx_body(msg(0, 3, true, disc_ack_fixture_wire()));      // ctrl
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == 1);
  CHECK(c.rf_pool.has_ema);
  CHECK(c.cls[int(RfClass::S0)].frames == 1);
  CHECK(c.cls[int(RfClass::Msp)].frames == 1);
  CHECK(c.cls[int(RfClass::Ctrl)].frames == 1);
}

TEST(rf_pool_with_no_enh_equals_the_base_series) {
  // "enh unavailable -> base only" needs no fallback branch: with no enh
  // frames the pool simply contains the base samples.
  Aggregator agg(vec_layers(), 4096, 1);
  for (uint16_t i = 1; i <= 5; ++i) agg.on_rx_body(msg(0, i, true, sbi_body(0)));
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == c.cls[int(RfClass::S0)].frames);
  CHECK(c.rf_pool.snr_ema == c.cls[int(RfClass::S0)].snr_ema);
  CHECK(c.rf_pool.rssi_ema == c.cls[int(RfClass::S0)].rssi_ema);
}

// The drone's chip numbers RC frames (DISC_ACK here) from the same
// hardware seq counter as video (EN_HWSEQ), so an interleaved RC frame
// occupies a seq and must advance the walk — otherwise the next video body
// books a phantom lost frame. Routing/class accounting is unchanged.
TEST(rc_frame_in_sequence_advances_video_seq_accounting) {
  auto ack = disc_ack_fixture_wire();
  Aggregator agg(vec_layers(), 512, 1);
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t){});
  std::vector<uint8_t> junk(20, 0);  // not SBI, not RC -> misroutes as "video"
  agg.on_rx_body(msg(0, 10, true, junk));    // video-ish body, seq 10
  agg.on_rx_body(msg(0, 11, true, ack));     // RC frame, next hw seq
  agg.on_rx_body(msg(0, 12, true, junk));    // next video-ish body, seq 12
  const CardTrack& c = agg.card(0);
  CHECK(c.seq_received == 3);            // all three frames delivered
  CHECK(c.seq_expected == 3);            // contiguous 10,11,12: no gap
  CHECK(agg.last_video_seq() == 12);
  CHECK(c.rc_frames == 1);               // the ack is still routed normally
  CHECK(c.cls[int(RfClass::Ctrl)].frames == 1);
}

TEST(crc_clean_unparseable_body_gets_no_class) {
  Aggregator agg(vec_layers(), 512, 1);
  std::vector<uint8_t> junk(20, 0);   // not SBI, not RC -> misroutes, still counted
  agg.on_rx_body(msg(0, 1, true, junk));
  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 1);
  for (int i = 0; i < kNumRfClasses; ++i) CHECK(c.cls[i].frames == 0);
}

// RxBody.mcs must flow through on_rx_body -> UepDecoder::add_body so the
// transition-attribution boundary can classify arrivals. No helper in this
// file builds a stream-1 body (video_body() above is s0, see
// evm_absent_until_first_sample_and_class_scoped), so build one directly
// via UepEncoder, same pattern as run_transition_sim in test_uep_decoder.cpp.
TEST(rx_mcs_reaches_decoder_boundary) {
  Aggregator agg(vec_layers(), /*seq_horizon=*/512,
                 /*n_cards=*/1);
  // mark_transition establishes expected mcs 4 after a first mark at 5
  // (prev known -> boundary opens). A body heard at mcs 4 must CLOSE the
  // boundary -- proving RxBody.mcs flows through on_rx_body into
  // UepDecoder::add_body.
  agg.decoder().mark_transition(1, 5, 1000);
  agg.decoder().mark_transition(1, 4, 1000);
  CHECK(agg.decoder().last_boundary_close_ms(1) < 0);

  mabur::UepEncoder enc(vec_layers(), /*flush_ms=*/1'000'000'000ULL);
  std::vector<uint8_t> unit(10, 0xAB);  // payload content is irrelevant here
  auto bodies = enc.add_frame(/*stream_id=*/1, unit.data(), unit.size(), 1000);
  REQUIRE(!bodies.empty());

  auto m = msg(0, 1, true, bodies[0].body);
  m.mcs = 4;
  m.mono_us = 2000 * 1000;  // now_ms = 2000, within kBoundaryExpiryMs of 1000
  agg.on_rx_body(m);
  CHECK(agg.decoder().last_boundary_close_ms(1) >= 0);  // closed by mcs 4 body
}
MTEST_MAIN
