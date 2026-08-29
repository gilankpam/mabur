#include "mtest.h"
#include "frame_fixture.h"
#include "aggregator.h"
#include "mabur/uep_encoder.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include "mabur/rc_proto.h"
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
  r.fec_overhead = 0.5;
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
  Aggregator agg(vec_layers(), 200, 512, 2);
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
  // last_video_seq() tracks ONLY the video/decoder-bound seq counter: RC
  // frames (DISC_ACK here) carry their own independent dot11 seq on the
  // drone and must never advance it, even though the ack is the last
  // crc-ok frame fed in this test — so the mark stays on the last video
  // body's seq (one behind the ack's).
  CHECK(agg.last_video_seq() == seq - 2);
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

TEST(evm_ema_best_chain_min_and_per_chain) {
  Aggregator agg(vec_layers(), 50, 1024, 1);
  agg.on_rx_body(msg(0, 1, true, video_body()));
  const auto& c = agg.card(0);
  CHECK(c.evm_has); CHECK(c.evm_a_has); CHECK(c.evm_b_has);
  CHECK(c.evm_ema == -48.0);    // min(-48, -44): more negative = better chain
  CHECK(c.evm_a_ema == -48.0);
  CHECK(c.evm_b_ema == -44.0);
}

TEST(evm_zero_samples_are_skipped_not_folded) {
  Aggregator agg(vec_layers(), 50, 1024, 1);
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
  Aggregator agg(vec_layers(), 50, 1024, 1);
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
  Aggregator agg(vec_layers(), 50, 1024, 1);
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
  Aggregator agg(vec_layers(), 200, 512, 1);
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
  Aggregator agg(vec_layers(), 200, 512, 1);
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
  Aggregator agg(vec_layers(), 200, 512, 1);
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

  Aggregator agg(vec_layers(), 200, 512, 1);
  agg.on_rx_body(msg(0, 1, true, bodies[0]));
  const CardTrack& c = agg.card(0);
  CHECK(c.cls[int(RfClass::Msp)].frames == 1);
  CHECK(c.cls[int(RfClass::Msp)].has_ema);
}

// --- s1+s3 pooled RF track (spec 2026-08-15) ---
// The RF label source and the predictive fade trigger read this instead of
// cls[S1]: s1+s3 is 97% of frames at the same PHY rate (one-rate ladder,
// overhead-only differentiation), so the two are statistically homogeneous,
// while msp/ctrl may carry a different per-rate TX power.

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

TEST(rf_pool_folds_s1_and_s3) {
  Aggregator agg(vec_layers(), 200, 4096, 1);
  agg.on_rx_body(msg(0, 1, true, sbi_body(1)));
  agg.on_rx_body(msg(0, 2, true, sbi_body(3)));
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == 2);
  CHECK(c.cls[int(RfClass::S1)].frames == 1);
  CHECK(c.cls[int(RfClass::S3)].frames == 1);
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
  Aggregator agg(vec_layers(), 200, 4096, 1);
  auto m = msg(0, 1, true, sbi_body(1));
  m.evm[0] = -30;
  m.evm[1] = 0;  // not sampled
  agg.on_rx_body(m);
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.evm_a_has);
  CHECK(!c.rf_pool.evm_b_has);
  CHECK(c.rf_pool.evm_ema == -30.0);
}

TEST(rf_pool_excludes_s0_msp_and_ctrl) {
  Aggregator agg(vec_layers(), 200, 4096, 1);
  agg.on_rx_body(msg(0, 1, true, sbi_body(0)));                  // s0
  agg.on_rx_body(msg(0, 2, true, sbi_body(mabur::kMspStreamId)));  // msp
  agg.on_rx_body(msg(0, 3, true, disc_ack_fixture_wire()));      // ctrl
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == 0);
  CHECK(!c.rf_pool.has_ema);
  CHECK(c.cls[int(RfClass::S0)].frames == 1);
  CHECK(c.cls[int(RfClass::Msp)].frames == 1);
  CHECK(c.cls[int(RfClass::Ctrl)].frames == 1);
}

TEST(rf_pool_with_no_s3_equals_the_s1_series) {
  // "s3 unavailable -> s1 only" needs no fallback branch: with no s3 frames
  // the pool simply contains the s1 samples.
  Aggregator agg(vec_layers(), 200, 4096, 1);
  for (uint16_t i = 1; i <= 5; ++i) agg.on_rx_body(msg(0, i, true, sbi_body(1)));
  const auto& c = agg.card(0);
  CHECK(c.rf_pool.frames == c.cls[int(RfClass::S1)].frames);
  CHECK(c.rf_pool.snr_ema == c.cls[int(RfClass::S1)].snr_ema);
  CHECK(c.rf_pool.rssi_ema == c.cls[int(RfClass::S1)].rssi_ema);
}

// RC frames (DISC_ACK here) carry their own independent 802.11 seq counter
// on the drone — a wildly different mac_seq on an interleaved RC frame must
// not perturb the video seq-gap walk (seq_expected/seq_received/
// last_video_seq), only the RC-frame routing/class accounting.
TEST(rc_frame_with_wild_seq_does_not_perturb_video_seq_accounting) {
  auto ack = disc_ack_fixture_wire();
  Aggregator agg(vec_layers(), 200, 512, 1);
  std::vector<uint8_t> junk(20, 0);  // not SBI, not RC -> misroutes as "video"
  agg.on_rx_body(msg(0, 10, true, junk));    // video-ish body, seq 10
  agg.on_rx_body(msg(0, 4000, true, ack));   // RC frame, wildly different seq
  agg.on_rx_body(msg(0, 11, true, junk));    // next video-ish body, seq 11
  const CardTrack& c = agg.card(0);
  CHECK(c.seq_received == 2);            // only the two video-ish bodies
  CHECK(c.seq_expected == 2);            // contiguous 10->11: no gap from the ack
  CHECK(agg.last_video_seq() == 11);     // ack's seq (4000) never touches this
  CHECK(c.rc_frames == 1);               // the ack is still routed normally
  CHECK(c.cls[int(RfClass::Ctrl)].frames == 1);
}

TEST(crc_clean_unparseable_body_gets_no_class) {
  Aggregator agg(vec_layers(), 200, 512, 1);
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
  Aggregator agg(vec_layers(), /*decode_deadline_ms=*/200, /*seq_horizon=*/512,
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
