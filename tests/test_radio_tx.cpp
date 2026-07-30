#include <cstdint>
#include <cstring>
#include <vector>

#include "mtest.h"
#include "mabur/profile.h"
#include "radio_tx.h"

#include "RadiotapBuilder.h"
#include "TxMode.h"

using namespace mabur;
using namespace mabur::rc;

namespace {

// Canonical SA devourer's build_dot11_probe_req() uses (streamtx/main.cpp).
const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

// Records every frame handed to send(); can be told to reject the next N
// frames to exercise the drop-counting path.
class CaptureSink : public FrameSink {
 public:
  bool send(const uint8_t* frame, size_t len) override {
    if (reject_next_ > 0) {
      --reject_next_;
      return false;
    }
    frames_.emplace_back(frame, frame + len);
    return true;
  }

  void reject_next(int n) { reject_next_ = n; }

  std::vector<std::vector<uint8_t>> frames_;
  int reject_next_ = 0;
};

devourer::TxMode to_tx_mode(const LayerTxSpec& s, uint8_t bw) {
  devourer::TxMode m;
  if (s.mode == PhyMode::VHT) {
    m.mode = devourer::TxMode::Mode::VHT;
    m.vht_mcs = s.mcs;
    m.vht_nss = 1;
  } else {
    m.mode = devourer::TxMode::Mode::HT;
    m.ht_mcs = s.mcs;
  }
  m.bw_mhz = bw;
  m.sgi = s.sgi;
  m.ldpc = s.ldpc;
  m.stbc = s.stbc;
  return m;
}

uint16_t read_le16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

}  // namespace

TEST(send_body_second_frame_has_ht_radiotap_and_dot11_header) {
  CaptureSink sink;
  RadioTx tx(sink);
  auto ladder = ladder_from(PhyMode::HT, 2, 20);
  tx.set_ladder(ladder);

  const uint8_t body0[] = {0xaa, 0xbb};
  const uint8_t body1[] = {0x01, 0x02, 0x03, 0x04};
  CHECK(tx.send_body(0, body0, sizeof(body0)));
  CHECK(tx.send_body(0, body1, sizeof(body1)));

  REQUIRE(sink.frames_.size() == 2);
  const auto& f = sink.frames_[1];  // seq 1

  REQUIRE(f.size() > 4);
  uint16_t rl = read_le16(&f[2]);
  CHECK(rl == 13);  // HT path radiotap length

  REQUIRE(f.size() >= static_cast<size_t>(rl) + 24 + sizeof(body1));
  const uint8_t* hdr = &f[rl];
  CHECK(hdr[0] == 0x40);
  CHECK(hdr[1] == 0x00);
  CHECK(hdr[2] == 0x00);
  CHECK(hdr[3] == 0x00);
  for (int i = 0; i < 6; ++i) CHECK(hdr[4 + i] == 0xff);
  for (int i = 0; i < 6; ++i) CHECK(hdr[10 + i] == kCanonicalSa[i]);
  for (int i = 0; i < 6; ++i) CHECK(hdr[16 + i] == kCanonicalSa[i]);
  uint16_t seq_ctl = read_le16(&hdr[22]);
  CHECK(seq_ctl == (1 << 4));

  const uint8_t* body_out = &f[static_cast<size_t>(rl) + 24];
  CHECK(std::memcmp(body_out, body1, sizeof(body1)) == 0);

  CHECK(tx.sent() == 2);
  CHECK(tx.drops() == 0);
  CHECK(tx.seq() == 2);
}

TEST(seq_wraps_at_4096) {
  CaptureSink sink;
  RadioTx tx(sink);
  auto ladder = ladder_from(PhyMode::HT, 0, 20);
  tx.set_ladder(ladder);

  const uint8_t body[] = {0x00};
  for (int i = 0; i < 4096; ++i) {
    CHECK(tx.send_body(0, body, sizeof(body)));
  }
  CHECK(tx.seq() == 0);
}

TEST(vht_ladder_radiotap_length_is_22) {
  CaptureSink sink;
  RadioTx tx(sink);
  auto ladder = ladder_from(PhyMode::VHT, 4, 80);
  tx.set_ladder(ladder);

  const uint8_t body[] = {0xde, 0xad, 0xbe, 0xef};
  CHECK(tx.send_body(0, body, sizeof(body)));
  CHECK(tx.send_body(0, body, sizeof(body)));

  REQUIRE(sink.frames_.size() == 2);
  const auto& f = sink.frames_[1];
  uint16_t rl = read_le16(&f[2]);
  CHECK(rl == 22);  // VHT path radiotap length
}

TEST(sink_rejection_increments_drops_and_still_consumes_seq) {
  CaptureSink sink;
  RadioTx tx(sink);
  auto ladder = ladder_from(PhyMode::HT, 0, 20);
  tx.set_ladder(ladder);

  const uint8_t body[] = {0x7};
  sink.reject_next(1);
  CHECK(tx.send_body(0, body, sizeof(body)) == false);
  CHECK(tx.drops() == 1);
  CHECK(tx.sent() == 0);
  CHECK(tx.seq() == 1);  // seq still consumed despite drop

  CHECK(tx.send_body(0, body, sizeof(body)) == true);
  CHECK(tx.sent() == 1);
  CHECK(tx.drops() == 1);
  CHECK(tx.seq() == 2);
}

TEST(send_body_before_set_ladder_drops_and_consumes_seq) {
  CaptureSink sink;
  RadioTx tx(sink);
  // Intentionally do NOT call set_ladder() so radiotap cache is empty.

  const uint8_t body[] = {0xaa, 0xbb};
  CHECK(tx.send_body(0, body, sizeof(body)) == false);

  CHECK(sink.frames_.size() == 0);  // No frame was sent to sink
  CHECK(tx.drops() == 1);
  CHECK(tx.sent() == 0);
  CHECK(tx.seq() == 1);  // Sequence was still consumed
}

namespace {
// Records send_many batch sizes; accepts everything.
class BatchSink : public FrameSink {
 public:
  bool send(const uint8_t* frame, size_t len) override {
    frames_.emplace_back(frame, frame + len);
    return true;
  }
  size_t send_many(const View* v, size_t n) override {
    batches_.push_back(n);
    for (size_t i = 0; i < n; ++i) frames_.emplace_back(v[i].data, v[i].data + v[i].len);
    return n;
  }
  std::vector<std::vector<uint8_t>> frames_;
  std::vector<size_t> batches_;
};
}  // namespace

TEST(send_bodies_matches_send_body_framing) {
  // Batch and per-body paths must produce byte-identical frames and the
  // same seq/sent accounting (CaptureSink exercises the default send_many
  // per-frame fallback).
  CaptureSink a_sink;
  CaptureSink b_sink;
  RadioTx a(a_sink);
  RadioTx b(b_sink);
  auto ladder = ladder_from(PhyMode::HT, 2, 20);
  a.set_ladder(ladder);
  b.set_ladder(ladder);

  std::vector<UepBody> bodies;
  for (uint8_t i = 0; i < 7; ++i)
    bodies.push_back(UepBody{static_cast<uint8_t>(i % 4),
                             std::vector<uint8_t>(40 + i, static_cast<uint8_t>(i))});

  for (auto& bd : bodies) a.send_body(bd.stream_id, bd.body.data(), bd.body.size());
  CHECK(b.send_bodies(bodies) == bodies.size());

  REQUIRE(a_sink.frames_.size() == b_sink.frames_.size());
  for (size_t i = 0; i < a_sink.frames_.size(); ++i)
    CHECK(a_sink.frames_[i] == b_sink.frames_[i]);
  CHECK(a.seq() == b.seq());
  CHECK(a.sent() == b.sent());
  CHECK(b.drops() == 0);
}

TEST(send_bodies_submits_one_batch) {
  BatchSink sink;
  RadioTx tx(sink);
  tx.set_ladder(ladder_from(PhyMode::HT, 2, 20));
  std::vector<UepBody> bodies;
  for (int i = 0; i < 5; ++i) bodies.push_back(UepBody{1, std::vector<uint8_t>(32, 0x5A)});
  CHECK(tx.send_bodies(bodies) == 5);
  REQUIRE(sink.batches_.size() == 1);
  CHECK(sink.batches_[0] == 5);
  CHECK(tx.sent() == 5);
}

MTEST_MAIN
