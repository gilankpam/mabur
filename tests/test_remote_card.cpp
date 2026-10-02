// RemoteCard (gs/src/remote_card.h): the CPE510 relay as a LinkCard.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include "mtest.h"
#include "own_air.h"
#include "remote_card.h"
#include "relay_wire.h"
using namespace maburgs;
using namespace maburgs::relay;

namespace {
// Scripted transport: the test pushes relay->client bytes, the card's RX
// thread recv()s them; everything the card sends is kept for inspection.
struct FakeTransport final : public RelayTransport {
  std::mutex mu; std::condition_variable cv;
  std::deque<std::vector<uint8_t>> inbox;
  std::vector<std::vector<uint8_t>> sent;
  std::atomic<bool> closed{false};
  // Outlives the transport (the card destroys it on reopen): the Rig keeps it.
  std::shared_ptr<std::atomic<bool>> closed_flag = std::make_shared<std::atomic<bool>>(false);
  bool send(const uint8_t* p, size_t n) override {
    std::lock_guard<std::mutex> lk(mu); sent.emplace_back(p, p + n); return true;
  }
  int recv(uint8_t* buf, size_t cap, int timeout_ms) override {
    std::unique_lock<std::mutex> lk(mu);
    cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return closed || !inbox.empty(); });
    if (closed) return -1;
    if (inbox.empty()) return 0;
    auto m = std::move(inbox.front()); inbox.pop_front();
    const size_t n = std::min(cap, m.size()); std::memcpy(buf, m.data(), n); return (int)n;
  }
  void close() override { { std::lock_guard<std::mutex> lk(mu); closed = true; } *closed_flag = true; cv.notify_all(); }
  void push(std::vector<uint8_t> m) { { std::lock_guard<std::mutex> lk(mu); inbox.push_back(std::move(m)); } cv.notify_all(); }
  int count(Type t) { std::lock_guard<std::mutex> lk(mu); int c = 0; for (auto& m : sent) c += msg_type(m.data(), m.size()) == t; return c; }
};

struct Rig {
  BodyQueue q;
  std::atomic<uint64_t> now_ms{1000};
  std::vector<FakeTransport*> opened;   // every transport the card opened, in order (only back() is live)
  std::vector<std::shared_ptr<std::atomic<bool>>> closed;   // each one's close() flag, safe after it is freed
  std::unique_ptr<RemoteCard> card;
  Rig(uint8_t ch = 136, uint8_t w = 40) {
    RemoteCard::Cfg c; c.addr = "10.83.11.1:8310"; c.channel = ch; c.width_mhz = w; c.card_id = 1;
    card = std::make_unique<RemoteCard>(c, q,
        [this](const std::string&, std::string&) { auto t = std::make_unique<FakeTransport>(); opened.push_back(t.get()); closed.push_back(t->closed_flag); return std::unique_ptr<RelayTransport>(std::move(t)); },
        [this] { return now_ms.load(); });
  }
  FakeTransport& t() { return *opened.back(); }
  // Bounded wait for the RX thread: predicate within 1 s.
  template <class P> bool soon(P pred) {
    for (int i = 0; i < 200 && !pred(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return pred();
  }
};

std::vector<uint8_t> status(uint8_t state, uint8_t ch, uint8_t sec, uint8_t you_own) {
  std::vector<uint8_t> b(kStatusLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 3; b[3] = kStatus;
  b[6] = state; b[7] = ch; b[8] = sec; b[9] = 1; b[10] = you_own;
  return b;
}
std::vector<uint8_t> frame(uint32_t seq, uint8_t rx_ch, uint8_t flags, uint8_t mcs, bool canonical = true) {
  std::vector<uint8_t> b(kFrameHdrLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 3; b[3] = kFrame;
  for (int i = 0; i < 4; ++i) b[4 + i] = (uint8_t)(seq >> (8 * i));
  b[8] = rx_ch; b[9] = 2; b[10] = flags; b[11] = mcs;
  b[12] = (uint8_t)-50; b[13] = (uint8_t)-52; b[14] = (uint8_t)-95; b[15] = (uint8_t)-95;
  std::vector<uint8_t> d(26 + 3, 0);
  d[0] = 0x88;
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::memcpy(d.data() + 10, sa, 6);
  if (!canonical) d[10] = 0x00;
  d[22] = 0x30; d[23] = 0x12; d[26] = 0xAA; d[27] = 0xBB; d[28] = 0xCC;
  b.insert(b.end(), d.begin(), d.end());
  return b;
}
size_t drained(BodyQueue& q, std::vector<mabur::node::RxBody>& out) { out.clear(); return q.drain(out, 0); }
}  // namespace

TEST(static_identity) {
  Rig r;
  CHECK(!r.card->can_scout());
  const CardCaps c = r.card->caps();
  CHECK(c.valid && c.chip == "ath9k" && c.gen == "CPE510" && c.rx_chains == 2 && c.tx_chains == 2);
  CHECK(!c.snr_ok && !c.fa_ok && !c.nhm_ok && !c.fast_retune);
  CHECK(c.bw_mask == (uint8_t)((1u << 2) | (1u << 3)));   // devourer kBw20|kBw40, same bits as the C record
  CHECK(r.card->tuned_central() == -1);
  CHECK(r.card->channel() == 136 && r.card->width() == 40);
  CHECK(r.card->relay_stats().has_value());               // USB cards return nullopt; the relay never does
}

TEST(open_sends_hello_and_tune_for_channel_and_width) {
  Rig r(136, 40);
  REQUIRE(r.card->open_and_start());
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  CHECK(r.t().count(kHello) >= 1);
  {   // released before stop(): the RX thread reacquires it inside recv() to exit
    std::lock_guard<std::mutex> lk(r.t().mu);
    auto& tune = *std::find_if(r.t().sent.begin(), r.t().sent.end(), [](auto& m) { return msg_type(m.data(), m.size()) == kTune; });
    CHECK(tune[6] == 136 && tune[7] == 2);   // 136 is HT40-: sec 2 (mabur::ht40_offset)
  }
  CHECK(r.card->alive() && !r.card->ready());
  r.card->stop();
}

TEST(owned_and_tuned_reads_ready_and_bodies_reach_the_queue_with_card_id_and_rx_channel) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.t().push(frame(1, 136, kFlagPhyValid | kFlagStbc, 4));
  r.t().push(frame(2, 0, 0, 4));               // mid-retune stamp passes through as 0
  std::vector<mabur::node::RxBody> out;
  REQUIRE(r.soon([&] { return r.card->rx_frames() >= 2; }));
  REQUIRE(drained(r.q, out) == 2);
  CHECK(out[0].card_id == 1 && out[0].rx_channel == 136 && out[0].mcs == 4);
  CHECK(out[1].rx_channel == 0);
  CHECK(out[0].mono_us == 1000 * 1000);        // now_ms * 1000
  CHECK(r.card->frames().own == 2);
  CHECK(r.card->frames().own_air_us > 0);      // 29-byte frame at mcs4/40 + preamble
  r.card->stop();
}

TEST(frames_dropped_while_not_owned_and_tuned) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(3, 132, 0, 0));            // someone else owns it, elsewhere
  r.t().push(frame(1, 132, kFlagPhyValid, 4));
  REQUIRE(r.soon([&] { return r.card->rx_frames() >= 1; }));
  std::vector<mabur::node::RxBody> out;
  CHECK(drained(r.q, out) == 0);
  CHECK(!r.card->ready());
  // Owned, then ordered elsewhere: not ready again until STATUS confirms.
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  CHECK(r.card->retune(144));
  CHECK(r.card->channel() == 144 && !r.card->ready());
  r.t().push(frame(2, 136, kFlagPhyValid, 4)); // still coming off the old channel
  REQUIRE(r.soon([&] { return r.card->rx_frames() >= 2; }));
  CHECK(drained(r.q, out) == 0);
  r.t().push(status(0, 144, 2, 1));   // 144 is HT40- (pairs with 140): sec 2
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.card->stop();
}

TEST(own_air_width_follows_tuned_sec_not_commanded_width) {
  Rig r(165, 40);                              // 165 has no HT40 pair: sec 0, the relay tunes 20 MHz
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 165, 0, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.t().push(frame(1, 165, kFlagPhyValid, 4));
  REQUIRE(r.soon([&] { return r.card->frames().own == 1; }));
  OwnAirAcc at20;
  at20.on_frame(29, 4, true, 20, false, false);
  CHECK(r.card->frames().own_air_us == at20.total_us());
  r.card->stop();
}

TEST(foreign_counted_not_queued) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.t().push(frame(1, 136, kFlagPhyValid, 4, /*canonical=*/false));
  REQUIRE(r.soon([&] { return r.card->foreign() == 1; }));
  std::vector<mabur::node::RxBody> out;
  CHECK(drained(r.q, out) == 0);
  CHECK(r.card->frames().foreign == 1 && r.card->frames().own == 0);
  r.card->stop();
}

TEST(send_control_packs_tx_only_when_owner) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  const std::vector<uint8_t> body = {1, 2, 3, 4};
  CHECK(!r.card->send_control(body));
  CHECK(r.card->tx_fail() == 1 && r.card->tx_frames() == 0);
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  CHECK(r.card->send_control(body));
  CHECK(r.card->tx_frames() == 1);
  REQUIRE(r.soon([&] { return r.t().count(kTx) == 1; }));
  {   // released before stop(): the RX thread reacquires it inside recv() to exit
    std::lock_guard<std::mutex> lk(r.t().mu);
    auto& tx = r.t().sent.back();
    CHECK(tx[4] == 0 /*mcs0*/ && (tx[5] & (kTxLdpc | kTxStbc)) == (kTxLdpc | kTxStbc));
    CHECK(tx.size() == kTxHdrLen + 24 + body.size());   // FCS-less probe-req + body, radiotap stripped
  }
  r.card->stop();
}

TEST(set_width_retunes_with_new_sec) {
  Rig r(136, 20);
  REQUIRE(r.card->open_and_start());
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  CHECK(r.card->set_width(136, 40));
  CHECK(r.card->width() == 40);
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 2; }));
  {   // released before stop(): the RX thread reacquires it inside recv() to exit
    std::lock_guard<std::mutex> lk(r.t().mu);
    CHECK(r.t().sent.back()[6] == 136 && r.t().sent.back()[7] == 2);
  }
  r.card->stop();
}

TEST(lost_reads_not_alive_and_reopen_restarts_cleanly) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.now_ms = 1000 + 2001;                       // > kLostMs with no STATUS
  r.card->tick(r.now_ms);
  CHECK(!r.card->alive() && !r.card->ready());
  r.card->stop();
  REQUIRE(r.card->open_and_start());            // what main.cpp's reopen loop does
  CHECK(r.opened.size() == 2);
  CHECK(*r.closed[0]);                         // opened[0] itself is freed by the reopen
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  CHECK(r.card->relay_stats()->reconnects == 1);
  r.card->stop();
}

TEST(refused_restarts_client_every_5s) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(3, 132, 0, 0));
  REQUIRE(r.soon([&] { return r.card->relay_stats()->state == 3; }));
  // The real relay sends STATUS every 500 ms; one per tick keeps lost() false
  // so the card reads Refused, not Lost (Lost wins: the core loop reopens it).
  // Waiting for the inbox to drain means the RX thread has taken this STATUS,
  // so every earlier one is fully applied before the tick reads lost().
  auto tick_with_status = [&](uint64_t t) {
    r.now_ms = t;
    r.t().push(status(3, 132, 0, 0));
    REQUIRE(r.soon([&] { std::lock_guard<std::mutex> lk(r.t().mu); return r.t().inbox.empty(); }));
    r.card->tick(t);
  };
  for (uint64_t t = 1000; t <= 1000 + 2600; t += 100) tick_with_status(t);
  const int tunes_at_window_end = r.t().count(kTune);   // start + 500 + ... + 2500 = 6
  CHECK(tunes_at_window_end == 6);
  for (uint64_t t = 1000 + 2700; t <= 1000 + 4900; t += 100) tick_with_status(t);
  CHECK(r.t().count(kTune) == tunes_at_window_end);     // window closed, nothing more
  tick_with_status(1000 + 5000);                        // 5 s after open (the clock seeds at open, not at refusal)
  CHECK(r.t().count(kTune) == tunes_at_window_end + 1); // restart: HELLO + TUNE again
  CHECK(r.card->relay_stats()->reconnects == 1);
  r.card->stop();
}

TEST(relay_stats_mirror_status_and_counters) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));   // your_drops etc. are parse_status's job (test_relay_wire); the card only copies them
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.t().push(frame(1, 136, 0, 4));
  r.t().push(frame(5, 136, 0, 4));             // seq gap of 3
  REQUIRE(r.soon([&] { return r.card->rx_frames() >= 2; }));
  const auto s = *r.card->relay_stats();
  CHECK(s.owned && s.state == 0 && s.ch == 136 && s.sec == 2);
  CHECK(s.frames == 2 && s.gaps == 3);
  r.card->stop();
}
TEST(retune_after_stop_records_target_without_sending) {
  Rig r(136, 20);
  REQUIRE(r.card->open_and_start());
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  r.card->stop();
  const int tunes = r.t().count(kTune);         // the stopped card still holds its closed transport
  CHECK(r.card->retune(149));
  CHECK(r.card->set_width(149, 40));
  CHECK(r.card->channel() == 149 && r.card->width() == 40);
  CHECK(r.t().count(kTune) == tunes);           // nothing sent on the closed transport
  REQUIRE(r.card->open_and_start());            // the reopen re-asserts the recorded target
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  {
    std::lock_guard<std::mutex> lk(r.t().mu);
    auto& tune = *std::find_if(r.t().sent.begin(), r.t().sent.end(), [](auto& m) { return msg_type(m.data(), m.size()) == kTune; });
    CHECK(tune[6] == 149 && tune[7] == 1);      // 149 is HT40+: sec 1
  }
  r.card->stop();
}
MTEST_MAIN
