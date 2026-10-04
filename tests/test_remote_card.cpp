// RemoteCard (gs/src/remote_card.h): the CPE510 relay as a LinkCard.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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
  Rig(uint8_t ch = 136, uint8_t w = 40, bool restart_when_refused = true) {
    RemoteCard::Cfg c; c.addr = "10.83.11.1:8310"; c.channel = ch; c.width_mhz = w; c.card_id = 1;
    c.restart_when_refused = restart_when_refused;
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
  b[0] = 0x4D; b[1] = 0x52; b[2] = 4; b[3] = kStatus;
  b[6] = state; b[7] = ch; b[8] = sec; b[9] = 1; b[10] = you_own;
  return b;
}
std::vector<uint8_t> frame(uint32_t seq, uint8_t rx_ch, uint8_t flags, uint8_t mcs, bool canonical = true) {
  std::vector<uint8_t> b(kFrameHdrLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 4; b[3] = kFrame;
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
std::vector<uint8_t> survey(uint16_t gen, uint32_t act, uint32_t busy, uint32_t rx, uint32_t err, uint32_t foreign) {
  std::vector<uint8_t> b(kSurveyLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 4; b[3] = kSurvey; b[4] = 136; b[5] = 2;
  b[6] = (uint8_t)gen; b[7] = (uint8_t)(gen >> 8);
  auto put = [&](size_t o, uint32_t v) { for (int i = 0; i < 4; ++i) b[o + i] = (uint8_t)(v >> (8 * i)); };
  put(8, act); put(12, busy); put(16, rx); put(24, err); put(28, foreign);
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
  CHECK(!r.card->alive() && !r.card->ready());   // no STATUS yet: not alive
  REQUIRE(r.soon([&] { return r.t().count(kTune) >= 1; }));
  CHECK(r.t().count(kHello) >= 1);
  {   // released before stop(): the RX thread reacquires it inside recv() to exit
    std::lock_guard<std::mutex> lk(r.t().mu);
    auto& tune = *std::find_if(r.t().sent.begin(), r.t().sent.end(), [](auto& m) { return msg_type(m.data(), m.size()) == kTune; });
    CHECK(tune[6] == 136 && tune[7] == 2);   // 136 is HT40-: sec 2 (mabur::ht40_offset)
  }
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->alive(); }));
  r.card->stop();
}

TEST(dead_relay_reads_not_alive_across_reopen) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->alive(); }));
  r.now_ms = 1000 + 2001;                       // > kLostMs with no further STATUS
  r.card->tick(r.now_ms);
  CHECK(!r.card->alive());
  r.card->stop();
  REQUIRE(r.card->open_and_start());            // reopen: no STATUS ever arrives on the new transport
  CHECK(!r.card->alive());                      // not alive for the usual post-open grace period
  r.card->tick(r.now_ms);
  CHECK(!r.card->alive());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->alive(); }));
  CHECK(r.card->relay_stats()->reconnects == 1);
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
  CHECK(r.card->alive() && !r.card->ready());   // refused, but a STATUS flowed: alive stands
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

// Refused from open, ticked with a STATUS every 100 ms out to 7 s (past
// kRefusedRestartMs): TUNE count after the 2.5 s tune window, and at 7 s.
// TUNE, not HELLO: HELLO is RelayClient's keepalive and is sent either way;
// a TUNE after the window closes is only ever a restart's.
namespace {
std::pair<int, int> refused_hello_tunes(Rig& r) {
  REQUIRE(r.card->open_and_start());
  auto tick_with_status = [&](uint64_t t) {
    r.now_ms = t;
    r.t().push(status(3, 132, 0, 0));
    REQUIRE(r.soon([&] { std::lock_guard<std::mutex> lk(r.t().mu); return r.t().inbox.empty(); }));
    r.card->tick(t);
  };
  for (uint64_t t = 1000; t <= 1000 + 2700; t += 100) tick_with_status(t);
  const int at_window = r.t().count(kTune);
  for (uint64_t t = 1000 + 2800; t <= 1000 + 7000; t += 100) tick_with_status(t);
  return {at_window, r.t().count(kTune)};
}
}  // namespace

TEST(refused_no_restart_when_restart_when_refused_false) {
  Rig r(136, 40, false);
  const auto [at_window, at_7s] = refused_hello_tunes(r);
  CHECK(at_7s == at_window);                               // never restarted
  CHECK(r.card->health() == RemoteCard::Health::Refused);  // the refusal stays visible
  CHECK(r.card->relay_stats()->reconnects == 0);
  r.card->stop();
}

TEST(refused_restarts_by_default) {
  Rig r;                                                   // default: restart_when_refused = true
  const auto [at_window, at_7s] = refused_hello_tunes(r);
  CHECK(at_7s > at_window);                                // the 5 s restart re-sent TUNE
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
TEST(health_connecting_then_owned) {
  Rig r;
  CHECK(r.card->health() == RemoteCard::Health::Connecting);   // not opened
  REQUIRE(r.card->open_and_start());
  CHECK(r.card->health() == RemoteCard::Health::Connecting);   // no STATUS yet
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  CHECK(r.card->health() == RemoteCard::Health::Owned);
}
TEST(health_refused_past_the_tune_window) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(3, 136, 2, 0));
  REQUIRE(r.soon([&] { return r.card->alive(); }));
  CHECK(r.card->health() == RemoteCard::Health::Connecting);   // inside the window
  r.now_ms += 2600;
  r.t().push(status(3, 136, 2, 0));
  REQUIRE(r.soon([&] { return r.card->health() == RemoteCard::Health::Refused; }));
}
TEST(health_tune_failed) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(2, 100, 0, 1));            // owner, never on our channel
  REQUIRE(r.soon([&] { return r.card->alive(); }));
  r.now_ms += 2600;
  r.t().push(status(2, 100, 0, 1));
  REQUIRE(r.soon([&] { return r.card->health() == RemoteCard::Health::TuneFailed; }));
}
TEST(health_lost_and_taken) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.now_ms += 2100;                            // no STATUS for > kLostMs
  CHECK(r.card->health() == RemoteCard::Health::Lost);
  Rig s;
  REQUIRE(s.card->open_and_start());
  s.t().push(status(0, 136, 2, 1));
  REQUIRE(s.soon([&] { return s.card->ready(); }));
  s.t().push(status(0, 136, 2, 0));            // another client took it
  // Wait for the RX thread to consume this STATUS (and thus set
  // not_owner_since_ms_ at the pre-increment clock) before moving the
  // clock, same idiom as tick_with_status above -- otherwise the RX
  // thread can read the already-advanced clock and ownership_lost()
  // never crosses its 1000 ms window.
  REQUIRE(s.soon([&] { std::lock_guard<std::mutex> lk(s.t().mu); return s.t().inbox.empty(); }));
  s.now_ms += 1100;
  s.t().push(status(0, 136, 2, 0));
  REQUIRE(s.soon([&] { return s.card->health() == RemoteCard::Health::Taken; }));
}
TEST(relay_stats_carry_you_own_and_transport_drops) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  const auto st = r.card->relay_stats();
  REQUIRE(st.has_value());
  CHECK(st->you_own);
  CHECK(st->rx_drops == 0 && st->tx_drops == 0);   // FakeTransport reports none
}
// Plan 2 review carry-over: a search-burst TUNE (owner, mid-retune) is not a
// state transition; the card no longer logs waiting/owned on every burst.
TEST(owner_mid_retune_is_not_a_logged_transition) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.card->tick(r.now_ms += 10);
  CHECK(r.card->transitions() == 2);              // "connecting", "owned and tuned"
  REQUIRE(r.card->retune(40));                    // 40 pairs with 36 (HT40-: sec 2)
  r.t().push(status(1, 136, 2, 1));               // STATUS: retuning, still on 136, we own it
  REQUIRE(r.soon([&] { return !r.card->ready(); }));
  r.card->tick(r.now_ms += 10);
  CHECK(r.card->transitions() == 2);              // not "waiting for STATUS"
  r.t().push(status(0, 40, 2, 1));                // tuned
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.card->tick(r.now_ms += 10);
  CHECK(r.card->transitions() == 2);              // not "owned and tuned" again
  CHECK(r.card->health() == RemoteCard::Health::Owned);
  // losing ownership mid-session still transitions (Refused via ownership_lost)
  r.t().push(status(0, 40, 2, 0));
  REQUIRE(r.soon([&] { return !r.card->ready(); }));
  r.card->tick(r.now_ms += 1100);
  CHECK(r.card->transitions() == 3);
}
// Plan 2 review carry-over, fix round 1: the mid-retune suppression is
// bounded to 1 s -- a relay stuck in a failed TUNE (owner, never reaching
// our channel) still logs "waiting for STATUS" once the grace runs out,
// instead of going silent forever.
TEST(owner_stuck_in_a_failed_tune_logs_after_the_grace) {
  Rig r;
  REQUIRE(r.card->open_and_start());
  r.t().push(status(0, 136, 2, 1));
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.card->tick(r.now_ms += 10);
  CHECK(r.card->transitions() == 2);              // "connecting", "owned and tuned"
  REQUIRE(r.card->retune(40));
  r.t().push(status(2, 136, 2, 1));               // STATUS: owner, refused TUNE, stuck on 136
  REQUIRE(r.soon([&] { return !r.card->ready(); }));
  r.card->tick(r.now_ms += 100);                  // first suppressed tick (grace clock starts)
  CHECK(r.card->transitions() == 2);              // within the 1 s grace
  r.card->tick(r.now_ms += 400);                  // +400 ms since the first suppressed tick
  CHECK(r.card->transitions() == 2);
  r.card->tick(r.now_ms += 700);                  // +1100 ms since the first suppressed tick
  CHECK(r.card->transitions() == 3);              // grace spent: "waiting for STATUS"
  r.t().push(status(0, 40, 2, 1));                // tuned
  REQUIRE(r.soon([&] { return r.card->ready(); }));
  r.card->tick(r.now_ms += 10);
  CHECK(r.card->transitions() == 4);              // "owned and tuned" again
}
// The survey tests poll frames().foreign to know the RX thread has processed
// a pushed SURVEY (messages are handled in order): each sync sample carries a
// foreign increment. The first SURVEY of a session is a baseline and adds
// nothing, so it is always followed by such a sync sample.
TEST(survey_window_is_the_delta_between_calls_on_one_gen) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 1));
  g.t().push(survey(1, 100, 10, 5, 3, 1));          // session baseline: the relay's backlog is not counted
  g.t().push(survey(1, 100, 10, 5, 3, 2));          // sync: foreign +1, no airtime
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 1; }));
  CHECK(!g.card->read_survey_window().valid);       // first call: baseline only
  g.t().push(survey(1, 250, 160, 10, 13, 4));       // +150 ms: busy +150 (100 %), rx +5, foreign +2
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 3; }));
  const auto w = g.card->read_survey_window();
  CHECK(w.valid);
  CHECK(std::abs(w.busy_pct - 100.0) < 0.01);
  CHECK(std::abs(w.rx_pct - 100.0 * 5 / 150) < 0.01);
  const auto e = g.card->read_energy_scout();
  CHECK(e.fa_valid);
  CHECK(e.fa_ofdm == 10);                           // since the session baseline: 13 - 3
  CHECK(g.card->read_energy_scout().fa_ofdm == 0);  // a delta: consumed
  CHECK(g.card->frames().foreign == 3);             // 4 - 1
}

TEST(survey_window_invalid_across_a_gen_change_or_short_span) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 1));
  g.t().push(survey(1, 100, 10, 5, 0, 0));          // session baseline
  g.t().push(survey(1, 100, 10, 5, 0, 1));          // sync: foreign +1
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 1; }));
  g.card->read_survey_window();                     // window baseline
  g.t().push(survey(2, 250, 10, 5, 0, 1));          // new gen (a retune or sweep): raw span 150 ms, still invalid
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 2; }));   // a new gen adds its full count
  CHECK(!g.card->read_survey_window().valid);
  g.t().push(survey(2, 300, 20, 5, 0, 2));          // only 50 ms on gen 2
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 3; }));
  CHECK(!g.card->read_survey_window().valid);
}

TEST(survey_reopen_does_not_re_add_the_relay_backlog) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 1));
  g.t().push(survey(1, 100, 10, 5, 3, 1));          // baseline
  g.t().push(survey(1, 100, 10, 5, 5, 2));          // +2 ofdm, +1 foreign
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 1; }));
  CHECK(g.card->read_energy_scout().fa_ofdm == 2);
  g.card->stop();
  REQUIRE(g.card->open_and_start());                // reopen after a lost relay: same gen, counters kept
  g.t().push(status(0, 136, 2, 1));
  g.t().push(survey(1, 200, 20, 5, 5, 2));          // new session's baseline: cumulative 5 / 2 not re-added
  g.t().push(survey(1, 200, 20, 5, 6, 3));          // +1 ofdm, +1 foreign
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 2; }));
  CHECK(g.card->read_energy_scout().fa_ofdm == 1);
}

// Final review item 6: a counter that goes DOWN within one gen (a fast relay
// restart that reused the gen) is a gen change, not a u32 wrap: the window
// across it is invalid, and the totals add the new sample's full count (the
// restarted counters began at 0), never ~4e9.
TEST(survey_counter_decrease_within_a_gen_is_a_gen_change) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 1));
  g.t().push(survey(1, 1000, 100, 50, 30, 10));     // session baseline
  g.t().push(survey(1, 1000, 100, 50, 30, 11));     // sync: foreign +1
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 1; }));
  g.card->read_survey_window();                     // window baseline
  CHECK(g.card->read_energy_scout().fa_ofdm == 0);
  g.t().push(survey(1, 20, 5, 1, 2, 1));            // restarted relay, same gen: everything dropped
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 2; }));   // +1 (full), not a wrap
  CHECK(!g.card->read_survey_window().valid);       // across the restart: invalid
  CHECK(g.card->read_energy_scout().fa_ofdm == 2);  // the new sample's full count
  g.t().push(survey(1, 170, 20, 1, 4, 2));          // +150 ms on the restarted counters
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 3; }));
  const auto w = g.card->read_survey_window();
  CHECK(w.valid);
  CHECK(std::abs(w.busy_pct - 10.0) < 0.01);        // 15 / 150
  CHECK(g.card->read_energy_scout().fa_ofdm == 2);
  // only ONE counter regressing still counts (a restart that already
  // accumulated more active time than the old sample)
  g.t().push(survey(1, 400, 25, 2, 1, 3));          // ofdm 4 -> 1
  REQUIRE(g.soon([&] { return g.card->frames().foreign == 6; }));   // full 3 added
  CHECK(!g.card->read_survey_window().valid);
  CHECK(g.card->read_energy_scout().fa_ofdm == 1);
}

TEST(start_sweep_sends_scan_and_result_comes_back_once) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 1));
  REQUIRE(g.soon([&] { return g.card->ready(); }));
  CHECK(g.card->can_sweep());
  CHECK(g.card->start_sweep({40, 64}, 2, 20));
  CHECK(g.card->sweeping());
  CHECK(g.t().count(kScan) == 1);
  std::vector<uint8_t> scan_msg;
  { std::lock_guard<std::mutex> lk(g.t().mu); for (auto& m : g.t().sent) if (msg_type(m.data(), m.size()) == kScan) scan_msg = m; }
  const uint16_t id = (uint16_t)(scan_msg[4] | (scan_msg[5] << 8));
  std::vector<uint8_t> res = {0x4D, 0x52, 0x04, 0x08, (uint8_t)id, (uint8_t)(id >> 8), 0, 136, 2, 1,
                              64, 0, 1, 20, 0, 14, 0, 1, 0, 3, 0, 9, 0};
  g.t().push(res);
  REQUIRE(g.soon([&] { return !g.card->sweeping(); }));
  auto r = g.card->take_sweep_result();
  REQUIRE(r.has_value());
  CHECK(r->entries.size() == 1 && r->entries[0].ch == 64 && r->entries[0].ofdm_err == 9);
  CHECK(!g.card->take_sweep_result().has_value());
  CHECK(g.card->relay_stats()->sweeps == 1);
}

TEST(start_sweep_refused_when_not_owned) {
  Rig g; REQUIRE(g.card->open_and_start());
  g.t().push(status(0, 136, 2, 0));                 // someone else owns it
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  CHECK(!g.card->start_sweep({40}, 2, 20));
  CHECK(g.t().count(kScan) == 0);
}

MTEST_MAIN
