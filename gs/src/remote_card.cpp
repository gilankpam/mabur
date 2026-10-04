#include "remote_card.h"

#include <chrono>
#include <cstdio>

#include "dot11.h"
#include "mabur/ht40.h"

namespace maburgs {
#ifndef __EMSCRIPTEN__
namespace {
uint64_t mono_ms_now() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace
#endif

uint8_t RemoteCard::sec_for(uint8_t ch, uint8_t width_mhz) {
  return width_mhz == 40 ? mabur::ht40_offset(ch) : 0;
}

#ifndef __EMSCRIPTEN__
RemoteCard::RemoteCard(Cfg cfg, BodyQueue& out)
    : RemoteCard(std::move(cfg), out,
                 [](const std::string& a, std::string& e) { return open_udp_transport(a, e); },
                 mono_ms_now) {}
#endif

RemoteCard::RemoteCard(Cfg cfg, BodyQueue& out, OpenFn open, NowMsFn now_ms)
    : cfg_(std::move(cfg)), out_(out), open_(std::move(open)), now_ms_(std::move(now_ms)),
      c_(cfg_.channel, sec_for(cfg_.channel, cfg_.width_mhz),
         [this](const std::vector<uint8_t>& m) { if (t_) t_->send(m.data(), m.size()); }) {
  channel_.store(cfg_.channel);
  width_.store(cfg_.width_mhz);
}

RemoteCard::~RemoteCard() { stop(); }

bool RemoteCard::open_and_start() {
  stop();
  std::string err;
  auto t = open_(cfg_.addr, err);
  if (!t) {
    std::fprintf(stderr, "maburgs relay card %d: %s\n", cfg_.card_id, err.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    t_ = std::move(t);
    stop_.store(false);
    // Re-assert the current target (not cfg_.channel): a reopen after a
    // lost relay must come back where the plan wants the card, and the
    // mechanical retune loop corrects any mismatch next tick anyway.
    c_.retune(channel_.load(), sec_for(channel_.load(), width_.load()), now_ms_());
    c_.start(now_ms_());         // forgets the old STATUS: lost()/refused() count from here
    last_restart_ms_ = now_ms_(); // first refused-restart no sooner than kRefusedRestartMs after open
    if (opened_once_) reconnects_.fetch_add(1);
    last_st_ = St::Down;          // a reopened card logs its transitions afresh
    opened_once_ = true;
  }
  running_.store(true);
  rx_ = std::thread([this] { rx_loop(); });
  log_transition("connecting");
  return true;
}

void RemoteCard::stop() {
  stop_.store(true);
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (t_) t_->close();
  }
  if (rx_.joinable()) rx_.join();
  running_.store(false);
}

bool RemoteCard::ready() const {
  std::lock_guard<std::mutex> lk(mu_);
  // A silent relay's last STATUS still reads owned_and_tuned(); lost wins.
  return running_.load() && c_.owned_and_tuned() && !c_.lost(now_ms_());
}

bool RemoteCard::alive() const {
  std::lock_guard<std::mutex> lk(mu_);
  // A relay we have never heard from since (re)start is not alive: a dead
  // CPE would otherwise read UP for ~2 s after every reopen (bench 2026-10-02).
  return running_.load() && c_.have_status() && !c_.lost(now_ms_());
}

RemoteCard::Health RemoteCard::health() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!running_.load()) return opened_once_ ? Health::Lost : Health::Connecting;
  const uint64_t now = now_ms_();
  if (c_.lost(now)) return Health::Lost;
  if (c_.owned_and_tuned()) return Health::Owned;
  if (c_.ownership_lost(now)) return Health::Taken;
  if (c_.refused(now)) return Health::Refused;
  if (c_.tune_failed(now)) return Health::TuneFailed;
  return Health::Connecting;
}

CardCaps RemoteCard::caps() const {
  CardCaps c;
  c.valid = true;
  c.chip = "ath9k";
  c.gen = "CPE510";
  c.tx_chains = 2;
  c.rx_chains = 2;
  c.bw_mask = (1u << 2) | (1u << 3);   // devourer AdapterCaps kBw20|kBw40 -- the C record's bit encoding
  c.tune5g_lo = 36;
  c.tune5g_hi = 177;
  c.fast_retune = false;
  c.fa_ok = c.igi_ok = c.nhm_ok = c.floor_ok = false;
  c.snr_ok = false;
  return c;
}

void RemoteCard::tick(uint64_t now_ms) {
  const char* what = nullptr;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running_.load()) return;
    c_.tick(now_ms);
    St st = St::Waiting;
    if (c_.lost(now_ms)) st = St::Lost;
    else if (c_.owned_and_tuned()) st = St::Owned;
    else if (c_.refused(now_ms) || c_.ownership_lost(now_ms)) st = St::Refused;
    // An owner mid-retune (a search-burst or hop TUNE: STATUS state 1, or a
    // STATUS still on the old channel) is not a transition out of Owned --
    // the pair waiting/owned used to print on every burst (plan 2 bench).
    else if (last_st_ == St::Owned && c_.status().you_own) st = St::Owned;
    if (cfg_.restart_when_refused && st == St::Refused && now_ms >= last_restart_ms_ && now_ms - last_restart_ms_ >= kRefusedRestartMs) {
      last_restart_ms_ = now_ms;
      c_.start(now_ms);          // HELLO + TUNE again, fresh retry window
      reconnects_.fetch_add(1);
    }
    if (st != last_st_) {
      last_st_ = st;
      what = st == St::Owned ? "owned and tuned" : st == St::Refused ? "refused (another client owns the relay)"
           : st == St::Lost ? "lost (no STATUS for 2 s)" : "waiting for STATUS";
    }
  }
  if (what) log_transition(what);
}

bool RemoteCard::send_control(const std::vector<uint8_t>& body) {
  std::lock_guard<std::mutex> lk(mu_);
  if (!running_.load() || !c_.owned_and_tuned()) { tx_fail_.fetch_add(1); return false; }
  const auto frame = build_control_frame(tx_seq_++, body.data(), body.size());
  if (!c_.send_control(frame)) { tx_fail_.fetch_add(1); return false; }
  tx_frames_.fetch_add(1);
  return true;
}

bool RemoteCard::retune(uint8_t ch) {
  std::lock_guard<std::mutex> lk(mu_);
  channel_.store(ch);
  // Stopped: record the target only -- no TUNE on a closed transport;
  // open_and_start() re-asserts channel_/width_.
  if (!running_.load()) return true;
  c_.retune(ch, sec_for(ch, width_.load()), now_ms_());
  return true;
}

bool RemoteCard::set_width(uint8_t ch, uint8_t width_mhz) {
  std::lock_guard<std::mutex> lk(mu_);
  width_.store(width_mhz);
  channel_.store(ch);
  if (!running_.load()) return true;   // see retune()
  c_.retune(ch, sec_for(ch, width_mhz), now_ms_());
  return true;
}

ScoutFrames RemoteCard::frames() const {
  return ScoutFrames{own_.load(), foreign_.load(), own_air_us_.load()};
}

std::optional<RelayStatsIn> RemoteCard::relay_stats() const {
  std::lock_guard<std::mutex> lk(mu_);
  const relay::Status& s = c_.status();
  RelayStatsIn r;
  r.state = s.state; r.ch = s.channel; r.sec = s.sec;
  r.owned = c_.owned_and_tuned();
  r.frames = c_.frames(); r.gaps = c_.seq_gaps();
  r.your_drops = s.your_drops; r.tx = s.tx; r.tx_fail = s.tx_fail; r.tx_refused = s.tx_refused;
  r.reconnects = reconnects_.load();
  r.you_own = s.you_own;
  r.rx_drops = t_ ? t_->rx_drops() : 0;
  r.tx_drops = t_ ? t_->tx_drops() : 0;
  return r;
}

void RemoteCard::rx_loop() {
  std::vector<uint8_t> buf(8192);
  while (!stop_.load()) {
    RelayTransport* t = nullptr;
    { std::lock_guard<std::mutex> lk(mu_); t = t_.get(); }
    if (!t) break;
    const int n = t->recv(buf.data(), buf.size(), 100);
    if (n < 0) break;
    if (n == 0) continue;
    on_datagram(buf.data(), static_cast<size_t>(n));
  }
  running_.store(false);
}

void RemoteCard::on_datagram(const uint8_t* b, size_t n) {
  const uint64_t now_ms = now_ms_();
  mabur::node::RxBody m;
  RelayClient::Rx r;
  bool ready_now = false;
  uint8_t air_width = 20;
  {
    std::lock_guard<std::mutex> lk(mu_);
    r = c_.on_message(b, n, now_ms, m);
    ready_now = c_.owned_and_tuned() && !c_.lost(now_ms);   // same predicate as ready()
    // Width the relay actually tuned (confirmed sec), not the commanded
    // width_: 40 on an unpaired channel tunes 20, and set_width may land
    // between here and the airtime charge.
    air_width = c_.status().sec != 0 ? 40 : 20;
  }
  if (r == RelayClient::Rx::Status || r == RelayClient::Rx::None) return;
  rx_frames_.fetch_add(1);
  if (r == RelayClient::Rx::Foreign) { foreign_.fetch_add(1); return; }
  // A relay another client has tuned elsewhere, or one still swinging to
  // our order, must not feed the aggregator (spec §2).
  if (!ready_now) return;
  if (m.crc_ok) {
    own_.fetch_add(1);
    // Own airtime (spec 2026-09-25-nhm-airtime §5). Width from the tuned
    // sec, never the per-frame bit; ath9k marks the LAST A-MPDU subframe
    // phy_valid (not the first, as devourer does), so the latch applies to
    // the following PPDU -- one preamble per PPDU is still counted once.
    relay::FrameMeta fm;
    const uint8_t* d; size_t dl;
    relay::parse_frame(b, n, fm, d, dl);
    own_air_.on_frame(dl, m.mcs, m.phy_valid, air_width,
                      (fm.flags & relay::kFlagStbc) != 0, (fm.flags & relay::kFlagSgi) != 0);
    own_air_us_.store(own_air_.total_us());
  }
  m.card_id = cfg_.card_id;
  m.mono_us = now_ms * 1000;
  out_.push(std::move(m));
}

void RemoteCard::log_transition(const char* what) {
  transitions_.fetch_add(1);
  std::fprintf(stderr, "maburgs relay card %d (%s): %s\n", cfg_.card_id, cfg_.addr.c_str(), what);
}

}  // namespace maburgs
