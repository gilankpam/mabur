#include "relay_link.h"

#include <cstdio>

namespace webgs {

RelayLink::RelayLink(std::unique_ptr<RelayTransport> t, uint8_t ch, uint8_t sec,
                     maburgs::BodyQueue& q, std::function<uint64_t()> now_us)
    : t_(std::move(t)), q_(q), now_us_(std::move(now_us)),
      c_(ch, sec, [this](const std::vector<uint8_t>& m) { t_->send(m.data(), m.size()); }) {}

RelayLink::~RelayLink() { stop(); }

void RelayLink::start() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    c_.start(now_us_() / 1000);
  }
  rx_ = std::thread([this] {
    std::vector<uint8_t> buf(8192);
    while (!stop_.load(std::memory_order_acquire)) {
      const int n = t_->recv(buf.data(), buf.size(), 100);
      if (n < 0) { closed_.store(true, std::memory_order_release); break; }
      if (n == 0) continue;
      mabur::node::RxBody b;
      const uint64_t now = now_us_();
      maburgs::RelayClient::Rx r;
      {
        std::lock_guard<std::mutex> lk(mu_);
        r = c_.on_message(buf.data(), static_cast<size_t>(n), now / 1000, b);
      }
      if (r != maburgs::RelayClient::Rx::Body) continue;
      b.card_id = 0;
      b.mono_us = now;
      q_.push(std::move(b));
    }
  });
}

void RelayLink::tick() {
  std::lock_guard<std::mutex> lk(mu_);
  c_.tick(now_us_() / 1000);
}

void RelayLink::send_frame(const std::vector<uint8_t>& f) {
  std::lock_guard<std::mutex> lk(mu_);
  c_.send_control(f);
}

RelayLink::Ready RelayLink::ready(bool gs_mode) {
  if (closed_.load(std::memory_order_acquire)) return Ready::Unreachable;
  std::lock_guard<std::mutex> lk(mu_);
  const uint64_t now = now_us_() / 1000;
  if (c_.owned_and_tuned()) return Ready::Owned;
  if (c_.tune_failed(now)) return Ready::TuneFailed;
  if (!c_.have_status()) return c_.lost(now) ? Ready::Unreachable : Ready::Waiting;
  if (!gs_mode) return Ready::Listening;
  return c_.refused(now) ? Ready::Refused : Ready::Waiting;
}

bool RelayLink::lost() {
  if (closed_.load(std::memory_order_acquire)) return true;
  std::lock_guard<std::mutex> lk(mu_);
  return c_.lost(now_us_() / 1000);
}

bool RelayLink::ownership_lost() {
  std::lock_guard<std::mutex> lk(mu_);
  return c_.ownership_lost(now_us_() / 1000);
}

std::string RelayLink::stats_fields() {
  std::lock_guard<std::mutex> lk(mu_);
  const auto& s = c_.status();
  char b[512];
  std::snprintf(b, sizeof b,
                ",\"radio\":\"relay\",\"relay_state\":%u,\"relay_ch\":%u,\"relay_sec\":%u,"
                "\"relay_owned\":%d,\"relay_you_own\":%d,\"relay_frames\":%llu,\"relay_gaps\":%llu,"
                "\"relay_rx_drops\":%llu,\"relay_tx_ring_drops\":%llu,\"relay_tx\":%u,\"relay_tx_fail\":%u,"
                "\"relay_tx_refused\":%u,\"relay_your_drops\":%u",
                s.state, s.channel, s.sec, c_.owned_and_tuned() ? 1 : 0, s.you_own ? 1 : 0,
                static_cast<unsigned long long>(c_.frames()),
                static_cast<unsigned long long>(c_.seq_gaps()),
                static_cast<unsigned long long>(t_->rx_drops()),
                static_cast<unsigned long long>(t_->tx_drops()), s.tx, s.tx_fail, s.tx_refused,
                s.your_drops);
  return b;
}

void RelayLink::stop() {
  if (stop_.exchange(true)) return;
  t_->close();
  if (rx_.joinable()) rx_.join();
}

}  // namespace webgs
