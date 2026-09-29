#include "relay_client.h"

#include <algorithm>

#include "dot11.h"

namespace maburgs {
namespace {
// RxBody RSSI is raw dBm + 110 (0 = absent); SNR is raw half-dB
// (snr_units.h). The relay's noise floor is a constant -95 dBm.
uint8_t rssi_raw(int8_t dbm) {
  if (dbm == relay::kDbmAbsent) return 0;
  return static_cast<uint8_t>(std::clamp(dbm + 110, 1, 255));
}
int8_t snr_raw(int8_t rssi, int8_t noise) {
  if (rssi == relay::kDbmAbsent || noise == relay::kDbmAbsent) return 0;
  return static_cast<int8_t>(std::clamp(2 * (rssi - noise), -128, 127));
}
}  // namespace

RelayClient::RelayClient(uint8_t channel, uint8_t sec, SendFn send)
    : ch_(channel), sec_(sec), send_(std::move(send)) {}

void RelayClient::start(uint64_t now_ms) {
  started_ = true;
  start_ms_ = last_hello_ms_ = now_ms;
  send_(relay::pack_hello());
  send_tune(now_ms);
}

void RelayClient::send_tune(uint64_t now_ms) {
  last_tune_ms_ = now_ms;
  send_(relay::pack_tune(++tune_id_, ch_, sec_));
}

void RelayClient::tick(uint64_t now_ms) {
  if (!started_) return;
  if (now_ms - last_hello_ms_ >= kHelloMs) {
    last_hello_ms_ = now_ms;
    send_(relay::pack_hello());
  }
  if (!owned_and_tuned() && now_ms - start_ms_ <= kTuneWindowMs &&
      now_ms - last_tune_ms_ >= kTuneRetryMs)
    send_tune(now_ms);
}

RelayClient::Rx RelayClient::on_message(const uint8_t* b, size_t n, uint64_t now_ms,
                                        mabur::node::RxBody& out) {
  const int t = relay::msg_type(b, n);
  if (t == relay::kStatus) {
    relay::Status s;
    if (!relay::parse_status(b, n, s)) { ++bad_; return Rx::None; }
    st_ = s;
    have_status_ = true;
    last_status_ms_ = now_ms;
    return Rx::Status;
  }
  if (t != relay::kFrame) { ++bad_; return Rx::None; }
  relay::FrameMeta m;
  const uint8_t* d = nullptr;
  size_t dl = 0;
  if (!relay::parse_frame(b, n, m, d, dl)) { ++bad_; return Rx::None; }
  ++frames_;
  if (have_seq_ && m.seq != last_seq_ + 1) gaps_ += static_cast<uint32_t>(m.seq - last_seq_ - 1);
  have_seq_ = true;
  last_seq_ = m.seq;
  RxMeta meta;
  meta.crc_err = (m.flags & relay::kFlagBadFcs) != 0;
  meta.data_rate = m.mcs <= 7 ? static_cast<uint16_t>(0x0C + m.mcs) : 0;
  meta.physt = (m.flags & relay::kFlagPhyValid) != 0;
  for (int i = 0; i < 2; ++i) {
    meta.rssi[i] = rssi_raw(m.rssi[i]);
    meta.snr[i] = snr_raw(m.rssi[i], m.noise[i]);
  }
  meta.tsfl = m.tsf_lo;
  if (fill_rx_body(d, dl, meta, out) != RxVerdict::Body) return Rx::None;
  out.rx_channel = m.rx_channel;
  return Rx::Body;
}

bool RelayClient::send_control(const std::vector<uint8_t>& f) {
  if (f.size() < 4) return false;
  const size_t rt = static_cast<size_t>(f[2] | (f[3] << 8));
  if (rt < 8 || rt >= f.size()) return false;
  // The rate is max_range_radiotap()'s (dot11.cpp): MCS0, LDPC, STBC, 20 MHz.
  send_(relay::pack_tx(0, relay::kTxLdpc | relay::kTxStbc, f.data() + rt, f.size() - rt));
  ++tx_;
  return true;
}

bool RelayClient::owned_and_tuned() const {
  return have_status_ && st_.you_own && st_.state == 0 && st_.channel == ch_ && st_.sec == sec_;
}

bool RelayClient::refused(uint64_t now_ms) const {
  return started_ && !owned_and_tuned() && now_ms - start_ms_ > kTuneWindowMs;
}

bool RelayClient::lost(uint64_t now_ms) const {
  if (!started_) return false;
  const uint64_t since = have_status_ ? last_status_ms_ : start_ms_;
  return now_ms - since > kLostMs;
}

}  // namespace maburgs
