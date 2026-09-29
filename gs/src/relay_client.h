#pragma once
// RelayClient: the pure mabur-relay v3 client (HELLO keepalive, TUNE +
// retry on refusal, ownership, FRAME -> RxBody, uplink TX). No sockets, no
// clock: the caller passes now_ms and a SendFn. Not thread-safe -- a caller
// with an RX thread wraps it in a mutex (web/src/relay_link.h).
// TUNE retry: every kTuneRetryMs while not yet owned (only within
// kTuneWindowMs of start), and again -- with no window limit -- whenever we
// own the link but read back mistuned (channel/sec mismatch, not already
// mid-retune per the relay's own state).
#include <cstdint>
#include <functional>
#include <vector>

#include "mabur/node.h"
#include "relay_wire.h"

namespace maburgs {

class RelayClient {
 public:
  using SendFn = std::function<void(const std::vector<uint8_t>&)>;
  static constexpr uint64_t kHelloMs = 500, kTuneRetryMs = 500, kTuneWindowMs = 2500, kLostMs = 2000;

  RelayClient(uint8_t channel, uint8_t sec, SendFn send);
  void start(uint64_t now_ms);
  void tick(uint64_t now_ms);
  enum class Rx { None, Body, Status };
  Rx on_message(const uint8_t* b, size_t n, uint64_t now_ms, mabur::node::RxBody& out);
  bool send_control(const std::vector<uint8_t>& radiotap_frame);

  bool have_status() const { return have_status_; }
  bool owned_and_tuned() const;
  bool refused(uint64_t now_ms) const;
  bool lost(uint64_t now_ms) const;
  const relay::Status& status() const { return st_; }
  uint64_t frames() const { return frames_; }
  uint64_t seq_gaps() const { return gaps_; }
  uint64_t bad_msgs() const { return bad_; }
  uint64_t tx_sent() const { return tx_; }

 private:
  void send_tune(uint64_t now_ms);
  // Saturating "time since": now < since (an older now_ms than a stored
  // timestamp, e.g. a message processed with a stamp ahead of the ticker)
  // reads as 0 elapsed rather than wrapping to a huge uint64.
  static uint64_t elapsed(uint64_t now, uint64_t since) { return now > since ? now - since : 0; }
  uint8_t ch_, sec_;
  SendFn send_;
  bool started_ = false, have_status_ = false, have_seq_ = false;
  uint64_t start_ms_ = 0, last_hello_ms_ = 0, last_tune_ms_ = 0, last_status_ms_ = 0;
  uint16_t tune_id_ = 0;
  uint32_t last_seq_ = 0;
  relay::Status st_;
  uint64_t frames_ = 0, gaps_ = 0, bad_ = 0, tx_ = 0;
};

}  // namespace maburgs
