#pragma once
// RelayLink: wraps maburgs::RelayClient with a transport (UDP native, the
// ring in the browser) and an RX thread, feeding decoded bodies into the
// existing BodyQueue -- the same consumer live_loop already drains for the
// USB radio path. Thread-safe: RelayClient itself is not, so every access
// goes through mu_.
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "body_queue.h"
#include "relay_client.h"
#include "relay_transport.h"

namespace webgs {

class RelayLink {
 public:
  RelayLink(std::unique_ptr<RelayTransport> t, uint8_t ch, uint8_t sec, maburgs::BodyQueue& q,
            std::function<uint64_t()> now_us);
  ~RelayLink();                                     // stop()
  void start();                                     // RelayClient::start + RX thread
  void tick();                                      // loop thread, every pass
  void send_frame(const std::vector<uint8_t>& f);   // build_control_frame() output
  enum class Ready { Waiting, Owned, Listening, Refused, Unreachable, TuneFailed };
  Ready ready(bool gs_mode);
  bool lost();
  std::string stats_fields();                       // ",\"radio\":\"relay\",..."
  void stop();

 private:
  std::unique_ptr<RelayTransport> t_;
  maburgs::BodyQueue& q_;
  std::function<uint64_t()> now_us_;
  std::mutex mu_;
  maburgs::RelayClient c_;
  std::thread rx_;
  std::atomic<bool> stop_{false}, closed_{false};
};

}  // namespace webgs
