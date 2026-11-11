#ifndef MABUR_PLAYER_CHANNEL_OBSERVE_H_
#define MABUR_PLAYER_CHANNEL_OBSERVE_H_
// Holds maburgs' channel-picker measuring lease (docs/gs-live-config.md
// "observe <ms>") open for as long as the goggles' own Channel picker is:
// `observe 3000` on open, a renewal every second while it stays open,
// `observe 0` on close. One exchange in flight at a time, same as any
// other LiveUdpClient owner on 8402 -- ChannelObserve never queues a
// second send behind the first; a level change just swaps what the next
// send will ask for, so a close followed by a reopen before the close's
// send has even gone out turns straight back into an open request.
#include <cstdint>
#include <optional>
#include <string>

#include "live_udp_client.h"

namespace maburplay {

// The verb side of a LiveUdpClient, narrowed to what ChannelObserve needs
// so a test can fake it without a socket.
class ObserveLink {
 public:
  virtual ~ObserveLink() = default;
  virtual bool idle() const = 0;
  virtual bool send_line(const std::string& line, uint64_t now_ms, std::string* why) = 0;
  virtual std::optional<LiveReply> poll(uint64_t now_ms) = 0;
};

// The real thing: an ordinary LiveUdpClient exchange against maburgs' 8402.
class UdpObserveLink final : public ObserveLink {
 public:
  explicit UdpObserveLink(LiveUdpClient& client) : client_(client) {}
  bool idle() const override { return !client_.in_flight(); }
  bool send_line(const std::string& line, uint64_t now_ms, std::string* why) override {
    return client_.send_line(line, now_ms, why);
  }
  std::optional<LiveReply> poll(uint64_t now_ms) override { return client_.poll(now_ms); }

 private:
  LiveUdpClient& client_;
};

// Renews the lease while the picker is open, on its own ObserveLink so it
// never collides with a GS row's own set/get or the view refresh's own
// exchange on a shared client.
class ChannelObserve {
 public:
  static constexpr uint64_t kLeaseMs = 3000;
  static constexpr uint64_t kRenewMs = 1000;

  explicit ChannelObserve(ObserveLink& link) : link_(link) {}

  // Call once per main-loop iteration with whether the Channel picker is
  // open right now.
  void tick(bool picker_open, uint64_t now_ms);

  // The measurer named by the last "ok observe <state>" reply received
  // while the picker was open; "" from the close edge until the first
  // reply to a new open lands -- a reply to the close's own "observe 0"
  // (or to a renewal sent just before the close) that only arrives after
  // the picker has shut never counts.
  const std::string& state() const { return state_; }

 private:
  enum class Want { kNone, kOpen, kClose };

  ObserveLink& link_;
  bool open_level_ = false;
  Want want_ = Want::kNone;
  uint64_t last_send_ms_ = 0;
  std::string state_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_CHANNEL_OBSERVE_H_
