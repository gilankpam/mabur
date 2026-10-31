#ifndef MABUR_PLAYER_LIVE_UDP_CLIENT_H_
#define MABUR_PLAYER_LIVE_UDP_CLIENT_H_
// One outstanding set/get against a GS live-config listener
// (mabur::LiveConfigControl, docs/gs-live-config.md) as seen from the far
// side of its loopback socket: maburgs' own "ok"/"err <reason>" wording
// passes straight through, so nothing above this has to re-spell it.
#include <cstdint>
#include <optional>
#include <string>

namespace maburplay {

// ok: whether the reply was "ok..." (true) or "err <reason>" (false).
// text: the value for a keyed `get`, the reason for an `err`, or the
// whole dump for a bare `get`.
struct LiveReply {
  bool ok;
  std::string text;
};

// A non-blocking client for one GS live-config listener, 127.0.0.1 only
// (maburgs on 8402; maburplay's own listener goes through
// PlayerConfigControl::submit_local instead, never this). One datagram in
// flight at a time: a second send while one is outstanding is refused
// with "busy", the same word the listener gives a second `set`.
class LiveUdpClient {
 public:
  static constexpr uint64_t kReplyTimeoutMs = 8000;

  LiveUdpClient() = default;
  ~LiveUdpClient();
  LiveUdpClient(const LiveUdpClient&) = delete;
  LiveUdpClient& operator=(const LiveUdpClient&) = delete;

  // A fresh socket per call, connected to 127.0.0.1:<port> -- a fresh
  // ephemeral source port each time, so a stale reply arriving after a
  // timeout can never be mistaken for a later exchange's.
  bool open(int port, std::string* err);

  bool send_set(const std::string& key, const std::string& value, uint64_t now_ms, std::string* why);
  bool send_get(const std::string& key, uint64_t now_ms, std::string* why);  // "" = bare get
  bool in_flight() const { return in_flight_; }

  // nullopt while still waiting (and under the timeout); {false, "no
  // answer"} once kReplyTimeoutMs has passed with nothing back -- frees
  // in_flight() either way.
  std::optional<LiveReply> poll(uint64_t now_ms);

 private:
  bool send(const std::string& line, uint64_t now_ms, std::string* why);

  int fd_ = -1;
  bool in_flight_ = false;
  uint64_t sent_ms_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LIVE_UDP_CLIENT_H_
