#ifndef MABUR_PLAYER_MENU_SESSION_H_
#define MABUR_PLAYER_MENU_SESSION_H_
// The running settings menu between its inputs and the two daemons: a
// button event goes through MenuModel::handle, the sets it hands out go to
// the MenuApplier, and maburgs' live view (GsLiveView) is refreshed with a
// bare `get` when the menu opens and after every GS set that lands.
//
// The refresh `get` and the GS setter share one LiveUdpClient, which has
// one exchange in flight at a time: a `get` sent between two steps of a
// row would make the next step's send fail "busy". So a refresh is only
// ever marked pending, and sent once no row is running and the client is
// idle; a row with a GS step that arrives while a refresh is in flight is
// held (one slot), shown applying, and started when the reply lands -- or
// ended at "no answer" with it, when maburgs did not answer the get. A
// player-only row never touches that client and starts at once.
//
// Until the first `get` lands the view holds placeholders: GS rows refuse
// ▶ (MenuModel::set_gs_known), and while the menu is open the `get` is
// asked again every kRetryMs.
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "gs_live_view.h"
#include "key_timer.h"
#include "live_udp_client.h"
#include "menu_apply.h"
#include "menu_model.h"

namespace maburplay {

// Where the refresh `get` goes: maburgs' live-config client, the same one
// GsLiveSetter sends on.
class GsViewSource {
 public:
  virtual ~GsViewSource() = default;
  virtual bool idle() const = 0;
  virtual bool send_get_all(uint64_t now_ms, std::string* why) = 0;
  // The reply once, as LiveUdpClient::poll gives it.
  virtual std::optional<LiveReply> poll(uint64_t now_ms) = 0;
};

class UdpGsViewSource final : public GsViewSource {
 public:
  explicit UdpGsViewSource(LiveUdpClient& client) : client_(client) {}
  bool idle() const override { return !client_.in_flight(); }
  bool send_get_all(uint64_t now_ms, std::string* why) override {
    return client_.send_get("", now_ms, why);
  }
  std::optional<LiveReply> poll(uint64_t now_ms) override { return client_.poll(now_ms); }

 private:
  LiveUdpClient& client_;
};

class MenuSession {
 public:
  static constexpr uint64_t kRetryMs = 2000;

  // Takes over applier.on_ok.
  MenuSession(MenuModel& model, MenuApplier& applier, GsViewSource& gs, GsLiveView& view);

  // One button event, from the bank or injected: the model handles it and
  // whatever sets it hands out start (or wait behind a refresh). Returns
  // the auto-repeat setting the bank must take now, nullopt to leave it.
  std::optional<bool> key(const KeyEvent& e, uint64_t now_ms);

  // Every loop iteration: lands a refresh reply (then starts a held row),
  // advances the running row, and sends a pending refresh when the client
  // is free.
  void tick(uint64_t now_ms);

  bool refresh_pending() const { return refresh_pending_; }
  bool refresh_in_flight() const { return refresh_in_flight_; }
  bool holding() const { return held_.has_value(); }
  // Moves whenever what the menu shows may have changed through this
  // session: a key, a refresh landing, a row's step answering or the row
  // settling.
  uint64_t generation() const { return generation_; }

  // "maburplay: menu: open" / "close" as the menu opens and closes.
  std::function<void(const std::string&)> log;

 private:
  struct Held {
    std::string row_id;
    std::vector<SetStep> steps;
  };
  // `shown`: the model already has the row applying (it was held).
  void start(const std::string& row_id, std::vector<SetStep> steps, bool shown, uint64_t now_ms);

  MenuModel& model_;
  MenuApplier& applier_;
  GsViewSource& gs_;
  GsLiveView& view_;
  bool refresh_pending_ = false;
  bool refresh_in_flight_ = false;
  std::optional<Held> held_;
  uint64_t generation_ = 0;
  uint64_t last_get_ms_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_SESSION_H_
