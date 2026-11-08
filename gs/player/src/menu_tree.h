#ifndef MABUR_PLAYER_MENU_TREE_H_
#define MABUR_PLAYER_MENU_TREE_H_
// The GS rows of the goggles settings menu: what each row reads, offers and
// sends, as MenuNodes for MenuModel. Every row reads live state through
// MenuTreeDeps when its lambdas run, so a tree built once stays current;
// MenuModel rebuilds it on every call anyway. A row's sets are the TOML
// text its live key takes (docs/gs-live-config.md): Owner::kPlayer steps go
// to maburplay's own setter, Owner::kGs steps to maburgs' (8402).
// maburgs' rows read "—" until GsLiveView::have: the view's defaults are not
// maburgs' values.
#include <functional>
#include <string>
#include <vector>

#include "gs_live_view.h"
#include "gs_snapshot.h"
#include "menu_model.h"
#include "mode_pick.h"
#include "player_config.h"

namespace maburplay {

struct MenuTreeDeps {
  const Config* cfg = nullptr;        // the player's live config
  const GsLiveView* gs = nullptr;     // maburgs' live keys
  const GsSnapshot* snap = nullptr;   // may be null; link/session/pinned
  std::function<bool()> rec_running;         // any recording in progress
  std::function<bool()> burned_rec_running;  // a burned one
  std::function<std::vector<ModeCand>()> modes;   // the connector's modes (empty on host)
  // follow_camera_hz: the drone's reported mode else the measured rate, 0 = unknown
  std::function<double()> camera_hz;
  std::function<std::string()> running_mode;      // "WxH@R" the presenter runs, "" = none
  // null = unknown (no warning; the host build); otherwise whether refresh
  // sync can act on this GS now: a lock built (display.phase_lock on and a
  // presenter), its frame-length register mapped (an RK356x) and not
  // refused in its first second of vblanks (the controller's total not the mode's).
  std::function<bool()> refresh_sync_available;
};

std::vector<MenuNode> build_menu_tree(const MenuTreeDeps& d);

// The drone group's header line, drawn under the crumb (MenuModel::
// set_header): "" with no snapshot, a stale session, or no link.vtx block;
// the update text under a table mismatch; "Reading…" while the drone's
// table is still being fetched (list == "pending" and some row has no
// value yet); the partial-answer text once the fetch gave up with some
// rows still unanswered; "" once every row has landed.
std::string drone_header(const GsSnapshot* snap, bool stale);

// The amber mark drawn beside the link pill (MenuModel::set_pill_mark):
// "no heartbeat" once the drone is linked but its MAVLink heartbeat has
// gone quiet; "" otherwise -- including before the first heartbeat ever
// arrives on this link, which reads as "no heartbeat yet", not a fault.
std::string drone_pill_mark(const GsSnapshot* snap, bool stale);

// The System group's "Menu size" row. maburplay's loop draws the menu at
// that row's MenuModel::preview() while one is running, so the operator
// sees a size before saving it.
inline constexpr const char* kMenuSizeRow = "sys.size";

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_TREE_H_
