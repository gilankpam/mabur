#ifndef MABUR_PLAYER_MENU_TREE_H_
#define MABUR_PLAYER_MENU_TREE_H_
// The GS rows of the goggles settings menu: what each row reads, offers and
// sends, as MenuNodes for MenuModel. Every row reads live state through
// MenuTreeDeps when its lambdas run, so a tree built once stays current;
// MenuModel rebuilds it on every call anyway. A row's sets are the TOML
// text its live key takes (docs/gs-live-config.md): Owner::kPlayer steps go
// to maburplay's own setter, Owner::kGs steps to maburgs' (8402).
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
  std::function<double()> camera_hz;              // the camera's measured rate, 0 = unknown
  std::function<std::string()> running_mode;      // "WxH@R" the presenter runs, "" = none
};

std::vector<MenuNode> build_menu_tree(const MenuTreeDeps& d);

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_TREE_H_
