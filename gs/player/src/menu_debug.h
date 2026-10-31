#ifndef MABUR_PLAYER_MENU_DEBUG_H_
#define MABUR_PLAYER_MENU_DEBUG_H_
// Debug verbs maburplay's 8403 listener answers under MABUR_MENU_DEBUG=1:
// the bench walk-through runs over ssh, with no hands on the GPIO buttons
// and no eyes on the goggles, so a script needs a way to inject a key
// event, read the menu's current view as text, and pull the OSD plane's
// last published buffer to a file. Host-testable: nothing here touches a
// socket or a device; main.cpp wires these into cfg_ctl.extra_verb
// (mabur/live_apply.h) and the bank's per-tick key queue.
#include <cstdint>
#include <optional>
#include <string>

#include "key_timer.h"
#include "menu_model.h"
#include "osd_raster.h"

namespace maburplay {

// `rest` = "<up|down|left|right|sel> <tap|hold|repeat>". Anything else:
// nullopt, *err set to the usage line (no "err " prefix -- the caller
// spells the reply).
std::optional<KeyEvent> parse_key_verb(const std::string& rest, std::string* err);

// A stable, line-oriented dump of a MenuView for a script to grep by fixed
// line prefix ("row ", "choice ", ...). Every field prints even when
// empty, so a missing line always means the view lacks that line, never
// that the field happened to read "".
std::string menu_view_text(const MenuView& v);

// PAM (P7, TUPLTYPE RGB_ALPHA, 8-bit) of `s`'s pixels, converted from the
// OSD plane's premultiplied alpha (gs_draw.h) to the straight alpha PAM
// readers expect. Written to `path + ".tmp"` then renamed onto `path` --
// never read mid-write. False with *err ("no osd surface") for a surface
// with no pixels.
bool write_surface_pam(const Surface& s, const std::string& path, std::string* err);

// `key` and `view` need an actual running menu to mean anything: with
// `menu_on` false (no buttons wired, or the GS font failed to load) the
// menu is never driven, so a queued `key` would sit forever behind a
// lying "ok" (nothing ever drains it), and `view` would dump a MenuView
// nothing ever moves. Both answer "err menu off" instead, and `key`
// leaves `*out_key` untouched (menu_debug's caller must not queue it).
//
// With `menu_on` true: `key` parses `rest` via parse_key_verb -- "err
// <usage>" on failure, "ok" with `*out_key` set on success; `view` dumps
// `menu_view_text(view)` in an "ok\n..." reply. `view` must be the
// caller's own fresh, unpaced `model.view(now)` -- a script sees the
// state right after its key, not whatever the composer's paced feed last
// drew.
//
// nullopt for any other verb ("snap", an unknown one), so the caller's
// own dispatch still runs and falls through to the usage error.
std::optional<std::string> handle_menu_debug_verb(const std::string& verb, const std::string& rest,
                                                   bool menu_on, const MenuView& view,
                                                   std::optional<KeyEvent>* out_key);

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_DEBUG_H_
