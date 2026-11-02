#ifndef MABUR_PLAYER_MENU_MODEL_H_
#define MABUR_PLAYER_MENU_MODEL_H_

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "key_timer.h"

namespace maburplay {

// Who applies a row's sets: maburplay itself, maburgs (over its live-config
// port) or the drone. One set is in flight per owner at a time.
enum class Owner : uint8_t { kPlayer, kGs, kDrone };
enum class NodeKind : uint8_t { kGroup, kChoice, kNumber, kAction, kInfo };

struct Choice {
  std::string value;
  std::string label;
  std::string note;
  bool pickable = true;
  bool operator==(const Choice&) const = default;
};

struct NumberSpec {
  double min = 0, max = 0, step = 1;
  std::function<double()> get;
  std::function<std::string(double)> fmt;
};

// What a row does when the operator applies a value: one or more sets, run in
// order by the applier; a later step is not sent after an earlier err.
struct SetStep {
  Owner owner;
  std::string key;
  std::string value;
  bool operator==(const SetStep&) const = default;
};

struct MenuNode {
  std::string id;            // unique within the tree, e.g. "link.rate"; status is keyed on it
  std::string label;
  NodeKind kind = NodeKind::kInfo;
  Owner owner = Owner::kPlayer;
  bool advanced = false;
  std::function<bool()> visible;                       // null = always
  std::function<std::string()> help;
  std::function<std::string()> value_text;             // the row's right-hand text
  std::function<std::string()> warn;                   // "" = none; rendered amber before the value
  // "" = usable; otherwise the row is dim (even when linked) and ▶ toasts
  // this text instead of opening its picker. Checked after the "needs the
  // drone linked" refusal, so that one still wins for an unlinked drone row.
  std::function<std::string()> refuse;
  std::function<std::vector<Choice>()> choices;        // kChoice
  std::function<std::string()> current;                // kChoice: the current value key
  // List the current choice even when it is not pickable, so a row whose
  // live value the operator cannot pick still shows where it is.
  bool keep_current_if_unpickable = false;
  NumberSpec number;                                   // kNumber
  std::function<std::string(const std::string& v)> confirm;  // "" = no prompt
  std::function<std::vector<SetStep>(const std::string& v)> steps;  // kChoice/kNumber: the sets
  std::function<std::string()> saved_text;             // null = "✓ saved"
  // "" = none; otherwise, when the row's apply settles ok, the saved status
  // is drawn amber (ViewRow::status_warn) and the help line reads this text
  // until the row's next apply starts -- not timed out like a plain
  // "✓ saved". Used for LDPC's wall warning.
  std::function<std::string()> warn_after_ok;
  std::function<std::vector<SetStep>()> action;        // kAction: its sets (empty = nothing to send)
  std::function<std::vector<MenuNode>()> children;     // kGroup
  std::function<std::string()> group_value;            // kGroup: "5 rungs" or "" (= "▸")
};

struct ViewRow {
  std::string id, label, value, warn, status;
  bool vtx = false, adv = false, dim = false, selected = false, busy = false,
       status_ok = false, status_err = false, status_warn = false;
  bool operator==(const ViewRow&) const = default;
};

struct MenuView {
  bool open = false;
  std::string crumb;
  bool linked = false;
  // "" = none, otherwise an amber line under the crumb (a drone/GS table
  // mismatch, or "reading the drone's settings" while rows have no value
  // yet).
  std::string header;
  // "" = none, otherwise an amber mark beside the link pill (e.g. "no
  // heartbeat" while linked but the drone has gone quiet).
  std::string pill_mark;
  std::vector<ViewRow> rows;        // the visible window (<= 10), scrolled around the cursor
  std::string help;
  bool help_is_error = false;
  std::string keys;                  // "▶ change · ◀ back · hold ◀ close" etc.
  enum class Picker { kNone, kChoices, kNumber, kConfirm } picker = Picker::kNone;
  std::vector<Choice> choices;
  int picker_cursor = -1;
  int picker_current = -1;
  std::string number_text, number_range, confirm_text;
  std::string toast, banner;
  bool operator==(const MenuView& o) const;   // field-wise; main composes only on a change
};

// The goggles settings menu as a pure state machine: button events in,
// a MenuView and the sets to run out. Rows are re-read from the tree on
// every call (its lambdas read live state), so nothing here goes stale
// when a value changes underneath an open menu.
class MenuModel {
 public:
  static constexpr uint64_t kToastMs = 1800;
  static constexpr uint64_t kSavedMs = 1300;
  static constexpr uint64_t kSavedLongMs = 2200;   // a custom saved text needs reading time
  static constexpr uint64_t kErrorMs = 6000;
  static constexpr size_t kMaxRows = 10;

  using Tree = std::function<std::vector<MenuNode>()>;
  explicit MenuModel(Tree root);

  void set_linked(bool linked);         // header pill; drone rows dim when false
  void set_has_sel(bool has_sel);
  // False until maburgs' live settings have been read: a GS row's ▶ is then
  // refused with a toast, since its values are placeholders.
  void set_gs_known(bool known);
  void set_show_advanced(bool on);
  void set_banner(std::string text);
  // The amber line under the crumb, and the amber mark beside the link
  // pill; both shown only while the menu is open (view()'s crumb/pill are
  // empty when closed too). "" clears either.
  void set_header(std::string text);
  void set_pill_mark(std::string text);
  void toast(std::string text, uint64_t now_ms);

  // One button event; returns what the caller must do next.
  struct Effect {
    std::vector<SetStep> steps;
    std::string row_id;
    bool repeat_on = false, repeat_off = false;
  };
  Effect handle(const KeyEvent& e, uint64_t now_ms);

  // Apply progress from the applier.
  void apply_started(const std::string& row_id, int n_steps);
  void apply_step(const std::string& row_id, int step_i);     // 1-based, shown as "applying 2/4"
  void apply_done(const std::string& row_id, bool ok, const std::string& reason, uint64_t now_ms);

  bool busy(Owner o) const;
  bool is_open() const;
  void open();
  void close();

  MenuView view(uint64_t now_ms) const;   // pure; the times decide what is still shown
  // What a choice row previews right now, or "" for nothing: the picker's
  // cursor while that row's picker is open, then the value it applied for
  // as long as that apply runs -- so a caller that shows the row's effect
  // live (the menu size) never snaps back to the saved value in between.
  // Cancelling the picker, closing the menu, or the apply settling (either
  // way) ends it, and the caller falls back to the saved value.
  std::string preview(const std::string& row_id) const;

 private:
  // A level is a group id (the root is "") and the cursor's index into
  // that group's visible rows.
  struct Level {
    std::string group_id;
    int cursor = 0;
  };
  // A button event reduced to what it means inside the open menu.
  enum class Act : uint8_t { kNone, kUp, kDown, kLeft, kRight };
  enum class EditKind : uint8_t { kChoices, kNumber, kConfirm };
  struct Edit {
    EditKind kind = EditKind::kChoices;
    MenuNode node;                 // the row being changed
    std::vector<Choice> choices;   // kChoices: what the picker lists
    int cursor = 0;
    double value = 0, orig = 0;    // kNumber
    double min = 0, max = 0;
    std::string confirm_text;      // kConfirm
    std::vector<SetStep> steps;    // kConfirm: sent on "go ahead"
  };
  using Locks = std::array<bool, 3>;   // indexed by Owner
  // Who a row's apply locks and what it reads as once saved, learnt when
  // its Effect is handed out (a row's sets may reach past its own owner).
  struct Pending {
    Locks locks{};
    Owner owner = Owner::kPlayer;
    std::function<std::string()> saved_text;
    std::function<std::string()> warn_after_ok;
  };
  struct Status {
    enum class State : uint8_t { kBusy, kOk, kErr } state = State::kBusy;
    int n_steps = 1, step = 0;
    uint64_t t_ms = 0;
    std::string text;      // kOk: the saved text; kErr: the reason
    // kOk only: non-empty keeps the saved status amber and this text in the
    // help line, un-timed, until the row's next apply_started.
    std::string warn;
    Pending pending;
  };
  struct Resolved {
    std::vector<MenuNode> rows;   // the level's visible rows
    std::string label;            // the group's label ("Menu" at the top)
  };

  std::vector<MenuNode> rows_of(const std::vector<MenuNode>& nodes) const;
  // One entry per stack level that still resolves; a group that has gone
  // (or is hidden now) ends the path there.
  std::vector<Resolved> resolve() const;
  std::optional<MenuNode> find(const std::string& id) const;
  bool dim(const MenuNode& n) const;
  std::string row_value(const MenuNode& n) const;
  std::string status_help(const Status& st) const;
  std::string keys_text(const MenuNode* sel) const;

  Effect press_right(const MenuNode& n, uint64_t now_ms);
  Effect press_in_edit(Act act, uint64_t now_ms);
  Effect emit(const MenuNode& n, std::vector<SetStep> steps, uint64_t now_ms);
  void refuse_busy(Owner o, uint64_t now_ms);
  void sync_stack();

  Tree root_;
  bool open_ = false;
  bool linked_ = false;
  bool has_sel_ = false;
  bool gs_known_ = true;
  bool show_advanced_ = false;
  std::string banner_;
  std::string header_;
  std::string pill_mark_;
  std::string toast_;
  uint64_t toast_ms_ = 0;
  std::vector<Level> stack_;
  std::optional<Edit> edit_;
  std::map<std::string, Status> status_;
  std::map<std::string, Pending> pending_;
  std::map<std::string, std::string> picked_;   // a choice row's applied value, until it settles
  Locks busy_{};
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_MODEL_H_
