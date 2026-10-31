#include "menu_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <tuple>
#include <utility>

namespace maburplay {
namespace {

constexpr const char* kTopCrumb = "Menu";
constexpr const char* kNeedsDrone = "Needs the drone linked";
constexpr const char* kReadingGs = "Reading GS settings…";
constexpr int kScrollLead = 4;   // rows kept above the cursor once the list scrolls

size_t idx(Owner o) { return static_cast<size_t>(o); }

// A shown-for-a-while text: still on screen at now_ms? A clock reading
// before the stamp counts as fresh rather than wrapping to "long ago".
bool within(uint64_t now_ms, uint64_t t_ms, uint64_t span_ms) {
  return now_ms < t_ms || now_ms - t_ms < span_ms;
}

// The shortest fixed-point text that reads back as exactly v: "5", "0.55",
// "-1". Fixed-point, never exponent form, because the text is a TOML value
// and a row label.
std::string shortest(double v) {
  if (v == 0) return "0";   // and never "-0"
  char buf[64];
  for (int d = 0; d <= 15; ++d) {
    std::snprintf(buf, sizeof buf, "%.*f", d, v);
    if (std::strtod(buf, nullptr) == v) return buf;
  }
  std::snprintf(buf, sizeof buf, "%.17g", v);
  return buf;
}

int decimals(double x) {
  std::string s = shortest(x);
  auto dot = s.find('.');
  return dot == std::string::npos ? 0 : static_cast<int>(s.size() - dot - 1);
}

// Snaps v to the step grid that starts at min (min + k*step, k rounded)
// and to the decimals the grid is written in, so repeated 0.05 steps land
// on 0.15 and not on 0.15000000000000002; then clamps to [min, max].
double snap(double v, double min, double max, double step) {
  if (step > 0) {
    double k = std::round((v - min) / step);
    v = min + k * step;
    int d = std::min(9, std::max(decimals(step), decimals(min)));
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", d, v);
    v = std::strtod(buf, nullptr);
  }
  return std::clamp(v, min, std::max(min, max));
}

std::string num_text(const NumberSpec& s, double v) { return s.fmt ? s.fmt(v) : shortest(v); }

std::string busy_text(Owner o) {
  return o == Owner::kDrone ? "One drone change at a time" : "One GS change at a time";
}

int index_of(const std::vector<Choice>& cs, const std::string& value) {
  for (size_t i = 0; i < cs.size(); ++i)
    if (cs[i].value == value) return static_cast<int>(i);
  return -1;
}

int wrap(int i, int delta, int n) { return n <= 0 ? 0 : ((i + delta) % n + n) % n; }

}  // namespace

bool MenuView::operator==(const MenuView& o) const {
  auto tie = [](const MenuView& v) {
    return std::tie(v.open, v.crumb, v.linked, v.rows, v.help, v.help_is_error, v.keys, v.picker,
                    v.choices, v.picker_cursor, v.picker_current, v.number_text, v.number_range,
                    v.confirm_text, v.toast, v.banner);
  };
  return tie(*this) == tie(o);
}

MenuModel::MenuModel(Tree root) : root_(std::move(root)) {}

void MenuModel::set_linked(bool linked) { linked_ = linked; }
void MenuModel::set_has_sel(bool has_sel) { has_sel_ = has_sel; }
void MenuModel::set_gs_known(bool known) { gs_known_ = known; }
void MenuModel::set_show_advanced(bool on) { show_advanced_ = on; }
void MenuModel::set_banner(std::string text) { banner_ = std::move(text); }

void MenuModel::toast(std::string text, uint64_t now_ms) {
  toast_ = std::move(text);
  toast_ms_ = now_ms;
}

bool MenuModel::busy(Owner o) const { return busy_[idx(o)]; }
bool MenuModel::is_open() const { return open_; }

void MenuModel::open() {
  open_ = true;
  stack_.assign(1, Level{});
  edit_.reset();
}

void MenuModel::close() {
  open_ = false;
  stack_.clear();
  edit_.reset();
}

std::vector<MenuNode> MenuModel::rows_of(const std::vector<MenuNode>& nodes) const {
  std::vector<MenuNode> out;
  for (const auto& n : nodes) {
    if (n.advanced && !show_advanced_) continue;
    if (n.visible && !n.visible()) continue;
    out.push_back(n);
  }
  return out;
}

std::vector<MenuModel::Resolved> MenuModel::resolve() const {
  std::vector<Resolved> out;
  out.push_back({rows_of(root_ ? root_() : std::vector<MenuNode>{}), kTopCrumb});
  for (size_t i = 1; i < stack_.size(); ++i) {
    const auto& prev = out.back().rows;
    auto it = std::find_if(prev.begin(), prev.end(), [&](const MenuNode& n) {
      return n.kind == NodeKind::kGroup && n.id == stack_[i].group_id;
    });
    if (it == prev.end()) break;
    Resolved r{rows_of(it->children ? it->children() : std::vector<MenuNode>{}), it->label};
    out.push_back(std::move(r));
  }
  return out;
}

std::optional<MenuNode> MenuModel::find(const std::string& id) const {
  std::function<std::optional<MenuNode>(const std::vector<MenuNode>&)> walk =
      [&](const std::vector<MenuNode>& nodes) -> std::optional<MenuNode> {
    for (const auto& n : nodes) {
      if (n.id == id) return n;
      if (n.kind == NodeKind::kGroup && n.children)
        if (auto f = walk(n.children())) return f;
    }
    return std::nullopt;
  };
  return root_ ? walk(root_()) : std::nullopt;
}

void MenuModel::sync_stack() {
  auto levels = resolve();
  stack_.resize(levels.size());
  for (size_t i = 0; i < stack_.size(); ++i) {
    int n = static_cast<int>(levels[i].rows.size());
    stack_[i].cursor = std::clamp(stack_[i].cursor, 0, std::max(0, n - 1));
  }
}

bool MenuModel::dim(const MenuNode& n) const { return n.owner == Owner::kDrone && !linked_; }

std::string MenuModel::row_value(const MenuNode& n) const {
  if (dim(n)) return "—";
  switch (n.kind) {
    case NodeKind::kGroup: {
      std::string g = n.group_value ? n.group_value() : "";
      return g.empty() ? "▸" : g;
    }
    case NodeKind::kAction:
      return n.value_text ? n.value_text() : "▸";
    case NodeKind::kNumber:
      if (n.value_text) return n.value_text();
      return n.number.get ? num_text(n.number, n.number.get()) : "";
    case NodeKind::kChoice: {
      if (n.value_text) return n.value_text();
      std::string cur = n.current ? n.current() : "";
      auto cs = n.choices ? n.choices() : std::vector<Choice>{};
      int i = index_of(cs, cur);
      return i >= 0 ? cs[i].label : cur;
    }
    case NodeKind::kInfo:
      return n.value_text ? n.value_text() : "";
  }
  return "";
}

// The applier reports its own failures as bare words; the row reads them
// as the operator would say them.
std::string MenuModel::status_help(const Status& st) const {
  if (st.text == "no answer") return "No answer — it may not have applied";
  if (st.text == "busy") return busy_text(st.pending.owner);
  return "err " + st.text;
}

std::string MenuModel::keys_text(const MenuNode* sel) const {
  std::string k;
  if (edit_ && edit_->kind == EditKind::kConfirm) k = "▶ go ahead · ◀ cancel";
  else if (edit_) k = "▲▼ choose · ▶ apply · ◀ cancel";
  else if (!sel) k = "◀ back";
  else if (sel->kind == NodeKind::kGroup) k = "▶ open · ◀ back · hold ◀ close";
  else if (sel->kind == NodeKind::kAction) k = "▶ run · ◀ back";
  else if (sel->kind == NodeKind::kInfo) k = "read-only · ◀ back";
  else k = "▶ change · ◀ back · hold ◀ close";
  if (has_sel_) k += "   (SEL = ▶)";
  return k;
}

void MenuModel::refuse_busy(Owner o, uint64_t now_ms) { toast(busy_text(o), now_ms); }

MenuModel::Effect MenuModel::emit(const MenuNode& n, std::vector<SetStep> steps,
                                  uint64_t now_ms) {
  Effect e;
  e.row_id = n.id;
  if (steps.empty()) return e;
  Pending p;
  p.owner = n.owner;
  p.saved_text = n.saved_text;
  p.locks[idx(n.owner)] = true;
  for (const auto& s : steps) p.locks[idx(s.owner)] = true;
  // A row whose sets reach another owner (OSD's FC switch: player, then
  // maburgs) waits for every owner it touches.
  for (size_t o = 0; o < p.locks.size(); ++o) {
    if (p.locks[o] && busy_[o]) {
      refuse_busy(static_cast<Owner>(o), now_ms);
      return Effect{};
    }
  }
  pending_[n.id] = std::move(p);
  e.steps = std::move(steps);
  return e;
}

MenuModel::Effect MenuModel::press_right(const MenuNode& n, uint64_t now_ms) {
  if (dim(n)) {
    toast(kNeedsDrone, now_ms);
    return {};
  }
  switch (n.kind) {
    case NodeKind::kGroup:
      stack_.push_back(Level{n.id, 0});
      return {};
    case NodeKind::kInfo: {
      std::string h = n.help ? n.help() : "";
      if (!h.empty()) toast(h, now_ms);
      return {};
    }
    default:
      break;
  }
  if (n.owner == Owner::kGs && !gs_known_) {
    toast(kReadingGs, now_ms);
    return {};
  }
  if (busy(n.owner)) {
    refuse_busy(n.owner, now_ms);
    return {};
  }
  Edit ed;
  ed.node = n;
  if (n.kind == NodeKind::kAction) {
    auto steps = n.action ? n.action() : std::vector<SetStep>{};
    std::string c = n.confirm ? n.confirm("") : "";
    if (c.empty()) return emit(n, std::move(steps), now_ms);
    ed.kind = EditKind::kConfirm;
    ed.confirm_text = std::move(c);
    ed.steps = std::move(steps);
    edit_ = std::move(ed);
    return {};
  }
  if (n.kind == NodeKind::kChoice) {
    std::string cur = n.current ? n.current() : "";
    for (auto& c : n.choices ? n.choices() : std::vector<Choice>{})
      if (c.pickable || (n.keep_current_if_unpickable && c.value == cur)) ed.choices.push_back(c);
    ed.kind = EditKind::kChoices;
    ed.cursor = std::max(0, index_of(ed.choices, cur));
    edit_ = std::move(ed);
    return {};
  }
  // kNumber. max may follow another row's live value: read it now.
  ed.kind = EditKind::kNumber;
  ed.min = n.number.min;
  ed.max = std::max(n.number.min, n.number.max);
  ed.orig = n.number.get ? n.number.get() : ed.min;
  ed.value = std::clamp(ed.orig, ed.min, ed.max);
  edit_ = std::move(ed);
  Effect e;
  e.repeat_on = true;
  return e;
}

MenuModel::Effect MenuModel::press_in_edit(Act act, uint64_t now_ms) {
  Edit& ed = *edit_;
  if (ed.kind == EditKind::kConfirm) {
    if (act == Act::kLeft) {
      edit_.reset();
    } else if (act == Act::kRight) {
      MenuNode n = std::move(ed.node);
      auto steps = std::move(ed.steps);
      edit_.reset();
      return emit(n, std::move(steps), now_ms);
    }
    return {};
  }

  const bool number = ed.kind == EditKind::kNumber;
  if (act == Act::kUp || act == Act::kDown) {
    if (number) {
      double step = ed.node.number.step;
      ed.value = snap(ed.value + (act == Act::kUp ? step : -step), ed.min, ed.max, step);
    } else {
      ed.cursor = wrap(ed.cursor, act == Act::kUp ? -1 : 1, static_cast<int>(ed.choices.size()));
    }
    return {};
  }
  if (act != Act::kLeft && act != Act::kRight) return {};

  Effect off;
  off.repeat_off = number;
  if (act == Act::kLeft) {
    edit_.reset();
    return off;
  }

  // Apply. The current value (or an empty picker) closes with nothing sent.
  std::string v;
  if (number) {
    if (ed.value == ed.orig) {
      edit_.reset();
      return off;
    }
    v = shortest(ed.value);
  } else {
    if (ed.choices.empty() || ed.choices[ed.cursor].value == (ed.node.current ? ed.node.current() : "")) {
      edit_.reset();
      return off;
    }
    v = ed.choices[ed.cursor].value;
  }
  MenuNode n = ed.node;
  auto steps = n.steps ? n.steps(v) : std::vector<SetStep>{};
  std::string c = n.confirm ? n.confirm(v) : "";
  if (!c.empty()) {
    Edit cf;
    cf.kind = EditKind::kConfirm;
    cf.node = std::move(n);
    cf.confirm_text = std::move(c);
    cf.steps = std::move(steps);
    edit_ = std::move(cf);
    return off;
  }
  edit_.reset();
  Effect e = emit(n, std::move(steps), now_ms);
  e.repeat_off = number;
  return e;
}

MenuModel::Effect MenuModel::handle(const KeyEvent& e, uint64_t now_ms) {
  if (!open_) {
    if ((e.button == Button::kRight && e.kind == KeyKind::kHold) ||
        (e.button == Button::kSel && e.kind == KeyKind::kTap)) {
      open();
    } else if (e.button == Button::kRight && e.kind == KeyKind::kTap) {
      toast("Hold ▶ to open the menu", now_ms);
    }
    return {};
  }

  // Hold ◀ closes from anywhere; nothing an open picker or prompt held is sent.
  if (e.button == Button::kLeft && e.kind == KeyKind::kHold) {
    Effect out;
    out.repeat_off = edit_ && edit_->kind == EditKind::kNumber;
    close();
    return out;
  }

  const bool in_number = edit_ && edit_->kind == EditKind::kNumber;
  Act act = Act::kNone;
  switch (e.button) {
    case Button::kUp:
    case Button::kDown:
      // Auto-repeat only steps a number; elsewhere it is ignored.
      if (e.kind == KeyKind::kTap || (e.kind == KeyKind::kRepeat && in_number))
        act = e.button == Button::kUp ? Act::kUp : Act::kDown;
      break;
    case Button::kLeft:
      if (e.kind == KeyKind::kTap) act = Act::kLeft;
      break;
    case Button::kRight:
    case Button::kSel:
      // A held ▶ acts once, like a tap: the timer fires the hold and then
      // nothing on release.
      if (e.kind == KeyKind::kTap || e.kind == KeyKind::kHold) act = Act::kRight;
      break;
    default:
      break;
  }
  if (act == Act::kNone) return {};
  if (edit_) return press_in_edit(act, now_ms);

  sync_stack();
  auto levels = resolve();
  const auto& rows = levels.back().rows;
  Level& lv = stack_.back();
  const int n = static_cast<int>(rows.size());
  switch (act) {
    case Act::kUp:
    case Act::kDown:
      lv.cursor = wrap(lv.cursor, act == Act::kUp ? -1 : 1, n);
      return {};
    case Act::kLeft:
      if (stack_.size() > 1) stack_.pop_back();
      else close();
      return {};
    case Act::kRight:
      if (n == 0) return {};
      return press_right(rows[lv.cursor], now_ms);
    default:
      return {};
  }
}

void MenuModel::apply_started(const std::string& row_id, int n_steps) {
  Status st;
  auto it = pending_.find(row_id);
  auto shown = status_.find(row_id);
  if (it != pending_.end()) {
    st.pending = std::move(it->second);
    pending_.erase(it);
  } else if (shown != status_.end() && shown->second.state == Status::State::kBusy) {
    // Already shown applying (held, then started): keep the locks it learnt.
    st.pending = std::move(shown->second.pending);
  } else if (auto n = find(row_id)) {
    st.pending.owner = n->owner;
    st.pending.saved_text = n->saved_text;
    st.pending.locks[idx(n->owner)] = true;
  }
  st.state = Status::State::kBusy;
  st.n_steps = n_steps;
  for (size_t o = 0; o < busy_.size(); ++o)
    if (st.pending.locks[o]) busy_[o] = true;
  status_[row_id] = std::move(st);
}

void MenuModel::apply_step(const std::string& row_id, int step_i) {
  auto it = status_.find(row_id);
  if (it != status_.end() && it->second.state == Status::State::kBusy) it->second.step = step_i;
}

void MenuModel::apply_done(const std::string& row_id, bool ok, const std::string& reason,
                           uint64_t now_ms) {
  Status& st = status_[row_id];
  for (size_t o = 0; o < busy_.size(); ++o)
    if (st.pending.locks[o]) busy_[o] = false;
  st.t_ms = now_ms;
  if (ok) {
    st.state = Status::State::kOk;
    st.text = st.pending.saved_text ? st.pending.saved_text() : "";
    if (st.text.empty()) st.text = "✓ saved";
  } else {
    st.state = Status::State::kErr;
    st.text = reason;
  }
}

MenuView MenuModel::view(uint64_t now_ms) const {
  MenuView v;
  v.banner = banner_;
  if (!toast_.empty() && within(now_ms, toast_ms_, kToastMs)) v.toast = toast_;
  if (!open_) return v;

  v.open = true;
  v.linked = linked_;
  auto levels = resolve();
  const size_t depth = levels.size();
  if (depth == 1) v.crumb = kTopCrumb;
  else if (depth == 2) v.crumb = levels[1].label;
  else v.crumb = levels[depth - 2].label + " › " + levels[depth - 1].label;

  const auto& rows = levels.back().rows;
  const int n = static_cast<int>(rows.size());
  const int cursor = std::clamp(stack_[depth - 1].cursor, 0, std::max(0, n - 1));
  const int max_rows = static_cast<int>(kMaxRows);
  const int start = std::max(0, std::min(cursor - kScrollLead, n - max_rows));

  const Status* sel_status = nullptr;
  for (int i = start; i < n && i < start + max_rows; ++i) {
    const MenuNode& node = rows[i];
    ViewRow r;
    r.id = node.id;
    r.label = node.label;
    r.vtx = node.owner == Owner::kDrone;
    r.adv = node.advanced;
    r.dim = dim(node);
    r.value = row_value(node);
    if (!r.dim && node.warn) r.warn = node.warn();
    r.selected = i == cursor;
    auto it = status_.find(node.id);
    if (it != status_.end()) {
      const Status& st = it->second;
      switch (st.state) {
        case Status::State::kBusy:
          r.busy = true;
          r.status = "applying";
          if (st.n_steps > 1 && st.step > 0)
            r.status += " " + std::to_string(st.step) + "/" + std::to_string(st.n_steps);
          break;
        case Status::State::kOk:
          if (within(now_ms, st.t_ms, st.text == "✓ saved" ? kSavedMs : kSavedLongMs)) {
            r.status = st.text;
            r.status_ok = true;
          }
          break;
        case Status::State::kErr:
          if (within(now_ms, st.t_ms, kErrorMs)) {
            r.status = "✗ failed";
            r.status_err = true;
            if (r.selected) sel_status = &st;
          }
          break;
      }
    }
    v.rows.push_back(std::move(r));
  }

  const MenuNode* sel = n > 0 ? &rows[cursor] : nullptr;
  if (sel) {
    std::string h = sel->help ? sel->help() : "";
    if (dim(*sel)) {
      v.help = h.empty() ? std::string(kNeedsDrone) + "." : std::string(kNeedsDrone) + ". " + h;
    } else if (sel_status) {
      v.help = status_help(*sel_status);
      v.help_is_error = true;
    } else {
      v.help = std::move(h);
    }
  }
  v.keys = keys_text(sel);

  if (edit_) {
    const Edit& ed = *edit_;
    switch (ed.kind) {
      case EditKind::kChoices:
        v.picker = MenuView::Picker::kChoices;
        v.choices = ed.choices;
        v.picker_cursor = ed.choices.empty() ? -1 : ed.cursor;
        v.picker_current = index_of(ed.choices, ed.node.current ? ed.node.current() : "");
        break;
      case EditKind::kNumber:
        v.picker = MenuView::Picker::kNumber;
        v.number_text = num_text(ed.node.number, ed.value);
        v.number_range = num_text(ed.node.number, ed.min) + " – " + num_text(ed.node.number, ed.max);
        break;
      case EditKind::kConfirm:
        v.picker = MenuView::Picker::kConfirm;
        v.confirm_text = ed.confirm_text;
        break;
    }
  }
  return v;
}

}  // namespace maburplay
