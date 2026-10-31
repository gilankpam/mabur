// MenuModel: the goggles menu's navigation, pickers, confirm prompt and
// apply status, driven by KeyEvents and read back through view().
#include <string>
#include <vector>

#include "key_timer.h"
#include "menu_model.h"
#include "mtest.h"

using maburplay::Button;
using maburplay::Choice;
using maburplay::KeyEvent;
using maburplay::KeyKind;
using maburplay::MenuModel;
using maburplay::MenuNode;
using maburplay::MenuView;
using maburplay::NodeKind;
using maburplay::Owner;
using maburplay::SetStep;

namespace {

// The live state the fixture tree's lambdas read.
struct World {
  std::string colour = "true";
  double num = 2;
  double frac = 0.5;
  double frac_max = 1.0;
  bool act_visible = true;
};

std::vector<MenuNode> fixture(World* w) {
  MenuNode colour;
  colour.id = "v.colour";
  colour.label = "Colour correction";
  colour.kind = NodeKind::kChoice;
  colour.owner = Owner::kPlayer;
  colour.help = [] { return std::string("Corrects the camera's colours."); };
  colour.choices = [] {
    return std::vector<Choice>{{"true", "On", "", true},
                               {"auto", "Auto", "needs a LUT", false},
                               {"false", "Off", "", true}};
  };
  colour.current = [w] { return w->colour; };
  colour.value_text = [w] {
    return w->colour == "true" ? std::string("On") : w->colour == "false" ? std::string("Off")
                                                                          : std::string("Mixed");
  };
  colour.steps = [](const std::string& v) {
    return std::vector<SetStep>{{Owner::kPlayer, "colortrans.enable", v}};
  };

  MenuNode num;
  num.id = "v.num";
  num.label = "Number";
  num.kind = NodeKind::kNumber;
  num.owner = Owner::kGs;
  num.help = [] { return std::string("A number."); };
  num.number.min = 0;
  num.number.max = 5;
  num.number.step = 1;
  num.number.get = [w] { return w->num; };
  num.confirm = [](const std::string& v) {
    return v == "5" ? std::string("Five is a lot.") : std::string();
  };
  num.steps = [](const std::string& v) {
    return std::vector<SetStep>{{Owner::kGs, "num.key", v}};
  };

  MenuNode frac;
  frac.id = "v.frac";
  frac.label = "Fraction";
  frac.kind = NodeKind::kNumber;
  frac.owner = Owner::kGs;
  frac.number.min = 0.1;
  frac.number.max = 0;   // read live below
  frac.number.step = 0.05;
  frac.number.get = [w] { return w->frac; };
  frac.steps = [](const std::string& v) {
    return std::vector<SetStep>{{Owner::kGs, "frac.key", v}};
  };
  frac.saved_text = [] { return std::string("✓ at next recording"); };

  MenuNode adv;
  adv.id = "v.adv";
  adv.label = "Internals";
  adv.kind = NodeKind::kInfo;
  adv.advanced = true;
  adv.value_text = [] { return std::string("42"); };

  MenuNode drone;
  drone.id = "v.drone";
  drone.label = "Camera mode";
  drone.kind = NodeKind::kChoice;
  drone.owner = Owner::kDrone;
  drone.help = [] { return std::string("The drone's camera."); };
  drone.choices = [] { return std::vector<Choice>{{"60", "60 fps", "", true}}; };
  drone.current = [] { return std::string("60"); };
  drone.value_text = [] { return std::string("60 fps"); };

  MenuNode video;
  video.id = "video";
  video.label = "Video";
  video.kind = NodeKind::kGroup;
  video.children = [=] { return std::vector<MenuNode>{colour, num, frac, adv, drone}; };

  MenuNode act;
  act.id = "l.act";
  act.label = "Do it";
  act.kind = NodeKind::kAction;
  act.owner = Owner::kGs;
  act.visible = [w] { return w->act_visible; };
  act.action = [] { return std::vector<SetStep>{{Owner::kGs, "act.key", "1"}}; };

  MenuNode info;
  info.id = "l.info";
  info.label = "Build";
  info.kind = NodeKind::kInfo;
  info.value_text = [] { return std::string("abc"); };

  MenuNode link;
  link.id = "link";
  link.label = "Link";
  link.kind = NodeKind::kGroup;
  link.group_value = [] { return std::string("2 rows"); };
  link.children = [=] { return std::vector<MenuNode>{act, info}; };

  return {video, link};
}

// frac's max follows another row: patch it in through the world.
MenuModel::Tree tree(World* w) {
  return [w] {
    auto t = fixture(w);
    auto kids = t[0].children();
    for (auto& k : kids)
      if (k.id == "v.frac") k.number.max = w->frac_max;
    t[0].children = [kids] { return kids; };
    return t;
  };
}

MenuModel::Effect key(MenuModel& m, Button b, KeyKind k, uint64_t now = 0) {
  return m.handle(KeyEvent{b, k}, now);
}
MenuModel::Effect tap(MenuModel& m, Button b, uint64_t now = 0) {
  return key(m, b, KeyKind::kTap, now);
}

int selected(const MenuView& v) {
  for (size_t i = 0; i < v.rows.size(); ++i)
    if (v.rows[i].selected) return static_cast<int>(i);
  return -1;
}

std::string selected_id(const MenuView& v) {
  int i = selected(v);
  return i < 0 ? "" : v.rows[i].id;
}

// Opens the menu and enters the Video group.
void into_video(MenuModel& m) {
  m.open();
  tap(m, Button::kRight);
}

void cursor_to(MenuModel& m, const std::string& id) {
  for (int i = 0; i < 20 && selected_id(m.view(0)) != id; ++i) tap(m, Button::kDown);
  REQUIRE(selected_id(m.view(0)) == id);
}

}  // namespace

TEST(closed_menu_opens_only_on_hold_right_or_sel_tap) {
  World w;
  MenuModel m(tree(&w));
  key(m, Button::kUp, KeyKind::kTap);
  key(m, Button::kDown, KeyKind::kTap);
  key(m, Button::kLeft, KeyKind::kTap);
  key(m, Button::kLeft, KeyKind::kHold);
  CHECK(!m.is_open());
  key(m, Button::kRight, KeyKind::kHold);
  CHECK(m.is_open());
  MenuView v = m.view(0);
  CHECK(v.open);
  CHECK(v.crumb == "Menu");
  REQUIRE(v.rows.size() == 2);
  CHECK(v.rows[0].label == "Video" && v.rows[0].selected);
  CHECK(v.rows[0].value == "▸");
  CHECK(v.rows[1].value == "2 rows");

  MenuModel m2(tree(&w));
  key(m2, Button::kSel, KeyKind::kTap);
  CHECK(m2.is_open());
}

TEST(a_tap_on_right_while_closed_only_toasts) {
  World w;
  MenuModel m(tree(&w));
  auto e = tap(m, Button::kRight, 1000);
  CHECK(e.steps.empty());
  CHECK(!m.is_open());
  CHECK(m.view(1000).toast == "Hold ▶ to open the menu");
  CHECK(m.view(2799).toast == "Hold ▶ to open the menu");
  CHECK(m.view(2800).toast.empty());
  CHECK(!m.view(1000).open);
  CHECK(m.view(1000).rows.empty());
}

TEST(cursor_wraps_both_ways_and_skips_nothing) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  // Video without advanced rows: colour, num, frac, drone.
  CHECK(selected_id(m.view(0)) == "v.colour");
  tap(m, Button::kUp);
  CHECK(selected_id(m.view(0)) == "v.drone");
  tap(m, Button::kDown);
  CHECK(selected_id(m.view(0)) == "v.colour");
  std::vector<std::string> seen;
  for (int i = 0; i < 4; ++i) {
    seen.push_back(selected_id(m.view(0)));
    tap(m, Button::kDown);
  }
  CHECK((seen == std::vector<std::string>{"v.colour", "v.num", "v.frac", "v.drone"}));
  CHECK(selected_id(m.view(0)) == "v.colour");
  // A repeat outside a number picker does nothing.
  key(m, Button::kDown, KeyKind::kRepeat);
  CHECK(selected_id(m.view(0)) == "v.colour");
}

TEST(right_on_a_group_pushes_and_left_pops_then_closes_at_top) {
  World w;
  MenuModel m(tree(&w));
  m.open();
  tap(m, Button::kDown);
  tap(m, Button::kRight);
  MenuView v = m.view(0);
  CHECK(v.crumb == "Link");
  CHECK(selected_id(v) == "l.act");
  tap(m, Button::kLeft);
  v = m.view(0);
  CHECK(v.crumb == "Menu");
  CHECK(selected_id(v) == "link");   // the cursor is kept per level
  tap(m, Button::kLeft);
  CHECK(!m.is_open());
  // SEL behaves like right, and a held right in the list acts once.
  m.open();
  key(m, Button::kSel, KeyKind::kTap);
  CHECK(m.view(0).crumb == "Video");
  m.close();
  m.open();
  key(m, Button::kRight, KeyKind::kHold);
  CHECK(m.view(0).crumb == "Video");
}

TEST(hold_left_closes_from_a_picker_without_sending) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  tap(m, Button::kRight);
  CHECK(m.view(0).picker == MenuView::Picker::kChoices);
  auto e = key(m, Button::kLeft, KeyKind::kHold);
  CHECK(e.steps.empty());
  CHECK(!m.is_open());
  // From a number picker the repeat is handed back.
  into_video(m);
  cursor_to(m, "v.num");
  tap(m, Button::kRight);
  tap(m, Button::kUp);
  e = key(m, Button::kLeft, KeyKind::kHold);
  CHECK(e.steps.empty());
  CHECK(e.repeat_off);
  CHECK(!m.is_open());
  // Reopening starts at the top with no picker.
  m.open();
  CHECK(m.view(0).crumb == "Menu");
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
}

TEST(a_drone_row_is_dim_unlinked_and_refuses_with_a_toast) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  cursor_to(m, "v.drone");
  MenuView v = m.view(0);
  int i = selected(v);
  CHECK(v.rows[i].vtx);
  CHECK(v.rows[i].dim);
  CHECK(v.rows[i].value == "—");
  CHECK(v.help == "Needs the drone linked. The drone's camera.");
  CHECK(!v.linked);
  auto e = tap(m, Button::kRight, 100);
  CHECK(e.steps.empty());
  v = m.view(100);
  CHECK(v.picker == MenuView::Picker::kNone);
  CHECK(v.toast == "Needs the drone linked");

  m.set_linked(true);
  v = m.view(100);
  CHECK(v.linked);
  CHECK(!v.rows[selected(v)].dim);
  CHECK(v.rows[selected(v)].value == "60 fps");
  CHECK(v.help == "The drone's camera.");
  // A GS row is never dim.
  m.set_linked(false);
  CHECK(!m.view(0).rows[0].dim);
}

TEST(picker_opens_on_the_current_choice_and_hides_unpickable_ones) {
  World w;
  w.colour = "false";
  MenuModel m(tree(&w));
  into_video(m);
  tap(m, Button::kRight);
  MenuView v = m.view(0);
  CHECK(v.picker == MenuView::Picker::kChoices);
  REQUIRE(v.choices.size() == 2);
  CHECK(v.choices[0].label == "On" && v.choices[1].label == "Off");
  CHECK(v.picker_current == 1);
  CHECK(v.picker_cursor == 1);
  CHECK(v.keys == "▲▼ choose · ▶ apply · ◀ cancel");
  tap(m, Button::kDown);
  CHECK(m.view(0).picker_cursor == 0);   // wraps
  tap(m, Button::kUp);
  CHECK(m.view(0).picker_cursor == 1);
  auto e = tap(m, Button::kLeft);        // cancel
  CHECK(e.steps.empty());
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
  CHECK(m.is_open());

  // A value the choices do not hold: nothing marked, the cursor on the first.
  w.colour = "mixed";
  tap(m, Button::kRight);
  v = m.view(0);
  CHECK(v.picker_current == -1);
  CHECK(v.picker_cursor == 0);
  CHECK(v.rows[selected(v)].value == "Mixed");
}

TEST(the_current_unpickable_choice_stays_listed_when_the_row_keeps_it) {
  World w;
  w.colour = "auto";
  auto base = tree(&w);
  MenuModel m([base] {
    auto t = base();
    auto kids = t[0].children();
    kids[0].keep_current_if_unpickable = true;
    t[0].children = [kids] { return kids; };
    return t;
  });
  into_video(m);
  tap(m, Button::kRight);
  MenuView v = m.view(0);
  REQUIRE(v.choices.size() == 3);
  CHECK(v.choices[1].value == "auto" && v.choices[1].note == "needs a LUT");
  CHECK(v.picker_current == 1 && v.picker_cursor == 1);
  auto e = tap(m, Button::kRight);       // the current one: just closes
  CHECK(e.steps.empty());
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
}

TEST(applying_the_current_value_sends_nothing) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  tap(m, Button::kRight);
  auto e = tap(m, Button::kRight);
  CHECK(e.steps.empty());
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
  CHECK(!m.busy(Owner::kPlayer));
  // A number picker moved and moved back is unchanged too.
  cursor_to(m, "v.num");
  e = tap(m, Button::kRight);
  CHECK(e.repeat_on);
  tap(m, Button::kUp);
  tap(m, Button::kDown);
  e = key(m, Button::kRight, KeyKind::kHold);
  CHECK(e.steps.empty());
  CHECK(e.repeat_off);
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
}

TEST(a_changed_choice_returns_its_steps_and_locks_the_owner) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  tap(m, Button::kRight);
  tap(m, Button::kDown);
  auto e = key(m, Button::kSel, KeyKind::kTap);
  CHECK(e.row_id == "v.colour");
  CHECK((e.steps == std::vector<SetStep>{{Owner::kPlayer, "colortrans.enable", "false"}}));
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
  CHECK(!m.busy(Owner::kPlayer));        // the applier's apply_started locks it
  m.apply_started("v.colour", 1);
  CHECK(m.busy(Owner::kPlayer));
  CHECK(!m.busy(Owner::kGs));
  MenuView v = m.view(0);
  CHECK(v.rows[0].busy);
  CHECK(v.rows[0].status == "applying");
  m.apply_done("v.colour", true, "", 10);
  CHECK(!m.busy(Owner::kPlayer));
}

TEST(a_confirm_is_asked_when_the_row_says_so_and_left_cancels) {
  World w;
  w.num = 4;
  MenuModel m(tree(&w));
  into_video(m);
  cursor_to(m, "v.num");
  tap(m, Button::kRight);
  tap(m, Button::kUp);
  auto e = tap(m, Button::kRight);
  CHECK(e.steps.empty());
  CHECK(e.repeat_off);
  MenuView v = m.view(0);
  CHECK(v.picker == MenuView::Picker::kConfirm);
  CHECK(v.confirm_text == "Five is a lot.");
  CHECK(v.keys == "▶ go ahead · ◀ cancel");
  e = tap(m, Button::kLeft);
  CHECK(e.steps.empty());
  CHECK(m.view(0).picker == MenuView::Picker::kNone);
  CHECK(m.is_open());

  tap(m, Button::kRight);
  tap(m, Button::kUp);
  tap(m, Button::kRight);
  e = tap(m, Button::kRight);            // go ahead
  CHECK(e.row_id == "v.num");
  CHECK((e.steps == std::vector<SetStep>{{Owner::kGs, "num.key", "5"}}));
  CHECK(m.view(0).picker == MenuView::Picker::kNone);

  // An action without a confirm hands out its sets on the first press.
  tap(m, Button::kLeft);
  tap(m, Button::kDown);
  tap(m, Button::kRight);
  e = tap(m, Button::kRight);
  CHECK(e.row_id == "l.act");
  CHECK((e.steps == std::vector<SetStep>{{Owner::kGs, "act.key", "1"}}));
}

TEST(number_picker_steps_within_range_and_asks_for_repeat) {
  World w;
  w.num = 4;
  MenuModel m(tree(&w));
  into_video(m);
  cursor_to(m, "v.num");
  auto e = tap(m, Button::kRight);
  CHECK(e.repeat_on && !e.repeat_off);
  MenuView v = m.view(0);
  CHECK(v.picker == MenuView::Picker::kNumber);
  CHECK(v.number_text == "4");
  CHECK(v.number_range == "0 – 5");
  CHECK(v.keys == "▲▼ choose · ▶ apply · ◀ cancel");
  tap(m, Button::kUp);
  key(m, Button::kUp, KeyKind::kRepeat);
  key(m, Button::kUp, KeyKind::kRepeat);
  CHECK(m.view(0).number_text == "5");
  for (int i = 0; i < 8; ++i) key(m, Button::kDown, KeyKind::kRepeat);
  CHECK(m.view(0).number_text == "0");
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  e = tap(m, Button::kRight);
  CHECK(e.repeat_off);
  CHECK((e.steps == std::vector<SetStep>{{Owner::kGs, "num.key", "2"}}));
}

TEST(number_values_snap_to_the_step_grid_and_clamp_to_a_live_max) {
  World w;
  w.frac = 0.5;
  w.frac_max = 0.62;
  MenuModel m(tree(&w));
  into_video(m);
  cursor_to(m, "v.frac");
  CHECK(m.view(0).rows[selected(m.view(0))].value == "0.5");
  tap(m, Button::kRight);
  CHECK(m.view(0).number_range == "0.1 – 0.62");
  tap(m, Button::kUp);
  CHECK(m.view(0).number_text == "0.55");
  tap(m, Button::kUp);
  CHECK(m.view(0).number_text == "0.6");
  tap(m, Button::kUp);
  CHECK(m.view(0).number_text == "0.62");
  for (int i = 0; i < 20; ++i) tap(m, Button::kDown);
  CHECK(m.view(0).number_text == "0.1");
  tap(m, Button::kUp);
  CHECK(m.view(0).number_text == "0.15");   // 0.1 + 0.05 is 0.15000000000000002 in doubles
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  CHECK(m.view(0).number_text == "0.25");
  auto e = tap(m, Button::kRight);
  CHECK((e.steps == std::vector<SetStep>{{Owner::kGs, "frac.key", "0.25"}}));

  // A live value above the max opens clamped to it.
  w.frac = 0.9;
  tap(m, Button::kRight);
  CHECK(m.view(0).number_text == "0.62");
  tap(m, Button::kLeft);

  // fmt shapes the picker's and the row's text, never the value sent.
  auto base = tree(&w);
  MenuModel f([base] {
    auto t = base();
    auto kids = t[0].children();
    for (auto& k : kids)
      if (k.id == "v.num")
        k.number.fmt = [](double v) { return std::to_string(static_cast<int>(v)) + " Mb/s"; };
    t[0].children = [kids] { return kids; };
    return t;
  });
  into_video(f);
  cursor_to(f, "v.num");
  CHECK(f.view(0).rows[selected(f.view(0))].value == "2 Mb/s");
  tap(f, Button::kRight);
  CHECK(f.view(0).number_text == "2 Mb/s");
  CHECK(f.view(0).number_range == "0 Mb/s – 5 Mb/s");
  tap(f, Button::kUp);
  e = tap(f, Button::kRight);
  CHECK((e.steps == std::vector<SetStep>{{Owner::kGs, "num.key", "3"}}));
}

TEST(busy_owner_refuses_a_second_edit_with_the_right_toast) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  m.apply_started("v.num", 1);
  CHECK(m.busy(Owner::kGs));
  cursor_to(m, "v.frac");
  auto e = tap(m, Button::kRight, 50);
  CHECK(e.steps.empty() && !e.repeat_on);
  CHECK(m.view(50).picker == MenuView::Picker::kNone);
  CHECK(m.view(50).toast == "One GS change at a time");
  // A player row is a different owner: still editable.
  cursor_to(m, "v.colour");
  tap(m, Button::kRight);
  CHECK(m.view(50).picker == MenuView::Picker::kChoices);
  tap(m, Button::kLeft);
  // An action on the busy owner is refused too.
  tap(m, Button::kLeft);
  tap(m, Button::kDown);
  tap(m, Button::kRight);
  e = tap(m, Button::kRight, 60);
  CHECK(e.steps.empty());
  CHECK(m.view(60).toast == "One GS change at a time");
  m.apply_done("v.num", true, "", 70);
  CHECK(!m.busy(Owner::kGs));

  // A drone row while the drone is busy.
  m.set_linked(true);
  tap(m, Button::kLeft);
  tap(m, Button::kUp);
  tap(m, Button::kRight);
  cursor_to(m, "v.drone");
  m.apply_started("v.drone", 1);
  tap(m, Button::kRight, 80);
  CHECK(m.view(80).toast == "One drone change at a time");
  CHECK(m.view(80).picker == MenuView::Picker::kNone);
}

TEST(a_row_whose_sets_reach_a_busy_owner_is_refused_at_apply) {
  // A player row that also sets a maburgs key (the FC OSD switch's shape).
  MenuNode both;
  both.id = "o.both";
  both.label = "Both halves";
  both.kind = NodeKind::kChoice;
  both.owner = Owner::kPlayer;
  both.choices = [] { return std::vector<Choice>{{"on", "On", "", true}, {"off", "Off", "", true}}; };
  both.current = [] { return std::string("off"); };
  both.steps = [](const std::string& v) {
    return std::vector<SetStep>{{Owner::kPlayer, "osd.enable", v}, {Owner::kGs, "msp.enable", v}};
  };
  MenuModel m([both] { return std::vector<MenuNode>{both}; });
  m.open();
  m.apply_started("elsewhere", 1);   // not in the tree: locks nothing
  CHECK(!m.busy(Owner::kGs) && !m.busy(Owner::kPlayer));
  tap(m, Button::kRight);
  tap(m, Button::kUp);
  auto e = tap(m, Button::kRight);
  REQUIRE(e.steps.size() == 2);
  m.apply_started("o.both", 2);
  CHECK(m.busy(Owner::kPlayer) && m.busy(Owner::kGs));   // every owner its sets touch
  m.apply_done("o.both", true, "", 0);
  CHECK(!m.busy(Owner::kPlayer) && !m.busy(Owner::kGs));

  // maburgs busy with another row: the player row opens, but applying it
  // would queue a maburgs set behind that one, so it is refused.
  MenuNode gs_row;
  gs_row.id = "g.row";
  gs_row.owner = Owner::kGs;
  MenuModel m2([both, gs_row] { return std::vector<MenuNode>{both, gs_row}; });
  m2.open();
  m2.apply_started("g.row", 1);
  CHECK(m2.busy(Owner::kGs));
  tap(m2, Button::kRight);
  CHECK(m2.view(0).picker == MenuView::Picker::kChoices);
  tap(m2, Button::kUp);
  e = tap(m2, Button::kRight, 40);
  CHECK(e.steps.empty());
  CHECK(m2.view(40).toast == "One GS change at a time");
  CHECK(m2.view(40).picker == MenuView::Picker::kNone);
}

TEST(status_text_follows_the_prototype_timings) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  m.apply_started("v.num", 4);
  MenuView v = m.view(0);
  CHECK(v.rows[1].status == "applying");
  CHECK(v.rows[1].busy);
  m.apply_step("v.num", 2);
  CHECK(m.view(0).rows[1].status == "applying 2/4");
  m.apply_done("v.num", true, "", 1000);
  v = m.view(1000);
  CHECK(!v.rows[1].busy);
  CHECK(v.rows[1].status == "✓ saved" && v.rows[1].status_ok);
  CHECK(m.view(2299).rows[1].status == "✓ saved");
  CHECK(m.view(2300).rows[1].status.empty());
  CHECK(!m.view(2300).rows[1].status_ok);

  // A row's own saved text holds longer.
  m.apply_started("v.frac", 1);
  m.apply_step("v.frac", 1);
  CHECK(m.view(3000).rows[2].status == "applying");
  m.apply_done("v.frac", true, "", 3000);
  CHECK(m.view(5199).rows[2].status == "✓ at next recording");
  CHECK(m.view(5200).rows[2].status.empty());

  // An error stays 6 s, red in the row and the help line of a selected row.
  m.apply_started("v.colour", 1);
  m.apply_done("v.colour", false, "publish failed", 10000);
  CHECK(!m.busy(Owner::kPlayer));
  v = m.view(10000);
  CHECK(v.rows[0].status == "✗ failed" && v.rows[0].status_err);
  CHECK(v.help == "err publish failed");
  CHECK(v.help_is_error);
  v = m.view(15999);
  CHECK(v.rows[0].status == "✗ failed" && v.help_is_error);
  v = m.view(16000);
  CHECK(v.rows[0].status.empty() && !v.help_is_error);
  CHECK(v.help == "Corrects the camera's colours.");

  // The applier's own reasons read as the operator's words.
  m.apply_started("v.colour", 1);
  m.apply_done("v.colour", false, "no answer", 20000);
  CHECK(m.view(20000).help == "No answer — it may not have applied");
  m.apply_started("v.colour", 1);
  m.apply_done("v.colour", false, "busy", 30000);
  CHECK(m.view(30000).help == "One GS change at a time");
}

TEST(advanced_rows_show_only_with_the_switch) {
  World w;
  MenuModel m(tree(&w));
  into_video(m);
  auto ids = [&] {
    std::vector<std::string> r;
    for (const auto& row : m.view(0).rows) r.push_back(row.id);
    return r;
  };
  CHECK((ids() == std::vector<std::string>{"v.colour", "v.num", "v.frac", "v.drone"}));
  m.set_show_advanced(true);
  CHECK((ids() == std::vector<std::string>{"v.colour", "v.num", "v.frac", "v.adv", "v.drone"}));
  MenuView v = m.view(0);
  CHECK(v.rows[3].adv && v.rows[3].value == "42");
  // The cursor is clamped when rows disappear under it.
  cursor_to(m, "v.drone");
  m.set_show_advanced(false);
  CHECK(selected_id(m.view(0)) == "v.drone");
  m.set_show_advanced(true);
  cursor_to(m, "v.drone");
  // A hidden row: the Link group's action goes, the cursor stays in range.
  tap(m, Button::kLeft);
  tap(m, Button::kDown);
  tap(m, Button::kRight);
  tap(m, Button::kDown);
  CHECK(selected_id(m.view(0)) == "l.info");
  w.act_visible = false;
  v = m.view(0);
  REQUIRE(v.rows.size() == 1);
  CHECK(v.rows[0].id == "l.info" && v.rows[0].selected);
  tap(m, Button::kDown);
  CHECK(selected_id(m.view(0)) == "l.info");
}

TEST(keys_line_names_the_buttons_for_each_state) {
  World w;
  MenuModel m(tree(&w));
  m.open();
  CHECK(m.view(0).keys == "▶ open · ◀ back · hold ◀ close");
  m.set_has_sel(true);
  CHECK(m.view(0).keys == "▶ open · ◀ back · hold ◀ close   (SEL = ▶)");
  m.set_has_sel(false);
  tap(m, Button::kRight);
  CHECK(m.view(0).keys == "▶ change · ◀ back · hold ◀ close");
  tap(m, Button::kRight);
  CHECK(m.view(0).keys == "▲▼ choose · ▶ apply · ◀ cancel");
  tap(m, Button::kLeft);
  tap(m, Button::kLeft);
  tap(m, Button::kDown);
  tap(m, Button::kRight);
  CHECK(m.view(0).keys == "▶ run · ◀ back");
  tap(m, Button::kDown);
  CHECK(m.view(0).keys == "read-only · ◀ back");
}

TEST(the_list_scrolls_a_window_of_ten_around_the_cursor) {
  std::vector<MenuNode> many;
  for (int i = 0; i < 14; ++i) {
    MenuNode n;
    n.id = "r" + std::to_string(i);
    n.label = n.id;
    n.kind = NodeKind::kInfo;
    many.push_back(n);
  }
  MenuModel m([many] { return many; });
  m.open();
  MenuView v = m.view(0);
  CHECK(v.rows.size() == 10);
  CHECK(v.rows[0].id == "r0");
  for (int i = 0; i < 8; ++i) tap(m, Button::kDown);
  v = m.view(0);
  CHECK(v.rows.size() == 10);
  CHECK(v.rows[0].id == "r4");
  CHECK(selected_id(v) == "r8");
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);
  tap(m, Button::kUp);   // wraps to r13
  v = m.view(0);
  CHECK(v.rows[0].id == "r4" && v.rows[9].id == "r13");
  CHECK(selected_id(v) == "r13");
}

TEST(a_nested_group_crumb_names_its_parent_and_a_vanished_group_pops) {
  bool rung_there = true;
  MenuNode mcs;
  mcs.id = "rung.0.mcs";
  mcs.label = "MCS";
  MenuNode rung;
  rung.id = "rung.0";
  rung.label = "R0";
  rung.kind = NodeKind::kGroup;
  rung.visible = [&rung_there] { return rung_there; };
  rung.children = [mcs] { return std::vector<MenuNode>{mcs}; };
  MenuNode ladder;
  ladder.id = "ladder";
  ladder.label = "Ladder";
  ladder.kind = NodeKind::kGroup;
  ladder.children = [rung] { return std::vector<MenuNode>{rung}; };
  MenuModel m([ladder] { return std::vector<MenuNode>{ladder}; });
  m.open();
  tap(m, Button::kRight);
  CHECK(m.view(0).crumb == "Ladder");
  tap(m, Button::kRight);
  CHECK(m.view(0).crumb == "Ladder › R0");
  CHECK(selected_id(m.view(0)) == "rung.0.mcs");
  rung_there = false;   // e.g. the rung was deleted
  CHECK(m.view(0).crumb == "Ladder");
  CHECK(m.view(0).rows.empty());
  CHECK(m.view(0).keys == "◀ back");
  tap(m, Button::kLeft);
  CHECK(m.view(0).crumb == "Menu");
}

TEST(views_compare_field_wise) {
  World w;
  MenuModel m(tree(&w));
  m.open();
  CHECK(m.view(0) == m.view(0));
  MenuView a = m.view(0);
  tap(m, Button::kDown);
  CHECK(!(a == m.view(0)));
  m.set_banner("⚠ Camera 100 fps");
  MenuView b = m.view(0);
  CHECK(b.banner == "⚠ Camera 100 fps");
  m.close();
  CHECK(m.view(0).banner == "⚠ Camera 100 fps");   // shown with the menu closed too
  CHECK(!(b == m.view(0)));
}

MTEST_MAIN
