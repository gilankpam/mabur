// menu_debug.h: the MABUR_MENU_DEBUG verb helpers -- key-verb parsing,
// MenuView -> text, and the OSD surface -> PAM writer -- exercised with no
// socket and no device.
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gs_draw.h"
#include "menu_debug.h"
#include "mtest.h"

using namespace maburplay;

namespace {

std::string slurp_binary(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

TEST(key_verb_parses_every_button_and_kind) {
  const std::vector<std::pair<std::string, Button>> buttons = {
      {"up", Button::kUp},     {"down", Button::kDown}, {"left", Button::kLeft},
      {"right", Button::kRight}, {"sel", Button::kSel},
  };
  const std::vector<std::pair<std::string, KeyKind>> kinds = {
      {"tap", KeyKind::kTap}, {"hold", KeyKind::kHold}, {"repeat", KeyKind::kRepeat}};
  for (const auto& bb : buttons) {
    for (const auto& kk : kinds) {
      std::string err;
      const std::optional<KeyEvent> e = parse_key_verb(bb.first + " " + kk.first, &err);
      REQUIRE(e.has_value());
      CHECK(e->button == bb.second);
      CHECK(e->kind == kk.second);
      CHECK(err.empty());
    }
  }
}

TEST(key_verb_rejects_junk_with_usage) {
  const std::string usage = "key <up|down|left|right|sel> <tap|hold|repeat>";
  const std::vector<std::string> bad = {"", "up", "up tap extra", "foo tap", "up foo", "   "};
  for (const std::string& line : bad) {
    std::string err;
    CHECK(!parse_key_verb(line, &err).has_value());
    CHECK(err == usage);
  }
}

TEST(view_text_has_every_prefix_for_an_open_menu_with_a_picker) {
  MenuView v;
  v.open = true;
  v.crumb = "Video";
  v.linked = true;
  ViewRow row;
  row.id = "link.rate";
  row.label = "Rate";
  row.value = "40 MHz";
  row.warn = "slow";
  row.status = "applying";
  row.vtx = true;
  row.adv = true;
  row.dim = true;
  row.selected = true;
  row.busy = true;
  row.status_ok = false;
  row.status_err = true;
  v.rows.push_back(row);
  v.help = "Pick a width";
  v.help_is_error = true;
  v.keys = "up down select";
  v.picker = MenuView::Picker::kChoices;
  v.choices = {{"20", "20 MHz", "", true}, {"40", "40 MHz", "note", true}};
  v.picker_cursor = 1;
  v.picker_current = 0;
  v.number_text = "5";
  v.number_range = "0..10";
  v.confirm_text = "Are you sure?";
  v.toast = "Saved!";
  v.banner = "LINKED";

  const std::string text = menu_view_text(v);
  CHECK(text.find("open 1\n") != std::string::npos);
  CHECK(text.find("crumb Video\n") != std::string::npos);
  CHECK(text.find("linked 1\n") != std::string::npos);
  CHECK(text.find("row > link.rate | Rate | 40 MHz | warn=slow | status=applying | "
                   "flags=dim,busy,vtx,adv,err\n") != std::string::npos);
  CHECK(text.find("help Pick a width\n") != std::string::npos);
  CHECK(text.find("help_error 1\n") != std::string::npos);
  CHECK(text.find("keys up down select\n") != std::string::npos);
  CHECK(text.find("picker choices\n") != std::string::npos);
  CHECK(text.find("choice * 20 | 20 MHz | \n") != std::string::npos);
  CHECK(text.find("choice > 40 | 40 MHz | note\n") != std::string::npos);
  CHECK(text.find("number 5 0..10\n") != std::string::npos);
  CHECK(text.find("confirm Are you sure?\n") != std::string::npos);
  CHECK(text.find("toast Saved!\n") != std::string::npos);
  CHECK(text.find("banner LINKED\n") != std::string::npos);
}

TEST(view_text_for_a_closed_view_is_open_0_with_empty_fields) {
  MenuView v;
  const std::string expected =
      "open 0\n"
      "crumb \n"
      "linked 0\n"
      "help \n"
      "keys \n"
      "picker none\n"
      "number  \n"
      "confirm \n"
      "toast \n"
      "banner \n";
  CHECK(menu_view_text(v) == expected);
}

TEST(pam_round_trips_straight_alpha_from_premultiplied) {
  std::vector<uint32_t> buf(4, 0u);
  buf[0] = premul(0xFFFFFF, 128);  // half-alpha white; the other 3 px stay transparent
  Surface s{buf.data(), 2, 2, 2};
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_menu_debug-snap.pam";
  std::string err;
  REQUIRE(write_surface_pam(s, path, &err));
  const std::string data = slurp_binary(path);
  const std::string hdr = "P7\nWIDTH 2\nHEIGHT 2\nDEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n";
  REQUIRE(data.size() == hdr.size() + 2 * 2 * 4);
  CHECK(data.compare(0, hdr.size(), hdr) == 0);
  const auto* px = reinterpret_cast<const unsigned char*>(data.data() + hdr.size());
  CHECK(px[0] == 255);  // R
  CHECK(px[1] == 255);  // G
  CHECK(px[2] == 255);  // B
  CHECK(px[3] == 128);  // A
  CHECK(px[7] == 0);    // second pixel's A: fully transparent in, straight alpha 0 out
  std::remove(path.c_str());
}

TEST(pam_refuses_an_empty_surface) {
  Surface s{};
  std::string err;
  CHECK(!write_surface_pam(s, std::string(MABUR_TEST_SCRATCH_DIR) + "/test_menu_debug-unused.pam",
                           &err));
  CHECK(err == "no osd surface");
}

MTEST_MAIN
