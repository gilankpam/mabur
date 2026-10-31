#include "menu_debug.h"

#include <cctype>
#include <cstdio>
#include <vector>

#include "mabur/config_write.h"

namespace maburplay {
namespace {

constexpr const char* kKeyUsage = "key <up|down|left|right|sel> <tap|hold|repeat>";

std::vector<std::string> split_ws(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    size_t j = i;
    while (j < s.size() && !std::isspace(static_cast<unsigned char>(s[j]))) ++j;
    if (j > i) out.push_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

bool button_of(const std::string& s, Button* out) {
  if (s == "up") *out = Button::kUp;
  else if (s == "down") *out = Button::kDown;
  else if (s == "left") *out = Button::kLeft;
  else if (s == "right") *out = Button::kRight;
  else if (s == "sel") *out = Button::kSel;
  else return false;
  return true;
}

bool kind_of(const std::string& s, KeyKind* out) {
  if (s == "tap") *out = KeyKind::kTap;
  else if (s == "hold") *out = KeyKind::kHold;
  else if (s == "repeat") *out = KeyKind::kRepeat;
  else return false;
  return true;
}

void add_flag(std::string* flags, bool on, const char* name) {
  if (!on) return;
  if (!flags->empty()) *flags += ",";
  *flags += name;
}

const char* picker_name(MenuView::Picker p) {
  switch (p) {
    case MenuView::Picker::kChoices: return "choices";
    case MenuView::Picker::kNumber: return "number";
    case MenuView::Picker::kConfirm: return "confirm";
    case MenuView::Picker::kNone: break;
  }
  return "none";
}

uint8_t unpremul_channel(uint8_t premul, uint8_t a) {
  if (a == 0) return 0;
  const unsigned v = (static_cast<unsigned>(premul) * 255 + a / 2) / a;
  return static_cast<uint8_t>(v > 255 ? 255 : v);
}

}  // namespace

std::optional<KeyEvent> parse_key_verb(const std::string& rest, std::string* err) {
  const std::vector<std::string> tok = split_ws(rest);
  Button b;
  KeyKind k;
  if (tok.size() == 2 && button_of(tok[0], &b) && kind_of(tok[1], &k)) return KeyEvent{b, k};
  if (err) *err = kKeyUsage;
  return std::nullopt;
}

std::string menu_view_text(const MenuView& v) {
  std::string out;
  out += "open ";
  out += v.open ? "1" : "0";
  out += "\n";
  out += "crumb " + v.crumb + "\n";
  out += "linked ";
  out += v.linked ? "1" : "0";
  out += "\n";
  for (const ViewRow& r : v.rows) {
    std::string flags;
    add_flag(&flags, r.dim, "dim");
    add_flag(&flags, r.busy, "busy");
    add_flag(&flags, r.vtx, "vtx");
    add_flag(&flags, r.adv, "adv");
    add_flag(&flags, r.status_ok, "ok");
    add_flag(&flags, r.status_err, "err");
    out += "row " + std::string(r.selected ? "> " : "") + r.id + " | " + r.label + " | " +
           r.value + " | warn=" + r.warn + " | status=" + r.status + " | flags=" + flags + "\n";
  }
  out += "help " + v.help + "\n";
  if (v.help_is_error) out += "help_error 1\n";
  out += "keys " + v.keys + "\n";
  out += std::string("picker ") + picker_name(v.picker) + "\n";
  if (v.picker == MenuView::Picker::kChoices) {
    for (size_t i = 0; i < v.choices.size(); ++i) {
      const Choice& c = v.choices[i];
      std::string marker;
      if (static_cast<int>(i) == v.picker_cursor) marker += '>';
      if (static_cast<int>(i) == v.picker_current) marker += '*';
      out += "choice " + marker + (marker.empty() ? "" : " ") + c.value + " | " + c.label +
             " | " + c.note + "\n";
    }
  }
  out += "number " + v.number_text + " " + v.number_range + "\n";
  out += "confirm " + v.confirm_text + "\n";
  out += "toast " + v.toast + "\n";
  out += "banner " + v.banner + "\n";
  return out;
}

std::optional<std::string> handle_menu_debug_verb(const std::string& verb, const std::string& rest,
                                                   bool menu_on, const MenuView& view,
                                                   std::optional<KeyEvent>* out_key) {
  if (verb == "key") {
    if (!menu_on) return std::string("err menu off");
    std::string err;
    const std::optional<KeyEvent> e = parse_key_verb(rest, &err);
    if (!e) return std::string("err ") + err;
    if (out_key) *out_key = e;
    return std::string("ok");
  }
  if (verb == "view") {
    if (!menu_on) return std::string("err menu off");
    return "ok\n" + menu_view_text(view);
  }
  return std::nullopt;
}

bool write_surface_pam(const Surface& s, const std::string& path, std::string* err) {
  if (!s.pixels || s.width <= 0 || s.height <= 0) {
    if (err) *err = "no osd surface";
    return false;
  }
  std::string content;
  content += "P7\n";
  content += "WIDTH " + std::to_string(s.width) + "\n";
  content += "HEIGHT " + std::to_string(s.height) + "\n";
  content += "DEPTH 4\n";
  content += "MAXVAL 255\n";
  content += "TUPLTYPE RGB_ALPHA\n";
  content += "ENDHDR\n";
  content.reserve(content.size() + static_cast<size_t>(s.width) * static_cast<size_t>(s.height) * 4);
  for (int y = 0; y < s.height; ++y) {
    const uint32_t* row = s.pixels + static_cast<size_t>(y) * static_cast<size_t>(s.stride_px);
    for (int x = 0; x < s.width; ++x) {
      const uint32_t px = row[x];
      const uint8_t a = static_cast<uint8_t>(px >> 24);
      const uint8_t r = unpremul_channel(static_cast<uint8_t>((px >> 16) & 0xFF), a);
      const uint8_t g = unpremul_channel(static_cast<uint8_t>((px >> 8) & 0xFF), a);
      const uint8_t b = unpremul_channel(static_cast<uint8_t>(px & 0xFF), a);
      content.push_back(static_cast<char>(r));
      content.push_back(static_cast<char>(g));
      content.push_back(static_cast<char>(b));
      content.push_back(static_cast<char>(a));
    }
  }
  const std::string tmp = path + ".tmp";
  if (!mabur::write_file_fsync(tmp, content)) {
    if (err) *err = "failed to write " + tmp;
    return false;
  }
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    if (err) *err = "failed to rename " + tmp + " onto " + path;
    ::remove(tmp.c_str());
    return false;
  }
  return true;
}

}  // namespace maburplay
