#include "gs_live_view.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <map>

#include "mabur/live_apply.h"

namespace maburplay {

namespace {

using mabur::live_apply_detail::trim;

bool parse_int(const std::string& s, int* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (end != s.c_str() + s.size()) return false;
  *out = static_cast<int>(v);
  return true;
}

bool parse_real(const std::string& s, double* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size()) return false;
  *out = v;
  return true;
}

bool parse_bool(const std::string& s, bool* out) {
  if (s == "true") { *out = true; return true; }
  if (s == "false") { *out = false; return true; }
  return false;
}

std::string render_real_g(double v) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

// One key = value line's value text into *out's matching field. Unknown
// keys and values this cannot parse are no-ops: the rest of the dump
// still lands.
void apply_one(const std::string& key, const std::string& value, GsLiveView* out) {
  int i;
  double d;
  bool b;
  if (key == "radio.channel") {
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      out->pinned_channel = std::nullopt;  // "auto" (or any other quoted text)
    } else if (parse_int(value, &i)) {
      out->pinned_channel = i;
    }
  } else if (key == "link.static_mcs") {
    if (parse_int(value, &i)) out->static_mcs = i;
  } else if (key == "link.static_bw") {
    if (parse_int(value, &i)) out->static_bw = i;
  } else if (key == "link.static_overhead_base") {
    if (parse_real(value, &d)) out->static_base = d;
  } else if (key == "link.static_overhead_enh") {
    if (parse_real(value, &d)) out->static_enh = d;
  } else if (key == "link.ladder") {
    std::vector<Rung> r;
    if (parse_ladder_value(value, &r)) out->ladder = std::move(r);
  } else if (key == "link.nack.enable") {
    if (parse_bool(value, &b)) out->nack = b;
  } else if (key == "debug_log.enable") {
    if (parse_bool(value, &b)) out->debug_log = b;
  } else if (key == "msp.enable") {
    if (parse_bool(value, &b)) out->msp = b;
  }
}

}  // namespace

bool parse_get_all(const std::string& reply, GsLiveView* out, uint64_t now_ms) {
  if (reply.rfind("ok", 0) != 0) return false;
  out->have = true;
  out->read_ms = now_ms;
  size_t pos = 2;  // past "ok"
  while (pos < reply.size()) {
    const size_t nl = reply.find('\n', pos);
    const size_t end = nl == std::string::npos ? reply.size() : nl;
    const std::string line = trim(reply.substr(pos, end - pos));
    pos = nl == std::string::npos ? reply.size() : nl + 1;
    if (line.empty()) continue;
    const size_t eq = line.find(" = ");
    if (eq == std::string::npos) continue;
    const std::string key = trim(line.substr(0, eq));
    const std::string value = trim(line.substr(eq + 3));
    apply_one(key, value, out);
  }
  return true;
}

bool parse_ladder_value(const std::string& text, std::vector<Rung>* out) {
  const std::string v = trim(text);
  if (v.size() < 2 || v.front() != '[' || v.back() != ']') return false;
  const std::string inner = v.substr(1, v.size() - 2);

  std::vector<Rung> result;
  size_t pos = 0;
  auto skip_ws = [&] {
    while (pos < inner.size() && std::isspace(static_cast<unsigned char>(inner[pos]))) ++pos;
  };
  for (;;) {
    skip_ws();
    if (pos >= inner.size()) break;
    if (inner[pos] != '{') return false;
    const size_t close = inner.find('}', pos + 1);
    if (close == std::string::npos) return false;
    const std::string body = inner.substr(pos + 1, close - pos - 1);
    if (body.find('{') != std::string::npos) return false;

    std::map<std::string, std::string> fields;
    size_t start = 0;
    bool body_ok = true;
    for (;;) {
      size_t comma = body.find(',', start);
      if (comma == std::string::npos) comma = body.size();
      const std::string pair = body.substr(start, comma - start);
      const size_t eqp = pair.find('=');
      const std::string key = eqp == std::string::npos ? "" : trim(pair.substr(0, eqp));
      const std::string val = eqp == std::string::npos ? "" : trim(pair.substr(eqp + 1));
      if (key.empty() || val.empty() || !fields.emplace(key, val).second) {
        body_ok = false;
        break;
      }
      if (comma >= body.size()) break;
      start = comma + 1;
    }
    if (!body_ok || fields.size() != 4) return false;

    Rung r;
    const auto mcs = fields.find("mcs");
    const auto bw = fields.find("bw");
    const auto base = fields.find("overhead_base");
    const auto enh = fields.find("overhead_enh");
    if (mcs == fields.end() || bw == fields.end() || base == fields.end() || enh == fields.end())
      return false;
    if (!parse_int(mcs->second, &r.mcs) || !parse_int(bw->second, &r.bw) ||
        !parse_real(base->second, &r.base) || !parse_real(enh->second, &r.enh))
      return false;
    result.push_back(r);

    pos = close + 1;
    skip_ws();
    if (pos < inner.size()) {
      if (inner[pos] != ',') return false;
      ++pos;
    }
  }
  if (result.empty()) return false;
  *out = std::move(result);
  return true;
}

std::string render_ladder_value(const std::vector<Rung>& l) {
  std::string out = "[";
  for (size_t i = 0; i < l.size(); ++i) {
    if (i) out += ", ";
    out += "{mcs = " + std::to_string(l[i].mcs) + ", bw = " + std::to_string(l[i].bw) +
           ", overhead_base = " + render_real_g(l[i].base) +
           ", overhead_enh = " + render_real_g(l[i].enh) + "}";
  }
  return out + "]";
}

LadderWidth ladder_width(const std::vector<Rung>& l) {
  if (l.empty()) return LadderWidth::kMixed;
  const int bw0 = l.front().bw;
  for (const Rung& r : l)
    if (r.bw != bw0) return LadderWidth::kMixed;
  if (bw0 == 20) return LadderWidth::k20;
  if (bw0 == 40) return LadderWidth::k40;
  return LadderWidth::kMixed;
}

const std::vector<Rung>& default_ladder(int bw) {
  static const std::vector<Rung> k40 = {
      {0, 40, 0.5, 0.25}, {1, 40, 0.5, 0.25}, {2, 40, 0.5, 0.25}, {3, 40, 0.5, 0.25}, {4, 40, 0.5, 0.25},
  };
  static const std::vector<Rung> k20 = {
      {0, 20, 0.5, 0.25}, {1, 20, 0.5, 0.25}, {2, 20, 0.5, 0.25},
      {3, 20, 0.5, 0.25}, {4, 20, 0.5, 0.25}, {5, 20, 0.5, 0.25},
  };
  return bw == 40 ? k40 : k20;
}

namespace {
bool ladders_equal(const std::vector<Rung>& a, const std::vector<Rung>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (a[i].mcs != b[i].mcs || a[i].bw != b[i].bw || a[i].base != b[i].base || a[i].enh != b[i].enh)
      return false;
  return true;
}
}  // namespace

bool is_default_ladder(const std::vector<Rung>& l) {
  return ladders_equal(l, default_ladder(40)) || ladders_equal(l, default_ladder(20));
}

}  // namespace maburplay
