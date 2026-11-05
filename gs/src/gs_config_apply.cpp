#include "gs_config_apply.h"

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

#include "mabur/channel_set.h"
#include "mabur/config_write.h"
#include "mabur/toml.h"

namespace maburgs {

const std::vector<LiveKey> kGsLiveKeys = {
    {"radio.channel", "radio", "channel", LiveType::Channel},
    {"link.static_mcs", "link", "static_mcs", LiveType::Int},
    {"link.static_bw", "link", "static_bw", LiveType::Int},
    {"link.static_overhead_base", "link", "static_overhead_base", LiveType::Real},
    {"link.static_overhead_enh", "link", "static_overhead_enh", LiveType::Real},
    {"link.ladder", "link", "ladder", LiveType::Ladder},
    {"link.nack.enable", "link.nack", "enable", LiveType::Bool},
    {"debug_log.enable", "debug_log", "enable", LiveType::Bool},
    {"msp.enable", "msp", "enable", LiveType::Bool},
};

namespace {

constexpr const char* kLadderTable = "link.ladder";
constexpr std::array<const char*, 4> kRungKeys = {"mcs", "bw", "overhead_base", "overhead_enh"};

bool is(const LiveKey& k, const char* dotted) { return std::strcmp(k.key, dotted) == 0; }

using mabur::live_apply_detail::trim;

// The shortest text that reads back as `v`, always with a decimal point or
// exponent so it stays a TOML float: 0.5, 0.25, 2.0.
std::string render_real(double v) {
  char buf[40];
  for (int prec = 1; prec <= 17; ++prec) {
    std::snprintf(buf, sizeof(buf), "%.*g", prec, v);
    if (std::strtod(buf, nullptr) == v) break;
  }
  std::string s = buf;
  if (s.find_first_of(".eEni") == std::string::npos) s += ".0";
  return s;
}

std::string render_bool(bool b) { return b ? "true" : "false"; }

// One rung of a submitted ladder: each of kRungKeys' value text, verbatim.
using RungText = std::map<std::string, std::string>;

// Splits the inline ladder `[{k = v, ...}, ...]` into its rungs' value
// text. The config parser rejects inline tables, so the value is checked
// for shape here and its value syntax by parsing the same rungs written as
// [[...]] blocks; types and ranges stay the loader's job.
bool split_ladder(const std::string& value, std::vector<RungText>* rungs, std::string* why) {
  const std::string want =
      "link.ladder: want [{mcs = .., bw = .., overhead_base = .., overhead_enh = ..}, ...]";
  const std::string v = trim(value);
  if (v.size() < 2 || v.front() != '[' || v.back() != ']') {
    *why = want;
    return false;
  }
  const std::string inner = v.substr(1, v.size() - 2);
  std::vector<std::string> bodies;
  size_t pos = 0;
  auto skip_ws = [&] {
    while (pos < inner.size() && std::isspace(static_cast<unsigned char>(inner[pos]))) ++pos;
  };
  for (;;) {
    skip_ws();
    if (pos >= inner.size()) break;
    if (inner[pos] != '{') {
      *why = want;
      return false;
    }
    const size_t close = inner.find('}', pos + 1);
    if (close == std::string::npos) {
      *why = want;
      return false;
    }
    const std::string body = inner.substr(pos + 1, close - pos - 1);
    if (body.find('{') != std::string::npos) {
      *why = "link.ladder: a rung value cannot be a table";
      return false;
    }
    bodies.push_back(body);
    pos = close + 1;
    skip_ws();
    if (pos < inner.size()) {
      if (inner[pos] != ',') {
        *why = want;
        return false;
      }
      ++pos;
    }
  }
  if (bodies.empty()) {
    *why = "link.ladder: must be a non-empty array";
    return false;
  }

  std::vector<RungText> out;
  std::string doc;
  for (size_t i = 0; i < bodies.size(); ++i) {
    const std::string where = "link.ladder[" + std::to_string(i) + "]";
    RungText rung;
    size_t start = 0;
    const std::string& body = bodies[i];
    for (;;) {
      size_t comma = body.find(',', start);
      if (comma == std::string::npos) comma = body.size();
      const std::string pair = body.substr(start, comma - start);
      const size_t eq = pair.find('=');
      const std::string key = eq == std::string::npos ? "" : trim(pair.substr(0, eq));
      const std::string val = eq == std::string::npos ? "" : trim(pair.substr(eq + 1));
      if (key.empty() || val.empty()) {
        *why = where + ": want key = value";
        return false;
      }
      bool known = false;
      for (const char* k : kRungKeys) known = known || key == k;
      if (!known) {
        *why = where + ": unknown key " + key;
        return false;
      }
      if (!rung.emplace(key, val).second) {
        *why = where + ": " + key + " given twice";
        return false;
      }
      if (comma >= body.size()) break;
      start = comma + 1;
    }
    for (const char* k : kRungKeys) {
      if (!rung.count(k)) {
        *why = where + ": missing " + k;
        return false;
      }
    }
    doc += "[[r]]\n";
    for (const char* k : kRungKeys) doc += std::string(k) + " = " + rung[k] + "\n";
    out.push_back(std::move(rung));
  }
  try {
    (void)mabur::toml::parse_toml_string(doc, kLadderTable);
  } catch (const std::exception& e) {
    *why = std::string(kLadderTable) + ": " + mabur::loader_reason(e.what());
    return false;
  }
  *rungs = std::move(out);
  return true;
}

// The run of blank and comment lines that ends the [[link.ladder]] region
// right before the next table header: by position it is the next table's
// lead-in ("# Before stepping up, ..." above [link.probe]), so it survives
// the replace. "" when the region has none or there is no ladder.
std::string ladder_region_tail(const std::string& text) {
  const std::string ours = std::string("[[") + kLadderTable + "]]";
  bool in_region = false;
  size_t region_end = text.size();
  std::vector<std::pair<size_t, size_t>> lines;  // [begin, end) incl. '\n', inside the region
  size_t start = 0;
  while (start < text.size()) {
    size_t nl = text.find('\n', start);
    const size_t end = nl == std::string::npos ? text.size() : nl + 1;
    const std::string t = trim(text.substr(start, end - start));
    const bool header = !t.empty() && t[0] == '[';
    if (!in_region) {
      in_region = header && t == ours;
    } else if (header && t != ours) {
      region_end = start;
      break;
    }
    if (in_region) lines.emplace_back(start, end);
    start = end;
  }
  if (!in_region) return "";
  size_t tail = region_end;
  for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
    const std::string t = trim(text.substr(it->first, it->second - it->first));
    if (!t.empty() && t[0] != '#') break;
    tail = it->first;
  }
  return text.substr(tail, region_end - tail);
}

// Copies the one live field `k` names from `src` into `dst`.
void copy_live_field(const LiveKey& k, const Config& src, Config* dst) {
  if (is(k, "radio.channel")) dst->radio.pin = src.radio.pin;
  else if (is(k, "link.static_mcs")) dst->link.static_mcs = src.link.static_mcs;
  else if (is(k, "link.static_bw")) dst->link.static_bw = src.link.static_bw;
  else if (is(k, "link.static_overhead_base")) dst->link.static_overhead_base = src.link.static_overhead_base;
  else if (is(k, "link.static_overhead_enh")) dst->link.static_overhead_enh = src.link.static_overhead_enh;
  else if (is(k, "link.ladder")) dst->link.ladder_cfg.ladder = src.link.ladder_cfg.ladder;
  else if (is(k, "link.nack.enable")) dst->link.nack.enable = src.link.nack.enable;
  else if (is(k, "debug_log.enable")) dst->debug_log.enable = src.debug_log.enable;
  else if (is(k, "msp.enable")) dst->msp.enable = src.msp.enable;
}

}  // namespace

const LiveKey* gs_live_key(const std::string& dotted) {
  for (const LiveKey& k : kGsLiveKeys)
    if (dotted == k.key) return &k;
  return nullptr;
}

std::string render_live_value(const Config& c, const LiveKey& k) {
  if (is(k, "radio.channel"))
    return c.radio.pin ? std::to_string(static_cast<int>(*c.radio.pin)) : "\"auto\"";
  if (is(k, "link.static_mcs")) return std::to_string(c.link.static_mcs);
  if (is(k, "link.static_bw")) return std::to_string(c.link.static_bw);
  if (is(k, "link.static_overhead_base")) return render_real(c.link.static_overhead_base);
  if (is(k, "link.static_overhead_enh")) return render_real(c.link.static_overhead_enh);
  if (is(k, "link.ladder")) {
    std::string out = "[";
    const auto& ladder = c.link.ladder_cfg.ladder;
    for (size_t i = 0; i < ladder.size(); ++i) {
      if (i) out += ", ";
      out += "{mcs = " + std::to_string(ladder[i].mcs) + ", bw = " + std::to_string(ladder[i].bw) +
             ", overhead_base = " + render_real(ladder[i].overhead_base) +
             ", overhead_enh = " + render_real(ladder[i].overhead_enh) + "}";
    }
    return out + "]";
  }
  if (is(k, "link.nack.enable")) return render_bool(c.link.nack.enable);
  if (is(k, "debug_log.enable")) return render_bool(c.debug_log.enable);
  if (is(k, "msp.enable")) return render_bool(c.msp.enable);
  return "";
}

bool patch_live_key(std::string* text, const LiveKey& k, const std::string& value,
                    std::string* why) {
  if (value.find_first_of("\r\n") != std::string::npos) {
    *why = std::string(k.key) + ": value spans lines";
    return false;
  }
  if (k.type != LiveType::Ladder) {
    if (!mabur::patch_toml_key(text, k.section, k.name, value)) {
      *why = std::string(k.key) + ": not set in the config file";
      return false;
    }
    return true;
  }
  std::vector<RungText> rungs;
  if (!split_ladder(value, &rungs, why)) return false;
  // Written in the bundle's block layout. The replaced region's own
  // comments (between or inside the old blocks) are not carried over; the
  // comment run that leads into the next table is.
  std::string blocks;
  for (size_t i = 0; i < rungs.size(); ++i) {
    if (i) blocks += "\n";
    blocks += std::string("[[") + kLadderTable + "]]\n" +
              "mcs = " + rungs[i]["mcs"] + "\n" +
              "bw  = " + rungs[i]["bw"] + "\n" +
              "overhead_base = " + rungs[i]["overhead_base"] + "\n" +
              "overhead_enh  = " + rungs[i]["overhead_enh"] + "\n";
  }
  const std::string tail = ladder_region_tail(*text);
  if (tail.empty() || tail[0] != '\n') blocks += "\n";
  blocks += tail;
  if (!mabur::replace_toml_array_tables(text, kLadderTable, blocks)) {
    *why = std::string(k.key) + ": no [[link.ladder]] in the config file";
    return false;
  }
  return true;
}

// --- GsConfigApplier -------------------------------------------------------

namespace {

// The loader drops rungs above link.max_mcs without a word; a set that
// names one is refused rather than flown short. The filter keeps order, so
// the first rung that does not line up is the dropped one. The rewrite
// replaces only the first run of [[link.ladder]] blocks, so rungs left
// over past the set came from another run further down the file, which
// the loader appends: refused too, rather than flown long.
std::string refuse_dropped_rung(const LiveKey& k, const std::string& value, const Config& cand) {
  if (k.type != LiveType::Ladder) return "";
  std::vector<RungText> rungs;
  std::string ignored;
  split_ladder(value, &rungs, &ignored);
  const auto& eff = cand.link.ladder_cfg.ladder;
  size_t j = 0;
  for (size_t i = 0; i < rungs.size(); ++i) {
    if (j < eff.size() && eff[j].mcs == std::atoi(rungs[i]["mcs"].c_str()) &&
        eff[j].bw == std::atoi(rungs[i]["bw"].c_str())) {
      ++j;
      continue;
    }
    return "rung " + std::to_string(i) + " exceeds link.max_mcs";
  }
  if (j != eff.size()) return "the config file holds more [[link.ladder]] rungs than the set";
  return "";
}

// The member a radio.channel value pins, or nullopt for "auto" and for
// anything the loader refuses (a non-member, a wrong type), which then
// refuses it with its own message.
std::optional<uint8_t> member_pin(const std::string& value, const Config& live) {
  try {
    const mabur::toml::Value doc = mabur::toml::parse_toml_string("channel = " + value, "radio.channel");
    const mabur::toml::Value& v = doc["channel"];
    if (!v.is_number_integer()) return std::nullopt;
    const int64_t ch = v.get<int64_t>();
    if (ch < 1 || ch > 255 || !mabur::channel_set_member(live.radio.channels, static_cast<uint8_t>(ch)))
      return std::nullopt;
    return static_cast<uint8_t>(ch);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

// The receiver width a link.ladder / link.static_mcs / link.static_bw value
// would derive, read off the value's text on the owning thread before any
// file work: nullopt for any other key, and for a value the patch or the
// loader will refuse with its own message. A ladder rung above
// link.max_mcs is counted here although the loader drops it; that set is
// refused anyway (refuse_dropped_rung).
std::optional<uint8_t> width_of_value(const LiveKey& k, const std::string& value,
                                      const Config& live) {
  LinkCfg link = live.link;
  if (k.type == LiveType::Ladder) {
    std::vector<RungText> rungs;
    std::string ignored;
    if (!split_ladder(value, &rungs, &ignored)) return std::nullopt;
    link.ladder_cfg.ladder.clear();
    for (auto& r : rungs) {
      Rung rung;
      rung.mcs = std::atoi(r["mcs"].c_str());
      rung.bw = std::atoi(r["bw"].c_str());
      link.ladder_cfg.ladder.push_back(rung);
    }
    return derived_width(link);
  }
  if (!is(k, "link.static_mcs") && !is(k, "link.static_bw")) return std::nullopt;
  try {
    const mabur::toml::Value doc = mabur::toml::parse_toml_string("v = " + value, k.key);
    const mabur::toml::Value& v = doc["v"];
    if (!v.is_number_integer()) return std::nullopt;
    const int64_t n = v.get<int64_t>();
    if (is(k, "link.static_mcs")) link.static_mcs = static_cast<int>(n);
    else link.static_bw = static_cast<uint8_t>(n);
    return derived_width(link);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

// Cards first: the width the config derives is asked of the core and set
// before the static pin or the ladder that needs it is handed over. Asked
// again here, on the owning thread, because the submit-time check ran
// before the worker's file work and a scan, hop, sweep or calibration may
// have started since; refused, nothing changes and nothing is published.
// The revert after a failed publish comes through here too, with the
// previous config: refused, the cards keep the new width and the process
// the new ladder or pin -- the two stay in step, the file holds the old
// value, and the engine logs "revert failed: <refusal>".
bool apply_width_then(const GsApplyHooks& h, const Config& c, std::string* reason) {
  const uint8_t want = derived_width(c.link);
  if (std::optional<std::string> r = h.width_refusal(want)) {
    *reason = *r;
    return false;
  }
  h.set_width(want);
  return true;
}

bool apply_to_process(const GsApplyHooks& h, const LiveKey& k, const Config& c, std::string* reason) {
  switch (k.type) {
    case LiveType::Int:
    case LiveType::Real:
      // Every link.static_* key routes here, including the two overhead
      // keys that cannot move derived_width; the flip is harmless then --
      // the core neither refuses nor acts on the width it is already at.
      if (!apply_width_then(h, c, reason)) return false;
      h.set_static(c);  // link.static_*
      return true;
    case LiveType::Ladder:
      if (!apply_width_then(h, c, reason)) return false;
      h.replace_ladder(c);
      return true;
    case LiveType::Bool:
      if (is(k, "link.nack.enable")) {
        h.nack(c.link.nack.enable);
        return true;
      }
      if (is(k, "debug_log.enable")) {
        std::string err;
        if (h.debug_log(c.debug_log.enable, &err)) return true;
        *reason = err.empty() ? "debug log refused" : err;
        return false;
      }
      if (is(k, "msp.enable")) {
        h.msp(c.msp.enable, c);
        return true;
      }
      break;
    case LiveType::Str:
    case LiveType::Channel:
      break;
  }
  *reason = std::string(k.key) + ": no apply for this key";
  return false;
}

mabur::LiveApplySpec<Config, LiveKey> gs_spec(GsApplyHooks hooks) {
  // An empty hook would throw bad_function_call on the core thread at the
  // first set that reaches it; refuse it here, where the wiring is.
  if (!hooks.set_pin || !hooks.set_auto || !hooks.set_static || !hooks.replace_ladder ||
      !hooks.nack || !hooks.debug_log || !hooks.msp || !hooks.pin_refusal ||
      !hooks.width_refusal || !hooks.set_width)
    throw std::invalid_argument("GsConfigApplier: every GsApplyHooks hook must be set");
  const auto h = std::make_shared<const GsApplyHooks>(std::move(hooks));
  mabur::LiveApplySpec<Config, LiveKey> s;
  s.daemon = "maburgs";
  s.thread_name = "mbr-cfgset";
  s.table = &kGsLiveKeys;
  s.render = render_live_value;
  s.patch = patch_live_key;
  s.load = [](const std::string& path) { return load_config(path); };
  // Worker thread: only the candidate's own checks, never the core's state.
  s.refuse_set = [](const LiveKey& k, const std::string& value,
                    const Config& cand) -> std::string {
    return refuse_dropped_rung(k, value, cand);
  };
  s.copy_field = copy_live_field;
  // Owning thread, before any file work: the core's refusals for a pin,
  // and for a set that would flip the receiver width (a 40 MHz flip over a
  // set with no HT40 pair is the core's "pair-valid" refusal here, ahead
  // of the loader's channel-set message). apply_width_then asks again.
  s.precheck = [h](const LiveKey& k, const std::string& value,
                   const Config& live) -> std::optional<std::string> {
    if (k.type == LiveType::Channel) {
      const std::optional<uint8_t> ch = member_pin(value, live);
      if (!ch) return std::nullopt;  // "auto", or the loader's to refuse
      return h->pin_refusal(*ch);
    }
    if (const std::optional<uint8_t> w = width_of_value(k, value, live)) return h->width_refusal(*w);
    return std::nullopt;
  };
  s.apply = [h](const LiveKey& k, const Config&, const Config& to, std::string* reason) {
    return apply_to_process(*h, k, to, reason);
  };
  s.order_after_publish = [](const LiveKey& k) { return k.type == LiveType::Channel; };
  s.order = [h](const LiveKey&, const Config& c) -> std::optional<std::string> {
    if (c.radio.pin) return h->set_pin(*c.radio.pin);
    h->set_auto();
    return std::nullopt;
  };
  return s;
}

}  // namespace

GsConfigApplier::GsConfigApplier(std::string cfg_path, Config live, GsApplyHooks hooks,
                                 std::function<void(const std::string&)> log)
    : LiveApplyEngine(std::move(cfg_path), std::move(live), gs_spec(std::move(hooks)),
                      std::move(log)) {}

}  // namespace maburgs
