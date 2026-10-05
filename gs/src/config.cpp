#include "config.h"
#include <arpa/inet.h>

#include <fstream>
#include <stdexcept>

#include "mabur/ht40.h"
#include "mabur/toml.h"
#include "nhm_busy.h"

namespace maburgs {
namespace {

namespace toml = mabur::toml;   // maburgs/maburplay are not inside mabur
using mabur::toml::Value;

// Set for the duration of load_config; collects keys that fell back to their
// struct default. Not reentrant, which is fine: it is called once at boot.
std::vector<std::string>* g_defaulted = nullptr;

std::string to_text(bool v) { return v ? "true" : "false"; }
std::string to_text(const std::string& v) { return v; }
template <typename T>
std::string to_text(const T& v) { return std::to_string(v); }

void note_default(const std::string& prefix, const char* key,
                  const std::string& value) {
  if (g_defaulted == nullptr) return;
  g_defaulted->push_back((prefix.empty() ? std::string(key)
                                         : prefix + "." + key) + "=" + value);
}

std::string g_file;      // set by load_config, "" outside it
// Line of the LAST key read, not necessarily the key currently being
// validated: a cross-key check (e.g. "up_util >= down_util") runs after both
// keys have been read, so it reports whichever key assign_if_present touched
// last, not necessarily the one actually at fault. Known, accepted
// limitation (see drone/src/config.cpp for the full rationale).
int g_line = 0;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  std::string where = "config: ";
  if (!g_file.empty() && g_line > 0)
    where += g_file + ":" + std::to_string(g_line) + ": ";
  throw std::runtime_error(where + field + ": " + why);
}

void check_keys(const Value& o, const std::string& where,
                std::initializer_list<const char*> allowed) {
  for (auto it = o.begin(); it != o.end(); ++it) {
    const std::string& k = it.key();
    bool ok = false;
    for (const char* a : allowed)
      if (k == a) { ok = true; break; }
    if (!ok) fail(where.empty() ? k : where + "." + k, "unknown key");
  }
}

// int64_t, not long: long is 32-bit on wasm32 (the web GS build).
int64_t get_int(const Value& o, const char* key, int64_t dflt, int64_t lo, int64_t hi,
                const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, to_text(dflt)); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_number_integer()) fail(where + "." + key, "not an integer");
  const int64_t v = o[key].get<int64_t>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

double get_num(const Value& o, const char* key, double dflt, double lo,
               double hi, const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, to_text(dflt)); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_number()) fail(where + "." + key, "not a number");
  const double v = o[key].get<double>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

std::string get_str(const Value& o, const char* key, const std::string& dflt,
                    const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, dflt); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_string()) fail(where + "." + key, "not a string");
  return o[key].get<std::string>();
}

// New: bools went through inline `if (o.contains(...))` blocks, which meant a
// missing bool could not be reported. Same shape as the others now.
bool get_bool(const Value& o, const char* key, bool dflt,
              const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, to_text(dflt)); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_boolean()) fail(where + "." + key, "not a boolean");
  return o[key].get<bool>();
}

// Overlay merge (spec 2026-09-27-web-ui §3.2): tables recurse, anything else
// replaces -- so an overlay [[link.ladder]] replaces the file's ladder whole.
void merge_overlay(Value& base, const Value& ov) {
  for (const auto& [k, v] : ov.items()) {
    Value* b = base.find(k);
    if (b && b->is_object() && v.is_object()) merge_overlay(*b, v);
    else if (b) *b = v;
    else base.set(k, v);
  }
}
}  // namespace

std::array<mabur::UepLayerCfg, 2> Config::uep_layers() const {
  std::array<mabur::UepLayerCfg, 2> out{};
  for (int s = 0; s < 2; ++s) {
    // window is TX-side; 128 here only feeds SwDecoder's auto-horizon
    // default, which the explicit seq_horizon (below) overrides. overhead
    // is decode-inert (SwDecoder never reads SwConfig::overhead -- see
    // common/include/mabur/uep_decoder.h), so 0.5 is a placeholder, not a
    // tuning knob; the real (literal, actual-air) overhead lives on the
    // drone's encoder side and on link.ladder_cfg for RC accounting.
    out[static_cast<size_t>(s)].fec = mabur::SwConfig{
        fec.symbol_size[static_cast<size_t>(s)], 128, 0.5};
    out[static_cast<size_t>(s)].blocks_per_body = 4;  // unused on decode
  }
  return out;
}

std::optional<ConfigIssue> radio_width_issue(uint8_t channel, int width) {
  if (width != 20 && width != 40)
    return ConfigIssue{"radio.width", "must be 20 or 40 (HT20 / HT40)"};
  if (width == 40 && mabur::ht40_offset(channel) == 0)
    return ConfigIssue{"radio.width", "40 MHz needs a standard 5 GHz pair and channel " +
                                          std::to_string(static_cast<int>(channel)) +
                                          " has none (common/include/mabur/ht40.h)"};
  return std::nullopt;
}

std::optional<ConfigIssue> link_width_issue(const LinkCfg& link, int width) {
  for (std::size_t i = 0; i < link.ladder_cfg.ladder.size(); ++i)
    if (link.ladder_cfg.ladder[i].bw == 40 && width != 40)
      return ConfigIssue{"link.ladder[" + std::to_string(i) + "].bw",
                         "40 MHz rung but radio.width is 20: the GS could not receive it"};
  if (link.static_bw == 40 && width != 40)
    return ConfigIssue{"link.static_bw", "40 MHz pin but radio.width is 20"};
  return std::nullopt;
}

Config load_config(const std::string& path, std::vector<std::string>* defaulted,
                   const std::string& overlay_path) {
  Value j;
  try {
    j = toml::parse_toml_file(path);
    if (!overlay_path.empty()) merge_overlay(j, toml::parse_toml_file(overlay_path));
  } catch (const toml::Error& e) {
    throw std::runtime_error(std::string("config: ") + e.what());
  }
  g_defaulted = defaulted;
  g_file = path;
  struct Clear {
    ~Clear() { g_defaulted = nullptr; g_file.clear(); g_line = 0; }
  } clear_on_exit;

  check_keys(j, "", {"radio", "fec", "link", "video", "msp", "stats", "au_ring",
                     "debug_log", "hop"});
  // Same reason as the drone's: a missing section visits none of its keys.
  // Kept in the exact order of the check_keys list above -- if they drift a
  // section goes silently unreported.
  for (const char* sec : {"radio", "fec", "link", "video", "msp", "stats",
                          "au_ring", "debug_log"})
    if (!j.contains(sec)) note_default("", sec, "(section absent)");
  Config c;

  // Set when "radio" is present but "cards" is absent from it: that is the
  // auto-scan case, reported below once the flag is settled.
  bool radio_cards_absent = false;
  if (j.contains("radio")) {
    const Value& r = j["radio"];
    check_keys(r, "radio", {"channel", "channels", "width", "cards", "tx_card", "scan", "relays"});
    c.radio.width = static_cast<uint8_t>(get_int(r, "width", 20, 20, 40, "radio"));
    if (r.contains("channels")) {
      g_line = r["channels"].line();
      if (!r["channels"].is_array()) fail("radio.channels", "not an array");
      c.radio.channels.clear();
      for (const Value& v : r["channels"]) {
        if (!v.is_number_integer()) fail("radio.channels", "not an integer");
        const long ch = v.get<int64_t>();
        if (ch < 1 || ch > 177) fail("radio.channels", "must be in [1,177]");
        c.radio.channels.push_back(static_cast<uint8_t>(ch));
      }
    } else {
      note_default("radio", "channels", "[40, 64, 112, 144]");
    }
    if (auto e = mabur::channel_set_issue(c.radio.channels, c.radio.width, "radio.channels"))
      fail(e->field, e->why);
    for (uint8_t ch : c.radio.channels)
      if (auto e = radio_width_issue(ch, c.radio.width)) fail(e->field, e->why);
    if (r.contains("channel")) {
      g_line = r["channel"].line();
      const Value& cv = r["channel"];
      if (cv.is_string()) {
        if (cv.get<std::string>() != "auto") fail("radio.channel", "must be \"auto\" or a member of radio.channels");
        c.radio.pin.reset();
      } else if (cv.is_number_integer()) {
        const int64_t ch = cv.get<int64_t>();
        // Range first: the uint8_t cast wraps (296 -> 40, a member).
        if (ch < 1 || ch > 177) fail("radio.channel", std::to_string(ch) + " is not a channel in [1,177]");
        if (!mabur::channel_set_member(c.radio.channels, static_cast<uint8_t>(ch)))
          fail("radio.channel", std::to_string(ch) + " is not a member of radio.channels");
        c.radio.pin = static_cast<uint8_t>(ch);
      } else {
        fail("radio.channel", "must be \"auto\" or a member of radio.channels");
      }
    } else {
      note_default("radio", "channel", "auto");
    }
    c.radio.tx_card = static_cast<int>(get_int(r, "tx_card", -1, -1, 15, "radio"));
    if (r.contains("relays")) {
      g_line = r["relays"].line();
      if (!r["relays"].is_array()) fail("radio.relays", "not an array");
      int i = 0;
      for (const Value& v : r["relays"]) {
        const std::string where = "radio.relays[" + std::to_string(i++) + "]";
        if (v.line() > 0) g_line = v.line();
        if (!v.is_string()) fail(where, "not a string");
        const std::string s = v.get<std::string>();
        const auto colon = s.rfind(':');
        if (colon == std::string::npos || colon == 0 || colon + 1 >= s.size())
          fail(where, "must be ipv4:port");
        // Numeric only: open_udp_transport() runs on the core thread on
        // every 2 s reopen, and a hostname there would block video on a
        // dead resolver.
        in_addr a4{};
        if (inet_pton(AF_INET, s.substr(0, colon).c_str(), &a4) != 1)
          fail(where, "host must be a dotted IPv4 address (the UDP transport resolves nothing)");
        const std::string port = s.substr(colon + 1);
        if (port.find_first_not_of("0123456789") != std::string::npos ||
            port.size() > 5)
          fail(where, "port is not a number");
        const long p = std::stol(port);
        if (p < 1 || p > 65535) fail(where, "port must be in [1,65535]");
        for (const auto& prev : c.radio.relays)
          if (prev == s) fail(where, "duplicate relay address");
        c.radio.relays.push_back(s);
      }
    } else {
      note_default("radio", "relays", "(none)");
    }
    if (r.contains("cards")) {
      if (!r["cards"].is_array() || r["cards"].empty())
        fail("radio.cards", "must be a non-empty array");
      c.radio.cards.clear();
      int i = 0;
      for (const Value& cj : r["cards"]) {
        const std::string where = "radio.cards[" + std::to_string(i++) + "]";
        check_keys(cj, where, {"usb_vid", "usb_pid", "index"});
        CardCfg card;
        card.usb_vid = static_cast<uint16_t>(get_int(cj, "usb_vid", 0x0bda, 0, 0xFFFF, where));
        card.usb_pid = static_cast<uint16_t>(get_int(cj, "usb_pid", 0, 0, 0xFFFF, where));
        card.index = static_cast<int>(get_int(cj, "index", 0, 0, 15, where));
        c.radio.cards.push_back(card);
      }
    } else {
      radio_cards_absent = true;
    }
    if (r.contains("scan")) {
      const Value& s = r["scan"];
      check_keys(s, "radio.scan",
                 {"dwell_ms", "settle_ms", "min_rounds", "search_ms", "op_window_ms",
                  "search_after_ms", "pick_margin", "one_card_ms", "max_ms", "busy_dbm", "blocked_pct"});
      ScanCfg& sc = c.radio.scan;
      sc.dwell_ms = static_cast<int>(get_int(s, "dwell_ms", 250, 50, 10000, "radio.scan"));
      sc.settle_ms = static_cast<int>(get_int(s, "settle_ms", 30, 0, 1000, "radio.scan"));
      sc.min_rounds = static_cast<int>(get_int(s, "min_rounds", 3, 1, 100, "radio.scan"));
      sc.search_ms = static_cast<int>(get_int(s, "search_ms", 100, 40, 2000, "radio.scan"));
      sc.op_window_ms = static_cast<int>(get_int(s, "op_window_ms", 300, 40, 10000, "radio.scan"));
      sc.search_after_ms = static_cast<int>(get_int(s, "search_after_ms", 5000, 0, 600000, "radio.scan"));
      sc.pick_margin = static_cast<int>(get_int(s, "pick_margin", 20, 0, 100000, "radio.scan"));
      sc.one_card_ms = static_cast<int>(get_int(s, "one_card_ms", 5000, 0, 60000, "radio.scan"));
      sc.max_ms = static_cast<int>(get_int(s, "max_ms", 30000, 1000, 600000, "radio.scan"));
      sc.busy.busy_dbm = static_cast<int>(get_int(s, "busy_dbm", -83, -104, -70, "radio.scan"));
      if (!maburgs::busy_dbm_is_edge(sc.busy.busy_dbm))
        fail("radio.scan.busy_dbm",
             "must be an NHM bucket edge: -104 -101 -98 -95 -92 -89 -86 -83 -80 -75 -70");
      sc.busy.blocked_pct = get_num(s, "blocked_pct", 50.0, 1.0, 100.0, "radio.scan");
    } else {
      note_default("radio", "scan", "(section absent)");
    }
  }
  // No list -> auto-scan (card_scan.h fills the list from the bus at
  // startup). A list -> pin exactly those, no probing. There is no longer a
  // silent "one default card": a GS with two cards and no config used to
  // receive on one of them.
  c.radio.auto_scan = c.radio.cards.empty();
  if (radio_cards_absent) note_default("radio", "cards", "(auto-scan)");
  // Only an explicit list is a fact at load time. Under auto-scan the count
  // is hardware, discovered after this returns; main.cpp warns and falls
  // back to auto-select when the scan finds fewer cards than the pin.
  // Relays count: they follow the explicit cards (card k+n = relays[n]).
  if (!c.radio.auto_scan &&
      c.radio.tx_card >= static_cast<int>(c.radio.cards.size() + c.radio.relays.size()))
    fail("radio.tx_card", "no such card");

  if (j.contains("hop")) {
    const Value& h = j["hop"];
    check_keys(h, "hop", {"window_ms", "persist", "dwell_observe_ms",
                          "dwell_period_ms", "rank_visits", "rank_max_age_ms", "confirm_ms", "confirm_extend_ms", "verify_ms",
                          "cooldown_ms", "max_hops_per_min", "backoff_ms", "one_card_repeats", "relay_burst_period_ms", "verdict"});
    HopCfg& hc = c.hop;
    hc.window_ms = (int)get_int(h, "window_ms", 150, 50, 2000, "hop");
    hc.persist = (int)get_int(h, "persist", 2, 1, 3, "hop");
    hc.dwell_observe_ms = (int)get_int(h, "dwell_observe_ms", 5, 1, 250, "hop");
    hc.dwell_period_ms = (int)get_int(h, "dwell_period_ms", 333, 20, 60000, "hop");
    hc.rank_visits = (int)get_int(h, "rank_visits", 5, 1, 100, "hop");
    hc.rank_max_age_ms = (int)get_int(h, "rank_max_age_ms", 10000, 1000, 600000, "hop");
    hc.confirm_ms = (int)get_int(h, "confirm_ms", 500, 100, 5000, "hop");
    hc.confirm_extend_ms = (int)get_int(h, "confirm_extend_ms", 3000, 0, 30000, "hop");
    hc.verify_ms = (int)get_int(h, "verify_ms", 1000, 200, 10000, "hop");
    hc.cooldown_ms = (int)get_int(h, "cooldown_ms", 2000, 0, 60000, "hop");
    hc.max_hops_per_min = (int)get_int(h, "max_hops_per_min", 4, 1, 60, "hop");
    hc.backoff_ms = (int)get_int(h, "backoff_ms", 30000, 1000, 600000, "hop");
    hc.one_card_repeats = (int)get_int(h, "one_card_repeats", 5, 1, 50, "hop");
    hc.relay_burst_period_ms = (int)get_int(h, "relay_burst_period_ms", 1000, 500, 60000, "hop");
    if (h.contains("verdict")) {
      const Value& v = h["verdict"];
      check_keys(v, "hop.verdict", {"loss_pct", "recovered_x", "weak_rssi_dbm", "weak_snr_db",
                                    "fading_drop_db", "foreign_pps", "fa_pps",
                                    "recovered_min", "starved_frac"});
      HopVerdictCfg& vc = hc.verdict;
      vc.loss_pct = get_num(v, "loss_pct", 3.0, 0.1, 100.0, "hop.verdict");
      vc.recovered_x = get_num(v, "recovered_x", 3.0, 1.0, 100.0, "hop.verdict");
      vc.recovered_min = (int)get_int(v, "recovered_min", 8, 0, 1000, "hop.verdict");
      vc.weak_rssi_dbm = (int)get_int(v, "weak_rssi_dbm", -78, -110, -20, "hop.verdict");
      vc.weak_snr_db = (int)get_int(v, "weak_snr_db", 12, 0, 40, "hop.verdict");
      vc.fading_drop_db = (int)get_int(v, "fading_drop_db", 6, 1, 40, "hop.verdict");
      vc.foreign_pps = (int)get_int(v, "foreign_pps", 50, 1, 100000, "hop.verdict");
      vc.fa_pps = (int)get_int(v, "fa_pps", 100, 1, 100000, "hop.verdict");
      vc.starved_frac = get_num(v, "starved_frac", 0.25, 0.0, 1.0, "hop.verdict");
    } else {
      note_default("hop", "verdict", "(section absent)");
    }
  } else {
    note_default("", "hop", "(section absent)");
  }

  if (j.contains("fec")) {
    const Value& r = j["fec"];
    check_keys(r, "fec", {"symbol_size", "seq_horizon"});
    if (r.contains("symbol_size")) {
      auto& s = r.at("symbol_size");
      if (s.is_array()) {
        if (s.size() != 2) fail("fec.symbol_size", "array must have 2 ints");
        try {
          for (size_t i = 0; i < 2; ++i)
            c.fec.symbol_size[i] = static_cast<int>(s.at(i).get<int64_t>());
        } catch (const toml::Error&) {
          fail("fec.symbol_size", "wrong type");
        }
      } else if (s.is_number_integer()) {
        c.fec.symbol_size.fill(static_cast<int>(s.get<int64_t>()));
      } else {
        fail("fec.symbol_size", "not an integer");
      }
      for (int v : c.fec.symbol_size)
        if (v < 32 || v > 1500) fail("fec.symbol_size", "must be in [32,1500]");
    } else {
      note_default("fec", "symbol_size", to_text(c.fec.symbol_size[0]));
    }
    c.fec.seq_horizon = static_cast<int>(get_int(r, "seq_horizon", 512, 16, 65536, "fec"));
  }

  // Raw [link] key overlay (spec 2026-10-01 link-pairing §2); empty when
  // absent or when only key_file was given. Resolved, with key_file, in the
  // unconditional block below (same reasoning as the sentinel resolution
  // further down: a config with no [link] table at all must still resolve
  // to the compiled-in default).
  std::string link_inline_key;
  if (j.contains("link")) {
    const Value& r = j["link"];
    check_keys(r, "link",
               {"feedback_ms", "beacon_keepalive_ms",
                "static_mcs", "static_overhead_base", "static_overhead_enh",
                "static_bw",
                "ladder", "max_mcs", "down_util", "up_util", "confirm_ms",
                "clean_ms", "probation_ms", "penalty_base_ms", "penalty_max_ms",
                "hold_after_down_ms", "min_between_changes_ms", "feedback_timeout_ms",
                "starved_confirm_ms", "s3_demote", "s3_down_util",
                "s3_settle_ms", "s3_min_syms",
                "rung_stats", "fade", "probe",
                "rcf_slot_hold_ms", "arrival_guard_syms",
                "nack",
                "key_file", "key"});
    c.link.key_file = get_str(r, "key_file", "/etc/mabur.key", "link");
    // Read by presence: link.key is an optional web-overlay key (spec §2),
    // not a tunable with a meaningful default, so its absence (the normal
    // case -- key_file is the real config surface) must not register as a
    // defaulted key (fix round 1, Task 4 review) -- that would print
    // "link.key=" under every plain boot's "config: N key(s) defaulted"
    // list, right next to the real DEFAULT-key warning, diluting it.
    if (r.contains("key")) link_inline_key = get_str(r, "key", "", "link");
    c.link.feedback_ms = static_cast<int>(get_int(r, "feedback_ms", 100, 20, 5000, "link"));
    c.link.rcf_slot_hold_ms = static_cast<int>(get_int(r, "rcf_slot_hold_ms", 30, 0, 1000, "link"));
    c.link.arrival_guard_syms = static_cast<int>(get_int(r, "arrival_guard_syms", 192, 16, 512, "link"));
    c.link.beacon_keepalive_ms = static_cast<int>(get_int(r, "beacon_keepalive_ms", 1000, 100, 60000, "link"));
    c.link.static_mcs = static_cast<int>(get_int(r, "static_mcs", -1, -1, 7, "link"));
    // Actual-air overhead (airtime-balance-uep): literal, not a scaled cmd
    // value -- old cmd default/range 0.25 [0.10, 1.0] x2 everywhere.
    // Same-rate-fixed-pairs (Task 3): base/enh pair, same default/range.
    c.link.static_overhead_base =
        get_num(r, "static_overhead_base", 0.5, 0.1, 2.0, "link");
    c.link.static_overhead_enh =
        get_num(r, "static_overhead_enh", 0.5, 0.1, 2.0, "link");
    c.link.static_bw = static_cast<int>(get_int(r, "static_bw", 20, 20, 40, "link"));
    if (c.link.static_bw != 20 && c.link.static_bw != 40)
      fail("link.static_bw", "must be 20 or 40");

    // Measured-loss ladder: rungs (c.link.ladder_cfg.ladder already holds the
    // struct default 6-rung ladder; an explicit "ladder" array replaces it
    // wholesale, in order) then the max_mcs feasibility filter, then the
    // change/probation/penalty thresholds.
    if (r.contains("ladder")) {
      if (!r["ladder"].is_array() || r["ladder"].empty())
        fail("link.ladder", "must be a non-empty array");
      if (r["ladder"].size() > 8)
        fail("link.ladder", "must have at most 8 entries");
      std::vector<Rung> parsed;
      int i = 0;
      for (const Value& rj : r["ladder"]) {
        const std::string where = "link.ladder[" + std::to_string(i++) + "]";
        check_keys(rj, where, {"mcs", "bw", "overhead_base", "overhead_enh"});
        Rung rung;
        rung.mcs = static_cast<int>(get_int(rj, "mcs", 0, 0, 7, where));
        // Per-rung width (2026-09-24 40 MHz top rungs): required, no
        // default -- a rung silently airing at 20 when the author meant 40
        // (or vice versa) is exactly the wrong-width mistake this is meant
        // to catch.
        if (!rj.contains("bw")) fail(where + ".bw", "required: 20 or 40 (per-rung width, 2026-09-24)");
        rung.bw = static_cast<int>(get_int(rj, "bw", 20, 20, 40, where));
        if (rung.bw != 20 && rung.bw != 40) fail(where + ".bw", "must be 20 or 40");
        // Actual-air overhead (airtime-balance-uep): literal, not a scaled
        // cmd value -- old cmd default/range 1.0 [0.05, 1.0] x2 everywhere.
        // Same-rate-fixed-pairs (Task 3): base/enh pair, same default/range.
        rung.overhead_base = get_num(rj, "overhead_base", 2.0, 0.1, 2.0, where);
        rung.overhead_enh = get_num(rj, "overhead_enh", 2.0, 0.1, 2.0, where);
        // Operator rule: the base layer every frame depends on never gets
        // less protection than the droppable enhance layer.
        if (rung.overhead_base < rung.overhead_enh)
          fail(where + ".overhead_base", "must be >= overhead_enh");
        parsed.push_back(rung);
      }
      c.link.ladder_cfg.ladder = parsed;
    } else {
      note_default("link", "ladder",
                   "(" + std::to_string(c.link.ladder_cfg.ladder.size()) +
                       " default rungs)");
    }
    const long max_mcs = get_int(r, "max_mcs", 7, 0, 7, "link");
    {
      std::vector<Rung> effective;
      for (const Rung& rung : c.link.ladder_cfg.ladder)
        if (rung.mcs <= max_mcs) effective.push_back(rung);
      if (effective.empty()) fail("link.ladder", "empty after max_mcs filter");
      c.link.ladder_cfg.ladder = effective;
    }
    c.link.ladder_cfg.down_util = get_num(r, "down_util", 0.6, 0.0, 1.0, "link");
    c.link.ladder_cfg.up_util = get_num(r, "up_util", 0.15, 0.0, 1.0, "link");
    if (c.link.ladder_cfg.up_util <= 0.0)
      fail("link.up_util", "must be > 0");
    if (c.link.ladder_cfg.up_util >= c.link.ladder_cfg.down_util)
      fail("link.up_util", "must be < down_util");
    c.link.ladder_cfg.confirm_ms =
        static_cast<int>(get_int(r, "confirm_ms", 250, 0, 600000, "link"));
    c.link.ladder_cfg.clean_ms =
        static_cast<int>(get_int(r, "clean_ms", 5000, 0, 600000, "link"));
    c.link.ladder_cfg.probation_ms =
        static_cast<int>(get_int(r, "probation_ms", 3000, 0, 600000, "link"));
    c.link.ladder_cfg.penalty_base_ms =
        static_cast<int>(get_int(r, "penalty_base_ms", 10000, 0, 600000, "link"));
    c.link.ladder_cfg.penalty_max_ms =
        static_cast<int>(get_int(r, "penalty_max_ms", 60000, 0, 600000, "link"));
    c.link.ladder_cfg.hold_after_down_ms =
        static_cast<int>(get_int(r, "hold_after_down_ms", 4000, 0, 600000, "link"));
    c.link.ladder_cfg.min_between_changes_ms =
        static_cast<int>(get_int(r, "min_between_changes_ms", 150, 0, 600000, "link"));
    c.link.ladder_cfg.feedback_timeout_ms =
        static_cast<int>(get_int(r, "feedback_timeout_ms", 1000, 0, 600000, "link"));
    c.link.ladder_cfg.starved_confirm_ms =
        static_cast<int>(get_int(r, "starved_confirm_ms", 300, 0, 600000, "link"));

    // s3 steady-state demote tuning (LadderCfg). s3_down_util keeps its
    // struct default (-1 sentinel) when absent from JSON and is resolved to
    // down_util below, AFTER down_util has parsed above.
    auto& lc = c.link.ladder_cfg;
    lc.s3_min_syms = static_cast<int>(get_int(r, "s3_min_syms", 50, 1, 100000, "link"));
    // Probe stream gate (spec 2026-09-04 §5). Optional block with live
    // defaults; the pre-2026-09-04 flat probe_* keys are gone and fail boot.
    if (r.contains("probe")) {
      const Value& pj = r["probe"];
      check_keys(pj, "link.probe", {"enable", "rung_offset", "clean_bodies", "max_util",
                                    "min_syms", "silence_ms", "pin_mcs"});
      auto& pc = lc.probe;
      pc.enable = get_bool(pj, "enable", pc.enable, "link.probe");
      pc.rung_offset = static_cast<int>(get_int(pj, "rung_offset", 1, 1, 7, "link.probe"));
      // Streak in expected probe BODIES (probe per AU, 2026-09-16), so the
      // gate's confidence is fixed by config alone, not by fps or the
      // layer split. clean_ms is gone: it fails boot like any unknown key.
      pc.clean_bodies = static_cast<int>(get_int(pj, "clean_bodies", 90, 10, 100000, "link.probe"));
      if (pj.contains("max_util"))
        pc.max_util = get_num(pj, "max_util", 0.35, 0.01, 2.0, "link.probe");
      else
        note_default("link.probe", "max_util", "(defaults to link.down_util)");
      pc.min_syms = static_cast<int>(get_int(pj, "min_syms", 16, 4, 100000, "link.probe"));
      pc.silence_ms = static_cast<int>(get_int(pj, "silence_ms", 500, 100, 10000, "link.probe"));
      pc.pin_mcs = static_cast<int>(get_int(pj, "pin_mcs", -1, -1, 7, "link.probe"));
    } else {
      // Whole sub-table absent: one line, same style as a missing top-level
      // section, rather than seven separate per-key lines the operator would
      // have to mentally group back together.
      note_default("link", "probe", "(section absent)");
    }
    // get_bool reports its own default when absent (no wrapping "section"
    // to collapse -- s3_demote is a single scalar, not a sub-table).
    lc.s3_demote = get_bool(r, "s3_demote", lc.s3_demote, "link");
    if (r.contains("s3_down_util"))
      lc.s3_down_util = get_num(r, "s3_down_util", 0.35, 0.01, 2.0, "link");
    else
      note_default("link", "s3_down_util", "(defaults to link.down_util)");
    lc.s3_settle_ms = static_cast<int>(get_int(r, "s3_settle_ms", 300, 0, 5000, "link"));

    // Fade-aware demotes (spec 2026-08-14 fade-demote). Config surface only:
    // nothing in this task consumes lc.fade yet.
    if (r.contains("fade")) {
      const Value& fj = r["fade"];
      check_keys(fj, "link.fade",
                 {"cascade", "predict", "hold_ms", "confirm_ms", "rssi_db",
                  "snr_db", "trigger_ms", "min_rung"});
      auto& fc = lc.fade;
      fc.cascade = get_bool(fj, "cascade", fc.cascade, "link.fade");
      fc.predict = get_bool(fj, "predict", fc.predict, "link.fade");
      fc.hold_ms = static_cast<int>(get_int(fj, "hold_ms", 2500, 0, 60000, "link.fade"));
      fc.confirm_ms = static_cast<int>(get_int(fj, "confirm_ms", 100, 20, 1000, "link.fade"));
      fc.rssi_db = get_num(fj, "rssi_db", 8.0, 0.5, 40.0, "link.fade");
      fc.snr_db = get_num(fj, "snr_db", 4.0, 0.5, 40.0, "link.fade");
      fc.trigger_ms = static_cast<int>(get_int(fj, "trigger_ms", 300, 50, 5000, "link.fade"));
      fc.min_rung = static_cast<int>(get_int(fj, "min_rung", 2, 0, 15, "link.fade"));
    } else {
      note_default("link", "fade", "(section absent)");
    }
    // Software NACK (spec 2026-10-05 fec-nack §7). settle is adaptive (no key).
    if (r.contains("nack")) {
      const Value& nj = r["nack"];
      check_keys(nj, "link.nack", {"enable", "lookback", "repeat_ms", "max_tries"});
      auto& nc = c.link.nack;
      nc.enable = get_bool(nj, "enable", nc.enable, "link.nack");
      nc.lookback = static_cast<int>(get_int(nj, "lookback", 256, 8, 4096, "link.nack"));
      nc.repeat_ms = static_cast<int>(get_int(nj, "repeat_ms", 16, 1, 1000, "link.nack"));
      nc.max_tries = static_cast<int>(get_int(nj, "max_tries", 2, 0, 16, "link.nack"));  // 0 = observe only
    }  // absent = off; not a defaulted key (bench knob)
    // Cross-section: [fec] is parsed above. A lookback at or past the
    // decoder's seq horizon would ask about seqs the decoder has already
    // forgotten (source_state reads them as unknown forever).
    if (c.link.nack.lookback >= c.fec.seq_horizon)
      fail("link.nack.lookback", "must be < fec.seq_horizon (" +
                                     std::to_string(c.fec.seq_horizon) + ")");

    if (r.contains("rung_stats")) {
      const Value& rs = r["rung_stats"];
      check_keys(rs, "link.rung_stats", {"half_life_samples"});
      c.link.ladder_cfg.rung_stats.half_life_samples = static_cast<int>(
          get_int(rs, "half_life_samples", 600, 10, 100000, "link.rung_stats"));
    } else {
      note_default("link", "rung_stats", "(section absent)");
    }
  }
  // Sentinel resolution: an absent link.probe.max_util/link.s3_down_util
  // tracks down_util. Runs unconditionally (not just when "link" was
  // present) so a wholly-absent config still resolves to down_util's
  // struct default.
  if (c.link.ladder_cfg.probe.max_util < 0)
    c.link.ladder_cfg.probe.max_util = c.link.ladder_cfg.down_util;
  if (c.link.ladder_cfg.s3_down_util < 0)
    c.link.ladder_cfg.s3_down_util = c.link.ladder_cfg.down_util;

  // Pairing key resolution (spec 2026-10-01 link-pairing §2): an inline
  // [link] key overrides key_file; otherwise the key file is read (missing
  // -> compiled-in default). Runs unconditionally so a config with no
  // [link] table at all still resolves to the default.
  if (!link_inline_key.empty()) {
    auto k = mabur::parse_key_hex(link_inline_key);
    if (!k) fail("link.key", "not 32 hex characters");
    c.link.key = *k;
    c.link.key_is_default = false;
    c.link.key_source = "link.key";
  } else {
    try {
      const auto kl = mabur::load_key_file(c.link.key_file);
      c.link.key = kl.key;
      c.link.key_is_default = kl.is_default;
      c.link.key_source = kl.source;
    } catch (const std::runtime_error& e) {
      fail("link.key_file", e.what());
    }
  }

  // ---- Width cross-checks (2026-09-24, 40 MHz top rungs) -------------------
  // A 40 MHz rung or pin needs the GS tuned 40 -- a 20-tuned receiver cannot
  // hear HT40 at all (docs/bw40.md). Runs unconditionally, after both radio
  // and link sections are settled, and after the max_mcs filter above: a
  // rung filtered out by max_mcs is not checked here, which matches "what
  // will fly".
  if (auto e = link_width_issue(c.link, c.radio.width)) fail(e->field, e->why);
  // ---------------------------------------------------------------------

  if (j.contains("video")) {
    const Value& r = j["video"];
    check_keys(r, "video",
               {"frame_gap_timeout_ms", "frame_gap_timeout_max_ms",
                "frame_lookahead"});
    c.video.frame_gap_timeout_ms = static_cast<int>(
        get_int(r, "frame_gap_timeout_ms", 50, 10, 1000, "video"));
    // Range top 400 keeps the ceiling under FrameStream's 500 ms
    // stall-reset backstop by construction.
    c.video.frame_gap_timeout_max_ms = static_cast<int>(
        get_int(r, "frame_gap_timeout_max_ms", 150, 0, 400, "video"));
    c.video.frame_lookahead = static_cast<int>(
        get_int(r, "frame_lookahead", 8, 2, 64, "video"));
  }

  if (j.contains("msp")) {
    const Value& r = j["msp"];
    check_keys(r, "msp", {"enable", "out", "symbol_size", "window"});
    c.msp.enable = get_bool(r, "enable", c.msp.enable, "msp");
    if (r.contains("out")) {
      const Value& o = r["out"];
      check_keys(o, "msp.out", {"host", "port"});
      c.msp.out_host = get_str(o, "host", "127.0.0.1", "msp.out");
      c.msp.out_port = static_cast<int>(get_int(o, "port", 14560, 1, 65535, "msp.out"));
    } else {
      // Whole sub-table absent: one line, same style as a missing top-level
      // section, rather than two separate per-key lines the operator would
      // have to mentally group back together.
      note_default("msp", "out", "(section absent)");
    }
    c.msp.symbol_size = static_cast<int>(get_int(r, "symbol_size", 1312, 16, 2048, "msp"));
    c.msp.window = static_cast<int>(get_int(r, "window", 16, 2, 255, "msp"));
  }

  if (j.contains("stats")) {
    const Value& r = j["stats"];
    check_keys(r, "stats", {"enable", "host", "port", "interval_ms", "out"});
    c.stats.enable = get_bool(r, "enable", c.stats.enable, "stats");
    c.stats.interval_ms =
        static_cast<int>(get_int(r, "interval_ms", 500, 100, 10000, "stats"));

    const bool has_legacy = r.contains("host") || r.contains("port");
    if (r.contains("out")) {
      // Ambiguity is a boot failure, not a precedence rule: a config with
      // both would silently ignore half of what its author wrote.
      if (has_legacy)
        fail("stats.out", "cannot be combined with stats.host / stats.port");
      if (!r["out"].is_array()) fail("stats.out", "not an array");
      if (r["out"].empty()) fail("stats.out", "must have at least one destination");
      c.stats.out.clear();
      for (const Value& e : r["out"]) {
        if (!e.is_object()) fail("stats.out[]", "not an object");
        check_keys(e, "stats.out[]", {"host", "port"});
        if (!e.contains("port")) fail("stats.out[].port", "missing");
        StatsOut o;
        o.host = get_str(e, "host", "127.0.0.1", "stats.out[]");
        o.port = static_cast<int>(get_int(e, "port", 8300, 1, 65535, "stats.out[]"));
        c.stats.out.push_back(o);
      }
    } else {
      StatsOut o;
      o.host = get_str(r, "host", "127.0.0.1", "stats");
      o.port = static_cast<int>(get_int(r, "port", 8300, 1, 65535, "stats"));
      c.stats.out = {o};
    }
  }

  if (j.contains("au_ring")) {
    const Value& r = j["au_ring"];
    check_keys(r, "au_ring", {"enable", "path", "socket", "slot_kb", "slot_count"});
    c.au_ring.enable = get_bool(r, "enable", c.au_ring.enable, "au_ring");
    c.au_ring.path = get_str(r, "path", "/dev/shm/mabur-au", "au_ring");
    c.au_ring.socket = get_str(r, "socket", "/run/mabur-au.sock", "au_ring");
    c.au_ring.slot_kb =
        static_cast<int>(get_int(r, "slot_kb", 512, 64, 4096, "au_ring"));
    c.au_ring.slot_count =
        static_cast<int>(get_int(r, "slot_count", 16, 4, 256, "au_ring"));
  }

  if (j.contains("debug_log")) {
    const Value& r = j["debug_log"];
    check_keys(r, "debug_log",
               {"enable", "dir", "ctl_period_ms", "rung_period_s"});
    c.debug_log.enable = get_bool(r, "enable", c.debug_log.enable, "debug_log");
    c.debug_log.dir = get_str(r, "dir", "/media/dvr/log", "debug_log");
    c.debug_log.ctl_period_ms = static_cast<int>(
        get_int(r, "ctl_period_ms", 1000, 50, 60000, "debug_log"));
    c.debug_log.rung_period_s = static_cast<int>(
        get_int(r, "rung_period_s", 10, 1, 600, "debug_log"));
  }
  return c;
}

}  // namespace maburgs
