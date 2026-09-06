#include "config.h"

#include <fstream>
#include <stdexcept>

#include "json.hpp"

namespace maburgs {
namespace {
using nlohmann::json;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  throw std::runtime_error("config: " + field + ": " + why);
}

void check_keys(const json& o, const std::string& where,
                std::initializer_list<const char*> allowed) {
  for (auto& [k, v] : o.items()) {
    bool ok = false;
    for (const char* a : allowed)
      if (k == a) { ok = true; break; }
    if (!ok) fail(where + "." + k, "unknown key");
  }
}

long get_int(const json& o, const char* key, long dflt, long lo, long hi,
             const std::string& where) {
  if (!o.contains(key)) return dflt;
  if (!o[key].is_number_integer()) fail(where + "." + key, "not an integer");
  const long v = o[key].get<long>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

double get_num(const json& o, const char* key, double dflt, double lo,
               double hi, const std::string& where) {
  if (!o.contains(key)) return dflt;
  if (!o[key].is_number()) fail(where + "." + key, "not a number");
  const double v = o[key].get<double>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

std::string get_str(const json& o, const char* key, const std::string& dflt,
                    const std::string& where) {
  if (!o.contains(key)) return dflt;
  if (!o[key].is_string()) fail(where + "." + key, "not a string");
  return o[key].get<std::string>();
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

Config load_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) fail(path, "cannot open");
  json j;
  try {
    j = json::parse(f);
  } catch (const std::exception& e) {
    fail(path, std::string("parse error: ") + e.what());
  }
  check_keys(j, "", {"radio", "fec", "link", "video", "msp", "stats", "au_ring",
                     "debug_log"});
  Config c;

  if (j.contains("radio")) {
    const json& r = j["radio"];
    check_keys(r, "radio", {"channel", "width", "cards", "tx_card"});
    c.radio.channel = static_cast<uint8_t>(get_int(r, "channel", 149, 1, 200, "radio"));
    c.radio.width = static_cast<uint8_t>(get_int(r, "width", 20, 20, 80, "radio"));
    c.radio.tx_card = static_cast<int>(get_int(r, "tx_card", -1, -1, 15, "radio"));
    if (r.contains("cards")) {
      if (!r["cards"].is_array() || r["cards"].empty())
        fail("radio.cards", "must be a non-empty array");
      c.radio.cards.clear();
      int i = 0;
      for (const json& cj : r["cards"]) {
        const std::string where = "radio.cards[" + std::to_string(i++) + "]";
        check_keys(cj, where, {"usb_vid", "usb_pid", "index"});
        CardCfg card;
        card.usb_vid = static_cast<uint16_t>(get_int(cj, "usb_vid", 0x0bda, 0, 0xFFFF, where));
        card.usb_pid = static_cast<uint16_t>(get_int(cj, "usb_pid", 0, 0, 0xFFFF, where));
        card.index = static_cast<int>(get_int(cj, "index", 0, 0, 15, where));
        c.radio.cards.push_back(card);
      }
    }
  }
  if (c.radio.cards.empty()) c.radio.cards.push_back(CardCfg{});
  if (c.radio.tx_card >= static_cast<int>(c.radio.cards.size()))
    fail("radio.tx_card", "no such card");

  if (j.contains("fec")) {
    const json& r = j["fec"];
    check_keys(r, "fec", {"symbol_size", "seq_horizon"});
    if (r.contains("symbol_size")) {
      auto& s = r.at("symbol_size");
      if (s.is_array()) {
        if (s.size() != 2) fail("fec.symbol_size", "array must have 2 ints");
        for (size_t i = 0; i < 2; ++i)
          c.fec.symbol_size[i] = static_cast<int>(s.at(i).get<int64_t>());
      } else if (s.is_number_integer()) {
        c.fec.symbol_size.fill(static_cast<int>(s.get<int64_t>()));
      } else {
        fail("fec.symbol_size", "not an integer");
      }
      for (int v : c.fec.symbol_size)
        if (v < 32 || v > 1500) fail("fec.symbol_size", "must be in [32,1500]");
    }
    c.fec.seq_horizon = static_cast<int>(get_int(r, "seq_horizon", 512, 16, 65536, "fec"));
  }

  if (j.contains("link")) {
    const json& r = j["link"];
    check_keys(r, "link",
               {"vtx_id", "feedback_ms", "beacon_keepalive_ms",
                "static_mcs", "static_overhead_base", "static_overhead_enh",
                "ladder", "max_mcs", "down_util", "up_util", "confirm_ms",
                "clean_ms", "probation_ms", "penalty_base_ms", "penalty_max_ms",
                "hold_after_down_ms", "min_between_changes_ms", "feedback_timeout_ms",
                "starved_confirm_ms", "s3_demote", "s3_down_util",
                "s3_settle_ms", "s3_min_syms",
                "rung_stats", "fade", "probe",
                "rcf_slot_hold_ms"});
    c.link.vtx_id = static_cast<uint32_t>(get_int(r, "vtx_id", 1, 0, 0xFFFFFFFFL, "link"));
    c.link.feedback_ms = static_cast<int>(get_int(r, "feedback_ms", 100, 20, 5000, "link"));
    c.link.rcf_slot_hold_ms = static_cast<int>(get_int(r, "rcf_slot_hold_ms", 30, 0, 1000, "link"));
    c.link.beacon_keepalive_ms = static_cast<int>(get_int(r, "beacon_keepalive_ms", 1000, 100, 60000, "link"));
    c.link.static_mcs = static_cast<int>(get_int(r, "static_mcs", -1, -1, 7, "link"));
    // Actual-air overhead (airtime-balance-uep): literal, not a scaled cmd
    // value -- old cmd default/range 0.25 [0.10, 1.0] x2 everywhere.
    // Same-rate-fixed-pairs (Task 3): base/enh pair, same default/range.
    c.link.static_overhead_base =
        get_num(r, "static_overhead_base", 0.5, 0.1, 2.0, "link");
    c.link.static_overhead_enh =
        get_num(r, "static_overhead_enh", 0.5, 0.1, 2.0, "link");

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
      for (const json& rj : r["ladder"]) {
        const std::string where = "link.ladder[" + std::to_string(i++) + "]";
        check_keys(rj, where, {"mcs", "overhead_base", "overhead_enh"});
        Rung rung;
        rung.mcs = static_cast<int>(get_int(rj, "mcs", 0, 0, 7, where));
        // Actual-air overhead (airtime-balance-uep): literal, not a scaled
        // cmd value -- old cmd default/range 1.0 [0.05, 1.0] x2 everywhere.
        // Same-rate-fixed-pairs (Task 3): base/enh pair, same default/range.
        rung.overhead_base = get_num(rj, "overhead_base", 2.0, 0.1, 2.0, where);
        rung.overhead_enh = get_num(rj, "overhead_enh", 2.0, 0.1, 2.0, where);
        parsed.push_back(rung);
      }
      c.link.ladder_cfg.ladder = parsed;
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
      const json& pj = r["probe"];
      check_keys(pj, "link.probe", {"enable", "rung_offset", "clean_ms", "max_util",
                                    "min_syms", "silence_ms", "pin_mcs"});
      auto& pc = lc.probe;
      if (pj.contains("enable")) {
        if (!pj["enable"].is_boolean()) fail("link.probe.enable", "not a boolean");
        pc.enable = pj["enable"].get<bool>();
      }
      pc.rung_offset = static_cast<int>(get_int(pj, "rung_offset", 1, 1, 7, "link.probe"));
      pc.clean_ms = static_cast<int>(get_int(pj, "clean_ms", 2000, 100, 60000, "link.probe"));
      if (pj.contains("max_util"))
        pc.max_util = get_num(pj, "max_util", 0.35, 0.01, 2.0, "link.probe");
      pc.min_syms = static_cast<int>(get_int(pj, "min_syms", 40, 4, 100000, "link.probe"));
      pc.silence_ms = static_cast<int>(get_int(pj, "silence_ms", 500, 100, 10000, "link.probe"));
      pc.pin_mcs = static_cast<int>(get_int(pj, "pin_mcs", -1, -1, 7, "link.probe"));
    }
    if (r.contains("s3_demote")) {
      if (!r["s3_demote"].is_boolean()) fail("link.s3_demote", "not a boolean");
      lc.s3_demote = r["s3_demote"].get<bool>();
    }
    if (r.contains("s3_down_util"))
      lc.s3_down_util = get_num(r, "s3_down_util", 0.35, 0.01, 2.0, "link");
    lc.s3_settle_ms = static_cast<int>(get_int(r, "s3_settle_ms", 300, 0, 5000, "link"));

    // Fade-aware demotes (spec 2026-08-14 fade-demote). Config surface only:
    // nothing in this task consumes lc.fade yet.
    if (r.contains("fade")) {
      const json& fj = r["fade"];
      check_keys(fj, "link.fade",
                 {"cascade", "predict", "hold_ms", "confirm_ms", "rssi_db",
                  "snr_db", "trigger_ms", "min_rung"});
      auto& fc = lc.fade;
      if (fj.contains("cascade")) {
        if (!fj["cascade"].is_boolean()) fail("link.fade.cascade", "not a boolean");
        fc.cascade = fj["cascade"].get<bool>();
      }
      if (fj.contains("predict")) {
        if (!fj["predict"].is_boolean()) fail("link.fade.predict", "not a boolean");
        fc.predict = fj["predict"].get<bool>();
      }
      fc.hold_ms = static_cast<int>(get_int(fj, "hold_ms", 2500, 0, 60000, "link.fade"));
      fc.confirm_ms = static_cast<int>(get_int(fj, "confirm_ms", 100, 20, 1000, "link.fade"));
      fc.rssi_db = get_num(fj, "rssi_db", 8.0, 0.5, 40.0, "link.fade");
      fc.snr_db = get_num(fj, "snr_db", 4.0, 0.5, 40.0, "link.fade");
      fc.trigger_ms = static_cast<int>(get_int(fj, "trigger_ms", 300, 50, 5000, "link.fade"));
      fc.min_rung = static_cast<int>(get_int(fj, "min_rung", 2, 0, 15, "link.fade"));
    }

    if (r.contains("rung_stats")) {
      const json& rs = r["rung_stats"];
      check_keys(rs, "link.rung_stats", {"half_life_samples"});
      c.link.ladder_cfg.rung_stats.half_life_samples = static_cast<int>(
          get_int(rs, "half_life_samples", 600, 10, 100000, "link.rung_stats"));
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

  if (j.contains("video")) {
    const json& r = j["video"];
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
    const json& r = j["msp"];
    check_keys(r, "msp", {"enable", "out", "symbol_size", "window"});
    if (r.contains("enable")) {
      if (!r["enable"].is_boolean()) fail("msp.enable", "not a boolean");
      c.msp.enable = r["enable"].get<bool>();
    }
    if (r.contains("out")) {
      const json& o = r["out"];
      check_keys(o, "msp.out", {"host", "port"});
      c.msp.out_host = get_str(o, "host", "127.0.0.1", "msp.out");
      c.msp.out_port = static_cast<int>(get_int(o, "port", 14560, 1, 65535, "msp.out"));
    }
    c.msp.symbol_size = static_cast<int>(get_int(r, "symbol_size", 1312, 16, 2048, "msp"));
    c.msp.window = static_cast<int>(get_int(r, "window", 16, 2, 255, "msp"));
  }

  if (j.contains("stats")) {
    const json& r = j["stats"];
    check_keys(r, "stats", {"enable", "host", "port", "interval_ms", "out"});
    if (r.contains("enable")) {
      if (!r["enable"].is_boolean()) fail("stats.enable", "not a boolean");
      c.stats.enable = r["enable"].get<bool>();
    }
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
      for (const json& e : r["out"]) {
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
    const json& r = j["au_ring"];
    check_keys(r, "au_ring", {"enable", "path", "socket", "slot_kb", "slot_count"});
    if (r.contains("enable")) {
      if (!r["enable"].is_boolean()) fail("au_ring.enable", "not a boolean");
      c.au_ring.enable = r["enable"].get<bool>();
    }
    c.au_ring.path = get_str(r, "path", "/dev/shm/mabur-au", "au_ring");
    c.au_ring.socket = get_str(r, "socket", "/run/mabur-au.sock", "au_ring");
    c.au_ring.slot_kb =
        static_cast<int>(get_int(r, "slot_kb", 512, 64, 4096, "au_ring"));
    c.au_ring.slot_count =
        static_cast<int>(get_int(r, "slot_count", 16, 4, 256, "au_ring"));
  }

  if (j.contains("debug_log")) {
    const json& r = j["debug_log"];
    check_keys(r, "debug_log",
               {"enable", "dir", "ctl_period_ms", "rung_period_s"});
    if (r.contains("enable")) {
      if (!r["enable"].is_boolean()) fail("debug_log.enable", "not a boolean");
      c.debug_log.enable = r["enable"].get<bool>();
    }
    c.debug_log.dir = get_str(r, "dir", "/media/dvr/log", "debug_log");
    c.debug_log.ctl_period_ms = static_cast<int>(
        get_int(r, "ctl_period_ms", 1000, 50, 60000, "debug_log"));
    c.debug_log.rung_period_s = static_cast<int>(
        get_int(r, "rung_period_s", 10, 1, 600, "debug_log"));
  }
  return c;
}

}  // namespace maburgs
