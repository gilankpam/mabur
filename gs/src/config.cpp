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

std::array<mabur::UepLayerCfg, 4> Config::uep_layers() const {
  std::array<mabur::UepLayerCfg, 4> out{};
  for (int s = 0; s < 4; ++s) {
    // window is TX-side; 128 here only feeds SwDecoder's auto-horizon
    // default, which the explicit seq_horizon (below) overrides.
    out[static_cast<size_t>(s)].fec = mabur::SwConfig{
        fec.symbol_size[static_cast<size_t>(s)], 128, mabur::kUepRefOverhead[s]};
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
  check_keys(j, "", {"radio", "fec", "link", "video_out"});
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
    check_keys(r, "fec", {"symbol_size", "decode_deadline_ms", "seq_horizon"});
    if (r.contains("symbol_size")) {
      auto& s = r.at("symbol_size");
      if (s.is_array()) {
        if (s.size() != 4) fail("fec.symbol_size", "array must have 4 ints");
        for (size_t i = 0; i < 4; ++i)
          c.fec.symbol_size[i] = static_cast<int>(s.at(i).get<int64_t>());
      } else if (s.is_number_integer()) {
        c.fec.symbol_size.fill(static_cast<int>(s.get<int64_t>()));
      } else {
        fail("fec.symbol_size", "not an integer");
      }
      for (int v : c.fec.symbol_size)
        if (v < 32 || v > 1500) fail("fec.symbol_size", "must be in [32,1500]");
    }
    c.fec.decode_deadline_ms = static_cast<int>(get_int(r, "decode_deadline_ms", 200, 20, 5000, "fec"));
    c.fec.seq_horizon = static_cast<int>(get_int(r, "seq_horizon", 512, 16, 65536, "fec"));
  }

  if (j.contains("link")) {
    const json& r = j["link"];
    check_keys(r, "link", {"vtx_id", "feedback_ms", "beacon_keepalive_ms", "video_silence_ms", "src_bitrate_mbps", "margin_db", "static_mcs", "static_overhead", "static_txagc"});
    c.link.vtx_id = static_cast<uint32_t>(get_int(r, "vtx_id", 1, 0, 0xFFFFFFFFL, "link"));
    c.link.feedback_ms = static_cast<int>(get_int(r, "feedback_ms", 100, 20, 5000, "link"));
    c.link.beacon_keepalive_ms = static_cast<int>(get_int(r, "beacon_keepalive_ms", 1000, 100, 60000, "link"));
    c.link.video_silence_ms = static_cast<int>(get_int(r, "video_silence_ms", 3000, 500, 60000, "link"));
    c.link.src_bitrate_mbps = get_num(r, "src_bitrate_mbps", 4.0, 0.5, 50.0, "link");
    c.link.margin_db = get_num(r, "margin_db", 2.0, 0.0, 50.0, "link");
    c.link.static_mcs = static_cast<int>(get_int(r, "static_mcs", -1, -1, 7, "link"));
    c.link.static_overhead = get_num(r, "static_overhead", 0.25, 0.10, 1.0, "link");
    c.link.static_txagc = static_cast<int>(get_int(r, "static_txagc", 63, 0, 63, "link"));
  }

  if (j.contains("video_out")) {
    const json& r = j["video_out"];
    check_keys(r, "video_out", {"host", "port"});
    c.video_out.host = get_str(r, "host", "127.0.0.1", "video_out");
    c.video_out.port = static_cast<int>(get_int(r, "port", 5600, 1, 65535, "video_out"));
  }
  return c;
}

}  // namespace maburgs
