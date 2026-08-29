#include "gs_snapshot.h"

#include <cmath>
#include <limits>
#include <string>

#include "json.hpp"

namespace maburplay {
namespace {
using nlohmann::json;

// Every accessor below is null- and type-tolerant by construction: the
// sideport is additive-only under v:1, so an unknown shape is a future
// version talking, not an error to escalate.
const json* obj(const json& o, const char* key) {
  if (!o.is_object()) return nullptr;
  auto it = o.find(key);
  if (it == o.end() || !it->is_object()) return nullptr;
  return &*it;
}

std::optional<double> num(const json& o, const char* key) {
  if (!o.is_object()) return std::nullopt;
  auto it = o.find(key);
  if (it == o.end() || !it->is_number()) return std::nullopt;
  return it->get<double>();
}

std::optional<int> integer(const json& o, const char* key) {
  const std::optional<double> v = num(o, key);
  if (!v) return std::nullopt;
  // A finite double outside int's range is well-formed JSON but a
  // double->int cast on it is UB (saturates to INT_MIN unsanitized on
  // x86-64/aarch64 -- garbage rendered as a real value, not "dropped" as
  // this parser's contract requires). Reject rather than clamp: a
  // clamped mcs of INT_MAX is exactly as wrong on screen as INT_MIN, and
  // "never received" is the honest rendering for a nonsense number.
  if (!std::isfinite(*v)) return std::nullopt;
  if (*v < static_cast<double>(std::numeric_limits<int>::min()) ||
      *v > static_cast<double>(std::numeric_limits<int>::max()))
    return std::nullopt;
  return static_cast<int>(*v);
}
}  // namespace

bool parse_gs_snapshot(const char* data, size_t n, GsSnapshot* out) {
  if (!out) return false;
  *out = GsSnapshot{};
  if (!data || n == 0) return false;

  json j;
  try {
    j = json::parse(data, data + n);
  } catch (const std::exception&) {
    return false;  // counted by the caller; never propagated
  }
  if (!j.is_object()) return false;

  if (const json* link = obj(j, "link")) {
    out->air_pct = num(*link, "air_pct");
    if (const std::optional<double> r = num(*link, "residual_loss"))
      out->post_loss_pct = *r * 100.0;
    if (const json* ctl = obj(*link, "ctl")) {
      if (const std::optional<double> p = num(*ctl, "pre_fec_loss"))
        out->pre_loss_pct = *p * 100.0;
      if (const json* rung = obj(*ctl, "rung")) {
        out->mcs = integer(*rung, "mcs");
        // link.ctl.rung.ov: value semantics only, no parsing change needed
        // here. Since 2026-08-29 (airtime-balance-uep) this is the LITERAL
        // FEC command overhead for the ladder's current rung -- the config
        // table's own values doubled (old cmd x2 = new actual) in the same
        // change, so the *100 below still lands on the right on-screen
        // percentage; this parser just forwards whatever number the
        // sideport sends, raw, like every other field here.
        if (const std::optional<double> ov = num(*rung, "ov"))
          out->fec_pct = *ov * 100.0;
      }
    }
  }

  if (j.contains("cards") && j["cards"].is_array()) {
    for (const json& c : j["cards"]) {
      if (!c.is_object()) continue;
      GsCard card;
      if (const std::optional<int> id = integer(c, "id")) card.id = *id;
      if (const json* classes = obj(c, "classes")) {
        if (const json* s0 = obj(*classes, "s0")) {
          card.rssi_dbm = num(*s0, "rssi");
          card.snr_db = num(*s0, "snr");
        }
      }
      // "Heard" needs both figures: the status colour is worst-of(rssi,snr)
      // and a half-populated row would colour itself off one of them.
      card.heard = card.rssi_dbm.has_value() && card.snr_db.has_value();
      out->cards.push_back(card);
    }
  }
  return true;
}

}  // namespace maburplay
