#include "gs_overlay.h"

#include <cmath>
#include <cstdio>

namespace maburplay {
namespace {
// Bars span -90 (nothing lit) to -45 (all six lit).
constexpr double kBarFloorDbm = -90.0;
constexpr double kBarCeilDbm = -45.0;
constexpr int kBars = 6;
}  // namespace

uint32_t status_rgb(Status s) {
  switch (s) {
    case Status::kOk: return tok::kStatusOk;
    case Status::kCaution: return tok::kStatusCaution;
    case Status::kCritical: return tok::kStatusCaution;
  }
  return tok::kStatusCaution;
}

Status rssi_status(double dbm) {
  if (dbm >= -65.0) return Status::kOk;
  if (dbm >= -80.0) return Status::kCaution;
  return Status::kCritical;
}

Status snr_status(double db) {
  if (db >= 12.0) return Status::kOk;
  if (db >= 6.0) return Status::kCaution;
  return Status::kCritical;
}

Status card_status(const GsCard& c) {
  if (!c.heard) return Status::kOk;
  const Status r = rssi_status(c.rssi_dbm.value_or(0.0));
  const Status s = snr_status(c.snr_db.value_or(0.0));
  return r > s ? r : s;  // enum order is ok < caution < critical
}

int rssi_bars(double dbm) {
  if (dbm <= kBarFloorDbm) return 0;
  if (dbm >= kBarCeilDbm) return kBars;
  const double frac = (dbm - kBarFloorDbm) / (kBarCeilDbm - kBarFloorDbm);
  // Round to nearest, not truncate: truncation understates the boundary
  // sample the design cites (-71 dBm -> 3 of 6; frac*6 = 2.53, which
  // truncates to 2). Rounding is also what keeps the mapping's two named
  // checkpoints (-58 -> 4, -71 -> 3) and monotonicity simultaneously true.
  int n = (int)std::lround(frac * kBars);
  if (n < 0) n = 0;
  if (n > kBars) n = kBars;
  return n;
}

int card_bars(const GsCard& c) {
  if (!c.heard || !c.rssi_dbm) return 0;
  return rssi_bars(*c.rssi_dbm);
}

Status airtime_status(double pct) {
  return pct >= 75.0 ? Status::kCaution : Status::kOk;
}

Status post_loss_status(double pct) {
  if (pct <= 0.0) return Status::kOk;
  if (pct <= 1.0) return Status::kCaution;
  return Status::kCritical;
}

std::string fmt_int(double v) {
  char b[32];
  double r = std::round(v);
  if (r == 0.0) r = 0.0;  // kill negative zero: "-0" would widen the box
  std::snprintf(b, sizeof(b), "%.0f", r);
  return b;
}

std::string fmt_one_dp(double v) {
  char b[32];
  // Round to one decimal place ourselves before handing off to snprintf.
  // "%.1f" alone rounds the raw binary value: 2.15 is actually stored as
  // 2.149999999999999911..., so naive "%.1f" prints "2.1", not the "2.2"
  // a human typing 2.15 expects. Scaling by 10 first lands on the nearest
  // representable double to the half-integer (21.5 is exact in binary),
  // so std::round's away-from-zero tie-break does the right thing.
  const double scaled = std::round(v * 10.0) / 10.0;
  std::snprintf(b, sizeof(b), "%.1f", scaled);
  return b;
}

std::string fmt_signed_int(double v) {
  const double r = std::round(v);
  if (r < 0.0) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.0f", -r);
    return std::string(kMinus) + b;
  }
  return fmt_int(r);
}

std::string fmt_clock(int seconds) {
  if (seconds < 0) seconds = 0;
  // Saturate rather than wrap: 60:00 reading as 00:00 mid-flight would say
  // "recording just started", which is the opposite of the truth. The cap
  // also keeps the string five characters wide forever, so the field box
  // sized at layout time stays correct.
  if (seconds > 99 * 60 + 59) seconds = 99 * 60 + 59;
  char b[16];
  std::snprintf(b, sizeof(b), "%02d:%02d", seconds / 60, seconds % 60);
  return b;
}

}  // namespace maburplay
