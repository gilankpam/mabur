#ifndef MABUR_PLAYER_GS_OVERLAY_H_
#define MABUR_PLAYER_GS_OVERLAY_H_

#include <cstdint>
#include <string>

#include "gs_snapshot.h"

namespace maburplay {

// Design tokens (docs/superpowers/specs/2026-08-04-gs-osd-essential-design.md).
// Opaque 0xRRGGBB: alpha always comes from glyph coverage, never from here.
namespace tok {
constexpr uint32_t kTextPrimary = 0xF2F3F5u;    // values
constexpr uint32_t kTextSecondary = 0xB0B4B8u;  // SNR, unit-suffixed %
constexpr uint32_t kTextLabel = 0x93989Du;      // labels, units, separators,
                                                // and the dimmed-stale state
constexpr uint32_t kTrack = 0x43474Bu;          // meter track, unlit bars
constexpr uint32_t kStatusOk = 0x3FC99Au;
constexpr uint32_t kStatusCaution = 0xDFA63Au;  // also the ceiling tick
// The seventh token, and the one deviation from the handoff's "no third
// colour" rule. That rule governs status ESCALATION -- it exists so a
// critical link state cannot invent a colour the alert layer owns.
// Recording is a mode indicator, not a status, and a white dot does not
// read as recording. Faults still use kStatusCaution.
constexpr uint32_t kStatusRec = 0xE5484Du;
}  // namespace tok

// Non-ASCII glyphs the design calls for by name.
constexpr const char* kEmDashPair = "——";  // never received
constexpr const char* kMinus = "−";             // NOT a hyphen
constexpr const char* kArrow = "→";
constexpr const char* kDotFilled = "●";
constexpr const char* kDotHollow = "○";

enum class Status { kOk, kCaution, kCritical };

// Critical maps to the caution hue: the essential OSD has no third colour.
uint32_t status_rgb(Status s);

Status rssi_status(double dbm);
Status snr_status(double db);
// Worst of the two. An unheard card returns kOk -- it has no signal to
// judge, and painting it critical would confuse "silent" with "terrible".
Status card_status(const GsCard& c);

// 0..6, six steps mapped over -90 -> -45 dBm.
int rssi_bars(double dbm);
int card_bars(const GsCard& c);  // 0 when unheard

Status airtime_status(double pct);     // caution at the 75 % ceiling
Status post_loss_status(double pct);   // ok only at exactly 0

// Player-measured half of the OSD's inputs. maburgs cannot supply these:
// its fps counts AUs published to the ring, which still reads 60 while a
// wedged decoder shows a frozen picture.
struct RecState {
  enum class Kind {
    kAbsent,     // dvr disabled in config: the block does not render
    kArmed,      // enabled, nothing written yet
    kRecording,  // file open and samples advancing
    kFault,      // open failed / samples stalled / disk nearly full
  };
  Kind kind = Kind::kAbsent;
  int elapsed_s = 0;
};

struct GsPlayerState {
  double fps = 0.0;
  double jitter_ms = 0.0;
  double mbps = 0.0;
  RecState rec;
};

// Formatting. Every value is fixed-width by construction so a field box
// never has to grow -- which is what makes tabular figures load-bearing
// here rather than merely tidy.
std::string fmt_int(double v);         // nearest integer, never negative-zero
std::string fmt_one_dp(double v);      // always one decimal place
std::string fmt_signed_int(double v);  // U+2212 for negatives
std::string fmt_clock(int seconds);    // mm:ss, saturating at 99:59

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_OVERLAY_H_
