#ifndef MABUR_PLAYER_GS_LAYER_H_
#define MABUR_PLAYER_GS_LAYER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gs_snapshot.h"
#include "osd_raster.h"  // Surface, DirtyRect

namespace maburplay {

// Design tokens.
// Opaque 0xRRGGBB: alpha always comes from glyph coverage, never from here.
namespace tok {
constexpr uint32_t kTextPrimary = 0xF2F3F5u;    // values
constexpr uint32_t kTextSecondary = 0xB0B4B8u;  // SNR, unit-suffixed %
constexpr uint32_t kTextLabel = 0x93989Du;      // labels, units, separators,
                                                // and the dimmed-stale state
constexpr uint32_t kTrack = 0x43474Bu;          // meter track, unlit bars
constexpr uint32_t kStatusOk = 0x3FC99Au;
constexpr uint32_t kStatusCaution = 0xDFA63Au;  // also the ceiling tick
// Recording is a mode indicator, not a status, and a white dot does not
// read as recording. Recorder faults use kStatusCaution.
constexpr uint32_t kStatusRec = 0xE5484Du;
// The one critical link state: the drone has stopped hearing the GS (the
// uplink cell, gs_compact.h). The same hue as kStatusRec but its own token,
// because it means a fault, not a mode -- a caller must never pick one for
// the other.
constexpr uint32_t kStatusCritical = 0xE5484Du;
// The settings menu's own surfaces (menu_layer.h). Never burned into a
// recording, so gs_palette_seeds() leaves them out.
constexpr uint32_t kMenuPanel = 0x0A0C10u;   // panel, picker, toast, banner fill
constexpr uint32_t kMenuSelect = 0x1D4FA3u;  // the selected row / choice
constexpr uint32_t kMenuTag = 0xE0B060u;     // the VTX tag on drone rows
}  // namespace tok

// Player-measured half of the OSD's inputs. maburgs cannot supply these:
// its fps counts AUs published to the ring, which still reads 60 while a
// wedged decoder shows a frozen picture.
struct RecState {
  enum class Kind {
    kArmed,      // nothing written yet
    kRecording,  // file open and samples advancing
    kFault,      // open failed / samples stalled / disk nearly full
  };
  Kind kind = Kind::kArmed;
  int elapsed_s = 0;
  // VTX onboard recorder leg. kNone = the VTX is not a
  // target or the button is off; with kNone and gs_target the field renders
  // exactly as it did before the VTX recorder existed.
  enum class Vtx { kNone, kWait, kRecording, kNoCard, kFull, kFault, kOff };
  Vtx vtx = Vtx::kNone;
  int vtx_elapsed_s = 0;
  bool gs_target = true;   // false with dvr.target = "vtx": the GS leg is ignored
};

struct GsPlayerState {
  double fps = 0.0;
  // Drone fps vs panel Hz (rate_match.h). rate_mismatch: too far apart
  // for the phase lock; the fps cell then shows src_hz/panel_hz. Both are
  // player-measured (pts cadence, vblank), so they never dim.
  bool rate_mismatch = false;
  double src_hz = 0.0;
  double panel_hz = 0.0;
  // The phase lock gave up (out of range) on a pair that otherwise matches:
  // the steer cannot reach the panel. Caution tint on the fps cell.
  bool phase_oor = false;
  double jitter_ms = 0.0;
  double mbps = 0.0;
  RecState rec;
  // Decoded picture size, latched from the DmaFrame the backend last
  // handed the presenter -- NOT the screen mode and NOT the burned-DVR
  // encode size, both of which stay at the panel's resolution while the
  // drone streams something smaller. 0 = nothing decoded yet, which the
  // compact bar renders as "--" rather than "0x0".
  int vid_w = 0;
  int vid_h = 0;

  // Latency OSD rows (latency accounting).
  // Player-measured, same as fps/jitter/mbps above -- current by
  // construction, never dimmed by `stale`. Sourced from
  // maburplay::LatTracker::p99_frame()/p50_frame() at 1 Hz: the REAL
  // p99- and p50-by-e2e frames' end-to-end times, not independently-ranked
  // per-segment percentiles (see lat_tracker.h). `lat_valid` false means
  // the anchor isn't usable yet (cold/discontinuous) -- the rows render
  // "LAT P99 --" rather than stale or fabricated numbers.
  //
  // Each frame's own 7-segment breakdown used to render beside these
  // headlines; it was dropped. Segment attribution is a
  // post-flight question the flight jsonl and flightreport.py answer
  // better, and on the glass it cost a full-width column that reached
  // into the centre-of-frame keep-clear band.
  bool lat_valid = false;
  // link-rtt: true once main folded the absolute network
  // floor (own anchor + sideport pts offset) into e2e — the rows then
  // read capture-stamp→scanout-start truth. False = today's relative
  // numbers (offset estimator cold, or sync lost at range); the headline
  // carries a ~ marker so the pilot always knows which one is showing.
  bool lat_abs = false;
  int lat_e2e_ms = 0;
  // The median frame, ranked the same way (p50-by-e2e). Shown as its own
  // row directly above the p99 one so the tail is read against the typical
  // frame rather than in isolation: before this existed, the OSD's only
  // latency figure was a tail statistic sitting among averages
  // (fps/jit/mbps) and read as if it were typical. Shares lat_valid --
  // both come from the same window and the same anchor.
  int lat_p50_e2e_ms = 0;
};

// The recording dot. Shared: both styles draw the same indicator, so the
// glyph it is made of cannot live in either one's header.
constexpr const char* kDotFilled = "●";

// The most receiving cards the OSD will render. A snapshot reporting more
// is truncated to this: the compact bar sizes its type from a line this
// wide.
constexpr int kMaxCards = 4;

// Formatting. Every value is fixed-width by construction so a field box
// never has to grow -- which is what makes tabular figures load-bearing
// here rather than merely tidy.
std::string fmt_int(double v);         // nearest integer, never negative-zero
std::string fmt_one_dp(double v);      // always one decimal place
std::string fmt_clock(int seconds);    // mm:ss, saturating at 99:59

// The REC field's text (gs_compact kRec): where recording happens (GS, VTX, GS+VTX) and why a selected
// target is not recording. aux = 1 paints the leading dot in kStatusRec.
struct RecText {
  std::string text;
  uint32_t rgb = 0;
  int aux = 0;
};
RecText rec_text(const RecState& r);
// Widest string rec_text() can return for ANY target (the GS+VTX combos).
extern const char* const kRecWorst;

// dvr.target: which recorders the button drives. The
// kRec box is sized on rec_worst(target) -- the widest string
// rec_text() can return for a player with that target -- rather than on
// kRecWorst for everyone: the box is cleared whole on every REC redraw and
// wins repaint_intersecting over MSP cells, so a box wider than anything
// the target can draw costs the FC OSD a strip of glyphs for nothing.
enum class RecTarget {
  kGs,    // "gs": byte- and pixel-identical to the pre-VTX-recorder field
  kVtx,   // "vtx"
  kBoth,  // "both"
};
// "gs" | "vtx" | "both" -> target. False on anything else.
bool parse_rec_target(const std::string& s, RecTarget* out);
const char* rec_worst(RecTarget t);

// What the GS overlay draws, as OsdComposer sees it. The one implementation
// is GsCompactBar (gs_compact.h), two plain-text rows along the bottom edge.
//
// OsdComposer owns THREE layers (buffer 0, buffer 1, the burn canvas) and
// drives them all through exactly these five calls. Everything else a
// layout offers -- the bar's field enum and debug hooks -- is its own
// business and stays off this interface, so another layout can slot in
// behind make_gs_layer() without the composer changing.
class GsLayer {
 public:
  virtual ~GsLayer() = default;

  // Computes every field box for this surface size. Fails (with *err set)
  // when the font cannot serve the layout at this size.
  virtual bool layout(int screen_w, int screen_h, std::string* err) = 0;

  // Formats every field, redraws the ones whose rendered state changed, and
  // appends one DirtyRect per redrawn field to *out (which is NOT cleared:
  // the caller batches GS and MSP rects into one quantize_rects call).
  // Returns the number of fields redrawn.
  //
  // `stale` dims every LINK field and holds its last value. Player-measured
  // fields ignore it -- they are current by construction.
  virtual int update(const GsSnapshot& snap, bool stale, const GsPlayerState& ps,
                     const Surface& s, std::vector<DirtyRect>* out) = 0;

  // Redraws every field whose box intersects any of `rects`, regardless of
  // whether its value changed. This is what makes GS pixels win a collision
  // with the MSP grid.
  virtual int repaint_intersecting(const DirtyRect* rects, size_t n,
                                   const Surface& s,
                                   std::vector<DirtyRect>* out) = 0;

  // Next update() is a full repaint. Call after anything that can have
  // clobbered the surface behind the layer's back (a buffer swap into a
  // slot this layer has never drawn, an MSP full-surface clear).
  virtual void invalidate() = 0;

  // Union of every field box -- the region the layer can ever touch.
  virtual DirtyRect bounds() const = 0;
};

class GsFont;
std::unique_ptr<GsLayer> make_gs_layer(GsFont& font,
                                       RecTarget rec_target = RecTarget::kGs);

// Every token colour at full alpha plus its shadow blend, for
// build_palette()'s extra seeds. Without these the burned DVR quantizes
// GS pixels against an MSP-only palette that has no green or amber in it.
// The compact bar draws only a subset of the tokens; seeding a palette with
// colours no pixel happens to take costs nothing.
const uint32_t* gs_palette_seeds(size_t* n);

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_LAYER_H_
