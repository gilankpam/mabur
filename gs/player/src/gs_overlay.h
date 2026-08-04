#ifndef MABUR_PLAYER_GS_OVERLAY_H_
#define MABUR_PLAYER_GS_OVERLAY_H_

#include <cstdint>
#include <string>
#include <vector>

#include "gs_snapshot.h"
#include "osd_raster.h"  // Surface, DirtyRect

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

// Field identity. The order here IS the draw order and the index into the
// overlay's shadow vector, so entries may be appended but never reordered.
// Cards are a fixed set of slots: kMaxCards bounds the block, and a slot
// past the reported card count simply renders nothing.
constexpr int kMaxCards = 4;

enum class GsFieldId {
  kRung = 0,
  kAirLabel,
  kAirValue,
  kAirMeter,
  kRec,
  kLossLabel,
  kLossPre,
  kLossArrow,
  kLossPost,
  kFpsValue,
  kFpsLabel,
  kJit,
  kMbps,
  // Card slots: FIVE fields per card, contiguous per slot. Five and not
  // four because the design colours every element of a row independently --
  // the id and unit in kTextLabel, the RSSI by status, the SNR in
  // kTextSecondary -- and one field can carry only one colour.
  kCard0Id, kCard0Bars, kCard0Rssi, kCard0Unit, kCard0Snr,
  kCard1Id, kCard1Bars, kCard1Rssi, kCard1Unit, kCard1Snr,
  kCard2Id, kCard2Bars, kCard2Rssi, kCard2Unit, kCard2Snr,
  kCard3Id, kCard3Bars, kCard3Rssi, kCard3Unit, kCard3Snr,
  kCount,
};

class GsFont;
struct MaskAtlas;

// The GS link-status overlay. Draws into the SAME ARGB surface the MSP
// rasterizer uses -- one plane, no compositing -- into four corner regions
// the MSP grid mostly does not reach. Where they do collide the caller
// draws MSP first and then calls repaint_intersecting(), so GS wins.
//
// Dirty tracking is per FIELD, not per block: each field has a box sized at
// layout() from its worst-case string, so a value change never reflows a
// row and a redraw touches ~40 k pixels rather than ~231 k. That margin is
// what keeps quantize_rects() inside maburplay's 2 ms pump loop.
class GsOverlay {
 public:
  explicit GsOverlay(GsFont& font) : font_(font) {}

  // Computes every field box for this surface size. Type sizes scale by
  // height/1080 and snap to an available atlas size; below an 18 px floor
  // the two TOP blocks are dropped and only the bottom two render.
  // Fails (with *err set) if a required atlas size is missing.
  bool layout(int screen_w, int screen_h, std::string* err);

  // Formats every field, redraws the ones whose rendered state changed, and
  // appends one DirtyRect per redrawn field to *out (which is NOT cleared:
  // the caller batches GS and MSP rects into one quantize_rects call).
  // Returns the number of fields redrawn.
  //
  // `stale` dims every LINK field to kTextLabel and holds its last value.
  // Player-measured fields ignore it -- they are current by construction.
  int update(const GsSnapshot& snap, bool stale, const GsPlayerState& ps,
             const Surface& s, std::vector<DirtyRect>* out);

  // Redraws every field whose box intersects any of `rects`, regardless of
  // whether its value changed. This is what makes GS pixels win a collision
  // with the MSP grid.
  int repaint_intersecting(const DirtyRect* rects, size_t n, const Surface& s,
                           std::vector<DirtyRect>* out);

  // Next update() is a full repaint. Call after anything that can have
  // clobbered the surface behind the overlay's back (a buffer swap into a
  // slot this overlay has never drawn, an MSP full-surface clear).
  void invalidate();

  // Union of every field box -- the region the overlay can ever touch.
  DirtyRect bounds() const { return bounds_; }
  int field_count() const { return (int)GsFieldId::kCount; }

  // Every token colour at full alpha plus its shadow blend, for
  // build_palette()'s extra seeds. Without these the burned DVR quantizes
  // GS pixels against an MSP-only palette that has no green or amber in it.
  static const uint32_t* palette_seeds(size_t* n);

  // Test hook: the string a field would render for these inputs.
  std::string debug_field_text(const GsSnapshot& snap, bool stale,
                               const GsPlayerState& ps, GsFieldId id) const;

  // Test hook: the box layout() computed for a field. Exists for the
  // magnitude-clamp regression test -- asserting "nothing draws outside
  // ITS box" needs to know what that box is, and bounds() only gives the
  // union of all of them.
  DirtyRect debug_field_box(GsFieldId id) const;

 private:
  struct FieldState {
    std::string text;
    uint32_t rgb = 0;
    int aux = -1;  // bars lit / meter fill px; -1 when unused
    bool operator==(const FieldState&) const = default;
  };
  struct Field {
    DirtyRect box;
    const MaskAtlas* atlas = nullptr;
    int baseline_y = 0;  // absolute, within the surface
    int pen_x = 0;       // absolute left edge of the pen
    bool active = false; // false => this field never renders (dropped block,
                         // or a card slot past the reported card count)
    FieldState last;
    bool valid = false;
  };

  // Per-row card geometry that doesn't depend on which slot or how many
  // cards are active -- computed once in layout(), reused by both layout()
  // itself (to seed the default kMaxCards-reserved positions) and by
  // update() (to re-anchor the ACTIVE rows to `bottom` whenever the
  // reported card count changes, so the block hugs the corner with 1-3
  // cards instead of always floating at the kMaxCards-reservation height).
  struct CardGeom {
    const MaskAtlas* cardid = nullptr;
    const MaskAtlas* primary = nullptr;
    const MaskAtlas* label = nullptr;
    const MaskAtlas* secondary = nullptr;
    int left = 0, bottom = 0, gap16 = 0, row_pitch = 0;
    int id_w = 0, bar_block_w = 0, bar_block_h = 0, rssi_w = 0, unit_w = 0;
    std::string rssi_worst;
  };

  FieldState state_of_(const GsSnapshot& snap, bool stale, const GsPlayerState& ps,
                       GsFieldId id) const;
  void draw_field_(GsFieldId id, const FieldState& st, const Surface& s);
  // Places slot `slot`'s five fields with their row's bottom edge at
  // `row_bottom`, using card_geom_. Shared by layout() (initial,
  // kMaxCards-reserved positions) and update() (re-anchored to `bottom`
  // for however many cards are actually active).
  void place_card_row_(int slot, int row_bottom);
  Field& f_(GsFieldId id) { return fields_[(size_t)id]; }
  const Field& f_(GsFieldId id) const { return fields_[(size_t)id]; }

  GsFont& font_;
  Field fields_[(size_t)GsFieldId::kCount];
  DirtyRect bounds_{0, 0, 0, 0};
  CardGeom card_geom_;
  // -1, not 0: layout() leaves every card slot active (it doesn't know the
  // card count yet), so a snapshot's first-ever report of ZERO cards must
  // still be treated as a change that deactivates all of them. 0 as the
  // sentinel would make that first update a no-op vs. an initial state
  // that happens to also read 0, leaving the reserved slots' bars fields
  // drawing unlit-track bars for cards that were never reported -- the
  // "empty shell" the design doc says zero cards must never render.
  int n_cards_ = -1;  // card slots currently active; -1 = never reconciled
  bool laid_out_ = false;
  double scale_ = 1.0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_OVERLAY_H_
