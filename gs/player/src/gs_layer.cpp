#include "gs_layer.h"

#include <cmath>
#include <cstdio>

#include "gs_compact.h"
#include "gs_draw.h"  // premul
#include "gs_overlay.h"

namespace maburplay {

// --- formatting ---------------------------------------------------------
// Shared by both styles. The compact bar uses fmt_int and fmt_one_dp only:
// it renders a plain ASCII '-' for negatives (fmt_signed_int's U+2212 is a
// typographic refinement the "no styling" bar deliberately does without).

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

// --- REC field text -------------------------------------------------------

const char* const kRecWorst = "\xE2\x97\x8F REC GS FAULT VTX NO CARD";

bool parse_rec_target(const std::string& s, RecTarget* out) {
  RecTarget t;
  if (s == "gs") t = RecTarget::kGs;
  else if (s == "vtx") t = RecTarget::kVtx;
  else if (s == "both") t = RecTarget::kBoth;
  else return false;
  if (out) *out = t;
  return true;
}

const char* rec_worst(RecTarget t) {
  switch (t) {
    // gs_target, VTX leg always kNone: "REC mm:ss" or "REC FAULT", both
    // eleven glyphs -- exactly the box the field had before the VTX
    // recorder existed.
    case RecTarget::kGs:   return "\xE2\x97\x8F REC FAULT";
    // GS leg ignored: "REC VTX mm:ss" (15) or "REC VTX <problem>", widest
    // "VTX NO CARD" (17). A recording VTX carries no problem text.
    case RecTarget::kVtx:  return "\xE2\x97\x8F REC VTX NO CARD";
    case RecTarget::kBoth: break;
  }
  return kRecWorst;
}

RecText rec_text(const RecState& r) {
  RecText out;
  const bool gs_rec = r.gs_target && r.kind == RecState::Kind::kRecording;
  const bool gs_bad = r.gs_target && r.kind == RecState::Kind::kFault;
  const bool vtx_rec = r.vtx == RecState::Vtx::kRecording;
  const bool vtx_shown = r.vtx != RecState::Vtx::kNone;
  const char* vtx_bad = nullptr;
  switch (r.vtx) {
    case RecState::Vtx::kWait:   vtx_bad = "VTX WAIT"; break;
    case RecState::Vtx::kNoCard: vtx_bad = "VTX NO CARD"; break;
    case RecState::Vtx::kFull:   vtx_bad = "VTX FULL"; break;
    case RecState::Vtx::kFault:  vtx_bad = "VTX FAULT"; break;
    case RecState::Vtx::kOff:    vtx_bad = "VTX OFF"; break;
    case RecState::Vtx::kNone:
    case RecState::Vtx::kRecording: break;
  }
  // Who is recording. A GS-only setup (vtx kNone) keeps the bare
  // "REC mm:ss" it always had.
  std::string who;
  if (gs_rec && vtx_rec) who = "GS+VTX";
  else if (gs_rec) who = vtx_shown ? "GS" : "";
  else if (vtx_rec) who = "VTX";
  // What is wrong, GS first.
  std::string bad;
  if (gs_bad) bad = vtx_shown ? "GS FAULT" : "FAULT";
  if (vtx_bad) bad += (bad.empty() ? "" : " ") + std::string(vtx_bad);

  if (gs_rec || vtx_rec) {
    out.text = std::string(kDotFilled) + " REC " + (who.empty() ? "" : who + " ") +
               fmt_clock(gs_rec ? r.elapsed_s : r.vtx_elapsed_s) +
               (bad.empty() ? "" : " " + bad);
    out.rgb = bad.empty() ? tok::kTextPrimary : tok::kStatusCaution;
    out.aux = 1;
  } else if (!bad.empty()) {
    out.text = std::string(kDotFilled) + " REC " + bad;
    out.rgb = tok::kStatusCaution;
  }
  return out;
}

// --- style selection ----------------------------------------------------

bool parse_gs_style(const std::string& s, GsStyle* out) {
  if (s == "essential") { if (out) *out = GsStyle::kEssential; return true; }
  if (s == "compact")   { if (out) *out = GsStyle::kCompact;   return true; }
  return false;
}

std::unique_ptr<GsLayer> make_gs_layer(GsStyle style, GsFont& font,
                                       RecTarget rec_target) {
  switch (style) {
    case GsStyle::kCompact:   return std::make_unique<GsCompactBar>(font, rec_target);
    case GsStyle::kEssential: break;
  }
  return std::make_unique<GsOverlay>(font, rec_target);
}

const uint32_t* gs_palette_seeds(size_t* n) {
  // Every token at a ramp of alphas, premultiplied. build_palette()'s
  // median cut needs the antialiased blends, not just the solid colours:
  // most GS pixels are partially covered glyph edges.
  static std::vector<uint32_t> seeds;
  static const ColorTrans* seeded_for = nullptr;
  static bool seeded = false;
  if (!seeded || seeded_for != colour_inverse()) {
    seeds.clear();
    const uint32_t toks[] = {tok::kTextPrimary,  tok::kTextSecondary,
                             tok::kTextLabel,    tok::kTrack,
                             tok::kStatusOk,     tok::kStatusCaution,
                             tok::kStatusRec};
    for (uint32_t t : toks)
      for (int a = 255; a >= 0; a -= 17) seeds.push_back(premul(t, (uint8_t)a));
    // The shadow blend: black at a ramp of alphas, which is what a
    // shadow-only pixel looks like.
    for (int a = 255; a >= 0; a -= 17) seeds.push_back(premul(0x000000u, (uint8_t)a));
    seeded = true;
    seeded_for = colour_inverse();
  }
  if (n) *n = seeds.size();
  return seeds.data();
}

}  // namespace maburplay
