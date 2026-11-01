#include "mav_params.h"

#include <cmath>
#include <cstdint>

#include "camera_modes.h"
#include "mabur/config_write.h"

namespace mabur::mavparam {

namespace {

std::string size_text(int width, int height) {
  return std::to_string(width) + "x" + std::to_string(height);
}

Value acc_wall_margin_db(const Config& c) {
  return Value{Type::Real32, false, 0, static_cast<float>(c.radio.wall_margin_db)};
}
Value acc_ldpc(const Config& c) { return Value{Type::Bool, c.radio.ldpc}; }
Value acc_venc_mode(const Config& c) {
  return Value{Type::Str, false, 0, 0,
               std::to_string(c.venc.core.fps) + "@" +
                   size_text(c.venc.core.width, c.venc.core.height) +
                   (c.venc.core.binning ? "/binned" : "/native")};
}
Value acc_venc_slices(const Config& c) {
  return Value{Type::Int32, false, static_cast<int32_t>(c.venc.core.slices)};
}
Value acc_bitrate_max_kbps(const Config& c) {
  return Value{Type::Int32, false, static_cast<int32_t>(c.encoder.bitrate_max_kbps)};
}
Value acc_airtime_budget(const Config& c) {
  return Value{Type::Real32, false, 0, static_cast<float>(c.encoder.airtime_budget)};
}
Value acc_rec_enable(const Config& c) { return Value{Type::Bool, c.record.enable}; }
Value acc_rec_bitrate_kbps(const Config& c) {
  return Value{Type::Int32, false, static_cast<int32_t>(c.record.bitrate_kbps)};
}
Value acc_rec_fps(const Config& c) {
  return Value{Type::Int32, false, static_cast<int32_t>(c.record.fps)};
}
// record.width/height 0x0 means "same as venc.size" (RecordCfg's own
// comment, drone/src/config.h) — fall back so a MAVLink reader always sees
// the size the recording will actually come out at, never a bare "0x0".
Value acc_rec_size(const Config& c) {
  int w = c.record.width, h = c.record.height;
  if (w == 0 && h == 0) {
    w = c.venc.core.width;
    h = c.venc.core.height;
  }
  return Value{Type::Str, false, 0, 0, size_text(w, h)};
}
Value acc_lowpwr_enable(const Config& c) { return Value{Type::Bool, c.low_power.enable}; }
Value acc_msp_enable(const Config& c) { return Value{Type::Bool, c.msp.enable}; }
Value acc_venc_mirror(const Config& c) { return Value{Type::Bool, c.venc.core.mirror}; }
Value acc_venc_flip(const Config& c) { return Value{Type::Bool, c.venc.core.flip}; }

using Accessor = Value (*)(const Config&);

// One accessor per row of cfgparam::kTable, same order. The static_assert
// below means a row added to that table without a matching accessor here
// fails the build instead of silently reading garbage at runtime.
constexpr Accessor kAccessors[] = {
    acc_wall_margin_db,    // RADIO_WALL_MARGN
    acc_ldpc,              // RADIO_LDPC
    acc_venc_mode,         // VENC_MODE
    acc_venc_slices,       // VENC_SLICES
    acc_bitrate_max_kbps,  // ENC_BITRATE_MAX
    acc_airtime_budget,    // ENC_AIRTIME
    acc_rec_enable,        // REC_ENABLE
    acc_rec_bitrate_kbps,  // REC_BITRATE
    acc_rec_fps,           // REC_FPS
    acc_rec_size,          // REC_SIZE
    acc_lowpwr_enable,     // LOWPWR_ENABLE
    acc_msp_enable,        // MSP_ENABLE
    acc_venc_mirror,       // VENC_MIRROR
    acc_venc_flip,         // VENC_FLIP
};
static_assert(sizeof(kAccessors) / sizeof(kAccessors[0]) == cfgparam::kCount,
              "mav_params accessor table must have one entry per cfgparam::kTable row");

bool is_control_or_quote(unsigned char c) {
  return c < 0x20 || c == 0x7f || c == '"' || c == '\\';
}

}  // namespace

bool read(const Config& cfg, size_t idx, Value* out) {
  if (idx >= cfgparam::kCount) return false;
  *out = kAccessors[idx](cfg);
  return true;
}

bool render_toml(const Value& v, std::string* out) {
  switch (v.type) {
    case Type::Bool:
      *out = v.b ? "true" : "false";
      return true;
    case Type::Int32:
      *out = std::to_string(v.i);
      return true;
    case Type::Real32: {
      // NaN/Inf have no TOML spelling %.9g would produce one anyway
      // ("nan", "inf") and the ".0" append below would mangle it further
      // ("inf.0") into something that doesn't even round-trip as a float.
      if (!std::isfinite(v.f)) return false;
      // paramval::float_text already picks the shortest text that reads
      // back as the same float; append ".0" when it picked the integer
      // form, so the TOML value is never read back as an Int32.
      std::string s = paramval::float_text(v.f);
      if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
          s.find('E') == std::string::npos) {
        s += ".0";
      }
      *out = s;
      return true;
    }
    case Type::Str: {
      for (unsigned char c : v.s) {
        if (is_control_or_quote(c)) return false;
      }
      *out = "\"" + v.s + "\"";
      return true;
    }
  }
  return false;
}

bool patch_row(std::string* text, size_t idx, const Value& v, const Config& current,
               std::string* followed, std::string* err) {
  if (followed) followed->clear();
  if (idx >= cfgparam::kCount) {
    if (err) *err = "no such parameter";
    return false;
  }
  const cfgparam::Row& row = cfgparam::kTable[idx];

  if (idx != cfgparam::kVencModeRow) {
    std::string rendered;
    if (!render_toml(v, &rendered)) {
      if (err) *err = "value unsupported";
      return false;
    }
    std::string candidate = *text;
    if (!patch_toml_key(&candidate, row.section, row.key, rendered)) {
      if (err) *err = std::string(row.section) + "." + row.key + ": key not found";
      return false;
    }
    *text = std::move(candidate);
    return true;
  }

  // VENC_MODE's Str value "<fps>@<WxH>/binned|native" spells venc.fps,
  // venc.size and venc.binning together. A malformed shape can't be
  // rendered into any of the three lines, so it is "value unsupported",
  // the same answer render_toml's own refusal gives above -- never a
  // report of a missing key.
  int fps = 0, w = 0, h = 0;
  bool binning = false;
  if (v.type != Type::Str || !cfgparam::parse_mode_text(v.s, &fps, &w, &h, &binning)) {
    if (err) *err = "value unsupported";
    return false;
  }
  std::string candidate = *text;
  if (!patch_toml_key(&candidate, "venc", "fps", std::to_string(fps))) {
    if (err) *err = "venc.fps: key not found";
    return false;
  }
  const std::string size_text = "\"" + std::to_string(w) + "x" + std::to_string(h) + "\"";
  if (!patch_toml_key(&candidate, "venc", "size", size_text)) {
    if (err) *err = "venc.size: key not found";
    return false;
  }
  if (!patch_toml_key(&candidate, "venc", "binning", binning ? "true" : "false")) {
    if (err) *err = "venc.binning: key not found";
    return false;
  }
  // The recorder follows this set only when it is on and pinned to an
  // explicit size (width > 0; 0x0 means "same as venc.size" and already
  // tracks it with no line to patch), and either the aspect ratio changed
  // (cross-multiplied so this never depends on float rounding) or the
  // pinned recording is larger than 1920x1080 and than the new readout's
  // sensor mode in either dimension, above 30 fps -- the loader's 4K rule
  // (drone/venc/camera_modes.c, drone/src/config.cpp) would otherwise
  // refuse the candidate. At 30 fps or below the loader allows a
  // recording larger than the tier (it is served from the 4K mode), so
  // needs_4k must stay false there or a legal recording gets rewritten
  // for no reason. camera_mode_for's own w == 0 ("no mode for this
  // readout at this rate") counts as "no follow" here: a candidate with
  // no mode is refused by the loader's venc.fps check regardless of what
  // record.size holds.
  if (current.record.enable && current.record.width > 0) {
    const bool aspect_changed = static_cast<int64_t>(w) * current.record.height !=
                                 static_cast<int64_t>(current.record.width) * h;
    const CameraMode tier = camera_mode_for(binning ? 1 : 0, static_cast<uint32_t>(fps), 0, 0);
    const bool needs_4k = fps > 30 && tier.w != 0 &&
                           (current.record.width > 1920 || current.record.height > 1080) &&
                           (current.record.width > tier.w || current.record.height > tier.h);
    if (aspect_changed || needs_4k) {
      if (!patch_toml_key(&candidate, "record", "size", size_text)) {
        if (err) *err = "record.size: key not found";
        return false;
      }
      if (followed) *followed = "record.size";
    }
  }
  *text = std::move(candidate);
  return true;
}

}  // namespace mabur::mavparam
