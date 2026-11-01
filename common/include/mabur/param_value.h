#pragma once
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "mabur/config_params.h"

// Parameter value coding shared by the drone's MAVLink extended-parameter
// component (drone/src/mav_params.*, cfgparam::kTable) and a ground-side
// client that edits the same rows over the MAVLink tunnel: the Value
// variant, wire coding for PARAM_EXT_* bytes, and text coding for a UI's
// text fields. Header-only and MAVLink-free, so a browser WASM build can
// use it without pulling in the MAVLink library.

namespace mabur::paramval {

using cfgparam::Type;

// A decoded parameter value, one alternative live per cfgparam::Type.
struct Value {
  Type type;
  bool b = false;
  int32_t i = 0;
  float f = 0;
  std::string s;
};

// Decodes the 128-byte wire buffer for row `idx`, given the sender's
// MAV_PARAM_EXT_TYPE (as a plain number, see cfgparam::mav_type). False
// (the caller should answer PARAM_ACK_VALUE_UNSUPPORTED) when idx is out
// of range, mav_type doesn't match the row's type, a Bool byte isn't 0 or
// 1, a Real32 buffer decodes to NaN or Inf, or a CUSTOM buffer has no NUL
// within 128 bytes or a non-printable byte (outside 0x20..0x7e, so DEL
// 0x7f included) before it.
inline bool decode_wire(size_t idx, uint8_t mav_type, const char wire[128], Value* out) {
  if (idx >= cfgparam::kCount) return false;
  const Type type = cfgparam::kTable[idx].type;
  if (mav_type != cfgparam::mav_type(type)) return false;
  switch (type) {
    case Type::Bool: {
      const uint8_t b = static_cast<uint8_t>(wire[0]);
      if (b != 0 && b != 1) return false;
      *out = Value{Type::Bool, b != 0};
      return true;
    }
    case Type::Int32: {
      int32_t v;
      std::memcpy(&v, wire, sizeof v);
      *out = Value{Type::Int32, false, v};
      return true;
    }
    case Type::Real32: {
      float v;
      std::memcpy(&v, wire, sizeof v);
      // A wire buffer decoding to NaN/Inf can never have come from a
      // legal config value (venc_cfg's rails, the bundle's %.9g text):
      // treat it the same as any other malformed payload.
      if (!std::isfinite(v)) return false;
      *out = Value{Type::Real32, false, 0, v};
      return true;
    }
    case Type::Str: {
      // A NUL must appear within the 128-byte buffer, and every byte
      // before it must be printable ASCII — otherwise the sender's buffer
      // is garbage, not a string this config could ever hold.
      size_t len = 0;
      for (; len < 128; ++len) {
        if (wire[len] == '\0') break;
        const unsigned char c = static_cast<unsigned char>(wire[len]);
        if (c < 0x20 || c > 0x7e) return false;
      }
      if (len == 128) return false;  // no NUL found
      *out = Value{Type::Str, false, 0, 0, std::string(wire, len)};
      // VENC_MODE packs <fps>@<WxH> into one Str value: a value whose
      // shape the loader could never make sense of is refused here,
      // before it ever reaches the config file.
      if (idx == cfgparam::kVencModeRow) {
        int fps, w, h;
        bool binning;
        if (!cfgparam::parse_mode_text(out->s, &fps, &w, &h, &binning)) return false;
      }
      return true;
    }
  }
  return false;
}

// Encodes `v` for PARAM_EXT_VALUE/PARAM_EXT_ACK: little-endian in bytes
// 0..3 for Bool/Int32/Real32, or NUL-terminated ASCII for Str. Zero-fills
// every byte past the value.
inline void encode_wire(const Value& v, char wire[128]) {
  std::memset(wire, 0, 128);
  switch (v.type) {
    case Type::Bool:
      wire[0] = v.b ? 1 : 0;
      break;
    case Type::Int32:
      std::memcpy(wire, &v.i, sizeof v.i);
      break;
    case Type::Real32:
      std::memcpy(wire, &v.f, sizeof v.f);
      break;
    case Type::Str: {
      const size_t n = std::min(v.s.size(), static_cast<size_t>(127));
      std::memcpy(wire, v.s.data(), n);
      wire[n] = '\0';
      break;
    }
  }
}

// The shortest `%.{6..9}g` text of `f` that reads back (via strtof) as the
// same float: %.9g always round-trips but spells 0.65f as 0.649999976, so
// a set of the value the bundle already holds would rewrite its line.
// mav_params.cpp's render_toml calls this and appends ".0" when the result
// has no '.' or 'e', so a TOML float is never read back as an Int32.
inline std::string float_text(float f) {
  char buf[64];
  for (int prec = 6; prec <= 9; ++prec) {
    std::snprintf(buf, sizeof buf, "%.*g", prec, static_cast<double>(f));
    if (std::strtof(buf, nullptr) == f) break;
  }
  return std::string(buf);
}

// Parses UI text for a row of type `t` into *out. False (leaving *out
// unspecified) for anything that isn't an exact, fully-consumed spelling:
// Bool accepts "0"/"1"/"true"/"false" case-insensitively; Int32 requires
// the whole string consumed by strtol and the result within int32 range;
// Real32 requires the whole string consumed by strtof and a finite
// result (so "inf" and "nan", which strtof happily parses, are refused);
// Str requires 1..127 bytes each in 0x20..0x7e, excluding '"' and '\\' so
// every parsed string is one render_toml can always quote without
// refusing it back.
inline bool parse_text(Type t, const std::string& text, Value* out) {
  switch (t) {
    case Type::Bool: {
      std::string lower = text;
      for (char& c : lower) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
      }
      if (lower == "1" || lower == "true") {
        *out = Value{Type::Bool, true};
        return true;
      }
      if (lower == "0" || lower == "false") {
        *out = Value{Type::Bool, false};
        return true;
      }
      return false;
    }
    case Type::Int32: {
      if (text.empty()) return false;
      errno = 0;
      char* end = nullptr;
      const long v = std::strtol(text.c_str(), &end, 10);
      if (end != text.c_str() + text.size()) return false;
      if (errno == ERANGE || v < INT32_MIN || v > INT32_MAX) return false;
      *out = Value{Type::Int32, false, static_cast<int32_t>(v)};
      return true;
    }
    case Type::Real32: {
      if (text.empty()) return false;
      char* end = nullptr;
      const float v = std::strtof(text.c_str(), &end);
      if (end != text.c_str() + text.size()) return false;
      if (!std::isfinite(v)) return false;
      *out = Value{Type::Real32, false, 0, v};
      return true;
    }
    case Type::Str: {
      if (text.empty() || text.size() > 127) return false;
      for (unsigned char c : text) {
        if (c < 0x20 || c > 0x7e || c == '"' || c == '\\') return false;
      }
      *out = Value{Type::Str, false, 0, 0, text};
      return true;
    }
  }
  return false;
}

// Renders a Value's own type as UI text: Bool "0"/"1" (not "true"/"false",
// since parse_text accepts either on the way back in); Int32 decimal;
// Real32 the shortest round-tripping text, without render_toml's ".0"
// append; Str as-is.
inline std::string format_text(const Value& v) {
  switch (v.type) {
    case Type::Bool:
      return v.b ? "1" : "0";
    case Type::Int32:
      return std::to_string(v.i);
    case Type::Real32:
      return float_text(v.f);
    case Type::Str:
      return v.s;
  }
  return {};
}

}  // namespace mabur::paramval
