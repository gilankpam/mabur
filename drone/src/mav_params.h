#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include "config.h"
#include "mabur/config_params.h"
#include "mabur/param_value.h"

// Drone-side accessors for the MAVLink extended-parameter component
// (cfgparam::kTable, common/include/mabur/config_params.h). This file
// reads/writes mabur::Config fields; it never includes the MAVLink
// library itself. The value coding itself (Value, decode_wire, encode_wire)
// lives in mabur::paramval (common/include/mabur/param_value.h), shared
// with a ground-side client, and is re-exposed here under its old names.

namespace mabur::mavparam {

using cfgparam::Type;
using Value = paramval::Value;
using paramval::decode_wire;
using paramval::encode_wire;

// Reads row `idx` of cfgparam::kTable out of a loaded Config into *out.
// False, *out untouched, when idx >= cfgparam::kCount — callers that drive
// idx from a wire param_index must not be able to read past the accessor
// table.
bool read(const Config& cfg, size_t idx, Value* out);

// Renders a Value as the TOML text for its row: "true"/"false", a decimal
// int, a float in the fewest significant digits (6..9) that read back as
// the same float and always containing a '.' or 'e', or a string quoted with
// "...". Returns false (and leaves *out unspecified) for a string
// containing '"', '\', a control character (including DEL, 0x7f), or for
// a non-finite float (NaN/Inf have no TOML spelling render_toml will
// produce) — render_toml never escapes.
bool render_toml(const Value& v, std::string* out);

// Patches `text` for row `idx`'s own config line(s), leaving `text`
// unchanged on failure. Every row but kVencModeRow owns one line:
// render_toml(v) must succeed and that line must exist in [section], or
// *err is "value unsupported" or "<section.key>: key not found".
// kVencModeRow's Str value "<fps>@<WxH>/binned|native" patches venc.fps,
// venc.size and venc.binning together (a value that doesn't parse to that
// shape is "value unsupported", never a report of a missing key); when
// `current.record.enable`, `current.record.width > 0` (an explicit
// record.size, not the "same as venc.size" default) and either the new
// WxH's aspect ratio differs from the current recording size's (compared
// by cross-multiplication, no float division) or the recording is larger
// than 1920x1080 and than the new readout's sensor mode (camera_mode_for)
// in either dimension, record.size is patched to the same WxH too and
// *followed is set to "record.size" -- cleared at the start of every call
// otherwise. record.fps never follows.
bool patch_row(std::string* text, size_t idx, const Value& v, const Config& current,
               std::string* followed, std::string* err);

}  // namespace mabur::mavparam
