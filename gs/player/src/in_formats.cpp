#include "in_formats.h"

#include <cstring>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

namespace maburplay {

bool in_formats_has_linear(const void* blob_data, uint32_t blob_len, uint32_t fourcc,
                            std::string* why) {
  bool ok = false;
  std::string reason = "IN_FORMATS blob unreadable";
  if (blob_data && blob_len >= sizeof(struct drm_format_modifier_blob)) {
    // memcpy throughout: the blob's interior offsets come from the driver,
    // so neither the format array nor the modifier array can be assumed
    // aligned for a direct struct/u32 load.
    struct drm_format_modifier_blob hdr {};
    std::memcpy(&hdr, blob_data, sizeof(hdr));
    if (hdr.version != FORMAT_BLOB_CURRENT) {
      // FORMAT_BLOB_CURRENT (1) is the only layout this parser understands.
      // A hypothetical future version could relocate or resize any of the
      // fields read below; misparsing it as v1 would be memory-safe (every
      // read stays inside the bounds-checked blob) but could fail OPEN --
      // reading garbage offsets as if they were valid ones. Reject instead.
      reason = "IN_FORMATS blob has an unsupported version";
    } else {
      const uint8_t* base = static_cast<const uint8_t*>(blob_data);
      const uint64_t len = blob_len;
      // count_formats/count_modifiers are driver-controlled u32s. Both
      // multiplications below are carried out in uint64_t specifically so
      // a UINT32_MAX count cannot wrap fend/mend back under `len` (a u32
      // computation could) and sail through the bounds check that follows.
      const uint64_t fend = static_cast<uint64_t>(hdr.formats_offset) + 4ull * hdr.count_formats;
      const uint64_t mend = static_cast<uint64_t>(hdr.modifiers_offset) +
                             static_cast<uint64_t>(sizeof(struct drm_format_modifier)) *
                                 hdr.count_modifiers;
      if (fend > len || mend > len) {
        reason = "IN_FORMATS blob is malformed";
      } else {
        uint32_t idx = 0;
        bool found = false;
        for (uint32_t f = 0; f < hdr.count_formats && !found; ++f) {
          uint32_t got = 0;
          std::memcpy(&got, base + hdr.formats_offset + 4ull * f, sizeof(got));
          if (got == fourcc) {
            idx = f;
            found = true;
          }
        }
        if (!found) {
          reason = "IN_FORMATS does not list the requested format";
        } else {
          reason = "lists the format but with no LINEAR modifier (compression-only window)";
          for (uint32_t m = 0; m < hdr.count_modifiers && !ok; ++m) {
            struct drm_format_modifier e {};
            std::memcpy(&e, base + hdr.modifiers_offset + sizeof(e) * m, sizeof(e));
            if (e.modifier != DRM_FORMAT_MOD_LINEAR) continue;
            // The UAPI's own comment on drm_format_modifier::formats says
            // this bitmask indexes drmModeGetPlane()'s format list, not the
            // blob's own array `idx` was just computed against above -- but
            // mainline drivers build both lists from the same
            // plane->format_types, so the two are byte-for-byte identical
            // and `idx` lands on the correct bit either way.
            if (idx < e.offset || idx - e.offset >= 64) continue;  // sliding 64-format window
            if (e.formats & (1ull << (idx - e.offset))) ok = true;
          }
        }
      }
    }
  }
  if (!ok && why) *why = reason;
  return ok;
}

}  // namespace maburplay
