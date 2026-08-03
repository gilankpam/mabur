#ifndef MABUR_PLAYER_IN_FORMATS_H_
#define MABUR_PLAYER_IN_FORMATS_H_

#include <cstdint>
#include <string>

namespace maburplay {

// Parses a KMS plane's IN_FORMATS blob (struct drm_format_modifier_blob,
// see <drm/drm_mode.h>) and decides whether `fourcc` can scan out under
// DRM_FORMAT_MOD_LINEAR (<drm/drm_fourcc.h>).
//
// Pure and host-buildable: no ioctls, no libdrm calls, just bytes in and a
// bool out. Lives outside drm_presenter.cpp (an HW-only, cross-build-only
// TU) specifically so the exact bench failure -- an AFBC-only plane that
// lists ARGB8888 in its plain format list but has NO LINEAR entry in
// IN_FORMATS -- is a unit test, not a hardware-only code path. See
// drm_presenter.cpp's plane_takes_linear_argb(), the only caller: it
// handles the "IN_FORMATS property absent entirely" case (a pre-modifier
// driver, where the plain format list is implicitly linear) itself, one
// layer up, because that is a property-existence question this function
// -- given only blob bytes -- cannot answer.
//
// Fails closed on anything it cannot positively parse and prove: a null or
// undersized blob, a header version other than FORMAT_BLOB_CURRENT, a
// format/modifier array that runs past blob_len, a format not listed at
// all, or a format listed with no LINEAR modifier. `why`, when non-null, is
// set to a short reason whenever the result is false; left untouched on
// success.
bool in_formats_has_linear(const void* blob_data, uint32_t blob_len, uint32_t fourcc,
                            std::string* why);

}  // namespace maburplay

#endif  // MABUR_PLAYER_IN_FORMATS_H_
