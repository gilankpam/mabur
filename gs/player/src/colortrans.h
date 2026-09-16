#ifndef MABUR_PLAYER_COLORTRANS_H_
#define MABUR_PLAYER_COLORTRANS_H_

#include <cstdint>

namespace maburplay {

// The GS-side reverse of the drone's "ColorTrans" sensor tuning
// (docs/colortrans.md). One evaluator, three consumers: the CRTC 3D LUT
// (build_cubic_lut), the OSD inverse (invert*), and -- as a verbatim GLSL
// copy in frame_colortrans.cpp -- the burned DVR. Host-buildable.
struct Vec3 {
  float r = 0.f, g = 0.f, b = 0.f;
};

// colortrans3.glsl's constants, in its order. Retune by editing
// kColorTrans3 in colortrans.cpp AND the shader string in
// frame_colortrans.cpp; test_colortrans pins forward() to the glsl.
struct ColorTransParams {
  bool reverse = true;              // apply the BT.709-style inverse first
  float y_offset_10b = 200.0f;      // camera luma offset, 10-bit units
  float y_offset_strength = 0.25f;  // fraction of it to remove
  float black_lift = 0.020f;
  float matrix[9] = {1.17866031f,  0.17460893f,  0.01571472f,
                     -0.11147506f, 1.55408099f,  -0.07362197f,
                     0.03243649f,  0.11275820f,  1.22378927f};  // row-major
  float gamma = 1.0f;
  float lift = -0.15f;
  float gain = 1.75f;
  float rgb_mult[3] = {1.0f, 1.0f, 1.0f};
  float saturation = -22.5f;  // mpv units, [-100, 100]
};

extern const ColorTransParams kColorTrans3;

class ColorTrans {
 public:
  explicit ColorTrans(const ColorTransParams& p = kColorTrans3);
  // colortrans3.glsl hook(), step for step, clamps included. In/out [0,1].
  Vec3 forward(Vec3 c) const;
  // The steps backwards. Exact wherever forward() did not clip; clamps
  // where it did (the OSD palette never does -- test_colortrans).
  Vec3 invert(Vec3 c) const;
  const ColorTransParams& params() const { return p_; }

 private:
  ColorTransParams p_;
  float sat_ = 1.f;    // clamp(1 + saturation/100, 0, 3)
  float minv_[9] = {}; // inverse of matrix
  float sinv_[9] = {}; // inverse of the saturation mix matrix
};

// 0xRRGGBB <-> 0xRRGGBB through the transform, rounded.
uint32_t forward_rgb8(const ColorTrans& t, uint32_t rgb);
uint32_t invert_rgb8(const ColorTrans& t, uint32_t rgb);
// Premultiplied 0xAARRGGBB: un-premultiply, transform, re-premultiply.
// Alpha is untouched; a fully transparent word stays 0.
uint32_t forward_premul_argb(const ColorTrans& t, uint32_t argb);
uint32_t invert_premul_argb(const ColorTrans& t, uint32_t argb);

// --- CRTC 3D LUT (VOP2 CUBIC_LUT, 9x9x9, 12-bit) -------------------------
// Which input channel varies fastest across the 729 entries. Undocumented;
// pinned on the bench (docs/colortrans.md). kRedFastest is the default.
enum class LutAxis { kRedFastest, kBlueFastest };
constexpr int kCubicLutEdge = 9;
constexpr int kCubicLutEntries = kCubicLutEdge * kCubicLutEdge * kCubicLutEdge;  // 729
struct CubicLut {
  uint16_t rgb[kCubicLutEntries][3] = {};  // 0..4095 per channel
};
// Flat index of grid node (ri, gi, bi), each 0..8, under `axis`.
int cubic_lut_index(int ri, int gi, int bi, LutAxis axis);
// t == nullptr yields the identity grid (the "off" table).
CubicLut build_cubic_lut(const ColorTrans* t, LutAxis axis);

}  // namespace maburplay

#endif  // MABUR_PLAYER_COLORTRANS_H_
