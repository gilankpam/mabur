#include "colortrans.h"

#include <algorithm>
#include <cmath>

namespace maburplay {

const ColorTransParams kColorTrans3{};

namespace {

float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

Vec3 mul(const float m[9], Vec3 v) {
  return {m[0] * v.r + m[1] * v.g + m[2] * v.b,
          m[3] * v.r + m[4] * v.g + m[5] * v.b,
          m[6] * v.r + m[7] * v.g + m[8] * v.b};
}

// Cofactor inverse; both matrices here are far from singular.
void invert3(const float m[9], float out[9]) {
  const float a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7],
              i = m[8];
  const float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const float det = a * A + b * B + c * C;
  const float inv = 1.f / det;
  out[0] = A * inv;
  out[1] = -(b * i - c * h) * inv;
  out[2] = (b * f - c * e) * inv;
  out[3] = B * inv;
  out[4] = (a * i - c * g) * inv;
  out[5] = -(a * f - c * d) * inv;
  out[6] = C * inv;
  out[7] = -(a * h - b * g) * inv;
  out[8] = (a * e - b * d) * inv;
}

constexpr float kLumaR = 0.2126f, kLumaG = 0.7152f, kLumaB = 0.0722f;

}  // namespace

ColorTrans::ColorTrans(const ColorTransParams& p) : p_(p) {
  sat_ = std::min(3.f, std::max(0.f, 1.f + p_.saturation / 100.f));
  invert3(p_.matrix, minv_);
  // mix(vec3(luma), c, sat) == ((1-sat) * L + sat * I) * c, L = luma rows.
  const float S[9] = {(1.f - sat_) * kLumaR + sat_, (1.f - sat_) * kLumaG, (1.f - sat_) * kLumaB,
                      (1.f - sat_) * kLumaR, (1.f - sat_) * kLumaG + sat_, (1.f - sat_) * kLumaB,
                      (1.f - sat_) * kLumaR, (1.f - sat_) * kLumaG, (1.f - sat_) * kLumaB + sat_};
  invert3(S, sinv_);
}

Vec3 ColorTrans::forward(Vec3 c) const {
  const ColorTransParams& p = p_;
  if (p.reverse) {
    const float yoff = (p.y_offset_10b / 1023.f) * p.y_offset_strength;
    const Vec3 x{c.r - yoff, c.g - yoff, c.b - yoff};
    Vec3 y = mul(p.matrix, x);
    c = {y.r + p.black_lift, y.g + p.black_lift, y.b + p.black_lift};
  }
  c = {clamp01(c.r), clamp01(c.g), clamp01(c.b)};
  const float gm = std::min(5.f, std::max(0.1f, p.gamma));
  const float l = std::min(1.f, std::max(-1.f, p.lift));
  const float k = std::min(10.f, std::max(0.f, p.gain));
  Vec3 y{std::pow(c.r, gm), std::pow(c.g, gm), std::pow(c.b, gm)};
  y = {(y.r + l) * k * std::min(4.f, std::max(0.f, p.rgb_mult[0])),
       (y.g + l) * k * std::min(4.f, std::max(0.f, p.rgb_mult[1])),
       (y.b + l) * k * std::min(4.f, std::max(0.f, p.rgb_mult[2]))};
  const float luma = kLumaR * y.r + kLumaG * y.g + kLumaB * y.b;
  y = {luma + (y.r - luma) * sat_, luma + (y.g - luma) * sat_, luma + (y.b - luma) * sat_};
  return {clamp01(y.r), clamp01(y.g), clamp01(y.b)};
}

Vec3 ColorTrans::invert(Vec3 c) const {
  const ColorTransParams& p = p_;
  Vec3 y = mul(sinv_, c);
  const float k = std::min(10.f, std::max(0.f, p.gain));
  const float l = std::min(1.f, std::max(-1.f, p.lift));
  const float gm = std::min(5.f, std::max(0.1f, p.gamma));
  y = {y.r / (k * std::min(4.f, std::max(0.f, p.rgb_mult[0]))) - l,
       y.g / (k * std::min(4.f, std::max(0.f, p.rgb_mult[1]))) - l,
       y.b / (k * std::min(4.f, std::max(0.f, p.rgb_mult[2]))) - l};
  y = {clamp01(y.r), clamp01(y.g), clamp01(y.b)};
  y = {std::pow(y.r, 1.f / gm), std::pow(y.g, 1.f / gm), std::pow(y.b, 1.f / gm)};
  if (p.reverse) {
    const float yoff = (p.y_offset_10b / 1023.f) * p.y_offset_strength;
    const Vec3 z{y.r - p.black_lift, y.g - p.black_lift, y.b - p.black_lift};
    Vec3 x = mul(minv_, z);
    y = {x.r + yoff, x.g + yoff, x.b + yoff};
  }
  return {clamp01(y.r), clamp01(y.g), clamp01(y.b)};
}

namespace {

uint32_t pack_rgb8(Vec3 v) {
  const auto q = [](float f) -> uint32_t { return (uint32_t)std::lround(clamp01(f) * 255.f); };
  return (q(v.r) << 16) | (q(v.g) << 8) | q(v.b);
}
Vec3 unpack_rgb8(uint32_t rgb) {
  return {((rgb >> 16) & 0xFF) / 255.f, ((rgb >> 8) & 0xFF) / 255.f, (rgb & 0xFF) / 255.f};
}

uint32_t map_premul(uint32_t argb, uint32_t (*f)(const ColorTrans&, uint32_t),
                    const ColorTrans& t) {
  const uint32_t a = argb >> 24;
  if (a == 0) return 0u;
  const uint32_t pr = (argb >> 16) & 0xFF, pg = (argb >> 8) & 0xFF, pb = argb & 0xFF;
  const uint32_t straight = (std::min(255u, pr * 255 / a) << 16) |
                            (std::min(255u, pg * 255 / a) << 8) | std::min(255u, pb * 255 / a);
  const uint32_t m = f(t, straight);
  const uint32_t r = ((m >> 16) & 0xFF) * a / 255, g = ((m >> 8) & 0xFF) * a / 255,
                 b = (m & 0xFF) * a / 255;
  return (a << 24) | (r << 16) | (g << 8) | b;
}

}  // namespace

uint32_t forward_rgb8(const ColorTrans& t, uint32_t rgb) {
  return pack_rgb8(t.forward(unpack_rgb8(rgb)));
}
uint32_t invert_rgb8(const ColorTrans& t, uint32_t rgb) {
  return pack_rgb8(t.invert(unpack_rgb8(rgb)));
}
uint32_t forward_premul_argb(const ColorTrans& t, uint32_t argb) {
  return map_premul(argb, forward_rgb8, t);
}
uint32_t invert_premul_argb(const ColorTrans& t, uint32_t argb) {
  return map_premul(argb, invert_rgb8, t);
}

}  // namespace maburplay
