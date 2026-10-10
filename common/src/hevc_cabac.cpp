#include "mabur/hevc_cabac.h"

#include <algorithm>

namespace mabur::hevc {

const uint8_t kRangeTabLps[64][4] = {
    {128, 176, 208, 240}, {128, 167, 197, 227}, {128, 158, 187, 216}, {123, 150, 178, 205},
    {116, 142, 169, 195}, {111, 135, 160, 185}, {105, 128, 152, 175}, {100, 122, 144, 166},
    {95, 116, 137, 158},  {90, 110, 130, 150},  {85, 104, 123, 142},  {81, 99, 117, 135},
    {77, 94, 111, 128},   {73, 89, 105, 122},   {69, 85, 100, 116},   {66, 80, 95, 110},
    {62, 76, 90, 104},    {59, 72, 86, 99},     {56, 69, 81, 94},     {53, 65, 77, 89},
    {51, 62, 73, 85},     {48, 59, 69, 80},     {46, 56, 66, 76},     {43, 53, 63, 72},
    {41, 50, 59, 69},     {39, 48, 56, 65},     {37, 45, 54, 62},     {35, 43, 51, 59},
    {33, 41, 48, 56},     {32, 39, 46, 53},     {30, 37, 43, 50},     {29, 35, 41, 48},
    {27, 33, 39, 45},     {26, 31, 37, 43},     {24, 30, 35, 41},     {23, 28, 33, 39},
    {22, 27, 32, 37},     {21, 26, 30, 35},     {20, 24, 29, 33},     {19, 23, 27, 31},
    {18, 22, 26, 30},     {17, 21, 25, 28},     {16, 20, 23, 27},     {15, 19, 22, 25},
    {14, 18, 21, 24},     {14, 17, 20, 23},     {13, 16, 19, 22},     {12, 15, 18, 21},
    {12, 14, 17, 20},     {11, 14, 16, 19},     {11, 13, 15, 18},     {10, 12, 15, 17},
    {10, 12, 14, 16},     {9, 11, 13, 15},      {9, 11, 12, 14},      {8, 10, 12, 14},
    {8, 9, 11, 13},       {7, 9, 11, 12},       {7, 9, 10, 12},       {7, 8, 10, 11},
    {6, 8, 9, 11},        {6, 7, 9, 10},        {6, 7, 8, 9},         {2, 2, 2, 2}};

const uint8_t kTransIdxLps[64] = {
    0,  0,  1,  2,  2,  4,  4,  5,  6,  7,  8,  9,  9,  11, 11, 12,
    13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
    24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63};

CabacCtx init_cabac_ctx(int init_value, int slice_qp_y) {
  const int slope = init_value >> 4, offset = init_value & 15;
  const int m = slope * 5 - 45, n = (offset << 3) - 16;
  const int qp = std::clamp(slice_qp_y, 0, 51);
  const int pre = std::clamp(((m * qp) >> 4) + n, 1, 126);
  CabacCtx c;
  c.mps = pre <= 63 ? 0 : 1;
  c.state = static_cast<uint8_t>(c.mps ? pre - 64 : 63 - pre);
  return c;
}

void CabacEncoder::put_bit(int b) {
  if (first_) first_ = false;
  else w_.put(b);
  for (; outstanding_ > 0; --outstanding_) w_.put(1 - b);
}

void CabacEncoder::renorm() {
  while (range_ < 256) {
    if (low_ < 256) {
      put_bit(0);
    } else if (low_ >= 512) {
      low_ -= 512;
      put_bit(1);
    } else {
      low_ -= 256;
      ++outstanding_;
    }
    range_ <<= 1;
    low_ <<= 1;
  }
}

void CabacEncoder::encode(CabacCtx& c, int bin) {
  const uint32_t lps = kRangeTabLps[c.state][(range_ >> 6) & 3];
  range_ -= lps;
  if (bin != c.mps) {
    low_ += range_;
    range_ = lps;
    if (c.state == 0) c.mps = static_cast<uint8_t>(1 - c.mps);
    c.state = kTransIdxLps[c.state];
  } else if (c.state < 62) {
    ++c.state;
  }
  renorm();
}

void CabacEncoder::terminate(int bin) {
  range_ -= 2;
  if (!bin) {
    renorm();
    return;
  }
  low_ += range_;
  range_ = 2;           // EncodeFlush
  renorm();
  put_bit((low_ >> 9) & 1);
  w_.u(((low_ >> 7) & 3) | 1, 2);
}

}  // namespace mabur::hevc
