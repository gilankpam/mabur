#pragma once
#include <cstdint>
#include <cstring>

// The RK3568's (and RK3566's) VOP2 frame-length register: VPn's
// DSP_VTOTAL_VS_END. It carries an undocumented immediate bit: a write with
// it set applies to the frame being scanned out right now and holds there
// until the next write, rather than waiting for the next vblank like the
// driver's own writes. Each video port has its own copy of the register,
// 0x100 further on per `crtc_index` from VP0's offset below.

namespace maburplay::vop2 {

constexpr uint32_t kBase = 0xfe040000u;   // RK3568 / RK3566
constexpr uint32_t kVtotalReg = 0x0c50u;  // VP0's DSP_VTOTAL_VS_END
constexpr uint32_t kImd = 1u << 15;       // applies to the frame now scanning out

// The register's fields: vtotal in [28:16], the sync width in [12:0].
constexpr int vtotal_of(uint32_t v) { return static_cast<int>((v >> 16) & 0x1fff); }
constexpr int sync_of(uint32_t v) { return static_cast<int>(v & 0x1fff); }

// Whether `v` (immediate bit clear) is a mode of `vtotal` lines and
// `sync_lines` of sync. A total off by one short of `vtotal` is ours, left
// by a run that did not get to put it back; anything beyond twice `vtotal`
// is some other mode entirely.
constexpr bool is_mode(uint32_t v, int vtotal, int sync_lines) {
  return sync_of(v) == sync_lines && vtotal_of(v) >= vtotal - 1 &&
         vtotal_of(v) <= 2 * vtotal;
}

// The mode's own value, recovered from whatever total the register holds.
constexpr uint32_t mode_value(uint32_t v, int vtotal) {
  return (v & ~(0x1fffu << 16)) | (static_cast<uint32_t>(vtotal) << 16);
}

// `orig` with the total set to `total` lines, applied at once.
constexpr uint32_t total_value(uint32_t orig, int total) {
  return (orig & ~(0x1fffu << 16)) |
         ((static_cast<uint32_t>(total) & 0x1fff) << 16) | kImd;
}

// Whether a device tree's compatible list (space-separated, as read from
// /proc/device-tree/compatible with its NUL separators turned to spaces)
// names this VOP2.
inline bool on_this_soc(const char* compat) {
  return std::strstr(compat, "rockchip,rk3568") != nullptr ||
         std::strstr(compat, "rockchip,rk3566") != nullptr;
}

}  // namespace maburplay::vop2
