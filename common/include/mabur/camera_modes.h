#pragma once
#include <string>
#include <vector>

// The drone's camera sensor modes, shared between the operator UIs (the
// goggles menu and the web VTX tab) that build a VENC_MODE choice list.
// Sizes and zoom factors mirror docs/camera-modes.md "Sizes" and "Which
// mode venc.binning and venc.fps select"; the web tab's MODE_SIZES /
// MODE_ZOOM (web/ui/src/lib/vtxparams.js) carries the same table, and a
// test keeps the two equal.

namespace mabur::cammode {

struct Size { int w, h; };

struct SensorMode {
  bool binned;
  int max_fps;
  const char* zoom;  // label text without the trailing "x", e.g. "1.0"
  std::vector<Size> sizes;
};

inline const std::vector<SensorMode>& modes() {
  static const std::vector<SensorMode> kModes = {
      {true, 90, "1.0", {{1920, 1080}, {1280, 720}}},
      {true, 100, "1.1", {{1728, 972}, {1280, 720}}},
      {true, 120, "1.1", {{1728, 816}, {1152, 544}, {864, 408}}},
      {false, 60, "1.3", {{1920, 1080}, {1600, 900}, {1280, 720}}},
      {false, 100, "1.6", {{1920, 1080}, {1600, 900}, {1280, 720}}},
  };
  return kModes;
}

// The first row with this readout whose max_fps covers fps; null outside
// either readout's range (fps < 1, fps > 120 binned, fps > 100 native --
// there is no native mode above it).
inline const SensorMode* mode_for(int fps, bool binned) {
  if (fps < 1) return nullptr;
  for (const SensorMode& m : modes()) {
    if (m.binned == binned && fps <= m.max_fps) return &m;
  }
  return nullptr;
}

inline constexpr int kRates[] = {60, 90, 100, 120};

// "<fps> fps · <W>×<H> · <zoom>×" with " bin" appended for a binned
// readout. The zoom comes from the sensor mode fps/binned select, not
// from the size, so a size off that mode's own list (e.g. a leftover
// REC_SIZE value) still labels by its mode's zoom.
inline std::string choice_label(int fps, int w, int h, bool binned) {
  const SensorMode* m = mode_for(fps, binned);
  std::string label = std::to_string(fps) + " fps · " + std::to_string(w) + "×" +
                       std::to_string(h) + " · " + (m ? m->zoom : "?") + "×";
  if (binned) label += " bin";
  return label;
}

}  // namespace mabur::cammode
