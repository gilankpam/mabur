#pragma once
#include <cstdint>
#include <memory>
#include <string>

// The display controller's frame length, for the GS-side phase lock: making
// a frame a line or two longer or shorter moves the vblank without the
// screen noticing anything beyond a slightly early or late refresh.

namespace maburplay {

class FrameLength {
 public:
  // What the controller holds now.
  struct Seen {
    uint32_t value = 0;  // its raw value (immediate bit clear), for the log
    int total = 0;       // the frame length in lines
    bool held = false;   // an earlier immediate write is still in force
  };

  virtual ~FrameLength() = default;
  // Takes the controller's value as the current mode's (`vtotal` lines,
  // `sync_lines` of sync); a total a little off is ours, left by a run that
  // did not get to put it back. False if it is not that mode's: the lock
  // then stays off.
  virtual bool take_mode(int vtotal, int sync_lines, Seen* seen) = 0;
  // The frame being scanned out is `total` lines long, from now on.
  virtual void set_total(int total) = 0;
  // Back to the mode's own value, applied at a frame boundary as the
  // driver's own writes are.
  virtual void restore() = 0;
};

// The frame-length control for CRTC `crtc_index`, or nullptr with `*why`
// set when this build or this board has none.
std::unique_ptr<FrameLength> map_frame_length(int crtc_index, std::string* why);

}  // namespace maburplay
