#ifndef MABUR_PLAYER_REC_BUTTON_H_
#define MABUR_PLAYER_REC_BUTTON_H_

#include <cstdint>
#include <string>

namespace maburplay {

// True when `name` is a GPIO line name for header pin `pin`. Accepts
// "PIN_<n>", "GPIO<n>" and a bare "<n>", and nothing else: the whole
// string must be consumed and the number must start with a digit, so
// "PIN_320" never answers to pin 32 and strtol's leading-space/sign
// tolerance cannot leak in.
bool match_pin_name(const char* name, int pin);

// Edge-triggered press detector. `level` is already polarity-corrected --
// the kernel inverts for us when the line is requested ACTIVE_LOW -- so
// true means pressed.
//
// Deliberately simple: a transition is accepted only if it is at least
// kDebounceMs after the last ACCEPTED transition, and a press is reported
// only on the accepted false->true edge. A held button therefore fires
// once and never repeats. The known limit is a switch whose bounce lasts
// longer than kDebounceMs, which can read as press-release-press; 50 ms
// is comfortably past the settling time of the tactile switches this is
// built for, and is the value PixelPilot_rk uses for the same job.
class ButtonDebounce {
 public:
  static constexpr uint64_t kDebounceMs = 50;

  // Returns true exactly once per accepted press. The FIRST call only
  // seeds the baseline and never fires, so a button held down (or a line
  // that idles pressed because the wiring is inverted) cannot toggle the
  // DVR at startup.
  bool feed(bool level, uint64_t now_ms);

 private:
  bool level_ = false;
  uint64_t last_change_ms_ = 0;
  bool seeded_ = false;
};

struct RecButtonCfg {
  int pin = 0;
  bool active_low = true;
  std::string bias = "pull-up";  // "pull-up" | "pull-down" | "none"
};

// One input line on the kernel's GPIO v2 character device, polled for a
// debounced press.
//
// No libgpiod: maburplay is a fully static musl binary and this is one
// input line, so <linux/gpio.h> -- which ships with the cross toolchain's
// kernel headers -- is the whole dependency. Not sysfs either: deprecated,
// no bias control, racy on export, absent on newer kernels.
class RecButton {
 public:
  RecButton() = default;
  ~RecButton();
  RecButton(const RecButton&) = delete;
  RecButton& operator=(const RecButton&) = delete;

  // Scans /dev/gpiochip* for a line whose name matches `pin` (see
  // match_pin_name). On failure fills *err with something a log reader can
  // act on and returns false.
  static bool resolve_pin(int pin, std::string* chip_path, unsigned* offset,
                          std::string* err);

  // resolve_pin, then request the line once with the bias/active-low
  // flags. A busy line names its current holder in *err.
  bool open(const RecButtonCfg& cfg, std::string* err);

  bool is_open() const { return line_fd_ >= 0; }

  // One GPIO_V2_LINE_GET_VALUES_IOCTL fed to the debouncer; true on an
  // accepted press. After kMaxIoErrors CONSECUTIVE ioctl failures the line
  // is closed and one line is logged, then silence -- /tmp is tmpfs, the
  // same reason the per-attempt DRM retry failures are silenced.
  bool poll(uint64_t now_ms);

  const std::string& chip_path() const { return chip_path_; }
  unsigned offset() const { return offset_; }

 private:
  static constexpr int kMaxIoErrors = 10;

  int line_fd_ = -1;
  std::string chip_path_;
  unsigned offset_ = 0;
  int io_errors_ = 0;
  ButtonDebounce deb_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_REC_BUTTON_H_
