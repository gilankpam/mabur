#ifndef MABUR_PLAYER_REC_BUTTON_H_
#define MABUR_PLAYER_REC_BUTTON_H_

#include <cstdint>
#include <optional>
#include <string>

namespace maburplay {

// True when `name` is a GPIO line name for header pin `pin`. Accepts
// "PIN_<n>", "GPIO<n>" and a bare "<n>", and nothing else: the whole
// string must be consumed and the number must start with a digit, so
// "PIN_320" never answers to pin 32 and strtol's leading-space/sign
// tolerance cannot leak in.
bool match_pin_name(const char* name, int pin);

// Debounced press detector. `level` is already polarity-corrected -- the
// kernel inverts for us when the line is requested ACTIVE_LOW -- so true
// means pressed.
//
// A level is accepted only once it has held for kDebounceMs: every sample
// that disagrees with the candidate restarts its clock, and a press is
// reported when the pressed level is accepted. A spike shorter than the
// window -- contact chatter, or noise coupled onto the button wire -- is
// therefore never a press, whichever way it goes. Accepting a change on its
// first sample instead (a leading-edge debounce) turns one noise spike into
// a phantom DVR toggle: on a line idling at the pressed level, a spike to
// released is accepted, the return a few ms later is swallowed as bounce,
// and the next sample past the window "presses". A held button fires once
// and never repeats. The cost is kDebounceMs of latency per press.
class ButtonDebounce {
 public:
  static constexpr uint64_t kDebounceMs = 50;

  // Returns true exactly once per accepted press. The FIRST call only
  // seeds the baseline and never fires, so a button held down (or a line
  // that idles pressed because the wiring is inverted) cannot toggle the
  // DVR at startup.
  bool feed(bool level, uint64_t now_ms);

  // The accepted level -- continuous, unlike feed()'s press-edge return.
  bool level() const { return level_; }

 private:
  bool level_ = false;      // the accepted level
  bool cand_ = false;       // the level the line has read since cand_ms_
  uint64_t cand_ms_ = 0;
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
  // flags. A busy line names its current holder in *err. `consumer` is
  // the name the kernel reports back to anything that inspects the line
  // (lsgpio, the busy-line error above) -- the rec button and the menu
  // bank's five lines want different names so a busy-line error tells you
  // which one is holding it.
  bool open(const RecButtonCfg& cfg, std::string* err,
            const char* consumer = "maburplay_rec");

  bool is_open() const { return line_fd_ >= 0; }

  // One GPIO_V2_LINE_GET_VALUES_IOCTL fed to the debouncer; true on an
  // accepted press. After kMaxIoErrors CONSECUTIVE ioctl failures the line
  // is closed and one line is logged, then silence -- /tmp is tmpfs, the
  // same reason the per-attempt DRM retry failures are silenced.
  bool poll(uint64_t now_ms);

  // The menu bank's path: one GPIO_V2_LINE_GET_VALUES_IOCTL fed to the
  // debouncer, returning the accepted LEVEL (not an edge) -- the menu
  // needs to know a line is still held, not just that it was pressed.
  // std::nullopt once the line has closed itself after kMaxIoErrors, same
  // as poll() above but reported rather than silently false forever.
  std::optional<bool> read_level(uint64_t now_ms);

  const std::string& chip_path() const { return chip_path_; }
  unsigned offset() const { return offset_; }

 private:
  static constexpr int kMaxIoErrors = 10;

  // Test-only seam (defined in tests/test_rec_button.cpp, the only
  // translation unit that calls it -- kept out of rec_button.cpp so it
  // never links into the shipped maburplay binary): lets a host test seed
  // line_fd_ with an arbitrary already-open fd, so open()'s
  // close-the-old-line guard can be exercised and its actual ::close()
  // observed without a real gpiochip. Not part of the public API -- do
  // not call this outside tests.
  friend void RecButtonSetLineFdForTest(RecButton& b, int fd);

  int line_fd_ = -1;
  std::string chip_path_;
  unsigned offset_ = 0;
  int io_errors_ = 0;
  ButtonDebounce deb_;
};

// Declared here (the friend declaration above requires it), defined only
// in tests/test_rec_button.cpp: test-only, not for production callers.
void RecButtonSetLineFdForTest(RecButton& b, int fd);

}  // namespace maburplay

#endif  // MABUR_PLAYER_REC_BUTTON_H_
