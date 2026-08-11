#include "rec_button.h"

#include <fcntl.h>
#include <glob.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace maburplay {
namespace {

// The tail of a line name must be exactly the decimal pin number: no
// sign, no leading space, no trailing characters. strtol alone accepts
// all three, which is why the first character is checked by hand.
bool tail_is_pin(const char* s, int pin) {
  if (!s || !std::isdigit(static_cast<unsigned char>(s[0]))) return false;
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  return end && *end == '\0' && v == static_cast<long>(pin);
}

}  // namespace

bool match_pin_name(const char* name, int pin) {
  if (!name || !*name) return false;
  if (std::strncmp(name, "PIN_", 4) == 0) return tail_is_pin(name + 4, pin);
  if (std::strncmp(name, "GPIO", 4) == 0) return tail_is_pin(name + 4, pin);
  return tail_is_pin(name, pin);
}

bool ButtonDebounce::feed(bool level, uint64_t now_ms) {
  if (!seeded_) {
    seeded_ = true;
    level_ = level;
    last_change_ms_ = now_ms;
    return false;
  }
  if (level == level_) return false;
  if (now_ms - last_change_ms_ < kDebounceMs) return false;  // bounce
  level_ = level;
  last_change_ms_ = now_ms;
  return level;  // a press, never a release
}

RecButton::~RecButton() {
  if (line_fd_ >= 0) ::close(line_fd_);
}

bool RecButton::resolve_pin(int pin, std::string* chip_path, unsigned* offset,
                            std::string* err) {
  glob_t g{};
  if (::glob("/dev/gpiochip*", 0, nullptr, &g) != 0) {
    *err = "no /dev/gpiochip* present";
    ::globfree(&g);
    return false;
  }
  bool found = false;
  int chips = 0;
  for (size_t i = 0; i < g.gl_pathc && !found; ++i) {
    const int fd = ::open(g.gl_pathv[i], O_RDONLY | O_CLOEXEC);
    if (fd < 0) continue;
    ++chips;
    struct gpiochip_info ci {};
    if (::ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &ci) == 0) {
      for (unsigned o = 0; o < ci.lines && !found; ++o) {
        struct gpio_v2_line_info li {};
        li.offset = o;
        if (::ioctl(fd, GPIO_V2_GET_LINEINFO_IOCTL, &li) != 0) continue;
        if (match_pin_name(li.name, pin)) {
          *chip_path = g.gl_pathv[i];
          *offset = o;
          found = true;
        }
      }
    }
    ::close(fd);
  }
  ::globfree(&g);
  if (!found) {
    *err = "no GPIO line named PIN_" + std::to_string(pin) + " / GPIO" +
           std::to_string(pin) + " on any of " + std::to_string(chips) + " chip(s)";
  }
  return found;
}

bool RecButton::open(const RecButtonCfg& cfg, std::string* err) {
  if (!resolve_pin(cfg.pin, &chip_path_, &offset_, err)) return false;

  const int chip_fd = ::open(chip_path_.c_str(), O_RDONLY | O_CLOEXEC);
  if (chip_fd < 0) {
    *err = chip_path_ + ": " + std::strerror(errno);
    return false;
  }

  struct gpio_v2_line_request req {};
  req.offsets[0] = offset_;
  req.num_lines = 1;
  req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
  if (cfg.active_low) req.config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
  if (cfg.bias == "pull-up") {
    req.config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
  } else if (cfg.bias == "pull-down") {
    req.config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
  } else {
    req.config.flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED;
  }
  std::snprintf(req.consumer, sizeof(req.consumer), "maburplay_rec");

  if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) != 0 || req.fd < 0) {
    const int e = errno;
    // Several lines on this board are already claimed (board-led,
    // bt_default_*, board-antenna). Name the holder rather than leaving
    // the reader to guess at EBUSY.
    struct gpio_v2_line_info li {};
    li.offset = offset_;
    if (e == EBUSY && ::ioctl(chip_fd, GPIO_V2_GET_LINEINFO_IOCTL, &li) == 0 &&
        li.consumer[0] != '\0') {
      *err = chip_path_ + " line " + std::to_string(offset_) + " is held by \"" +
             li.consumer + "\"";
    } else {
      *err = chip_path_ + " line " + std::to_string(offset_) + ": " + std::strerror(e);
    }
    ::close(chip_fd);
    return false;
  }
  ::close(chip_fd);  // the line fd is independent of the chip fd
  line_fd_ = req.fd;
  return true;
}

bool RecButton::poll(uint64_t now_ms) {
  if (line_fd_ < 0) return false;
  struct gpio_v2_line_values v {};
  v.mask = 1;
  if (::ioctl(line_fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) != 0) {
    if (++io_errors_ >= kMaxIoErrors) {
      std::fprintf(stderr,
                   "maburplay: rec button: %d consecutive read failures (%s) -- "
                   "button disabled for the rest of this run\n",
                   io_errors_, std::strerror(errno));
      ::close(line_fd_);
      line_fd_ = -1;
    }
    return false;
  }
  io_errors_ = 0;
  return deb_.feed((v.bits & 1u) != 0, now_ms);
}

}  // namespace maburplay
