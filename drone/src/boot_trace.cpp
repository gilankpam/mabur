#include "boot_trace.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <atomic>
#include <unistd.h>

namespace {

// Nanoseconds of CLOCK_MONOTONIC at boot_trace_init(). Zero means "not
// initialised" — the clock's own zero is unreachable in practice (it is
// system uptime), so no sentinel is wasted.
std::atomic<uint64_t> g_t0_ns{0};

// Private dup of stderr taken at init. The venc bring-up wraps its vendor
// calls in sdk_quiet, which dup2()s /dev/null over fds 1 and 2 to silence
// the MI blobs — a stamp written to fd 2 inside such a window simply
// vanishes (that is how the pre-init teardown's ten calls hid from the
// first attribution run). Writing to our own descriptor makes the stamp
// independent of whatever the process later does to fd 2.
std::atomic<int> g_fd{STDERR_FILENO};

uint64_t mono_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace

extern "C" {

void boot_trace_init(void) {
  uint64_t expected = 0;
  // compare_exchange, not a plain store: idempotent by construction, so a
  // stray second call cannot restart the timeline at zero.
  if (g_t0_ns.compare_exchange_strong(expected, mono_ns(), std::memory_order_relaxed)) {
    int fd = ::dup(STDERR_FILENO);
    if (fd >= 0) g_fd.store(fd, std::memory_order_relaxed);
  }
}

int boot_trace_fd(void) { return g_fd.load(std::memory_order_relaxed); }

uint64_t boot_trace_elapsed_us(void) {
  uint64_t t0 = g_t0_ns.load(std::memory_order_relaxed);
  if (t0 == 0) return 0;
  uint64_t now = mono_ns();
  if (now <= t0) return 0;
  return (now - t0) / 1000ULL;
}

size_t boot_trace_fmt(uint64_t us, char* out, size_t cap) {
  if (!out || cap == 0) return 0;
  char tmp[32];
  // Truncating division, not rounding: a stamp must never read later than
  // the event it marks, so the millisecond field is a floor.
  const unsigned long long sec = static_cast<unsigned long long>(us / 1000000ULL);
  const unsigned long long ms = static_cast<unsigned long long>((us / 1000ULL) % 1000ULL);
  // Two seconds columns so a normal (<100 s) startup lines up in the log and
  // a stage boundary can be read down the column; the pad is cosmetic and a
  // longer uptime widens past it rather than losing the value.
  int n = std::snprintf(tmp, sizeof(tmp), "+%2llu.%03llu", sec, ms);
  if (n < 0) { out[0] = '\0'; return 0; }
  size_t len = static_cast<size_t>(n);
  if (len > sizeof(tmp) - 1) len = sizeof(tmp) - 1;
  if (len > cap - 1) len = cap - 1;
  std::memcpy(out, tmp, len);
  out[len] = '\0';
  return len;
}

void bootlog(const char* fmt, ...) {
  char stamp[32];
  boot_trace_fmt(boot_trace_elapsed_us(), stamp, sizeof(stamp));

  char line[512];
  int n = std::snprintf(line, sizeof(line), "[boot %s] ", stamp);
  if (n < 0) return;
  size_t off = static_cast<size_t>(n);
  if (off > sizeof(line) - 2) off = sizeof(line) - 2;

  va_list ap;
  va_start(ap, fmt);
  int m = std::vsnprintf(line + off, sizeof(line) - off - 1, fmt, ap);
  va_end(ap);
  if (m > 0) {
    size_t body = static_cast<size_t>(m);
    if (body > sizeof(line) - off - 2) body = sizeof(line) - off - 2;
    off += body;
  }
  line[off++] = '\n';

  // One write(2), not fprintf: bootlog is called from the main thread and
  // from the radio bring-up thread concurrently, and stderr's own locking
  // would still let a second line land between this stamp and its text if
  // the two were separate calls. fd 2 directly also survives the _exit(3)
  // fault path with no buffered tail to lose.
  ssize_t ignored = ::write(g_fd.load(std::memory_order_relaxed), line, off);
  (void)ignored;
}

}  // extern "C"
