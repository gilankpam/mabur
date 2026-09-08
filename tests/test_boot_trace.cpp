// Host coverage for the boot-timeline stamps maburd writes into
// /tmp/mabur.log. The drone's UART0 TX pad is destroyed, so that log is the
// only place its startup timeline can be read (docs/boot-time-findings-
// 2026-09-07.md, "What is still blocked"); run_real_mode() itself needs a
// radio and an encoder and is not host-runnable, so the arithmetic is split
// into boot_trace_fmt() and tested here.
#include "boot_trace.h"

#include <cstring>
#include <string>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include "mtest.h"

namespace {

std::string fmt(uint64_t us) {
  char buf[32];
  size_t n = boot_trace_fmt(us, buf, sizeof(buf));
  CHECK(n == std::strlen(buf));
  return std::string(buf);
}

}  // namespace

TEST(fmt_zero_is_padded_to_the_common_width) {
  // Two seconds columns, so every stamp in a normal (<100 s) startup lines
  // up in the log and a stage boundary can be read down the column.
  CHECK(fmt(0) == "+ 0.000");
}

TEST(fmt_renders_milliseconds) {
  CHECK(fmt(1234000) == "+ 1.234");
  CHECK(fmt(999999) == "+ 0.999");
  CHECK(fmt(12345678) == "+12.345");
}

TEST(fmt_truncates_rather_than_rounds) {
  // A stamp must never read later than the event it marks: floor, so the
  // number is a lower bound on elapsed time, not a nearest-value estimate.
  CHECK(fmt(1234999) == "+ 1.234");
  CHECK(fmt(1999) == "+ 0.001");
}

TEST(fmt_widens_past_the_pad_rather_than_losing_the_value) {
  // The pad is cosmetic. A daemon that has been up for hours still reports
  // the true elapsed time (this fires if maburd is restarted by hand long
  // after boot, which is exactly how the A/B is measured).
  CHECK(fmt(123456789) == "+123.456");
  CHECK(fmt(3600000000ULL) == "+3600.000");
}

TEST(fmt_always_terminates_and_never_overruns_the_buffer) {
  char buf[8];
  std::memset(buf, 'x', sizeof(buf));
  size_t n = boot_trace_fmt(123456789, buf, 4);
  CHECK(buf[3] == '\0');
  CHECK(n == 3);
  CHECK(std::string(buf) == "+12");
  // Untouched past the cap.
  CHECK(buf[4] == 'x');
}

TEST(fmt_with_zero_capacity_writes_nothing) {
  char buf[4];
  std::memset(buf, 'x', sizeof(buf));
  CHECK(boot_trace_fmt(1234000, buf, 0) == 0);
  CHECK(buf[0] == 'x');
}

TEST(elapsed_is_zero_before_init) {
  // Reading the clock before main() has latched t0 must not report a
  // 50-year uptime from the raw monotonic value.
  CHECK(boot_trace_elapsed_us() == 0);
}

TEST(elapsed_advances_after_init) {
  boot_trace_init();
  uint64_t a = boot_trace_elapsed_us();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  uint64_t b = boot_trace_elapsed_us();
  CHECK(b > a);
  CHECK(b - a >= 4000);
}

TEST(init_is_idempotent) {
  // A stray second call must not re-zero a timeline that is already being
  // read, or every stamp after it silently restarts at + 0.000.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  uint64_t before = boot_trace_elapsed_us();
  boot_trace_init();
  uint64_t after = boot_trace_elapsed_us();
  CHECK(after >= before);
}

TEST(bootlog_survives_a_dup2_over_stderr) {
  // The venc bring-up's sdk_quiet dup2()s /dev/null over fd 2 around every
  // vendor call. A stamp emitted inside that window must still reach the
  // log, or the slowest calls in the whole startup are exactly the ones
  // that cannot be timed. Capture the real stderr through a pipe, then
  // blank fd 2 the way sdk_quiet does, and stamp.
  int saved = dup(STDERR_FILENO);
  REQUIRE(saved >= 0);
  int pfd[2];
  REQUIRE(pipe(pfd) == 0);
  REQUIRE(dup2(pfd[1], STDERR_FILENO) == STDERR_FILENO);
  // init (already ran in an earlier test, so it is idempotent here) took
  // its dup while fd 2 pointed at the original stderr, not our pipe. Force
  // the test's own view: re-point boot_trace's descriptor at the pipe via
  // a fresh dup so the assertion is about the mechanism, not test order.
  int bt = boot_trace_fd();
  CHECK(bt != STDERR_FILENO);  // init took a private dup
  REQUIRE(dup2(pfd[1], bt) == bt);

  int devnull = open("/dev/null", O_WRONLY);
  REQUIRE(devnull >= 0);
  REQUIRE(dup2(devnull, STDERR_FILENO) == STDERR_FILENO);  // sdk_quiet
  bootlog("inside quiet window %d", 42);
  REQUIRE(dup2(saved, STDERR_FILENO) == STDERR_FILENO);    // sdk_quiet_end

  char buf[256] = {0};
  ssize_t n = read(pfd[0], buf, sizeof(buf) - 1);
  REQUIRE(n > 0);
  std::string got(buf, static_cast<size_t>(n));
  CHECK(got.find("] inside quiet window 42\n") != std::string::npos);
  CHECK(got.rfind("[boot +", 0) == 0);

  close(devnull); close(pfd[0]); close(pfd[1]); close(saved);
}

MTEST_MAIN
