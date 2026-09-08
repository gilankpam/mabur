#pragma once
// Boot timeline for maburd's own startup, written into its stderr log.
//
// Why this exists as its own thing rather than a plain fprintf: the drone's
// UART0 TX solder pad is destroyed, so there is no serial console on the
// aircraft and /tmp/mabur.log is the only surface its startup timeline can
// be read from (docs/boot-time-findings-2026-09-07.md). Before these stamps
// the 2.64 s window between the first encoded frame and the radio RX loop
// was unattributed, because maburd's log lines carried no time of their own
// and ssh does not answer until ~7 s of uptime — too late to watch it live.
//
// Scope is deliberately the boot path only. Steady-state warnings are not
// stamped, and stderr as a whole is NOT filtered through a stamping thread:
// that would also catch devourer and the MI blobs, but it can lose the tail
// on the _exit(3) fault path in maburd's venc on_fault callback — which is
// precisely the log you would be reading.
#include <cstddef>
#include <cstdint>

namespace mabur {

// Latches t0 on CLOCK_MONOTONIC. Call once, first thing in main().
// Idempotent: a second call is ignored, so a stray one cannot re-zero a
// timeline that is already being read.
void boot_trace_init();

// Microseconds since boot_trace_init(). 0 if init has not run.
uint64_t boot_trace_elapsed_us();

// Renders `us` as the "+S.mmm" field of a boot line into `out`. Split out
// from bootlog() because it is the only part with arithmetic in it and
// run_real_mode() is not host-runnable (it needs a radio and an encoder).
// Always NUL-terminates when cap > 0; truncates rather than overrunning.
// Returns the number of characters written, excluding the NUL.
size_t boot_trace_fmt(uint64_t us, char* out, size_t cap);

// Writes "[boot +S.mmm] " + the formatted line + "\n" to stderr in ONE
// write, so a line from a concurrently-starting thread cannot land between
// a stamp and its own text. Newline is supplied here; do not pass one.
void bootlog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

}  // namespace mabur
