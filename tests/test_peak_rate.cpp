#include "../drone/src/peak_rate.h"
#include "mtest.h"

using mabur::PeakRate;

// The agent loop samples the monotonic enc_bytes counter every wake (~5 ms)
// and prints the busiest 100 ms window once per stats second. A flat
// 16 Mbit/s stream must read 16000 kbit/s (decimal kbit, like cmd_kbps),
// not the 1 s average.
TEST(peak_rate_flat_stream_reads_its_rate) {
  PeakRate pk(100);
  uint64_t bytes = 0;
  for (uint64_t now = 0; now <= 1000; now += 5) {
    pk.sample(now, bytes);
    bytes += 10000;  // 10 kB / 5 ms = 16 Mbit/s
  }
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15900 && peak <= 16100);
}

// A single 100 ms burst at twice the rate inside an otherwise flat second
// is what the 1 Hz telemetry average hides and what fills the TxQueue
// (flight-0011): the peak must report the burst window, and take() must
// reset so the next second starts clean.
TEST(peak_rate_reports_the_burst_window_then_resets) {
  PeakRate pk(100);
  uint64_t bytes = 0;
  for (uint64_t now = 0; now <= 1000; now += 5) {
    pk.sample(now, bytes);
    const bool burst = now >= 400 && now < 500;
    bytes += burst ? 20000 : 10000;
  }
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 31500 && peak <= 32500);
  // Nothing closed since the take: reads 0, not the stale burst.
  CHECK(pk.take_peak_kbps() == 0);
  // Continue flat: back to the flat rate, the burst is gone.
  for (uint64_t now = 1005; now <= 2000; now += 5) {
    pk.sample(now, bytes);
    bytes += 10000;
  }
  const uint32_t peak2 = pk.take_peak_kbps();
  CHECK(peak2 >= 15900 && peak2 <= 16100);
}

// Sparse wakes (a stalled loop, or a wake every 250 ms) close a longer
// window and average over its real span — never divide by the nominal 100.
TEST(peak_rate_sparse_samples_use_the_real_span) {
  PeakRate pk(100);
  pk.sample(0, 0);
  pk.sample(250, 500000);  // 500 kB / 250 ms = 16 Mbit/s
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15900 && peak <= 16100);
}

// A counter reset (encoder restart re-seeds enc_bytes at 0) must not print
// a ~2^64 rate: a backwards step re-seeds the window silently.
TEST(peak_rate_counter_going_backwards_reseeds) {
  PeakRate pk(100);
  pk.sample(0, 1000000);
  pk.sample(50, 1010000);
  pk.sample(120, 5000);  // went backwards
  CHECK(pk.take_peak_kbps() == 0);
  pk.sample(230, 5000 + 220000);  // 220 kB / 110 ms = 16 Mbit/s
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15900 && peak <= 16100);
}

MTEST_MAIN
