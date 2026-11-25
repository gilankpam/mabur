#pragma once
// Thread naming/placement helpers and the steady clock shared by maburd.
#include <chrono>
#include <cstdint>
#include <string>

namespace mabur::maburd {

void name_thread(const char* n);

// Core-placement policy (findings §19). The SSC338Q has two
// A7s and the frame producer is the pacer for the whole link: dq_split
// cpu_us is ~9 ms/frame unpinned but ~6.3 ms with the producer alone on a
// core, because every txq.push notify lets the TX writer preempt it
// mid-frame and the FEC worker migrates onto it (sink_us 1.9 -> 0.15 ms,
// GS arrival span 8.6 -> 7.9 ms, air spacing 382 -> 346 us).
//
// Applied as: process affinity -> core 0 BEFORE any thread exists (radio,
// venc SDK, USB pool, FEC worker all inherit it), then the hot thread
// moves itself to core 1. Cost: the FEC worker shares core 0 and slows to
// ~120 us/repair, so the producer's flush-join rises ~1 ms — a net win at
// today's geometry, NOT a free one. Revisit if per-frame repair count grows.
//
// Guarded on exactly-2 online CPUs: that is the deployment target, and it
// keeps a dev host (where this same binary builds and runs) from cramming
// every thread onto CPU 0.
constexpr int kHotCore = 1;
constexpr int kRestCore = 0;

bool two_core_target();
void pin_self_to(int cpu);
int repin_inherited_threads(const char* comm, int cpu);

void touch_update_file(const std::string& dir, const char* name);

inline uint64_t now_steady_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// µs sibling for the dq_split gauge: the intervals it separates (venc-ring
// wait 0–5 ms, FEC/SBI CPU, queue wait) are each of the same order as
// now_steady_ms()'s 1 ms quantum, so a ms clock cannot split them.
inline uint64_t now_steady_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace mabur::maburd
