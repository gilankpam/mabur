// Real mode: the IGI floor watchdog thread body (real_mode.h).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "real_mode.h"
#include "SignalStop.h"

namespace mabur::maburd {

using namespace mabur;

// IGI floor watchdog: every 100 ms, feed the longest video send to the
// state machine and set the radio's floor when it moves. Takes no maburd
// lock -- a sender stalled in the chip holds tx_gate shared (and
// DevourerSink::m on the direct path), and this thread exists to notice
// exactly that. SetWideTxIgiFloor takes only devourer's register lock,
// which no sender holds across a bulk-OUT. Not started when the cap is not
// above the base (watchdog off).
void igi_wd_loop(RealMode& m) {
  name_thread("mbr-igi-wd");
  using Ev = mabur::IgiFloorWatchdog::Event;
  mabur::IgiFloorWatchdog wd(m.igi_params, now_steady_ms());
  uint32_t last_seq = m.retune_seq.load(std::memory_order_relaxed);
  // Frames can back up inside the chip while USB sends still return quickly,
  // so the chip's TX queue fill is a second slow signal. One control read per
  // tick; a radio that cannot report it is read no more.
  bool queue_ok = true;
  // Foreign airtime fills the queue too and a higher floor does not cure it,
  // so a window on a busy channel is not judged. One 240 ms CCX window is
  // armed per second and read at the next; a reading counts for 2 s. This
  // thread is the only one that arms or reads the engine after startup.
  bool busy_ok = true, busy_armed = false;
  int busy_pct = -1;
  uint64_t busy_read_ms = 0;
  while (!g_devourer_should_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const uint32_t seq = m.retune_seq.load(std::memory_order_relaxed);
    const bool eligible = m.device_ready.load(std::memory_order_acquire) &&
                          !m.cal_active.load(std::memory_order_relaxed) &&
                          m.video_bw.load(std::memory_order_relaxed) == 40 && seq == last_seq;
    last_seq = seq;
    // No chip reads while video is not at 40 MHz: the window is not judged.
    uint32_t backlog_pct = 0;
    if (eligible && queue_ok) {
      try {
        uint16_t free_pages = 0, total_pages = 0;
        if (!m.rtl_device->GetTxQueuePages(free_pages, total_pages)) {
          queue_ok = false;
          std::fprintf(stderr, "maburd radio: IGI floor watchdog: no chip queue reading, sends only\n");
        } else if (total_pages > 0) {
          const uint32_t used = total_pages > free_pages ? total_pages - free_pages : 0;
          backlog_pct = std::min<uint32_t>(100, used * 100 / total_pages);
        }
      } catch (...) {
        // A failed USB read counts as an empty queue for this tick; a log line
        // per failure would flood at 10 Hz and the USB fault shows elsewhere.
      }
    }
    const uint64_t now_us = now_steady_us();
    const uint64_t now_ms = now_us / 1000;
    if (!eligible) {
      // A retune spoils the armed window; the next eligible tick re-arms.
      busy_armed = false;
      busy_pct = -1;
      busy_read_ms = 0;
    } else if (busy_ok && now_ms - busy_read_ms >= 1000) {
      busy_read_ms = now_ms;
      try {
        if (busy_armed) {
          const auto cb = m.rtl_device->GetChannelBusy();
          busy_pct = cb.valid && cb.valid_busy ? static_cast<int>(cb.busy_pct) : -1;
        }
        busy_armed = false;
        if (m.rtl_device->ArmChannelBusy(240000) != 0) {
          busy_armed = true;
        } else {
          busy_ok = false;
          busy_pct = -1;
          std::fprintf(stderr, "maburd radio: IGI floor watchdog: no busy reading, busy gate off\n");
        }
      } catch (...) {
        busy_armed = false;
        busy_pct = -1;
      }
    }
    if (busy_pct >= 0 && now_ms - busy_read_ms > 2000) busy_pct = -1;
    const auto r = wd.step(now_ms, eligible, m.send_timer.take_longest_us(now_us), backlog_pct, busy_pct);
    if (r.event == Ev::Raised || r.event == Ev::Reverted) {
      bool ok = true;
      try {
        ok = m.rtl_device->SetWideTxIgiFloor(r.level);
      } catch (...) {
        // The floor is recorded before the register write, so DIG enforces it.
        std::fprintf(stderr, "maburd radio: IGI floor 0x%02x: USB error on the IGI write, DIG applies it\n",
                     static_cast<unsigned>(r.level));
      }
      if (!ok) {
        std::fprintf(stderr, "maburd radio: IGI floor watchdog off (radio has no runtime floor)\n");
        return;
      }
      m.igi_floor_now.store(r.level, std::memory_order_relaxed);
    }
    if (r.event == Ev::Raised)
      std::fprintf(stderr, "maburd radio: IGI floor 0x%02x -> 0x%02x (longest send %u ms, chip queue %u %%, busy %s)\n",
                   static_cast<unsigned>(r.from), static_cast<unsigned>(r.level), r.longest_ms,
                   static_cast<unsigned>(r.backlog_pct),
                   r.busy_pct < 0 ? "?" : (std::to_string(r.busy_pct) + " %").c_str());
    else if (r.event == Ev::Kept)
      std::fprintf(stderr, "maburd radio: IGI floor 0x%02x kept\n", static_cast<unsigned>(r.level));
    else if (r.event == Ev::Reverted)
      std::fprintf(stderr, "maburd radio: IGI floor 0x%02x -> 0x%02x reverted, no help; holding %u s\n",
                   static_cast<unsigned>(r.from), static_cast<unsigned>(r.level), r.hold_s);
  }
}
}  // namespace mabur::maburd
