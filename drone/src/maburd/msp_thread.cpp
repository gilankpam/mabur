// Real mode: the MSP thread and its start/stop (real_mode.h).

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <thread>
#include <vector>

#include <unistd.h>

#include "mabur/msp_status.h"
#include "msp_serial.h"
#include "real_mode.h"
#include "SignalStop.h"

namespace mabur::maburd {

using namespace mabur;

void RealMode::run_msp(std::atomic<bool>& stop, const MspCfg m) {
  name_thread("mbr-msp");
  // Robust control modulation, same tier as DISC_ACK; MSP is a third
  // producer on the mutex-guarded dev_sink.send() path (never the pool).
  ControlRadiotap msp_radiotap = make_control_radiotap();
  uint16_t seq = 0;
  std::random_device rd;
  MspSource src(to_msp_source_cfg(m),
    [&](const uint8_t* body, size_t n) {
      // Quiesce for the calibration session's duration -- the same
      // "non-sweep PPDUs in the characterized airtime" telemetry
      // suppression exists to avoid. Reading/decoding serial bytes
      // continues regardless (src still gets on_serial_bytes() below),
      // only the TRANSMIT is skipped, so the UART side never desyncs or
      // overflows waiting out a session.
      if (cal_active.load(std::memory_order_relaxed)) return;
      const std::vector<uint8_t>& radiotap = msp_radiotap.get();
      const auto frame = build_control_frame(radiotap, seq, body, n);
      seq = static_cast<uint16_t>((seq + 1) & 0xFFF);
      dev_sink.send(frame.data(), frame.size());
    },
    rd());  // random initial_seq (SwEncoder restart-safety contract)
  std::fprintf(stderr,
      "maburd msp: enabled symbol_size=%d window=%d block_payload=%d update_rate_hz=%.2g serial=%s baud=%d\n",
      m.symbol_size, m.window,
      m.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen),
      m.update_rate_hz, m.serial.c_str(), m.baud);

  // Low-power mode: ask the FC for its arm state.
  // MSP_STATUS is request/reply -- the FC pushes only DisplayPort on
  // its own -- so poll at kArmPollMs and hand every decoded reply to
  // RcAgent. Kept polling after the arm latch: 6 B out + ~33 B in per
  // 500 ms is free and the live flag stays observable.
  //
  // COUPLED to low_power.stale_ms, which is operator-configurable while
  // this is not: a report is fresh for stale_ms and arrives at best once
  // per kArmPollMs, so stale_ms <= 500 makes the mode flap once per poll
  // (fresh on the reply tick, stale before the next one). Deliberately
  // NOT validated -- the flap direction is toward FULL power, i.e. the
  // safe side, so a too-small stale_ms costs bitrate, never safety.
  // Keep stale_ms comfortably above this (the bundle ships 2000), and if
  // this poll period ever changes, revisit that margin.
  constexpr uint64_t kArmPollMs = 500;
  std::vector<uint8_t> status_req;
  mabur::msp_append_status_request(status_req);
  uint64_t last_poll_ms = 0;
  // Installed whether or not low power is on: low_power.enable is a live
  // knob, and the poll below is what gates the replies.
  src.set_message_hook([&](const mabur::MspMessage& msg) {
    if (auto armed = mabur::msp_status_armed(msg))
      agent.note_arm_state(*armed, now_steady_ms());
  });

  MspSerial serial;
  uint8_t buf[512];
  while (!stop.load(std::memory_order_relaxed) && !g_devourer_should_stop) {
    if (!serial.is_open()) {
      if (!serial.open(m.serial, m.baud)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      std::fprintf(stderr, "maburd msp: reading %s @ %d\n",
                   m.serial.c_str(), m.baud);
    }
    int n = serial.read(buf, sizeof buf);
    if (n > 0) src.on_serial_bytes(buf, static_cast<size_t>(n), now_steady_ms());
    else if (n < 0) serial.close();  // error -> reconnect

    if (agent.low_power_enabled() && serial.is_open()) {
      uint64_t t = now_steady_ms();
      if (t - last_poll_ms >= kArmPollMs) {
        last_poll_ms = t;
        if (serial.write(status_req.data(), status_req.size()) < 0) serial.close();
      }
    }
  }
  std::fprintf(stderr, "maburd msp: stopped\n");
}

void RealMode::start_msp(const MspCfg& m) {
  if (msp_thread.joinable()) return;
  msp_stop.store(false);
  msp_thread = std::thread([this, m] { run_msp(msp_stop, m); });
}

void RealMode::stop_msp() {
  if (!msp_thread.joinable()) return;
  msp_stop.store(true);
  msp_thread.join();
  msp_thread = std::thread();
}

}  // namespace mabur::maburd
