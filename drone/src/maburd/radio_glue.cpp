#include "radio_glue.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#include "mabur/rc_proto.h"

namespace mabur::maburd {

using namespace mabur;

// A GS at a different RC_VERSION is refused by rc::frame_type(), so its RCFs
// never reach the agent -- and, because the uplink RSSI/SNR EMAs are fed
// inside that same accepted-frame branch, drone.uplink.snr_* goes stale too.
// From the drone's side that is indistinguishable from "no GS is talking",
// and from the GS's side it looks like the stale-caps restart deadlock, which
// sends the operator to `restart maburd` -- which cannot help. So say it out
// loud, but rarely: a mismatched peer transmits continuously and /tmp is
// tmpfs, hence the once-per-5 s gate.
// Not thread-safe; only rx_callback (the RX thread) calls it.
void log_foreign_rc_version(uint8_t peer_ver) {
  using clock = std::chrono::steady_clock;
  static clock::time_point last{};
  const auto now = clock::now();
  if (last.time_since_epoch().count() != 0 &&
      now - last < std::chrono::seconds(5))
    return;
  last = now;
  std::fprintf(stderr,
               "maburd: heard an RC frame at RC_VERSION %u but this build "
               "speaks %u -- ignoring it (rate-limited to 1/5s). The pair is "
               "half-deployed: there is no control link and no video in "
               "either direction. Finish the deploy on BOTH ends; restarting "
               "maburd will not help.\n",
               static_cast<unsigned>(peer_ver),
               static_cast<unsigned>(rc::RC_VERSION));
}

MspSourceCfg to_msp_source_cfg(const MspCfg& m) {
  MspSourceCfg c;
  c.update_rate_hz = m.update_rate_hz;
  c.symbol_size = m.symbol_size;
  c.window = m.window;
  c.overhead = m.overhead;
  return c;
}

uint16_t open_usb_and_get_pid(uint16_t vid, uint16_t configured_pid,
                              libusb_context* ctx, libusb_device_handle** out_handle) {
  std::vector<uint16_t> pids;
  if (configured_pid != 0) {
    pids.push_back(configured_pid);
  } else {
    // Scan order: 8812EU, then the Jaguar3 8822C dies (RTL8812CU c812,
    // RTL8822CU c82c -- the EMAX Wyvern Link board), then 8812AU. Which of
    // these the binary can actually drive is devourer's build-time chip
    // selection; an unbuilt chip fails at CreateRadio, not here.
    pids = {0xa81a, 0xc812, 0xc82c, 0x881a, 0x8812};
  }
  for (uint16_t pid : pids) {
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (h) {
      *out_handle = h;
      return pid;
    }
  }
  *out_handle = nullptr;
  return 0;
}

}  // namespace mabur::maburd
