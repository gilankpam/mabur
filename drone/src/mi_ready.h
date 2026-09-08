// Readiness of the SigmaStar MI kernel modules maburd's venc dlopens against.
//
// Boot-path context (docs/boot-time-findings-2026-09-07.md, "rcS, stamped"):
// the MI insmod chain (load_sigmastar: mhal, mi_common, mi_sys, ... mi_venc,
// sensor detect, sensor driver) is ~0.7 s on a cold boot, and maburd's own
// first ~1.2 s -- exec + shared-library page-in, the USB port reset,
// CreateRtlDevice -- touch none of it. Letting the init script background
// load_sigmastar and start maburd immediately overlaps the two, but only if
// the venc bring-up waits for the modules instead of failing MI_SYS_Init and
// exiting for a 2 s respawn. This is that wait.
//
// "Live" is read from sysfs, not from a sentinel the script would have to
// write: /sys/module/<name>/initstate is "live" once the module's init has
// returned, and load_sigmastar's last step is the sensor driver insmod, so
// mi_venc live + any sensor_*_mipi live means the whole chain has finished.
// (sensor_config.ko, which the script inserts and removes during detection,
// does not match the sensor_*_mipi glob on purpose.) On a warm restart both
// are already live and the wait costs one directory scan.
#pragma once
#include <dirent.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace mabur {

inline bool module_live(const std::string& sysfs_module_root, const std::string& name) {
  std::string path = sysfs_module_root + "/" + name + "/initstate";
  FILE* f = std::fopen(path.c_str(), "r");
  if (!f) return false;
  char buf[16] = {0};
  size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
  std::fclose(f);
  while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == ' ')) buf[--n] = 0;
  return std::strcmp(buf, "live") == 0;
}

// True when mi_venc is live and at least one sensor_*_mipi driver is live.
inline bool mi_modules_live(const std::string& sysfs_module_root) {
  if (!module_live(sysfs_module_root, "mi_venc")) return false;
  DIR* d = opendir(sysfs_module_root.c_str());
  if (!d) return false;
  bool sensor = false;
  while (dirent* e = readdir(d)) {
    const char* n = e->d_name;
    size_t len = std::strlen(n);
    if (len > 12 && std::strncmp(n, "sensor_", 7) == 0 && std::strcmp(n + len - 5, "_mipi") == 0 &&
        module_live(sysfs_module_root, n)) {
      sensor = true;
      break;
    }
  }
  closedir(d);
  return sensor;
}

struct MiReady {
  bool ready;
  int waited_ms;
};

// Polls until mi_modules_live() or timeout_ms elapses. Never throws; a
// timeout returns ready=false so the caller can proceed and let the MI init
// report the failure the way it always has.
inline MiReady wait_for_mi_modules(const std::string& sysfs_module_root, int timeout_ms,
                                   int poll_ms = 10) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  for (;;) {
    if (mi_modules_live(sysfs_module_root)) {
      return {true, static_cast<int>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t0).count())};
    }
    const int elapsed =
        static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t0).count());
    if (elapsed >= timeout_ms) return {false, elapsed};
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
  }
}

}  // namespace mabur
