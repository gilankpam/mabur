// mi_ready.h: the venc bring-up waits for the MI modules load_sigmastar
// inserts, read from a sysfs-shaped directory so the boot overlap (init
// script backgrounds load_sigmastar, maburd starts at once) cannot race
// MI_SYS_Init into a respawn.
#include "mi_ready.h"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "mtest.h"

namespace {

struct TmpRoot {
  std::string path;
  TmpRoot() {
    char tmpl[] = "/tmp/mi_ready_XXXXXX";
    path = mkdtemp(tmpl);
  }
  void module(const std::string& name, const std::string& state) {
    mkdir((path + "/" + name).c_str(), 0755);
    std::ofstream(path + "/" + name + "/initstate") << state << "\n";
  }
  ~TmpRoot() { (void)!std::system(("rm -rf " + path).c_str()); }
};

}  // namespace

TEST(live_when_venc_and_sensor_are_live) {
  TmpRoot r;
  r.module("mi_sys", "live");
  r.module("mi_venc", "live");
  r.module("sensor_imx415_mipi", "live");
  CHECK(mabur::mi_modules_live(r.path));
  auto w = mabur::wait_for_mi_modules(r.path, 1000);
  CHECK(w.ready);
  CHECK(w.waited_ms < 50);
}

TEST(not_live_without_a_sensor_driver) {
  TmpRoot r;
  r.module("mi_venc", "live");
  r.module("sensor_config", "live");  // the detect-time helper, not a driver
  CHECK(!mabur::mi_modules_live(r.path));
}

TEST(not_live_while_venc_is_still_initialising) {
  TmpRoot r;
  r.module("mi_venc", "coming");
  r.module("sensor_imx415_mipi", "live");
  CHECK(!mabur::mi_modules_live(r.path));
}

TEST(not_live_when_the_root_is_missing) {
  CHECK(!mabur::mi_modules_live("/nonexistent/sys/module"));
  auto w = mabur::wait_for_mi_modules("/nonexistent/sys/module", 30, 5);
  CHECK(!w.ready);
  CHECK(w.waited_ms >= 30);
}

TEST(wait_returns_once_the_sensor_driver_goes_live) {
  TmpRoot r;
  r.module("mi_venc", "live");
  std::thread late([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    r.module("sensor_imx415_mipi", "live");
  });
  auto w = mabur::wait_for_mi_modules(r.path, 2000, 5);
  late.join();
  CHECK(w.ready);
  CHECK(w.waited_ms >= 70);
  CHECK(w.waited_ms < 1000);
}

MTEST_MAIN
