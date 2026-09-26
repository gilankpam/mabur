#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "mtest.h"
#include "vtx_recorder.h"

using namespace mabur;
namespace fs = std::filesystem;

namespace {

struct FakeChannel : RecordChannel {
  int starts = 0, stops = 0, idrs = 0;
  bool ok = true;
  bool start() override { ++starts; return ok; }
  void stop() override { ++stops; }
  void request_idr() override { ++idrs; }
};

// A fake sysfs / dev / mounts tree under a fresh temp dir. `slot`, `card`,
// `mounted` switch each probe stage on.
struct Tree {
  fs::path base;
  RecorderPaths paths;
  std::string dir;
  explicit Tree(bool slot = true, bool card = true, bool mounted = true) {
    static int n = 0;
    base = fs::temp_directory_path() / ("vtxrec_" + std::to_string(::getpid()) + "_" + std::to_string(n++));
    fs::remove_all(base);
    fs::create_directories(base / "mmc_host");
    fs::create_directories(base / "dev");
    fs::create_directories(base / "card");
    dir = (base / "card").string();
    paths.mmc_host_dir = (base / "mmc_host").string();
    paths.card_dev = (base / "dev" / "mmcblk0p1").string();
    paths.mounts = (base / "mounts").string();
    if (slot) fs::create_directories(base / "mmc_host" / "mmc0");
    if (card) std::ofstream(paths.card_dev) << "";
    std::ofstream m(paths.mounts);
    m << "proc /proc proc rw 0 0\n";
    if (mounted) m << "/dev/mmcblk0p1 " << dir << " vfat rw 0 0\n";
  }
  ~Tree() { fs::remove_all(base); }
  RecordCfg cfg(int min_free_mb = 1) const {
    RecordCfg c;
    c.enable = true;
    c.dir = dir;
    c.min_free_mb = min_free_mb;
    return c;
  }
};

std::vector<uint8_t> cat(std::initializer_list<std::vector<uint8_t>> parts) {
  std::vector<uint8_t> out;
  for (auto& p : parts) out.insert(out.end(), p.begin(), p.end());
  return out;
}
const std::vector<uint8_t> kSc = {0x00, 0x00, 0x00, 0x01};
std::vector<uint8_t> nal(uint8_t type, std::vector<uint8_t> body) {
  return cat({kSc, {static_cast<uint8_t>(type << 1), 0x01}, body});
}
// Key AU with VPS/SPS/PPS so HevcParams completes (SPS layout as in
// tests/test_hevc_params.cpp: byte[2] then a 12-byte profile_tier_level).
std::vector<uint8_t> key_au() {
  return cat({nal(32, {0x0C, 0x0D, 0x0E}),
              nal(33, {0xAB, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0xFF, 0xFF}),
              nal(34, {0xCA, 0xFE}),
              nal(19, {0xAA, 0xBB, 0xCC})});
}
std::vector<uint8_t> p_au() { return nal(1, {0x01, 0x02, 0x03}); }

void feed(VtxRecorder& r, const std::vector<uint8_t>& au, uint32_t pts, bool key) {
  r.on_au(au.data(), au.size(), pts, key);
}

}  // namespace

TEST(rec_status_packing) {
  CHECK(pack_rec_status(RecState::Error, RecErr::LowSpace) == 0x16);
  CHECK(pack_rec_status(RecState::Recording, RecErr::None) == 0x01);
}

TEST(probe_reports_each_missing_stage_without_touching_the_channel) {
  struct Case { bool slot, card, mounted; int min_free; bool enable; RecErr want; };
  const Case cases[] = {
      {true, true, true, 1, false, RecErr::Disabled},
      {false, true, true, 1, true, RecErr::NoSlot},
      {true, false, true, 1, true, RecErr::NoCard},
      {true, true, false, 1, true, RecErr::NotMounted},
      {true, true, true, 1 << 30, true, RecErr::LowSpace},
  };
  for (const Case& c : cases) {
    Tree t(c.slot, c.card, c.mounted);
    FakeChannel ch;
    RecordCfg cfg = t.cfg(c.min_free);
    cfg.enable = c.enable;
    VtxRecorder r(cfg, ch, 1920, 1080, t.paths);
    r.request(true);
    r.service(0);
    CHECK(r.state() == RecState::Error);
    CHECK(r.err() == c.want);
    CHECK(ch.starts == 0);
  }
}

TEST(records_from_first_key_then_stops_on_off_wish) {
  Tree t;
  FakeChannel ch;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths);
  r.request(true);
  r.service(0);
  CHECK(ch.starts == 1);
  CHECK(r.state() == RecState::Off);          // armed: nothing written yet
  feed(r, p_au(), 1000, false);                // before the first IDR: skipped
  feed(r, key_au(), 17667, true);
  for (uint32_t i = 2; i < 50; ++i) feed(r, p_au(), 1000 + i * 16667, false);  // < queue_depth 64
  r.service(0);
  CHECK(r.state() == RecState::Recording);
  CHECK(r.files() == 1);
  const std::string path = r.current_path();
  CHECK(path.find("record-0000.mp4") != std::string::npos);
  r.request(false);
  r.service(0);
  CHECK(ch.stops == 1);
  CHECK(r.state() == RecState::Off);
  CHECK(r.err() == RecErr::None);
  std::ifstream f(path, std::ios::binary);
  char box[8] = {};
  f.read(box, 8);
  CHECK(std::string(box + 4, 4) == "ftyp");
  CHECK(fs::file_size(path) > 100);
}

TEST(queue_overflow_drops_then_resyncs_on_key) {
  Tree t;
  FakeChannel ch;
  RecorderLimits lim;
  lim.queue_depth = 2;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths, lim);
  r.request(true);
  r.service(0);
  feed(r, key_au(), 1000, true);
  feed(r, p_au(), 17667, false);
  feed(r, p_au(), 34334, false);   // queue full: dropped, resync armed
  feed(r, p_au(), 51001, false);   // still waiting for a key: skipped, not a drop
  CHECK(r.dropped() == 1);
  r.service(0);
  CHECK(ch.idrs == 1);             // the worker asked the channel for a fresh IDR
  feed(r, p_au(), 67668, false);   // still skipped
  feed(r, key_au(), 84335, true);  // accepted: resync done
  feed(r, p_au(), 101002, false);
  CHECK(r.dropped() == 1);
  r.service(0);
  CHECK(r.state() == RecState::Recording);
}

TEST(rotation_opens_the_next_file_on_the_next_key) {
  Tree t;
  FakeChannel ch;
  RecorderLimits lim;
  lim.rotate_bytes = 1;            // every written AU crosses the threshold
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths, lim);
  r.request(true);
  r.service(0);
  feed(r, key_au(), 1000, true);
  r.service(0);                    // file 1 opened, written, rotated
  feed(r, p_au(), 17667, false);   // skipped: rotation re-armed need_key
  feed(r, key_au(), 34334, true);
  r.service(0);                    // file 2
  CHECK(r.files() == 2);
  CHECK(fs::exists(t.dir + "/record-0000.mp4"));
  CHECK(fs::exists(t.dir + "/record-0001.mp4"));
  CHECK(ch.idrs >= 1);
}

TEST(write_error_stops_with_write_error) {
  Tree t;
  FakeChannel ch;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths);
  r.mint = [](const std::string&) { return std::string("/dev/full"); };
  r.request(true);
  r.service(0);
  feed(r, key_au(), 1000, true);
  r.service(0);
  CHECK(r.state() == RecState::Error);
  CHECK(r.err() == RecErr::WriteError);
  CHECK(ch.stops == 1);
  feed(r, key_au(), 17667, true);  // after the failure: not accepted, no crash
  r.service(0);
  CHECK(r.files() == 0);
  r.request(false);
  r.service(0);
  CHECK(r.state() == RecState::Off);
  CHECK(r.err() == RecErr::None);
}

TEST(rapid_toggles_coalesce) {
  Tree t;
  FakeChannel ch;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths);
  r.request(true);
  r.request(false);
  r.request(true);
  r.service(0);
  CHECK(ch.starts == 1);
  r.request(false);
  r.request(true);
  r.request(false);
  r.service(0);
  CHECK(ch.stops == 1);
  CHECK(ch.starts == 1);
  CHECK(r.state() == RecState::Off);
}

TEST(refused_start_retries_on_next_on_edge) {
  Tree t(/*slot=*/true, /*card=*/false, /*mounted=*/true);
  FakeChannel ch;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths);
  r.request(true);
  r.service(0);
  CHECK(r.err() == RecErr::NoCard);
  std::ofstream(t.paths.card_dev) << "";   // card inserted
  r.request(true);                          // same wish: no retry
  r.service(0);
  CHECK(ch.starts == 0);
  r.request(false);
  r.service(0);
  r.request(true);                          // off->on: retry
  r.service(0);
  CHECK(ch.starts == 1);
  CHECK(r.err() == RecErr::None);
}

TEST(channel_missing_reports_disabled) {
  Tree t;
  FakeChannel ch;
  ch.ok = false;
  VtxRecorder r(t.cfg(), ch, 1920, 1080, t.paths);
  r.request(true);
  r.service(0);
  CHECK(r.state() == RecState::Error);
  CHECK(r.err() == RecErr::Disabled);
  feed(r, key_au(), 1000, true);           // not accepting: ignored
  r.service(0);
  CHECK(r.files() == 0);
}

MTEST_MAIN
