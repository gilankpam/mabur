#pragma once
// VTX onboard SD recorder (spec docs/superpowers/specs/2026-09-26-vtx-recorder-design.md,
// background docs/sd-record-findings-2026-09-26.md).
//
// Threads:
//   - request() : RcAgent's agent thread (via Actuator::set_record). Never blocks.
//   - on_au()   : the record channel's drain thread (drone/venc/venc_record.c).
//                 Copies the AU into a bounded queue; never blocks on the card.
//   - service() : the worker thread ("mbr-rec"), the ONLY thread that touches
//                 the channel (start/stop/idr), the card and the file. Tests
//                 call service() directly instead of start_thread().
//
// A refused start (no slot/card/mount/space, channel missing) is NOT retried
// on its own: the next off->on wish retries, so a missing card never becomes
// a probe storm. Stops only on a received off-wish, LowSpace or WriteError.
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "mabur/dvr_mux.h"
#include "mabur/dvr_name.h"
#include "mabur/hevc_params.h"

namespace mabur {

enum class RecState : uint8_t { Off = 0, Recording = 1, Error = 2 };
enum class RecErr : uint8_t {
  None = 0, Disabled = 1, NoSlot = 2, NoCard = 3, NotMounted = 4, LowSpace = 5, WriteError = 6,
};

// Telem::rec_status layout: bits 0-1 state, bits 2-7 error code.
constexpr uint8_t pack_rec_status(RecState s, RecErr e) {
  return static_cast<uint8_t>(static_cast<uint8_t>(s) | (static_cast<uint8_t>(e) << 2));
}

// The encoder side (drone/venc/venc_record.c on the drone, a fake in tests).
// Called from the worker thread only.
class RecordChannel {
 public:
  virtual ~RecordChannel() = default;
  virtual bool start() = 0;        // StartRecvPic + IDR; false = no channel
  virtual void stop() = 0;         // synchronous: no on_au() after it returns
  virtual void request_idr() = 0;
};

struct RecorderPaths {
  std::string mmc_host_dir = "/sys/class/mmc_host";  // a slot = at least one host
  std::string card_dev = "/dev/mmcblk0p1";           // a card = its first partition
  std::string mounts = "/proc/mounts";
};

struct RecorderLimits {
  uint64_t rotate_bytes = 3900ull * 1000 * 1000;  // FAT32 caps a file at 4 GiB
  size_t queue_depth = 64;                         // AUs, not bytes: ~1.07 s at 60 fps
  int fragment_ms = 1000;                          // loss bound on power cut
};

class VtxRecorder {
 public:
  VtxRecorder(const RecordCfg& cfg, RecordChannel& ch, int width, int height,
              RecorderPaths paths = {}, RecorderLimits lim = {});
  ~VtxRecorder();
  VtxRecorder(const VtxRecorder&) = delete;
  VtxRecorder& operator=(const VtxRecorder&) = delete;

  void start_thread();   // production: spawn the worker
  void shutdown();       // join the worker, close the file; idempotent

  void request(bool on);
  void on_au(const uint8_t* au, size_t n, uint32_t pts_us, bool key);
  bool service(int wait_ms);   // one worker iteration; true if it did work

  uint8_t status_byte() const { return status_.load(std::memory_order_relaxed); }
  RecState state() const { return static_cast<RecState>(status_byte() & 0x03); }
  RecErr err() const { return static_cast<RecErr>(status_byte() >> 2); }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  uint64_t files() const { return files_; }                  // worker/test thread
  const std::string& current_path() const { return path_; }  // worker/test thread

  // Mints the next file path; default DvrNamer::next(dir) (record-NNNN.mp4).
  std::function<std::string(const std::string& dir)> mint;

 private:
  struct Au {
    std::vector<uint8_t> data;
    uint32_t pts = 0;
    bool key = false;
  };

  RecErr probe() const;
  void do_start();
  void do_stop();
  void fail(RecErr e);
  void write_au(const Au& a);
  void set_status(RecState s, RecErr e) {
    status_.store(pack_rec_status(s, e), std::memory_order_relaxed);
  }

  RecordCfg cfg_;
  RecordChannel& ch_;
  int width_, height_;
  RecorderPaths paths_;
  RecorderLimits lim_;
  DvrNamer namer_;

  std::mutex mx_;
  std::condition_variable cv_;
  // Guarded by mx_:
  std::deque<Au> q_;
  bool desired_ = false;
  uint64_t gen_ = 0, handled_gen_ = 0;
  bool need_key_ = true;     // drop until a key AU: a file never holds an orphan P
  bool idr_wanted_ = false;
  bool accepting_ = false;
  bool quit_ = false;

  // Worker thread only:
  bool running_ = false;
  bool file_open_ = false;
  DvrMux mux_;
  HevcParams params_;
  uint64_t last_frag_ = 0;
  uint64_t files_ = 0;
  std::string path_;

  std::atomic<uint8_t> status_{0};
  std::atomic<uint64_t> dropped_{0};
  std::thread thr_;
  bool shut_ = false;
};

}  // namespace mabur
