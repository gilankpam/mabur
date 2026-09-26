#include "vtx_recorder.h"

#include <dirent.h>
#include <pthread.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <utility>

namespace mabur {
namespace {

bool dir_has_entries(const std::string& d) {
  DIR* dp = ::opendir(d.c_str());
  if (!dp) return false;
  bool any = false;
  while (dirent* e = ::readdir(dp)) {
    if (e->d_name[0] == '.') continue;
    any = true;
    break;
  }
  ::closedir(dp);
  return any;
}

bool path_exists(const std::string& p) { return ::access(p.c_str(), F_OK) == 0; }

bool is_mountpoint(const std::string& mounts, const std::string& dir) {
  std::ifstream f(mounts);
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string dev, mnt;
    if (ss >> dev >> mnt && mnt == dir) return true;
  }
  return false;
}

// Free space in MiB; empty when statvfs fails (not evidence of a full card).
std::optional<uint64_t> free_mb(const std::string& dir) {
  struct statvfs v {};
  if (::statvfs(dir.c_str(), &v) != 0) return std::nullopt;
  return static_cast<uint64_t>(v.f_bavail) * static_cast<uint64_t>(v.f_frsize) / (1024 * 1024);
}

}  // namespace

VtxRecorder::VtxRecorder(const RecordCfg& cfg, RecordChannel& ch, int width, int height,
                         RecorderPaths paths, RecorderLimits lim)
    : cfg_(cfg), ch_(ch), width_(width), height_(height), paths_(std::move(paths)), lim_(lim) {
  mint = [this](const std::string& d) { return namer_.next(d); };
  set_status(RecState::Off, RecErr::None);
}

VtxRecorder::~VtxRecorder() { shutdown(); }

void VtxRecorder::start_thread() {
  thr_ = std::thread([this] {
    pthread_setname_np(pthread_self(), "mbr-rec");
    for (;;) {
      {
        std::lock_guard<std::mutex> lk(mx_);
        if (quit_) break;
      }
      service(200);
    }
  });
}

void VtxRecorder::shutdown() {
  if (shut_) return;
  shut_ = true;
  {
    std::lock_guard<std::mutex> lk(mx_);
    quit_ = true;
  }
  cv_.notify_all();
  if (thr_.joinable()) thr_.join();
  if (running_) do_stop();
}

void VtxRecorder::request(bool on) {
  {
    std::lock_guard<std::mutex> lk(mx_);
    if (on == desired_) return;
    desired_ = on;
    ++gen_;
  }
  cv_.notify_all();
}

void VtxRecorder::on_au(const uint8_t* au, size_t n, uint32_t pts_us, bool key, bool prefixed) {
  // An Annex-B AU only reaches here when the encoder gave no usable NAL
  // table (packNum == 0, a slice without a start code), so its key flag may
  // miss an IDR: check the bitstream. Scanned outside mx_ so the drain never
  // holds the writer off for a NAL walk. A prefixed AU's flag came from that
  // table and needs no second look.
  if (!prefixed) key = key || au_is_irap(au, n);
  {
    std::lock_guard<std::mutex> lk(mx_);
    if (!accepting_) return;
    if (need_key_ && !key) return;
    if (q_.size() >= lim_.queue_depth) {
      // Writer behind (flash GC stall): drop, then resync on a fresh IDR so
      // the file never holds a P frame whose reference it lacks.
      dropped_.fetch_add(1, std::memory_order_relaxed);
      need_key_ = true;
      idr_wanted_ = true;
    } else {
      need_key_ = false;
      q_.push_back(Au{std::vector<uint8_t>(au, au + n), pts_us, key, prefixed});
    }
  }
  cv_.notify_all();
}

bool VtxRecorder::service(int wait_ms) {
  std::unique_lock<std::mutex> lk(mx_);
  cv_.wait_for(lk, std::chrono::milliseconds(wait_ms), [&] {
    return quit_ || gen_ != handled_gen_ || idr_wanted_ || !q_.empty();
  });
  if (gen_ != handled_gen_) {
    handled_gen_ = gen_;
    const bool want = desired_;
    lk.unlock();
    if (want) do_start(); else do_stop();
    return true;
  }
  const bool idr = idr_wanted_;
  idr_wanted_ = false;
  std::deque<Au> batch;
  batch.swap(q_);
  lk.unlock();
  if (idr && running_) ch_.request_idr();
  for (Au& a : batch) {
    if (!running_) break;
    write_au(a);
  }
  return idr || !batch.empty();
}

RecErr VtxRecorder::probe() const {
  if (!cfg_.enable) return RecErr::Disabled;
  if (!dir_has_entries(paths_.mmc_host_dir)) return RecErr::NoSlot;
  if (!path_exists(paths_.card_dev)) return RecErr::NoCard;
  if (!is_mountpoint(paths_.mounts, cfg_.dir)) return RecErr::NotMounted;
  const auto mb = free_mb(cfg_.dir);
  if (!mb) return RecErr::NotMounted;
  if (*mb < static_cast<uint64_t>(cfg_.min_free_mb)) return RecErr::LowSpace;
  return RecErr::None;
}

void VtxRecorder::do_start() {
  if (running_) return;
  const RecErr e = probe();
  if (e != RecErr::None) {
    set_status(RecState::Error, e);
    std::fprintf(stderr, "maburd rec: START refused (err %u)\n", static_cast<unsigned>(e));
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mx_);
    q_.clear();
    need_key_ = true;
    idr_wanted_ = false;
    accepting_ = true;
  }
  if (!ch_.start()) {
    {
      std::lock_guard<std::mutex> lk(mx_);
      accepting_ = false;
      q_.clear();
    }
    set_status(RecState::Error, RecErr::Disabled);
    std::fprintf(stderr, "maburd rec: START refused (no record channel)\n");
    return;
  }
  running_ = true;
  file_open_ = false;
  set_status(RecState::Off, RecErr::None);  // Recording once the first file is open
  std::fprintf(stderr, "maburd rec: START armed (%s)\n", cfg_.dir.c_str());
}

void VtxRecorder::do_stop() {
  if (!running_) {
    set_status(RecState::Off, RecErr::None);
    return;
  }
  ch_.stop();  // synchronous: no on_au() after this
  std::deque<Au> rest;
  {
    std::lock_guard<std::mutex> lk(mx_);
    accepting_ = false;
    rest.swap(q_);
  }
  for (Au& a : rest) {
    if (!running_) break;
    write_au(a);
  }
  if (!running_) return;  // write_au() failed: fail() already reported it
  if (file_open_) {
    mux_.close(/*durable=*/true);
    file_open_ = false;
  }
  running_ = false;
  set_status(RecState::Off, RecErr::None);
  std::fprintf(stderr, "maburd rec: STOP (%s) files=%llu dropped=%llu\n", path_.c_str(),
               static_cast<unsigned long long>(files()),
               static_cast<unsigned long long>(dropped()));
}

void VtxRecorder::fail(RecErr e) {
  std::fprintf(stderr, "maburd rec: STOPPED on error %u (%s) files=%llu dropped=%llu\n",
               static_cast<unsigned>(e), path_.c_str(),
               static_cast<unsigned long long>(files()),
               static_cast<unsigned long long>(dropped()));
  ch_.stop();
  {
    std::lock_guard<std::mutex> lk(mx_);
    accepting_ = false;
    q_.clear();
  }
  if (file_open_) {
    // Low space: the card is healthy, so fsync what is there. A write
    // error: the card may be gone, and an fsync could block on it.
    mux_.close(/*durable=*/e == RecErr::LowSpace);
    file_open_ = false;
  }
  running_ = false;
  set_status(RecState::Error, e);
}

void VtxRecorder::write_au(Au& a) {
  if (a.key) {
    if (a.prefixed) params_.feed_prefixed(a.data.data(), a.data.size());
    else params_.feed(a.data.data(), a.data.size());
  }
  if (!file_open_) {
    if (!a.key || !params_.complete()) return;  // a file begins at a key AU with VPS/SPS/PPS
    path_ = mint(cfg_.dir);
    if (!mux_.open(path_, params_.hvcc(), width_, height_, lim_.fragment_ms)) {
      fail(RecErr::WriteError);
      return;
    }
    file_open_ = true;
    last_frag_ = 0;
    ++files_;
    set_status(RecState::Recording, RecErr::None);
    std::fprintf(stderr, "maburd rec: recording -> %s\n", path_.c_str());
  }
  if (a.prefixed) mux_.write_sample_prefixed(std::move(a.data), a.pts, a.key);
  else mux_.write_sample(a.data.data(), a.data.size(), a.pts, a.key);
  if (!mux_.ok()) {
    fail(RecErr::WriteError);
    return;
  }
  if (mux_.fragments() != last_frag_) {
    last_frag_ = mux_.fragments();
    // One fsync per fragment: a power cut loses at most the open fragment.
    if (!mux_.sync()) {
      fail(RecErr::WriteError);
      return;
    }
    const auto mb = free_mb(cfg_.dir);
    if (mb && *mb < static_cast<uint64_t>(cfg_.min_free_mb)) {
      fail(RecErr::LowSpace);
      return;
    }
  }
  if (mux_.bytes_written() >= lim_.rotate_bytes) {
    mux_.close(/*durable=*/true);
    file_open_ = false;
    std::lock_guard<std::mutex> lk(mx_);
    need_key_ = true;     // the next file starts on a fresh IDR
    idr_wanted_ = true;
  }
}

}  // namespace mabur
