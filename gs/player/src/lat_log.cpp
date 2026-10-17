#include "lat_log.h"

#include <fstream>

namespace maburplay {

namespace {
constexpr uint64_t kRecheckUs = 5'000'000;  // a session rotation (drone restart) lands here within 5 s

std::string read_marker(const char* path) {
  std::ifstream f(path);
  if (!f.good()) return {};
  std::string s;
  std::getline(f, s);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
    s.pop_back();
  return s;
}
}  // namespace

LatLog::LatLog(const char* marker_path, const char* file_name, const char* header,
               bool batched)
    : marker_(marker_path), file_name_(file_name), header_(header),
      buf_(batched ? new char[kBatchBytes] : nullptr), th_([this] { run_(); }) {}

LatLog::~LatLog() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    stop_ = true;
  }
  work_cv_.notify_one();
  th_.join();
  if (f_) std::fclose(f_);
}

void LatLog::push_(Rec r, size_t bytes) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (bytes > 0 && q_bytes_ + bytes > kQueueBytes) {
      // Full: drop the row. One kDrop record marks the gap however many
      // rows it spans; it costs no queue bytes, so it always fits.
      dropped_.fetch_add(1, std::memory_order_relaxed);
      if (!q_.empty() && q_.back().kind == Rec::kDrop) {
        ++q_.back().n;
        return;
      }
      r = Rec{Rec::kDrop, 0, 1, {}};
      bytes = 0;
    }
    q_bytes_ += bytes;
    q_.push_back(std::move(r));
    ++pushed_;
  }
  work_cv_.notify_one();
}

void LatLog::write(uint64_t mono_us, const char* payload) {
  std::string text(payload);
  const size_t bytes = text.size() + 24;  // + stamp, space, newline
  push_(Rec{Rec::kLine, mono_us, 0, std::move(text)}, bytes);
}

void LatLog::flush() { push_(Rec{Rec::kFlush, 0, 0, {}}, 0); }

void LatLog::sync() {
  std::unique_lock<std::mutex> lk(mu_);
  const uint64_t target = pushed_;
  done_cv_.wait(lk, [&] { return done_ >= target; });
}

std::string LatLog::path() const {
  std::lock_guard<std::mutex> lk(mu_);
  return path_;
}

void LatLog::run_() {
  std::deque<Rec> batch;
  for (;;) {
    {
      std::unique_lock<std::mutex> lk(mu_);
      work_cv_.wait(lk, [&] { return stop_ || !q_.empty(); });
      if (q_.empty()) return;  // stop_ and nothing left to land
      batch.swap(q_);
      q_bytes_ = 0;
    }
    // The disk work happens here, outside the lock: a stall holds up only
    // this thread, and the caller keeps queueing (or dropping) meanwhile.
    for (const Rec& r : batch) handle_(r);
    const uint64_t n = batch.size();
    batch.clear();
    {
      std::lock_guard<std::mutex> lk(mu_);
      done_ += n;
    }
    done_cv_.notify_all();
  }
}

void LatLog::handle_(const Rec& r) {
  switch (r.kind) {
    case Rec::kLine:
      // Gate on elapsed stamp time alone -- not on whether f_ is currently
      // open -- so a missing or unusable session backs off for kRecheckUs
      // instead of re-attempting read_marker()/fopen() on every row. A stamp
      // behind the last check (frame.log's rows carry their flip time) has
      // not elapsed anything.
      if (!checked_ ||
          (r.mono_us >= last_check_us_ && r.mono_us - last_check_us_ >= kRecheckUs))
        reopen_(r.mono_us);
      if (f_)
        std::fprintf(f_, "%llu %s\n", static_cast<unsigned long long>(r.mono_us),
                     r.text.c_str());
      break;
    case Rec::kFlush:
      if (f_) std::fflush(f_);
      break;
    case Rec::kDrop:
      if (f_)
        std::fprintf(f_, "# dropped %llu\n", static_cast<unsigned long long>(r.n));
      break;
  }
}

void LatLog::reopen_(uint64_t mono_us) {
  last_check_us_ = mono_us;
  checked_ = true;
  const std::string dir = read_marker(marker_);
  if (dir == dir_ && f_) return;  // same session, already open
  if (dir.empty()) {
    if (f_) {
      std::fclose(f_);
      f_ = nullptr;
      dir_.clear();
      std::lock_guard<std::mutex> lk(mu_);
      path_.clear();
    }
    return;
  }
  const std::string p = dir + "/" + file_name_;
  std::FILE* f = std::fopen(p.c_str(), "a");
  // A new session that can't be opened leaves any still-open previous
  // session's handle alone (deliberate: keep logging into the old
  // directory rather than go dark over a bad new target).
  if (!f) return;
  // The old session closes (flushing whatever it holds) before the new one
  // is set up: a batched instance hands the new stream the same buffer.
  if (f_) std::fclose(f_);
  if (buf_)
    setvbuf(f, buf_.get(), _IOFBF, kBatchBytes);
  else
    setvbuf(f, nullptr, _IOLBF, 0);
  std::fprintf(f, "%s\n", header_);
  f_ = f;
  dir_ = dir;
  std::lock_guard<std::mutex> lk(mu_);
  path_ = p;
}

}  // namespace maburplay
