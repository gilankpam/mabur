#include "feed_loop.h"

#include <poll.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <ctime>
#include <utility>

namespace maburplay {
namespace {

uint64_t now_ms() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000ull + static_cast<uint64_t>(ts.tv_nsec) / 1000000ull;
}
void poke(int fd) {
  if (fd < 0) return;
  const uint64_t one = 1;
  (void)!::write(fd, &one, sizeof one);
}
void clear(int fd) {
  uint64_t v;
  if (fd >= 0) (void)!::read(fd, &v, sizeof v);  // EFD_NONBLOCK: one read empties it
}

}  // namespace

FeedLoop::FeedLoop(RingClient& ring, AuRouter& router, StreamFeeder& feeder, Cfg cfg)
    : ring_(ring), router_(router), feeder_(feeder), cfg_(cfg),
      feed_efd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)),
      main_efd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)) {
  ring_.set_wake_fd(feed_efd_);
}

FeedLoop::~FeedLoop() {
  stop_and_join();
  ring_.set_wake_fd(-1);
  if (feed_efd_ >= 0) ::close(feed_efd_);
  if (main_efd_ >= 0) ::close(main_efd_);
}

void FeedLoop::set_periodic(std::function<void()> fn, uint32_t every_ms) {
  periodic_ = std::move(fn);
  every_ms_ = every_ms;
}

void FeedLoop::start() { th_ = std::thread([this] { run_(); }); }

void FeedLoop::run_() {
  // Signals are the main thread's: SIGTERM must end ITS wait.
  sigset_t s;
  sigemptyset(&s);
  sigaddset(&s, SIGINT);
  sigaddset(&s, SIGTERM);
  sigaddset(&s, SIGUSR1);
  pthread_sigmask(SIG_BLOCK, &s, nullptr);
  uint64_t next = now_ms() + every_ms_;
  while (!stop_.load()) {
    if (!checkpoint()) break;
    if (drain_req_.load()) {
      router_.set_draining();
      drain_result_ = drain_stream(
          feeder_, [this](int ms) { ring_.pump(ms); clear(feed_efd_); }, now_ms,
          cfg_.drain_budget_ms);
      drained_.store(true);
      break;
    }
    ring_.pump(cfg_.pump_ms);
    clear(feed_efd_);
    if (ring_.dead()) {
      dead_.store(true);
      break;
    }
    if (debug_req_.exchange(false))
      std::fprintf(stderr, "fps-log: STALL %s trunc_skip=%llu\n", ring_.debug_line().c_str(),
                   static_cast<unsigned long long>(router_.truncated_skipped()));
    if (periodic_ && now_ms() >= next) {
      periodic_();
      next = now_ms() + every_ms_;
    }
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    finished_.store(true);
    cv_.notify_all();  // a main thread waiting in park() sees the exit
  }
  poke(main_efd_);
}

void FeedLoop::wait_main(int timeout_ms) {
  pollfd p{main_efd_, POLLIN, 0};
  if (::poll(&p, 1, timeout_ms) > 0) clear(main_efd_);
}

void FeedLoop::take_notes(std::vector<FeedNote>* out) {
  std::lock_guard<std::mutex> lk(mu_);
  for (FeedNote& n : notes_) out->push_back(std::move(n));
  notes_.clear();
}

void FeedLoop::push_note(FeedNote&& n) {
  std::lock_guard<std::mutex> lk(mu_);
  if (notes_.size() >= cfg_.max_notes && !n.au.empty()) {
    std::vector<uint8_t>().swap(n.au);  // main is stalled: keep the record, not its bytes
    notes_dropped_.fetch_add(1);
  }
  notes_.push_back(std::move(n));
}

bool FeedLoop::flush_waiting() const {
  std::lock_guard<std::mutex> lk(mu_);
  // Resumed but not yet out (parked_ is the feed's to clear): that flush is
  // already served -- serving it again would race the feed's next decoder call.
  return parked_ == Park::kFlush && !resume_;
}

bool FeedLoop::park_locked_(std::unique_lock<std::mutex>& lk, Park why) {
  parked_ = why;
  resume_ = false;
  parks_.fetch_add(1);
  cv_.notify_all();
  cv_.wait(lk, [this] { return resume_ || stop_.load(); });
  // Out of the park, under mu_: only now is parked_ kNone. Between resume()
  // and here, parked_ still names the old park with resume_ set -- park() and
  // flush_waiting() read that pair as "not parked". Clearing resume_ here (and
  // at entry, against a stale resume()) lets the next park hold.
  parked_ = Park::kNone;
  resume_ = false;
  return !stop_.load();
}

bool FeedLoop::checkpoint() {
  if (!park_req_.load()) return !stop_.load();
  std::unique_lock<std::mutex> lk(mu_);
  if (!park_req_.load()) return !stop_.load();
  return park_locked_(lk, Park::kMain);
}

bool FeedLoop::park_for_flush() {
  std::unique_lock<std::mutex> lk(mu_);
  poke(main_efd_);  // main reads flush_waiting() under mu_: it sees kFlush once we wait
  // On true the router submits the flush record itself. If a watchdog park()
  // follows the resume() at once, cancel_ is already set again and that
  // submit may give way (BUFFER_FULL) and be lost -- fine: the watchdog
  // flushes and disarms anyway, and the next sync point re-arms.
  return park_locked_(lk, Park::kFlush);
}

bool FeedLoop::park(std::chrono::milliseconds limit) {
  std::unique_lock<std::mutex> lk(mu_);
  park_req_.store(true);
  cancel_.store(true);  // a BUFFER_FULL retry on the feed gives way at once
  poke(feed_efd_);      // and a pump wait ends
  // Parked = parked_ set AND not merely resumed: right after resume() the
  // feed has not left its old park yet (parked_ is stale until it runs), and
  // it makes decoder calls on its way out -- design §4 rule 1.
  const bool ok = cv_.wait_for(lk, limit, [this] {
    return (parked_ != Park::kNone && !resume_) || finished_.load();
  });
  if (!ok) {
    park_req_.store(false);
    cancel_.store(false);
  }
  return ok;
}

void FeedLoop::resume() {
  std::lock_guard<std::mutex> lk(mu_);
  park_req_.store(false);
  cancel_.store(false);
  resume_ = true;
  cv_.notify_all();
}

void FeedLoop::begin_drain() {
  drain_req_.store(true);
  poke(feed_efd_);
}

void FeedLoop::stop_and_join() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    stop_.store(true);
    cancel_.store(true);
    cv_.notify_all();
  }
  poke(feed_efd_);
  if (th_.joinable()) th_.join();
}

}  // namespace maburplay
