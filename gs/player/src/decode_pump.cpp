#include "decode_pump.h"

#include <cstdio>
#include <ctime>
#include <utility>

#include "au_ring.h"  // kRecFlagComplete
#include "poll_wait.h"
#include "worker_thread.h"

namespace maburplay {
namespace {

uint64_t mono_us() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000ull + static_cast<uint64_t>(ts.tv_nsec) / 1000ull;
}

}  // namespace

DecodePump::DecodePump(LatTracker& lat, FrameRegulator& regulator, WakeQueue<HandedFrame>& out,
                       WakeQueue<DmaFrame>& releases)
    : lat_(lat), regulator_(regulator), out_(out), releases_(releases) {}

DecodePump::~DecodePump() { stop_and_join(); }

void DecodePump::set_feed(FeedLoop* f) { feed_ = f; }

VideoBackend::FrameSink DecodePump::sink() {
  return [this](const DmaFrame& f) { on_frame_(f); };
}

void DecodePump::set_backend(VideoBackend* b) {
  // Unconditionally, even for the same pointer: a backend re-created after
  // the old one was destroyed often lands at the old address.
  std::vector<DmaFrame> stale;
  discarded_.fetch_add(releases_.drain(&stale));
  backend_ = b;
  auto* pw = dynamic_cast<PollWait*>(b);
  blocking_poll_ = pw && pw->set_poll_timeout_ms(kPollTimeoutMs);
  if (pw && !blocking_poll_)
    std::fprintf(stderr,
                 "maburplay: decode pump: backend refused a blocking poll timeout, "
                 "polling every 1 ms\n");
}

void DecodePump::set_prime(std::function<void(const DmaFrame&)> fn) { prime_ = std::move(fn); }

void DecodePump::set_pts_sink(std::function<void(uint32_t)> fn) { pts_sink_ = std::move(fn); }

double DecodePump::source_hz() const {
  std::lock_guard<std::mutex> lk(src_mu_);
  return source_rate_.hz();
}

void DecodePump::start() { th_ = start_worker_thread([this] { run_(); }); }

void DecodePump::drain_submit_notes_() {
  if (!feed_) return;
  notes_.clear();
  feed_->take_submit_notes(&notes_);
  for (const FeedNote& n : notes_) {
    lat_.on_submit(n.meta, n.t_us);
    // A salvaged (incomplete) AU is a loss event: its delivery time stays
    // out of the capture-time scheduler's lateness window.
    if (!(n.meta.flags & maburgs::kRecFlagComplete)) regulator_.note_incomplete(n.meta.pts_us);
  }
}

void DecodePump::on_frame_(const DmaFrame& f) {
  drain_submit_notes_();
  const uint64_t now = mono_us();
  lat_.on_decoded(f.pts_us, now);
  {
    std::lock_guard<std::mutex> lk(src_mu_);
    source_rate_.on_pts(f.pts_us, now);
  }
  last_pts_.store(f.pts_us);
  last_frame_us_.store(now);
  video_w_.store(f.width);
  video_h_.store(f.height);
  have_frame_.store(true);
  frames_.fetch_add(1);  // last: a reader that sees the count sees the rest
  out_.push(HandedFrame{f, now});
  if (prime_) prime_(f);
  if (pts_sink_) pts_sink_(f.pts_us);
}

void DecodePump::drain_releases_() {
  rel_.clear();
  if (releases_.drain(&rel_) == 0) return;
  if (!backend_) {
    discarded_.fetch_add(rel_.size());
    return;
  }
  for (const DmaFrame& f : rel_) backend_->release_frame(f);
}

bool DecodePump::checkpoint_() {
  if (!park_req_.load()) return !stop_.load();
  // Before reporting parked: everything queued so far goes back to the
  // backend it belongs to, so a set_backend() while parked finds only what
  // was queued after the park.
  drain_releases_();
  std::unique_lock<std::mutex> lk(mu_);
  if (!park_req_.load()) return !stop_.load();
  parked_ = true;
  resume_ = false;
  cv_.notify_all();
  cv_.wait(lk, [this] { return resume_ || stop_.load(); });
  // Only now, under mu_, is the thread out of the park: until here park()
  // reads parked_ && resume_ as "not parked" (a resumed thread may already
  // be heading into the backend).
  parked_ = false;
  resume_ = false;
  return !stop_.load();
}

void DecodePump::run_() {
  while (!stop_.load()) {
    if (!checkpoint_()) break;
    drain_releases_();
    bool got = false;
    if (backend_) {
      const uint64_t before = frames_.load();
      backend_->poll();
      got = frames_.load() != before;
    }
    // Notes for AUs still in the decoder (or for a decoder that emits
    // nothing): consumed now, so the feed's queue never waits on a frame.
    drain_submit_notes_();
    if (!got && !(backend_ && blocking_poll_)) {
      // Nothing came out and poll() did not wait for it: a short wait so the
      // thread does not spin, ended early by a park or a stop.
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait_for(lk, std::chrono::milliseconds(1),
                   [this] { return park_req_.load() || stop_.load(); });
    }
  }
  std::lock_guard<std::mutex> lk(mu_);
  finished_.store(true);
  cv_.notify_all();  // a park() waiting sees the exit
}

bool DecodePump::park(std::chrono::milliseconds limit) {
  std::unique_lock<std::mutex> lk(mu_);
  park_req_.store(true);
  cv_.notify_all();  // ends an idle wait
  const bool ok = cv_.wait_for(lk, limit, [this] {
    return (parked_ && !resume_) || finished_.load() || !th_.joinable();
  });
  if (!ok) park_req_.store(false);
  return ok;
}

void DecodePump::resume() {
  std::lock_guard<std::mutex> lk(mu_);
  park_req_.store(false);
  resume_ = true;
  cv_.notify_all();
}

void DecodePump::stop_and_join() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    stop_.store(true);
    cv_.notify_all();
  }
  if (th_.joinable()) th_.join();
}

}  // namespace maburplay
