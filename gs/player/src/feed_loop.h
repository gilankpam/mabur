#ifndef MABUR_PLAYER_FEED_LOOP_H_
#define MABUR_PLAYER_FEED_LOOP_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "au_router.h"
#include "ring_client.h"
#include "stream_feeder.h"

namespace maburplay {

// The decoder-input thread (design-reader-thread.md). It blocks on the AU
// doorbell, reads the ring and feeds the decoder through an AuRouter, so a
// slice reaches MPP within a doorbell wake of landing whatever the render
// loop is doing. Crossing to the main thread:
//  - FeedNotes (lat submit stamps, DVR bytes, delivery figures): a queue;
//  - flushes: the feed PARKS (park_for_flush) and the main thread flushes
//    while it is parked -- only it can release the frames the presenter,
//    regulator and recorder hold, which must precede mpi->reset;
//  - the watchdog: the main thread parks the feed (park) to flush/re-create.
// Parked = between two ring events, outside every decoder call: the main
// thread then owns the decoder's input side (MPP put/append/reset/destroy
// are not safe concurrently with each other; see the design §2).
class FeedLoop {
 public:
  struct Cfg {
    int pump_ms = 2;                 // ring wait ceiling; the doorbell wakes earlier
    uint32_t drain_budget_ms = 100;  // shutdown: how long a streaming picture's close may take
    size_t max_notes = 256;          // beyond: a note keeps its meta, loses its bytes
  };
  FeedLoop(RingClient& ring, AuRouter& router, StreamFeeder& feeder, Cfg cfg);
  ~FeedLoop();  // stops and joins, even a parked thread
  FeedLoop(const FeedLoop&) = delete;
  FeedLoop& operator=(const FeedLoop&) = delete;

  // Before start(): fn runs every every_ms ON THE FEED THREAD (it may read
  // feed-owned state: ring, feeder, router).
  void set_periodic(std::function<void()> fn, uint32_t every_ms);
  void start();

  // --- main thread ---
  void wait_main(int timeout_ms);              // ends early when a flush waits or the feed ends
  void take_notes(std::vector<FeedNote>* out); // appends, oldest first
  bool flush_waiting() const;
  bool park(std::chrono::milliseconds limit);  // false = not parked in time (nothing changed)
  void resume();                               // ends a park, flush or main
  void begin_drain();                          // shutdown: finish the streaming picture, then exit
  bool finished() const { return finished_.load(); }
  bool drained() const { return drained_.load(); }        // the drain ran to its end
  DrainResult drain_result() const { return drain_result_; }  // after stop_and_join()
  void stop_and_join();
  bool dead() const { return dead_.load(); }
  const std::atomic<bool>& cancel_flag() const { return cancel_; }
  uint64_t parks() const { return parks_.load(); }
  uint64_t notes_dropped() const { return notes_dropped_.load(); }
  void request_debug_line() { debug_req_.store(true); }   // fps-log STALL ring dump

  // --- feed thread (AuRouter hooks, the ring sink) ---
  void push_note(FeedNote&& n);
  bool park_for_flush();  // false = stopping
  bool checkpoint();      // parks iff the main thread asked; false = stopping

 private:
  enum class Park : uint8_t { kNone, kFlush, kMain };
  void run_();
  bool park_locked_(std::unique_lock<std::mutex>& lk, Park why);

  RingClient& ring_;
  AuRouter& router_;
  StreamFeeder& feeder_;
  Cfg cfg_;
  int feed_efd_ = -1;  // main -> feed: ends a pump wait
  int main_efd_ = -1;  // feed -> main: ends wait_main
  std::function<void()> periodic_;
  uint32_t every_ms_ = 5000;
  std::thread th_;

  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<FeedNote> notes_;
  Park parked_ = Park::kNone;
  bool resume_ = false;
  DrainResult drain_result_;

  std::atomic<bool> park_req_{false}, stop_{false}, drain_req_{false}, finished_{false},
      drained_{false}, dead_{false}, cancel_{false}, debug_req_{false};
  std::atomic<uint64_t> parks_{0}, notes_dropped_{0};
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FEED_LOOP_H_
