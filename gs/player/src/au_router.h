#ifndef MABUR_PLAYER_AU_ROUTER_H_
#define MABUR_PLAYER_AU_ROUTER_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "au_ring.h"
#include "ring_client.h"
#include "stream_feeder.h"
#include "video_backend.h"

namespace maburplay {

// What the decoder-input side tells the rest of the player about one closed
// record (design-reader-thread.md). kSubmit leaves BEFORE the decoder sees the
// AU, so the latency tracker knows the pts before its frame can exist;
// kDelivery after, with the AU's bytes moved in (raw DVR) -- MPP copied what it
// took (put_packet's copy path, STREAM_APPEND's memcpy).
struct FeedNote {
  enum class Kind : uint8_t { kSubmit, kDelivery };
  Kind kind = Kind::kDelivery;
  maburgs::AuRecordMeta meta{};
  uint64_t t_us = 0;        // kSubmit: submit time; kDelivery: delivery time (monotonic µs)
  uint32_t bytes = 0;       // kDelivery: AU size (au may be dropped under backpressure)
  bool complete = false;    // kDelivery
  bool decodable = false;   // kDelivery
  std::vector<uint8_t> au;  // kDelivery
};

// maburplay's ring sink: kOpen/kGrow to StreamFeeder; at kClose the flush
// barrier, the decodable gate, sid-0 arming and the whole/streamed submit.
// Everything that is not decoder input leaves as FeedNotes. One thread only
// (the feed thread, or main when there is none).
class AuRouter {
 public:
  struct Hooks {
    std::function<void(FeedNote&&)> note;
    // Before a flush_before record: held frames released, feeder.on_flush(),
    // backend->flush(), disarm(). false = stopping: drop the record.
    std::function<bool()> flush;
    std::function<uint64_t()> now_us;
  };
  AuRouter(StreamFeeder& feeder, std::unique_ptr<VideoBackend>& backend);
  void set_hooks(Hooks h) { hooks_ = std::move(h); }
  void on_event(AuEvent&& ev);

  void disarm() { armed_ = false; }
  bool armed() const { return armed_; }
  // Shutdown: nothing new reaches the decoder (no whole submit, no START);
  // the picture already streaming still finishes.
  void set_draining();

  uint64_t submits() const { return submits_.load(std::memory_order_relaxed); }
  uint64_t truncated_skipped() const { return truncated_.load(std::memory_order_relaxed); }
  bool synced() const { return t_sync_us_ != 0; }
  uint64_t t_sync_us() const { return t_sync_us_; }

 private:
  void note_submit_(const maburgs::AuRecordMeta& m);
  void deliver_(AuEvent& ev, bool complete, bool decodable, uint64_t t_deliver);

  StreamFeeder& feeder_;
  std::unique_ptr<VideoBackend>& backend_;  // re-created by the watchdog: never cached
  Hooks hooks_;
  bool armed_ = false;
  bool draining_ = false;
  uint64_t t_sync_us_ = 0;
  std::atomic<uint64_t> submits_{0};
  std::atomic<uint64_t> truncated_{0};
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_AU_ROUTER_H_
