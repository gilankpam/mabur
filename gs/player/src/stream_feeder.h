#ifndef MABUR_PLAYER_STREAM_FEEDER_H_
#define MABUR_PLAYER_STREAM_FEEDER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ring_client.h"
#include "stream_decoder.h"

namespace maburplay {

// Why a feeder does not stream (logged at start, oneshot JSON "stream").
enum class StreamOff : uint8_t { kNone = 0, kConfig, kNoDecoder, kProbe };
const char* stream_off_name(StreamOff r);  // "on", "off:config", "off:no_decoder", "off:probe"

// Turns ring v4 events into a streamed decode (spec 2026-10-10-h265-slices
// §6.3). Relies on ring v4's writer contract: for an AU with nslices >= 2,
// the valid bytes of a kOpen/kGrow always end on a NAL boundary (maburgs
// writes the one piece that may not -- the passthrough remainder -- outside
// valid_len). So a VCL NAL is whole as soon as it lies in the valid bytes:
// a split AU is started on its first slice the moment that slice is valid,
// and gets each later slice on the event that makes it valid -- never a
// partial NAL; never more than n-1 slices before the close; the last slice
// (with whatever follows it) at a decodable close, with LAST. Whatever this
// feeder started is finished or ended here and never also submitted whole
// (owns()). Everything else goes the whole-AU path at kClose, unchanged.
// One picture open at a time.
class StreamFeeder {
 public:
  enum class Close : uint8_t {
    kWhole,     // not streamed: the caller takes the whole-AU path
    kStreamed,  // the rest went in with LAST: the picture decodes
    kAborted,   // ended with an empty LAST (or earlier): nothing more to do
  };

  explicit StreamFeeder(bool enable) : enable_(enable) {}

  // Probes `dec` (nullptr = the backend cannot stream). Call after the
  // backend's init and again after any re-creation (on_flush() first).
  void set_decoder(StreamDecoder* dec);
  StreamOff off_reason() const { return off_; }

  // kOpen / kGrow. `armed`: the backend has seen its sync point (only a
  // complete sid-0 AU arms it, at kClose, as before).
  void on_open(const AuEvent& ev, bool armed);
  // This record started streaming (also after it was aborted early): its
  // close belongs here, never to the whole-AU path.
  bool owns(uint64_t rec_no, uint32_t pts_us) const;
  // kClose (aborted ones included). `decodable`: au_decodable(flags).
  Close on_close(const AuEvent& ev, bool decodable);
  // The decoder is about to be flushed, reset or re-created: end any
  // streamed picture first (counted stream_aborted).
  void on_flush();
  void count_whole() { ++whole_submits_; }

  uint64_t streamed() const { return streamed_; }
  uint64_t streamed_salvaged() const { return streamed_salvaged_; }
  uint64_t stream_aborted() const { return stream_aborted_; }
  uint64_t whole_submits() const { return whole_submits_; }

 private:
  struct Cur {
    bool seen = false;       // a kOpen for (rec_no, pts) arrived
    bool eligible = false;   // it may stream
    bool started = false;    // START reached the decoder
    bool ended = false;      // aborted before its close
    uint64_t rec_no = 0;
    uint32_t pts = 0;
    uint8_t n = 0;           // slices announced (MPP_PACKET_STREAM_SLICES)
    uint8_t parts = 0;       // VCL NALs handed to the decoder
    size_t fed = 0;          // bytes handed to the decoder: [0, fed)
    std::vector<size_t> nal; // start-code offsets of every NAL found
    std::vector<size_t> vcl; // of them, the VCL NALs
    size_t scanned = 0;      // bytes below this examined for start codes
  };
  struct Ended {             // the last stream aborted before its close
    bool valid = false;
    uint64_t rec_no = 0;
    uint32_t pts = 0;
  };
  void scan_(const std::vector<uint8_t>& b);
  // End of VCL NAL k: the next NAL's start code, else `valid` (the valid
  // bytes end on a NAL boundary).
  size_t end_of_(size_t k, size_t valid) const;
  void feed_ready_(const std::vector<uint8_t>& b);
  bool finish_(const std::vector<uint8_t>& au);
  void abort_cur_();

  bool enable_;
  StreamDecoder* dec_ = nullptr;
  StreamOff off_ = StreamOff::kNoDecoder;
  Cur cur_;
  Ended ended_;
  uint64_t streamed_ = 0, streamed_salvaged_ = 0, stream_aborted_ = 0, whole_submits_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_STREAM_FEEDER_H_
