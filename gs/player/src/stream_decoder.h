#ifndef MABUR_PLAYER_STREAM_DECODER_H_
#define MABUR_PLAYER_STREAM_DECODER_H_

#include <cstddef>
#include <cstdint>

namespace maburplay {

// A decoder that takes a picture slice by slice while it decodes (rkvdec2
// stream mode through the fpvOS-patched MPP; spec 2026-10-10-h265-slices
// §6.3). MppBackend implements it next to VideoBackend (whose shape stays
// frozen); StreamFeeder drives it; tests use a fake. One thread only: the
// decoder-input thread (main when there is none).
// Kept free of any SDK include: mpp_backend.h pulls it in on every build.
class StreamDecoder {
 public:
  virtual ~StreamDecoder() = default;
  // true iff pictures can be streamed here (MPP_DEC_SET_STREAM_APPEND probe).
  virtual bool probe() = 0;
  // The first part of a picture of `nslices` slices: its first slice and
  // any NAL units before it. false = not taken (nothing reached the decoder).
  virtual bool start(const uint8_t* p, size_t n, uint8_t nslices, uint32_t pts_us) = 0;
  // One more whole slice of the open picture; `last` ends it.
  virtual bool append(const uint8_t* p, size_t n, uint32_t pts_us, bool last) = 0;
  // End the open picture with what it has (an empty LAST append).
  virtual void abort(uint32_t pts_us) = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_STREAM_DECODER_H_
