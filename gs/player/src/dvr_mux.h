#ifndef MABUR_PLAYER_DVR_MUX_H_
#define MABUR_PLAYER_DVR_MUX_H_

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace maburplay {

// Fragmented MP4 (init segment + moof/mdat fragments). One video track,
// hvc1, timescale 1'000'000 (pts_us native). No B-frames in this encoder
// config (SVC-T TRAIL only): decode order == presentation order, no cts.
//
// Pure serialization: never reads the wall clock. Fragment cuts are
// decided from sample pts deltas alone, which keeps the muxer
// deterministic and testable without sleeps.
class DvrMux {
 public:
  // hvcc: from HevcParams::hvcc(). width/height: from config/mode
  // (1920x1080). fragment_ms: soft cap on fragment duration measured
  // against sample pts (not wall time); a key AU always cuts too.
  bool open(const std::string& path, const std::vector<uint8_t>& hvcc,
            int width, int height, int fragment_ms = 1000);

  // au: Annex-B bytes (converted internally via annexb_to_length_prefixed).
  // pts_us: 32-bit capture stamp, unwrapped internally to 64-bit monotonic.
  // key: AU is IRAP. Fragments are cut at each key AU or when fragment_ms
  // elapsed since the fragment's first sample, whichever first; a fragment
  // is flushed to disk whole (moof+mdat in one write) so a crash loses at
  // most the open fragment.
  void write_sample(const uint8_t* au, size_t n, uint32_t pts_us, bool key);

  void close();  // flushes the open fragment

  uint64_t samples() const { return samples_; }
  uint64_t fragments() const { return fragments_; }

 private:
  struct Sample {
    std::vector<uint8_t> data;  // length-prefixed NALs (annexb_to_length_prefixed)
    uint64_t pts64 = 0;
    bool key = false;
  };

  uint64_t unwrap_pts(uint32_t pts_us);
  void flush_fragment();

  FILE* f_ = nullptr;
  int width_ = 0;
  int height_ = 0;
  std::vector<uint8_t> hvcc_;
  int fragment_ms_ = 1000;

  uint64_t samples_ = 0;
  uint64_t fragments_ = 0;

  bool have_pts_ = false;
  uint32_t last_pts_raw_ = 0;
  uint64_t last_pts64_ = 0;

  std::vector<Sample> pending_;
  uint64_t fragment_start_pts_ = 0;

  // Running estimate of the inter-sample interval, in us, carried across
  // fragment boundaries. A sample's trun duration must never be 0 — some
  // players compute playback rate from it — so the last sample of any
  // fragment (including a lone-sample fragment, e.g. back-to-back IDRs
  // or close() right after a fragment-cutting key) falls back to this
  // instead of 0. Seeded to the 60 fps nominal frame interval (same
  // convention as RtpPacketizerCfg::nominal_frame_us) and updated
  // whenever a real delta is computed from two consecutive samples.
  uint32_t last_dur_us_ = 16667;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DVR_MUX_H_
