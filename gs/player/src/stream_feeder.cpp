#include "stream_feeder.h"

#include <algorithm>

#include "au_ring.h"
#include "mabur/frame_wire.h"

namespace maburplay {

const char* stream_off_name(StreamOff r) {
  switch (r) {
    case StreamOff::kNone: return "on";
    case StreamOff::kConfig: return "off:config";
    case StreamOff::kNoDecoder: return "off:no_decoder";
    case StreamOff::kProbe: return "off:probe";
  }
  return "off";
}

void StreamFeeder::set_decoder(StreamDecoder* dec) {
  dec_ = dec;
  if (!enable_) off_ = StreamOff::kConfig;
  else if (!dec_) off_ = StreamOff::kNoDecoder;
  else off_ = dec_->probe() ? StreamOff::kNone : StreamOff::kProbe;
  cur_ = Cur{};
  // ended_ is NOT reset here: Task 9 rebuilds the decoder after a watchdog
  // fires by calling on_flush() (which aborts any open picture and records
  // it in ended_) and then set_decoder(new). Clearing ended_ here would
  // make that picture's later kClose report kWhole instead of kAborted --
  // a picture that streamed and was aborted must never also go whole.
}

// NAL start-code offsets in b, incrementally (VCL ones also in vcl). A
// start code counts once its first NAL header byte is there (it names the
// type); a 4-byte code starts at its leading zero.
void StreamFeeder::scan_(const std::vector<uint8_t>& b) {
  const size_t n = b.size();
  size_t j = cur_.scanned;
  while (j + 3 < n) {
    if (b[j] == 0 && b[j + 1] == 0 && b[j + 2] == 1) {
      const size_t pos = j > 0 && b[j - 1] == 0 ? j - 1 : j;
      cur_.nal.push_back(pos);
      if (((b[j + 3] >> 1) & 0x3F) < 32) cur_.vcl.push_back(pos);
      j += 3;
    } else {
      ++j;
    }
  }
  cur_.scanned = j;
}

size_t StreamFeeder::end_of_(size_t k, size_t valid) const {
  const auto it = std::upper_bound(cur_.nal.begin(), cur_.nal.end(), cur_.vcl[k]);
  return it == cur_.nal.end() ? valid : *it;
}

void StreamFeeder::on_open(const AuEvent& ev, bool armed) {
  const bool same = cur_.seen && cur_.rec_no == ev.meta.rec_no && cur_.pts == ev.meta.pts_us;
  if (ev.kind == AuEventKind::kOpen && !same) {
    // One picture open at a time: a new record while the last streamed one
    // never closed (its close was lost) ends that one first. (A repeated
    // kOpen of the same record is just growth: never a second START.)
    if (cur_.started && !cur_.ended) abort_cur_();
    cur_ = Cur{};
    cur_.seen = true;
    cur_.rec_no = ev.meta.rec_no;
    cur_.pts = ev.meta.pts_us;
    cur_.n = ev.meta.nslices;
    cur_.eligible = off_ == StreamOff::kNone && !stop_new_ && armed && ev.meta.nslices >= 2 &&
                    !ev.flush_before &&
                    (ev.meta.flags & mabur::framewire::kFlagDiscont) == 0;
  } else if (!same) {
    return;  // growth of a record never seen open: nothing to stream
  }
  if (!cur_.eligible || cur_.ended) return;
  feed_ready_(ev.au);
}

void StreamFeeder::feed_ready_(const std::vector<uint8_t>& b) {
  scan_(b);
  if (!cur_.started) {
    if (cur_.vcl.empty()) return;  // only leading non-VCL NALs so far
    const size_t e0 = end_of_(0, b.size());
    if (!dec_->start(b.data(), e0, cur_.n, cur_.pts)) {
      cur_.eligible = false;  // never reached the decoder: it goes whole at the close
      return;
    }
    cur_.started = true;
    cur_.parts = 1;
    cur_.fed = e0;
  }
  // Ring v4 writer contract: the valid bytes end on a NAL boundary, so every
  // VCL NAL in them is whole. Hand each over at once; the picture's last
  // slice waits for the close (it alone carries LAST).
  while (cur_.parts < cur_.vcl.size() && cur_.parts + 1u < cur_.n) {
    const size_t e = end_of_(cur_.parts, b.size());
    if (!dec_->append(b.data() + cur_.fed, e - cur_.fed, cur_.pts, false)) {
      abort_cur_();
      return;
    }
    cur_.fed = e;
    ++cur_.parts;
  }
}

bool StreamFeeder::owns(uint64_t rec_no, uint32_t pts_us) const {
  return (cur_.started && cur_.rec_no == rec_no && cur_.pts == pts_us) ||
         (ended_.valid && ended_.rec_no == rec_no && ended_.pts == pts_us);
}

bool StreamFeeder::finish_(const std::vector<uint8_t>& au) {
  // The writer contract says the closed record extends what was fed. These
  // bytes come from another process's shm, though: a close shorter than the
  // fed prefix would underflow e - fed below into a ~4 GB read. Abort instead.
  if (au.size() < cur_.fed) return false;
  scan_(au);
  if (cur_.vcl.size() != cur_.n || cur_.parts == 0 || cur_.parts >= cur_.n) return false;
  for (size_t k = cur_.parts; k < cur_.n; ++k) {
    const bool last = k + 1 == cur_.n;
    const size_t e = last ? au.size() : end_of_(k, au.size());
    if (!dec_->append(au.data() + cur_.fed, e - cur_.fed, cur_.pts, last)) return false;
    cur_.fed = e;
    ++cur_.parts;
  }
  return true;
}

StreamFeeder::Close StreamFeeder::on_close(const AuEvent& ev, bool decodable) {
  const uint64_t rec = ev.meta.rec_no;
  const uint32_t pts = ev.meta.pts_us;
  if (ended_.valid && ended_.rec_no == rec && ended_.pts == pts) {
    // Aborted before its close (lost close, flush, refused append): done.
    ended_ = Ended{};
    if (cur_.seen && cur_.rec_no == rec && cur_.pts == pts) cur_ = Cur{};
    return Close::kAborted;
  }
  const bool mine = cur_.seen && cur_.rec_no == rec && cur_.pts == pts;
  if (!mine || !cur_.started) {
    if (mine) cur_ = Cur{};
    return Close::kWhole;
  }
  Close res = Close::kAborted;
  if (decodable && !ev.aborted && finish_(ev.au)) {
    res = Close::kStreamed;
    ++streamed_;
    if (ev.meta.flags & maburgs::kRecFlagSliceSalvaged) ++streamed_salvaged_;
  } else {
    // Passthrough, aborted, mis-counted or refused: one empty LAST, and not
    // a byte of the closed record past what was already fed (a passthrough
    // record ends mid-NAL beyond the NAL-aligned valid_len).
    abort_cur_();
  }
  ended_ = Ended{};
  cur_ = Cur{};
  return res;
}

void StreamFeeder::on_flush() {
  if (cur_.started && !cur_.ended) abort_cur_();
  // A picture seen open but not started yet (only leading non-VCL NALs so
  // far) must not START on the decoder just flushed: it goes whole at close.
  cur_.eligible = false;
}

void StreamFeeder::abort_cur_() {
  dec_->abort(cur_.pts);
  ++stream_aborted_;
  cur_.ended = true;
  ended_ = Ended{true, cur_.rec_no, cur_.pts};
}

const char* drain_name(DrainResult::Kind k) {
  switch (k) {
    case DrainResult::kIdle: return "stream idle";
    case DrainResult::kFinished: return "stream finished";
    case DrainResult::kAborted: return "stream aborted";
  }
  return "stream ?";
}

DrainResult drain_stream(StreamFeeder& f, const std::function<void(int)>& pump,
                         const std::function<uint64_t()>& now_ms, uint32_t budget_ms) {
  f.stop_new();
  DrainResult r;
  if (!f.streaming()) return r;
  r.pts = f.streaming_pts();
  const uint64_t aborted0 = f.stream_aborted();
  const uint64_t t0 = now_ms();
  while (f.streaming() && now_ms() - t0 < budget_ms) pump(1);
  r.waited_ms = static_cast<uint32_t>(now_ms() - t0);
  if (f.streaming()) {
    f.on_flush();  // its close never came: one empty LAST
    r.kind = DrainResult::kAborted;
  } else {
    r.kind = f.stream_aborted() != aborted0 ? DrainResult::kAborted : DrainResult::kFinished;
  }
  return r;
}

}  // namespace maburplay
