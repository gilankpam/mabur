#include "frame_stream.h"

#include <cstdio>

namespace maburgs {

void FrameStream::push_fragment(uint8_t sid, const uint8_t* pkt, size_t len,
                                uint64_t now_ms, const FragArrival& arr) {
  if (!pkt || len < 6) { ++bad_frags_; return; }
  // Stall watchdog: frames have been arriving for stall_reset_ms with nothing
  // emitted — the emit cursor is wedged above the incoming id64 space (e.g. a
  // producer restart whose discont signal never landed). A receiver must not
  // sit discarding a healthy stream; re-base from scratch.
  if (stall_armed_ && cfg_.stall_reset_ms &&
      now_ms - stall_arm_ms_ >= cfg_.stall_reset_ms) {
    if (last_stall_log_ms_ == 0 ||
        now_ms - last_stall_log_ms_ >= 5000) {  // one line per onset, not per frame
      last_stall_log_ms_ = now_ms;
      std::fprintf(stderr,
                   "maburgs: frame stall: no emit for %llu ms with frames "
                   "arriving (next_emit=%llu last_id64=%llu discont_seen=%d) "
                   "-> reset\n",
                   static_cast<unsigned long long>(now_ms - stall_arm_ms_),
                   static_cast<unsigned long long>(next_emit_id64_),
                   static_cast<unsigned long long>(last_id64_),
                   discont_seen_since_emit_ ? 1 : 0);
    }
    ++stall_resets_;
    reset();
  }
  uint16_t fseq = static_cast<uint16_t>(pkt[0] | (pkt[1] << 8));
  uint16_t idx = static_cast<uint16_t>(pkt[2] | (pkt[3] << 8));
  uint16_t count = static_cast<uint16_t>(pkt[4] | (pkt[5] << 8));
  if (count == 0 || idx >= count) { ++bad_frags_; return; }
  uint32_t key = (static_cast<uint32_t>(sid) << 16) | fseq;
  Slot& s = slots_[key];
  if (s.chunks.empty()) { s.sid = sid; s.fseq = fseq; s.first_ms = now_ms;
                          s.last_progress_ms = now_ms; s.count = count; }
  s.chunks[idx].assign(pkt + 6, pkt + len);
  if (arr.have_sw_seq && (!s.have_seq_at_max || idx > s.max_idx)) {
    s.max_idx = idx;
    s.seq_at_max = arr.sw_seq;
    s.have_seq_at_max = true;
  }
  s.last_arrival_ms = now_ms;
  if (arr.body_mono_us &&
      (s.lat.t_first_us == 0 || arr.body_mono_us < s.lat.t_first_us))
    s.lat.t_first_us = arr.body_mono_us;
  if (arr.body_mono_us > s.lat.t_last_arr_us)
    s.lat.t_last_arr_us = arr.body_mono_us;
  if (idx == 0 && !s.have_hdr) {
    auto h = mabur::framewire::parse_frame_hdr(s.chunks[0].data(), s.chunks[0].size());
    if (!h) { ++bad_frags_; slots_.erase(key); return; }
    s.have_hdr = true;
    s.lat.drone_q_ms = arr.q_ms;
    s.lat.enc_us = arr.enc_us;
    s.lat.drone_air_ms = arr.air_ms;
    s.lat.hdr_retx = arr.retx;
    s.hdr = *h;
    bool rebased = false;
    s.id64 = unwrap_id(h->frame_id, h->flags, &rebased);
    s.discont = rebased;
    if (rebased) params_.reset();  // producer restart may change resolution
    if ((h->flags & mabur::framewire::kFlagDiscont) != 0)
      discont_seen_since_emit_ = true;
    if (have_next_emit_ && !stall_armed_) {
      stall_armed_ = true;
      stall_arm_ms_ = now_ms;
    }
  }
  try_emit(now_ms);
}

uint64_t FrameStream::unwrap_id(uint16_t id, uint8_t flags, bool* rebased) {
  bool discont = (flags & mabur::framewire::kFlagDiscont) != 0;
  if (!have_id_base_ || (discont && !in_discont_run_)) {
    // First frame, or producer restart: re-base far above anything emitted
    // so ordering never waits on pre-discontinuity ids. The drone flags every
    // frame of its ~1 s discont window, so only the first flagged frame of a
    // run re-bases — the rest delta-track below to keep in-window ordering.
    have_id_base_ = true;
    uint64_t base = have_next_emit_ ? (next_emit_id64_ + 0x20000) : 0x10000;
    base &= ~static_cast<uint64_t>(0xFFFF);  // low16(base)=0 so low16(last_id64_)=id
    last_id64_ = base + id;
    in_discont_run_ = discont;
    *rebased = true;
    return last_id64_;
  }
  in_discont_run_ = discont;
  int16_t d = static_cast<int16_t>(id - static_cast<uint16_t>(last_id64_));
  last_id64_ = static_cast<uint64_t>(static_cast<int64_t>(last_id64_) + d);
  return last_id64_;
}

void FrameStream::try_emit(uint64_t now_ms) {
  for (;;) {
    // Evict late arrivals: a known-id slot behind the emit cursor decoded
    // after we already advanced past it — never emitted (cold-start emits
    // the first-known head immediately for zero start latency, so a frame
    // decoding entirely after its successor is late by definition).
    for (auto it = slots_.begin(); it != slots_.end();) {
      if (it->second.have_hdr && have_next_emit_ &&
          it->second.id64 < next_emit_id64_ && !it->second.began) {
        ++dropped_;
        it = slots_.erase(it);
      } else {
        ++it;
      }
    }
    // Head-of-line: known-header slot with the lowest id64.
    Slot* head = nullptr;
    uint64_t max_known = 0;
    for (auto& [k, s] : slots_) {
      if (!s.have_hdr) continue;
      if (s.id64 > max_known) max_known = s.id64;
      if (!head || s.id64 < head->id64) head = &s;
    }
    if (!head) return;
    if (have_next_emit_ && head->id64 > next_emit_id64_) {
      if (head->discont) {
        // Producer restart: the re-based id64 is a synthetic jump, not lost
        // frames. Advance the cursor without booking it as dropped.
        next_emit_id64_ = head->id64;
      } else {
        // Gap of whole frames before head. Skip only when the gap frame is
        // stale (timeout) or the pipeline has run ahead (lookahead). The
        // missing frame's sid is unknowable, so wait the slower stream out.
        bool stale = now_ms >= head->first_ms + gap_ms_max();
        bool ahead = max_known >= next_emit_id64_ + static_cast<uint64_t>(cfg_.lookahead);
        if (!stale && !ahead) return;
        dropped_ += head->id64 - next_emit_id64_;
        next_emit_id64_ = head->id64;
      }
    }
    if (!have_next_emit_) { have_next_emit_ = true; next_emit_id64_ = head->id64; }

    if (!head->began) {
      head->began = true;
      cb_.begin_frame(head->hdr, head->sid);
      if (head->hdr.slice_rows > 0 && params_.usable())
        head->sa.emplace(params_.sps(), params_.pps(), head->hdr.slice_rows, head->count,
                         mabur::framewire::kFrameHdrLen);
    }
    // Stream the contiguous chunk prefix (fragment 0 minus the FrameHdr) --
    // through the slice assembler for a split AU, raw otherwise.
    while (true) {
      auto it = head->chunks.find(head->emitted_upto);
      if (it == head->chunks.end()) break;
      if (!head->sa) {
        const auto& c = it->second;
        size_t skip = head->emitted_upto == 0 ? mabur::framewire::kFrameHdrLen : 0;
        if (c.size() > skip) cb_.frame_data(c.data() + skip, c.size() - skip);
      }
      ++head->emitted_upto;
      head->last_progress_ms = now_ms;
    }
    if (head->sa) head->sa->drain(head->chunks, cb_.frame_data);
    if (head->emitted_upto == head->count) { finish(*head, true); continue; }
    // Mid-frame gap: give repairs gap_timeout_ms to fill it; force-advance
    // if the stream has run lookahead frames ahead.
    bool stale = now_ms >= head->last_progress_ms + gap_ms(head->sid);
    bool ahead = max_known >= head->id64 + static_cast<uint64_t>(cfg_.lookahead);
    if (stale || ahead) { finish(*head, false); continue; }
    return;
  }
}

void FrameStream::finish(Slot& s, bool complete) {
  if (s.sa) {
    s.sa->finish(s.chunks, cb_.frame_data);
    s.lat.slice = s.sa->result();
  } else if (!complete && s.hdr.slice_rows > 0) {
    s.lat.slice.fallback = params_.unsupported() ? kSliceFbUnsupported : kSliceFbNoParams;
  }
  if (complete) feed_params(s);
  if (s.lat.slice.salvaged) {
    ++slice_salvaged_;
    slices_kept_ += s.lat.slice.kept;
    slices_filled_ += s.lat.slice.filled;
    slices_after_hole_ += s.lat.slice.kept_after_hole;
  } else if (!complete && s.lat.slice.fallback < kSliceFbCount && s.lat.slice.fallback != kSliceFbNone) {
    ++slice_fallback_[s.lat.slice.fallback];
  }
  cb_.end_frame(complete, s.lat);
  complete ? ++clean_ : ++truncated_;
  next_emit_id64_ = s.id64 + 1;
  stall_armed_ = false;
  discont_seen_since_emit_ = false;
  slots_.erase((static_cast<uint32_t>(s.sid) << 16) | s.fseq);
}

// A complete AU that opens with a parameter set (refresh start, IDR) feeds
// the slice-salvage tracker: one copy per such AU, 2 Hz at GOP 0.5 s.
void FrameStream::feed_params(const Slot& s) {
  const auto c0 = s.chunks.find(0);
  if (c0 == s.chunks.end()) return;
  const auto& b = c0->second;
  const size_t h = mabur::framewire::kFrameHdrLen;
  size_t p = h;
  if (b.size() >= h + 5 && b[h] == 0 && b[h + 1] == 0 && b[h + 2] == 0 && b[h + 3] == 1) p = h + 4;
  else if (b.size() >= h + 4 && b[h] == 0 && b[h + 1] == 0 && b[h + 2] == 1) p = h + 3;
  else return;
  const uint8_t type = static_cast<uint8_t>((b[p] >> 1) & 0x3F);
  if (type < 32 || type > 34) return;
  std::vector<uint8_t> au(b.begin() + static_cast<long>(h), b.end());
  for (uint16_t i = 1; i < s.count; ++i) {
    const auto it = s.chunks.find(i);
    if (it == s.chunks.end()) return;
    au.insert(au.end(), it->second.begin(), it->second.end());
  }
  params_.feed(au.data(), au.size());
}

void FrameStream::poll(uint64_t now_ms) {
  // Age out slots that never got fragment 0 (can't be ordered or begun).
  for (auto it = slots_.begin(); it != slots_.end();) {
    if (!it->second.have_hdr &&
        now_ms >= it->second.first_ms + gap_ms(it->second.sid)) {
      ++dropped_;
      it = slots_.erase(it);
    } else {
      ++it;
    }
  }
  try_emit(now_ms);
}

std::optional<TailView> FrameStream::tail_view(uint8_t sid) const {
  const Slot* best = nullptr;
  for (const auto& [k, s] : slots_)
    if (s.sid == sid && s.have_hdr && s.have_seq_at_max && s.emitted_upto < s.count &&
        (!best || s.id64 > best->id64))
      best = &s;
  if (!best) return std::nullopt;
  return TailView{best->count, best->max_idx, best->seq_at_max, best->last_arrival_ms, best->first_ms};
}

void FrameStream::reset() {
  // Close any in-flight frame at the packetizer with a truncated end so its
  // in-flight FU doesn't dangle across the session/format-flip boundary.
  for (auto& [k, s] : slots_)
    if (s.began) cb_.end_frame(false, s.lat);
  slots_.clear();
  have_id_base_ = false;
  last_id64_ = 0;
  next_emit_id64_ = 0;
  have_next_emit_ = false;
  in_discont_run_ = false;
  stall_armed_ = false;
  discont_seen_since_emit_ = false;
  params_.reset();
}

}  // namespace maburgs
