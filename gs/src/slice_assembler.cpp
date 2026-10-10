#include "slice_assembler.h"

#include <algorithm>
#include <optional>

#include "mabur/hevc_skip_slice.h"
#include "mabur/hevc_slice.h"

namespace maburgs {
namespace {

using mabur::hevc::SliceHeader;
using mabur::hevc::SliceParse;

struct Code { size_t pos, len; };

constexpr uint32_t kCtbLog2 = 6;          // FrameHdr.slice_rows unit: 64-px CTU rows
constexpr uint32_t kMaxSlices = 64;
constexpr size_t kReserveCap = 1u << 20;  // prefix buffer pre-size, bytes

void find_codes(const uint8_t* b, size_t n, size_t from, std::vector<Code>* out) {
  for (size_t j = from; j + 3 <= n; ++j) {
    if (b[j] != 0 || b[j + 1] != 0) continue;
    if (b[j + 2] == 1) { out->push_back({j, 3}); j += 2; continue; }
    if (b[j + 2] == 0 && j + 4 <= n && b[j + 3] == 1) { out->push_back({j, 4}); j += 3; }
  }
}

}  // namespace

SliceAssembler::SliceAssembler(const mabur::hevc::Sps& sps, const mabur::hevc::Pps& pps,
                               uint8_t slice_rows, uint16_t count, size_t hdr_len)
    : sps_(sps), pps_(pps), slice_rows_(slice_rows), count_(count), hdr_len_(hdr_len) {
  // slice_rows counts 64-px CTU rows (FrameHdr): the count means nothing at
  // another CTB size, and plan() refuses that geometry outright. Computed
  // wide and range-checked before narrowing: a wrapped count could look sane.
  if (sps_.log2_ctb != kCtbLog2 || slice_rows_ == 0) return;
  const uint32_t rows = sps_.pic_h_ctbs();
  const uint32_t n = (rows + slice_rows_ - 1) / slice_rows_;
  n_ = n <= kMaxSlices ? static_cast<uint8_t>(n) : 0;
}

void SliceAssembler::extend_prefix(const ChunkMap& chunks) {
  for (;;) {
    const auto it = chunks.find(prefix_chunks_);
    if (it == chunks.end()) break;
    const auto& c = it->second;
    if (prefix_chunks_ == 0) {
      frag_ = c.size();
      buf_.reserve(std::min(static_cast<size_t>(count_) * frag_, kReserveCap));
      if (c.size() > hdr_len_) buf_.insert(buf_.end(), c.begin() + static_cast<long>(hdr_len_), c.end());
    } else {
      buf_.insert(buf_.end(), c.begin(), c.end());
    }
    ++prefix_chunks_;
  }
}

void SliceAssembler::scan_prefix() {
  std::vector<Code> found;
  find_codes(buf_.data(), buf_.size(), scanned_, &found);
  // The rescan window (last 3 bytes) can sit inside a 4-byte code the
  // previous scan already found whole -- its "00 00 01" tail would read as
  // a second, 3-byte code one byte later. Codes never overlap.
  for (const Code& c : found)
    if (sc_.empty() || c.pos >= sc_.back().pos + sc_.back().len) sc_.push_back({c.pos, c.len});
  scanned_ = buf_.size() >= 3 ? buf_.size() - 3 : 0;
}

void SliceAssembler::drain(const ChunkMap& chunks, const ByteSink& out) {
  extend_prefix(chunks);
  scan_prefix();
  if (!sc_.empty() && sc_.back().pos > emitted_) {
    out(buf_.data() + emitted_, sc_.back().pos - emitted_);
    emitted_ = sc_.back().pos;
  }
}

void SliceAssembler::finish(const ChunkMap& chunks, const ByteSink& out, const ByteSink& raw_out) {
  extend_prefix(chunks);
  if (prefix_chunks_ == count_) {             // complete
    out(buf_.data() + emitted_, buf_.size() - emitted_);
    emitted_ = buf_.size();
    result_.slices = n_;
    result_.kept = n_;
    return;
  }
  std::vector<std::vector<uint8_t>> pieces;
  SliceSalvage res;
  const SliceFallback fb = plan(chunks, &pieces, &res);
  if (fb != kSliceFbNone) {
    result_.fallback = fb;
    (raw_out ? raw_out : out)(buf_.data() + emitted_, buf_.size() - emitted_);
    emitted_ = buf_.size();
    return;
  }
  for (const auto& p : pieces) out(p.data(), p.size());
  result_ = res;
}

SliceFallback SliceAssembler::plan(const ChunkMap& chunks,
                                   std::vector<std::vector<uint8_t>>* pieces, SliceSalvage* res) {
  if (sps_.log2_ctb != kCtbLog2) return kSliceFbUnsupported;   // slice_rows is in 64-px rows
  if (n_ == 0 || frag_ <= hdr_len_) return kSliceFbGeometry;
  // Every prefix chunk is a non-last fragment (the AU is incomplete): F each.
  if (buf_.size() != static_cast<size_t>(prefix_chunks_) * frag_ - hdr_len_) return kSliceFbGeometry;
  // 1. Runs of consecutive fragments as AU byte ranges; run 0 is the prefix.
  struct Run { size_t off; std::vector<uint8_t> b; };
  std::vector<Run> runs;
  runs.push_back({0, buf_});
  for (uint16_t i = static_cast<uint16_t>(prefix_chunks_ + 1); i < count_;) {
    if (!chunks.count(i)) { ++i; continue; }
    Run r;
    r.off = static_cast<size_t>(i) * frag_ - hdr_len_;
    for (; i < count_ && chunks.count(i); ++i) {
      const auto& c = chunks.at(i);
      if (i + 1 < count_ && c.size() != frag_) return kSliceFbGeometry;
      r.b.insert(r.b.end(), c.begin(), c.end());
    }
    runs.push_back(std::move(r));
  }
  const bool have_last = chunks.count(static_cast<uint16_t>(count_ - 1)) != 0;
  const size_t au_end = have_last ? static_cast<size_t>(count_ - 1) * frag_ - hdr_len_ +
                                        chunks.at(static_cast<uint16_t>(count_ - 1)).size()
                                  : SIZE_MAX;

  // 2. Complete NAL units of every run.
  struct Slice { std::vector<uint8_t> bytes; SliceHeader h; size_t off; };
  std::vector<std::optional<Slice>> slices(n_);
  std::vector<std::pair<size_t, std::vector<uint8_t>>> prefix_nonvcl;  // AU offset, bytes
  const uint32_t span = static_cast<uint32_t>(slice_rows_) * sps_.pic_w_ctbs();
  bool seen_vcl = false;
  for (size_t r = 0; r < runs.size(); ++r) {
    std::vector<Code> codes;
    find_codes(runs[r].b.data(), runs[r].b.size(), 0, &codes);
    const bool reaches_end = have_last && runs[r].off + runs[r].b.size() == au_end;
    for (size_t k = 0; k < codes.size(); ++k) {
      const size_t s = codes[k].pos, body = s + codes[k].len;
      size_t e;
      if (k + 1 < codes.size()) e = codes[k + 1].pos;
      else if (reaches_end) e = runs[r].b.size();
      else break;  // runs into a hole: incomplete
      // A 3-byte code opening a run after a hole may be the tail of a
      // 4-byte one whose first zero was lost: not the original bytes.
      if (r > 0 && s == 0 && codes[k].len == 3) continue;
      if (body + 2 > e) continue;
      const uint8_t* nal = runs[r].b.data() + body;
      const uint8_t type = static_cast<uint8_t>((nal[0] >> 1) & 0x3F);
      std::vector<uint8_t> bytes(runs[r].b.begin() + static_cast<long>(s),
                                 runs[r].b.begin() + static_cast<long>(e));
      if (type >= 32) {
        if (r == 0 && !seen_vcl) prefix_nonvcl.emplace_back(s, std::move(bytes));
        continue;
      }
      seen_vcl = true;
      SliceHeader h;
      const SliceParse st = mabur::hevc::parse_slice_header(nal, e - body, sps_, pps_, &h);
      if (st == SliceParse::kUnsupported || st == SliceParse::kDependent) return kSliceFbUnsupported;
      if (st != SliceParse::kOk) return kSliceFbGeometry;
      const uint32_t addr = h.first ? 0 : h.address;
      if (h.first != (addr == 0) || addr % span != 0 || addr / span >= n_ || slices[addr / span])
        return kSliceFbGeometry;
      slices[addr / span] = Slice{std::move(bytes), std::move(h), runs[r].off + s};
    }
  }

  // 3. Template and rules.
  const SliceHeader* tmpl = nullptr;
  for (const auto& s : slices)
    if (s) { tmpl = &s->h; break; }
  if (!tmpl) return kSliceFbNoTemplate;
  if (tmpl->slice_type == 2) return kSliceFbISlice;

  // 4. Slices already streamed by drain(): the first `done` must be 0..done-1.
  uint32_t done = 0;
  for (const Sc& c : sc_) {
    if (c.pos >= emitted_) break;
    if (c.pos + c.len < buf_.size() && ((buf_[c.pos + c.len] >> 1) & 0x3F) < 32) ++done;
  }
  for (uint32_t k = 0; k < done; ++k)
    if (!slices[k] || slices[k]->off >= emitted_) return kSliceFbGeometry;

  uint32_t first_missing = n_;
  for (uint32_t k = 0; k < n_; ++k)
    if (!slices[k]) { first_missing = k; break; }
  if (done == 0)
    for (auto& [off, bytes] : prefix_nonvcl)
      if (off >= emitted_) pieces->push_back(std::move(bytes));
  const uint32_t total = sps_.pic_w_ctbs() * sps_.pic_h_ctbs();
  for (uint32_t k = done; k < n_; ++k) {
    if (slices[k]) {
      pieces->push_back(std::move(slices[k]->bytes));
      continue;
    }
    auto fill = mabur::hevc::make_skip_slice(sps_, pps_, *tmpl, k * span, std::min((k + 1) * span, total));
    if (!fill) return kSliceFbGeometry;
    pieces->push_back(std::move(*fill));
  }
  res->salvaged = true;
  res->slices = n_;
  for (uint32_t k = 0; k < n_; ++k) {
    if (!slices[k]) continue;
    ++res->kept;
    if (k > first_missing) ++res->kept_after_hole;
  }
  res->filled = static_cast<uint8_t>(n_ - res->kept);
  return kSliceFbNone;
}

}  // namespace maburgs
