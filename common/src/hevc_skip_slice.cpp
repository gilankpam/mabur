#include "mabur/hevc_skip_slice.h"

#include "mabur/hevc_cabac.h"

namespace mabur::hevc {
namespace {

// Per-min-CB state of the coding units the fill has coded so far.
class Grid {
 public:
  Grid(const Sps& sps, uint32_t first_ctb)
      : w_(sps.width), h_(sps.height), log2_min_(sps.log2_min_cb), log2_ctb_(sps.log2_ctb),
        wctb_(sps.pic_w_ctbs()), first_ctb_(first_ctb),
        cols_(sps.width >> sps.log2_min_cb), rows_(sps.height >> sps.log2_min_cb),
        depth_(cols_ * rows_, 0), skip_(cols_ * rows_, 0), coded_(cols_ * rows_, 0) {}

  bool available(int x, int y) const {
    if (x < 0 || y < 0 || x >= static_cast<int>(w_) || y >= static_cast<int>(h_)) return false;
    const uint32_t ctb = (static_cast<uint32_t>(y) >> log2_ctb_) * wctb_ + (static_cast<uint32_t>(x) >> log2_ctb_);
    return ctb >= first_ctb_ && coded_[idx(x, y)];
  }
  int depth_at(int x, int y) const { return depth_[idx(x, y)]; }
  int skip_at(int x, int y) const { return skip_[idx(x, y)]; }
  void mark(int x0, int y0, int size, int depth) {
    for (int y = y0; y < y0 + size && y < static_cast<int>(h_); y += 1 << log2_min_)
      for (int x = x0; x < x0 + size && x < static_cast<int>(w_); x += 1 << log2_min_) {
        depth_[idx(x, y)] = static_cast<uint8_t>(depth);
        skip_[idx(x, y)] = 1;
        coded_[idx(x, y)] = 1;
      }
  }
  uint32_t w() const { return w_; }
  uint32_t h() const { return h_; }
  uint32_t log2_min() const { return log2_min_; }

 private:
  size_t idx(int x, int y) const {
    return (static_cast<size_t>(y) >> log2_min_) * cols_ + (static_cast<size_t>(x) >> log2_min_);
  }
  uint32_t w_, h_, log2_min_, log2_ctb_, wctb_, first_ctb_;
  size_t cols_, rows_;
  std::vector<uint8_t> depth_, skip_, coded_;
};

void walk(Grid& g, uint32_t max_merge, int x0, int y0, int log2, int depth,
          std::vector<FillStep>* out) {
  const int size = 1 << log2;
  bool split;
  if (x0 + size <= static_cast<int>(g.w()) && y0 + size <= static_cast<int>(g.h()) &&
      log2 > static_cast<int>(g.log2_min())) {
    const int inc = (g.available(x0 - 1, y0) && g.depth_at(x0 - 1, y0) > depth) +
                    (g.available(x0, y0 - 1) && g.depth_at(x0, y0 - 1) > depth);
    out->push_back({FillBin::kSplit, static_cast<uint8_t>(inc), 0});
    split = false;
  } else {
    split = log2 > static_cast<int>(g.log2_min());
  }
  if (split) {
    const int half = size >> 1;
    walk(g, max_merge, x0, y0, log2 - 1, depth + 1, out);
    if (x0 + half < static_cast<int>(g.w())) walk(g, max_merge, x0 + half, y0, log2 - 1, depth + 1, out);
    if (y0 + half < static_cast<int>(g.h())) walk(g, max_merge, x0, y0 + half, log2 - 1, depth + 1, out);
    if (x0 + half < static_cast<int>(g.w()) && y0 + half < static_cast<int>(g.h()))
      walk(g, max_merge, x0 + half, y0 + half, log2 - 1, depth + 1, out);
    return;
  }
  const int inc = (g.available(x0 - 1, y0) && g.skip_at(x0 - 1, y0)) +
                  (g.available(x0, y0 - 1) && g.skip_at(x0, y0 - 1));
  out->push_back({FillBin::kSkip, static_cast<uint8_t>(inc), 1});
  if (max_merge > 1) out->push_back({FillBin::kMergeIdx, 0, 0});
  g.mark(x0, y0, size, depth);
}

}  // namespace

std::vector<FillStep> skip_fill_steps(const Sps& sps, uint32_t max_num_merge_cand,
                                      uint32_t first_ctb, uint32_t end_ctb) {
  std::vector<FillStep> out;
  Grid g(sps, first_ctb);
  const uint32_t wctb = sps.pic_w_ctbs();
  for (uint32_t ctb = first_ctb; ctb < end_ctb; ++ctb) {
    const int x0 = static_cast<int>((ctb % wctb) << sps.log2_ctb);
    const int y0 = static_cast<int>((ctb / wctb) << sps.log2_ctb);
    walk(g, max_num_merge_cand, x0, y0, static_cast<int>(sps.log2_ctb), 0, &out);
    out.push_back({FillBin::kEndOfSlice, 0, static_cast<uint8_t>(ctb + 1 == end_ctb ? 1 : 0)});
  }
  return out;
}

std::optional<std::vector<uint8_t>> make_skip_slice(const Sps& sps, const Pps& pps,
                                                    const SliceHeader& tmpl, uint32_t first_ctb,
                                                    uint32_t end_ctb) {
  if (tmpl.slice_type == 2) return std::nullopt;
  const uint32_t total = sps.pic_w_ctbs() * sps.pic_h_ctbs();
  if (first_ctb >= end_ctb || end_ctb > total) return std::nullopt;

  BitWriter w;
  write_slice_header(w, tmpl, {first_ctb == 0, first_ctb, true}, sps, pps);

  // Context init (9.3.2.2; Tables 9-11, 9-13, 9-19): initType 1 = P without
  // cabac_init_flag, 2 = B (or P with it). split_cu_flag and cu_skip_flag
  // share one row for both.
  const int init_type = tmpl.slice_type == 1 ? (tmpl.cabac_init_flag ? 2 : 1)
                                             : (tmpl.cabac_init_flag ? 1 : 2);
  static const int kSplit[3] = {107, 139, 126};
  static const int kSkip[3] = {197, 185, 201};
  const int merge_init = init_type == 1 ? 122 : 137;
  CabacCtx split[3], skip[3];
  for (int i = 0; i < 3; ++i) {
    split[i] = init_cabac_ctx(kSplit[i], tmpl.slice_qp_y);
    skip[i] = init_cabac_ctx(kSkip[i], tmpl.slice_qp_y);
  }
  CabacCtx merge = init_cabac_ctx(merge_init, tmpl.slice_qp_y);

  CabacEncoder enc(w);
  for (const FillStep& s : skip_fill_steps(sps, tmpl.max_num_merge_cand, first_ctb, end_ctb)) {
    switch (s.kind) {
      case FillBin::kSplit: enc.encode(split[s.ctx_inc], s.value); break;
      case FillBin::kSkip: enc.encode(skip[s.ctx_inc], s.value); break;
      case FillBin::kMergeIdx: enc.encode(merge, s.value); break;
      case FillBin::kEndOfSlice: enc.terminate(s.value); break;
    }
  }
  w.align_zero();

  std::vector<uint8_t> out = {0x00, 0x00, 0x00, 0x01, tmpl.nal_hdr[0], tmpl.nal_hdr[1]};
  const auto esc = escape(w.bytes().data(), w.bytes().size());
  out.insert(out.end(), esc.begin(), esc.end());
  return out;
}

}  // namespace mabur::hevc
