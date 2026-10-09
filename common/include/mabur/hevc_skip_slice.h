#pragma once
#include <cstdint>
#include <optional>
#include <vector>

#include "mabur/hevc_ps.h"
#include "mabur/hevc_slice.h"

namespace mabur::hevc {

// A fill slice (spec 2026-10-10-h265-slices §5.2): an independent slice
// covering CTUs [first_ctb, end_ctb) in which every coding unit is a skip
// CU with merge_idx 0 -- "copy from the reference with predicted motion".
// The decoder then sees a complete, legal picture (no gap, no error).

enum class FillBin : uint8_t { kSplit, kSkip, kMergeIdx, kEndOfSlice };
struct FillStep {
  FillBin kind;
  uint8_t ctx_inc;  // split_cu_flag / cu_skip_flag ctxInc (0..2); 0 otherwise
  uint8_t value;
};

// Every bin of the fill's slice_segment_data, in bitstream order: the
// coding-quadtree walk of 7.3.8.4 (implicit splits where a block crosses the
// picture edge) with ctxInc from 9.3.4.2.2 (neighbours outside the slice are
// unavailable).
std::vector<FillStep> skip_fill_steps(const Sps& sps, uint32_t max_num_merge_cand,
                                      uint32_t first_ctb, uint32_t end_ctb);

// The fill as an Annex-B NAL unit (00 00 00 01 + tmpl's NAL header +
// escaped RBSP). Header = tmpl's with first/address set and SAO cleared.
// nullopt for an I-slice template (an IRAP has no reference to copy) or an
// empty / out-of-picture range.
std::optional<std::vector<uint8_t>> make_skip_slice(const Sps& sps, const Pps& pps,
                                                    const SliceHeader& tmpl, uint32_t first_ctb,
                                                    uint32_t end_ctb);

}  // namespace mabur::hevc
