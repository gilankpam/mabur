#pragma once

#include <cstdint>

#include "mabur/uep_decoder.h"

namespace maburgs {

// {arrived, expected} for the ladder's block-4 instant-demote input --
// post-FEC (residual) loss on the current rung. Extracted from the control
// step in gs/src/main.cpp so the decoder -> LinkHealth wiring is testable:
// test_vrx_controller.cpp hand-builds LinkHealth, so before 2026-09-02
// nothing covered which of the decoder's two loss measures the demote path
// actually reads. See tests/test_ladder_residual.cpp.
struct ResidualCounts {
  uint64_t arrived = 0;
  uint64_t expected = 0;
};

ResidualCounts ladder_residual_counts(const mabur::UepDecoder& dec);

}  // namespace maburgs
