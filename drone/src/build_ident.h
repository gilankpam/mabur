#pragma once

namespace mabur {

// What this binary is: the release it was built as and the board it was
// built for. Both are compiled in from the MABUR_RELEASE / MABUR_BOARD
// CMake cache strings ("local" for a hand build).
const char* build_release();
const char* build_board();

}  // namespace mabur
