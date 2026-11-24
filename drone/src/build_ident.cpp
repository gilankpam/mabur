#include "build_ident.h"

namespace mabur {

const char* build_release() { return MABUR_RELEASE_STR; }
const char* build_board() { return MABUR_BOARD_STR; }

}  // namespace mabur
