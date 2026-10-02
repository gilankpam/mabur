#pragma once
// Browser transport for RelayLink: the core<->worker SPSC ring. The
// interface and the native UDP transport live in gs/src/relay_transport.h.
#include <memory>
#include "relay_transport.h"   // gs/src/relay_transport.h (include path: MABUR_DIR/gs/src)
namespace webgs {
#ifdef __EMSCRIPTEN__
std::unique_ptr<maburgs::RelayTransport> open_ring_transport();
#endif
}  // namespace webgs
