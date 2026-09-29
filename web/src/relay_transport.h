#pragma once
// RelayTransport: the byte pipe RelayLink speaks over -- UDP natively
// (open_udp_transport), the core<->worker SPSC ring in the browser
// (open_ring_transport, Task 10). Pure interface, no protocol knowledge.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace webgs {

class RelayTransport {
 public:
  virtual ~RelayTransport() = default;
  virtual bool send(const uint8_t* p, size_t n) = 0;
  // One message into buf. >0 bytes, 0 = timeout, -1 = transport closed for good.
  virtual int recv(uint8_t* buf, size_t cap, int timeout_ms) = 0;
  virtual void close() = 0;              // also wakes a blocked recv()
  virtual uint64_t rx_drops() const { return 0; }
  virtual uint64_t tx_drops() const { return 0; }
};

#ifndef __EMSCRIPTEN__
std::unique_ptr<RelayTransport> open_udp_transport(const std::string& host_port, std::string& err);
#endif
#ifdef __EMSCRIPTEN__
std::unique_ptr<RelayTransport> open_ring_transport();
#endif

}  // namespace webgs
