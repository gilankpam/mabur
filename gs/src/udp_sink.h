#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace maburgs {

// Recovered-RTP UDP output (default 127.0.0.1:5600, the wfb-ng convention —
// gstreamer/PixelPilot/ffplay all consume it). Non-blocking fire-and-forget:
// failures are counted, never block decode; a bad address disables the sink
// (ok() == false) instead of throwing.
class UdpSink {
 public:
  UdpSink(const std::string& host, int port);
  ~UdpSink();
  UdpSink(const UdpSink&) = delete;
  UdpSink& operator=(const UdpSink&) = delete;

  bool ok() const { return fd_ >= 0; }
  bool send(const uint8_t* data, size_t len);
  uint64_t sent() const { return sent_; }
  uint64_t failed() const { return failed_; }

 private:
  int fd_ = -1;
  uint64_t sent_ = 0, failed_ = 0;
};

}  // namespace maburgs
