#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace mabur {

// Blocking-with-timeout raw serial reader/writer for the FC MSP UART.
// termios setup mirrors msposd (raw 8N1, no flow control). read() returns
// bytes read (>0), 0 on timeout, or -1 on error (caller should close() and
// reconnect).
class MspSerial {
 public:
  ~MspSerial() { close(); }
  bool open(const std::string& dev, int baud);
  bool is_open() const { return fd_ >= 0; }
  int read(uint8_t* buf, size_t n);
  // Blocking write of n bytes (the 6-byte MSP_STATUS request); returns bytes
  // written or -1 (caller closes and reconnects, same as read).
  int write(const uint8_t* p, size_t n);
  void close();

 private:
  int fd_ = -1;
};

}  // namespace mabur
