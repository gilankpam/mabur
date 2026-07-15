#include "msp_serial.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mabur {
namespace {
speed_t speed_by_value(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}
}  // namespace

bool MspSerial::open(const std::string& dev, int baud) {
  close();
  fd_ = ::open(dev.c_str(), O_RDONLY | O_NOCTTY);
  if (fd_ < 0) return false;
  struct termios o {};
  tcgetattr(fd_, &o);
  cfsetspeed(&o, speed_by_value(baud));
  cfmakeraw(&o);
  o.c_cflag |= (CLOCAL | CREAD);
  o.c_cflag &= ~CSIZE;
  o.c_cflag |= CS8;
  o.c_cflag &= ~PARENB;
  o.c_cflag &= ~CSTOPB;
  o.c_cc[VMIN] = 0;
  o.c_cc[VTIME] = 1;  // 100 ms read timeout
  tcsetattr(fd_, TCSANOW, &o);
  tcflush(fd_, TCIFLUSH);
  return true;
}

int MspSerial::read(uint8_t* buf, size_t n) {
  if (fd_ < 0) return -1;
  ssize_t r = ::read(fd_, buf, n);
  if (r < 0) return -1;
  return static_cast<int>(r);
}

void MspSerial::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

}  // namespace mabur
