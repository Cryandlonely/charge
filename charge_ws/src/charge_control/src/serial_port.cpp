#include "serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace charge_control {

SerialPort::SerialPort() : fd_(-1), baud_rate_(0) {}

SerialPort::~SerialPort() {
  Close();
}

bool SerialPort::Open(const std::string& port, int baud_rate) {
  if (fd_ >= 0) {
    Close();
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);

  speed_t baud;
  switch (baud_rate) {
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      baud = B19200;
      break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 0.1s read timeout

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  port_ = port;
  baud_rate_ = baud_rate;
  return true;
}

void SerialPort::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::IsOpen() const {
  return fd_ >= 0;
}

ssize_t SerialPort::Write(const uint8_t* data, size_t size) {
  if (fd_ < 0) {
    return -1;
  }
  return ::write(fd_, data, size);
}

ssize_t SerialPort::Read(uint8_t* buffer, size_t size) {
  if (fd_ < 0) {
    return -1;
  }
  return ::read(fd_, buffer, size);
}

uint8_t SerialPort::CurrentToByte(double current, double max_current) {
  const double clamped = std::clamp(current, 0.0, max_current);
  const double normalized = clamped / max_current;  // 0.0 - 1.0
  const int level = static_cast<int>(std::round(normalized * 255.0));
  return static_cast<uint8_t>(std::clamp(level, 0, 255));
}

bool SerialPort::SendCurrentCommand(double current1, double current2,
                                    double max_current) {
  (void)current1; (void)current2; (void)max_current;

  uint8_t buffer[4];
  buffer[0] = 0x5A;  // 固定帧头
  buffer[1] = 0xE0;    // 模块1电流
  buffer[2] = 0xE0;    // 模块2电流
  buffer[3] = 0xA5;  // 固定帧尾

  std::cout<< "Sending command: ";
  for (size_t i = 0; i < sizeof(buffer); ++i) {
    std::cout << "0x" << std::hex << static_cast<int>(buffer[i]) << " ";
  }
  std::cout << std::dec << std::endl;

  const ssize_t written = Write(buffer, sizeof(buffer));
  return written == static_cast<ssize_t>(sizeof(buffer));
}

}  // namespace charge_control
