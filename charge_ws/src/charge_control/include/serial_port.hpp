#ifndef SERIAL_PORT_HPP_
#define SERIAL_PORT_HPP_

#include <cstdint>
#include <string>

namespace charge_control {

// 串口通信类，用于控制电流模块
class SerialPort {
 public:
  // 默认最大电流值 12A
  static constexpr double kDefaultMaxCurrent = 12.0;

  SerialPort();
  ~SerialPort();

  // 禁止拷贝和赋值
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // 打开串口
  // port: 串口设备路径，如 "/dev/ttyACM0"
  // baud_rate: 波特率，如 9600, 19200, 38400, 57600, 115200
  // 返回: 成功返回 true，失败返回 false
  bool Open(const std::string& port, int baud_rate);

  // 关闭串口
  void Close();

  // 检查串口是否已打开
  bool IsOpen() const;

  // 发送原始数据
  // data: 数据指针
  // size: 数据长度
  // 返回: 成功发送的字节数，失败返回 -1
  ssize_t Write(const uint8_t* data, size_t size);

  // 读取数据
  // buffer: 接收缓冲区
  // size: 缓冲区大小
  // 返回: 成功读取的字节数，失败返回 -1
  ssize_t Read(uint8_t* buffer, size_t size);

  // 将电流值转换为字节值 (0-12A -> 0x00-0xFF)
  // current: 电流值 (0.0 - max_current)
  // max_current: 最大电流值，默认 12A
  // 返回: 对应的字节值 (0-255)
  static uint8_t CurrentToByte(double current,
                               double max_current = kDefaultMaxCurrent);

  // 发送双模块电流控制命令
  // current1: 模块1电流
  // current2: 模块2电流
  // max_current: 最大电流值
  // 返回: 成功返回 true，失败返回 false
  bool SendCurrentCommand(double current1, double current2,
                          double max_current = kDefaultMaxCurrent);

 private:
  int fd_;
  std::string port_;
  int baud_rate_;
};

}  // namespace charge_control

#endif  // SERIAL_PORT_HPP_
