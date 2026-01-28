#ifndef CHARGE_CONTROL_GPIO_CONTROL_HPP_
#define CHARGE_CONTROL_GPIO_CONTROL_HPP_

#include <string>

namespace charge_control {

class GpioControl {
 public:
  // gpio_num: 系统 GPIO 编号，如 382, 402 等
  explicit GpioControl(int gpio_num);
  ~GpioControl();

  // 基础初始化，direction: "out", "in", "high", "low"
  bool Init(const std::string& direction = "out");
  
  // 便捷函数：初始化为输出并设置高电平
  bool InitHigh();
  
  // 便捷函数：初始化为输出并设置低电平
  bool InitLow();
  
  // 便捷函数：初始化为输入模式
  bool InitInput();
  
  bool SetHigh();
  bool SetLow();
  
  // 读取当前电平值，返回 0 或 1，失败返回 -1
  int ReadValue();
  
  bool IsValid() const { return valid_; }
  int GetGpioNum() const { return gpio_num_; }

 private:
  bool WriteToFile(const std::string& path, const std::string& value);
  std::string ReadFromFile(const std::string& path);
  bool WaitForFileAccess(const std::string& path, int max_retries, int delay_ms);
  
  int gpio_num_;
  std::string gpio_path_;
  bool valid_{false};
  bool exported_{false};
};

}  // namespace charge_control

#endif  // CHARGE_CONTROL_GPIO_CONTROL_HPP_
