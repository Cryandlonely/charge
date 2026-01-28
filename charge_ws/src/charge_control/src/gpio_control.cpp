#include "gpio_control.hpp"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <unistd.h>

namespace charge_control {

GpioControl::GpioControl(int gpio_num) : gpio_num_(gpio_num) {
  gpio_path_ = "/sys/class/gpio/gpio" + std::to_string(gpio_num_);
}

GpioControl::~GpioControl() {
}

bool GpioControl::WriteToFile(const std::string& path, const std::string& value) {
  std::ofstream file(path, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << path << " (errno: " << errno << ")" << std::endl;
    return false;
  }
  file << value;
  if (file.fail()) {
    std::cerr << "Failed to write to " << path << std::endl;
    file.close();
    return false;
  }
  file.close();
  return true;
}

std::string GpioControl::ReadFromFile(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << path << " for reading" << std::endl;
    return "";
  }
  std::string value;
  file >> value;
  file.close();
  return value;
}

bool GpioControl::WaitForFileAccess(const std::string& path, int max_retries, int delay_ms) {
  for (int i = 0; i < max_retries; i++) {
    // 检查文件是否存在且可写
    if (access(path.c_str(), W_OK) == 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }
  return false;
}

bool GpioControl::Init(const std::string& direction) {
  // 检查是否已经 export
  bool newly_exported = false;
  if (!std::filesystem::exists(gpio_path_)) {
    // echo gpio_num > /sys/class/gpio/export
    if (!WriteToFile("/sys/class/gpio/export", std::to_string(gpio_num_))) {
      std::cerr << "Failed to export GPIO " << gpio_num_ << std::endl;
      return false;
    }
    exported_ = true;
    newly_exported = true;
    
    // 等待 GPIO 目录创建
    int retries = 20;
    while (!std::filesystem::exists(gpio_path_) && retries > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      retries--;
    }
    
    if (!std::filesystem::exists(gpio_path_)) {
      std::cerr << "GPIO " << gpio_num_ << " directory not created" << std::endl;
      return false;
    }
  }
  
  // 设置 direction
  std::string direction_path = gpio_path_ + "/direction";
  
  // 如果是新 export 的 GPIO，等待 udev 设置好权限
  if (newly_exported) {
    if (!WaitForFileAccess(direction_path, 30, 100)) {
      std::cerr << "Timeout waiting for permission on " << direction_path << std::endl;
      return false;
    }
  }
  
  // echo direction > /sys/class/gpio/gpioXXX/direction
  if (!WriteToFile(direction_path, direction)) {
    std::cerr << "Failed to set direction '" << direction << "' for GPIO " << gpio_num_ << std::endl;
    return false;
  }
  
  valid_ = true;
  std::cout << "GPIO " << gpio_num_ << " initialized successfully (direction: " << direction << ")" << std::endl;
  return true;
}

bool GpioControl::InitHigh() {
  return Init("high");
}

bool GpioControl::InitLow() {
  return Init("low");
}

bool GpioControl::InitInput() {
  return Init("in");
}

bool GpioControl::SetHigh() {
  if (!valid_) return false;
  
  std::string value_path = gpio_path_ + "/value";
  return WriteToFile(value_path, "1");
}

bool GpioControl::SetLow() {
  if (!valid_) return false;
  
  std::string value_path = gpio_path_ + "/value";
  return WriteToFile(value_path, "0");
}

int GpioControl::ReadValue() {
  if (!valid_) return -1;
  
  std::string value_path = gpio_path_ + "/value";
  std::string value = ReadFromFile(value_path);
  
  if (value.empty()) return -1;
  
  return (value[0] == '1') ? 1 : 0;
}

}  // namespace charge_control
