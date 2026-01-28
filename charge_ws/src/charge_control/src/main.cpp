#include <mqtt/async_client.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <log4cpp/Category.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/Priority.hh>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common.hpp"
#include "gpio_control.hpp"
#include "http_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "udp_protocol.hpp"

// 全局日志对象
log4cpp::Category& logger = log4cpp::Category::getRoot();

// 获取当前日期字符串 (格式: YYYY-MM-DD)
std::string getCurrentDateString() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now = *std::localtime(&time_t_now);
  std::ostringstream oss;
  oss << std::put_time(&tm_now, "%Y-%m-%d");
  return oss.str();
}

// 清理超过指定天数的旧日志文件
void cleanOldLogs(const std::string& log_dir, int keep_days) {
  try {
    namespace fs = std::filesystem;
    if (!fs::exists(log_dir)) {
      return;
    }

    auto now = std::chrono::system_clock::now();
    auto cutoff_time = now - std::chrono::hours(24 * keep_days);

    // 匹配日志文件名格式: charge_control_YYYY-MM-DD.log
    std::regex log_pattern(R"(charge_control_(\d{4}-\d{2}-\d{2})\.log)");

    for (const auto& entry : fs::directory_iterator(log_dir)) {
      if (!entry.is_regular_file()) continue;

      std::string filename = entry.path().filename().string();
      std::smatch match;

      if (std::regex_match(filename, match, log_pattern)) {
        std::string date_str = match[1].str();

        // 解析日期
        std::tm tm = {};
        std::istringstream ss(date_str);
        ss >> std::get_time(&tm, "%Y-%m-%d");

        if (!ss.fail()) {
          auto file_time = std::chrono::system_clock::from_time_t(std::mktime(&tm));

          // 如果文件日期早于截止时间，删除该文件
          if (file_time < cutoff_time) {
            fs::remove(entry.path());
            std::cout << "删除旧日志文件: " << filename << std::endl;
          }
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "清理旧日志失败: " << e.what() << std::endl;
  }
}

// 初始化日志系统
void initLogger() {
  try {
    std::string log_dir = "/home/sunrise/charge_ws/logs";
    int keep_days = 30;  // 保留30天的日志

    // 确保日志目录存在
    std::filesystem::create_directories(log_dir);

    // 清理超过30天的旧日志
    cleanOldLogs(log_dir, keep_days);

    // 创建按日期命名的日志文件
    std::string date_str = getCurrentDateString();
    std::string log_file = log_dir + "/charge_control_" + date_str + ".log";

    // 创建 layout
    log4cpp::PatternLayout* fileLayout = new log4cpp::PatternLayout();
    fileLayout->setConversionPattern("%d{%Y-%m-%d %H:%M:%S.%l} [%p] %m%n");

    // 使用 FileAppender (追加模式，第三个参数true表示追加)
    log4cpp::FileAppender* fileAppender = new log4cpp::FileAppender("fileAppender", log_file, true);
    fileAppender->setLayout(fileLayout);

    // 配置 root category
    logger.setPriority(log4cpp::Priority::DEBUG);
    logger.addAppender(fileAppender);

    logger.info("========================================");
    logger.info("日志系统初始化成功，日志文件: %s", log_file.c_str());
    logger.info("========================================");
  } catch (const std::exception& e) {
    std::cerr << "日志初始化失败: " << e.what() << std::endl;
  }
}

namespace charge_control {

// MQTT 回调类
class MqttCallback : public virtual mqtt::callback {
 public:
  MqttCallback(std::function<void(const std::string&)> on_message_cb) : on_message_cb_(on_message_cb) {}

  void message_arrived(mqtt::const_message_ptr msg) override {
    if (on_message_cb_) {
      on_message_cb_(msg->to_string());
    }
  }

  void connection_lost(const std::string& /*cause*/) override {
    // 连接丢失时的处理
  }

  void connected(const std::string& /*cause*/) override {
    // 连接成功时的处理
  }

 private:
  std::function<void(const std::string&)> on_message_cb_;
};

class SerialSenderNode : public rclcpp::Node {
 private:
  std::string port_;
  int baud_rate_{0};
  double max_current_{0.0};
  int gpio_num_enable_{0};
  int monitor_interval_ms_{100};
  int last_enable_value_{-1};
  // GPIO引脚配置
  std::vector<int> additional_gpio_pins_;
  std::map<int, std::shared_ptr<GpioControl>> additional_gpios_;

  SerialPort serial_port_;
  std::shared_ptr<GpioControl> gpio_enable_;
  std::shared_ptr<GpioControl> gpio_1_;
  std::shared_ptr<GpioControl> gpio_2_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // MQTT
  std::string mqtt_broker_;
  std::string mqtt_client_id_;
  std::string mqtt_topic_;
  std::shared_ptr<mqtt::async_client> mqtt_client_;
  std::shared_ptr<MqttCallback> mqtt_callback_;
  std::atomic<bool> mqtt_running_{false};

  // HTTP
  int http_port_{8080};
  std::shared_ptr<HttpServer> http_server_;

  // TCP Protocol
  int udp_port_{53100};
  std::shared_ptr<UdpProtocolHandler> udp_handler_;

  //默认AP设置
  std::string default_ap_ssid_{"AP"};
  std::string default_ap_password_{""};

 public:
  SerialSenderNode() : Node("serial_sender") {
    // 从yaml配置文件加载参数
    LoadConfig();

    OpenPort();

    // 上电默认10A
    SendStartupCommand();

    // 初始化GPIO 381 用于监控使能信号
    gpio_enable_ = std::make_shared<GpioControl>(gpio_num_enable_);
    if (gpio_enable_->InitInput()) last_enable_value_ = gpio_enable_->ReadValue();

    // 初始化GPIO引脚
    InitializeAdditionalGpios();

    SetGpioHigh(397);

    // 继电器高电平
    SetGpioHigh(382);
    SetGpioHigh(402);

    // 创建定时器监控 GPIO 381 电平变化
    monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(monitor_interval_ms_), std::bind(&SerialSenderNode::MonitorGpioCallback, this));

    // Start MQTT Client
    // StartMqttClient();

    // Start HTTP Server
    StartHttpServer();

    StartUdpServer();
  }

  ~SerialSenderNode() {
    // StopMqttClient();
    StopHttpServer();
    StopUdpServer();
  }

 private:
  // 从yaml配置文件加载参数
  void LoadConfig() {
    std::string config_path = "/home/sunrise/charge_ws/src/charge_control/config/config.yaml";
    try {
      YAML::Node config = YAML::LoadFile(config_path);

      // 串口配置
      port_ = config["serial"]["port"].as<std::string>("/dev/ttyS1");
      baud_rate_ = config["serial"]["baud_rate"].as<int>(19200);
      max_current_ = config["serial"]["max_current"].as<double>(10.0);
      gpio_num_enable_ = config["gpio"]["gpio_num_enable"].as<int>(381);
      monitor_interval_ms_ = config["gpio"]["monitor_interval_ms"].as<int>(100);

      // GPIO引脚列表
      additional_gpio_pins_ = config["gpio"]["gpio_pins"].as<std::vector<int>>();

      // MQTT配置
      mqtt_broker_ = config["mqtt"]["broker"].as<std::string>("tcp://localhost:1883");
      mqtt_client_id_ = config["mqtt"]["client_id"].as<std::string>("charge_control");
      mqtt_topic_ = config["mqtt"]["topic"].as<std::string>("charge/dog_connect");

      // HTTP配置
      http_port_ = config["http"]["port"].as<int>(8080);

    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
    }
  }

  void OpenPort() {
    if (serial_port_.Open(port_, baud_rate_)) {
      RCLCPP_INFO(this->get_logger(), "Opened %s at %d baud", port_.c_str(), baud_rate_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s at %d baud", port_.c_str(), baud_rate_);
    }
  }

  void SendStartupCommand() {
    if (!serial_port_.IsOpen()) {
      return;
    }
    serial_port_.SendCurrentCommand(10.0, 10.0, max_current_);
  }

  void StartMqttClient() {
    try {
      mqtt_client_ = std::make_shared<mqtt::async_client>(mqtt_broker_, mqtt_client_id_);

      // 设置回调
      mqtt_callback_ = std::make_shared<MqttCallback>([this](const std::string& msg) { this->OnMqttMessage(msg); });
      mqtt_client_->set_callback(*mqtt_callback_);

      // 连接选项
      mqtt::connect_options conn_opts;
      conn_opts.set_clean_session(true);
      conn_opts.set_automatic_reconnect(true);
      conn_opts.set_keep_alive_interval(20);

      mqtt_running_ = true;

      // 异步连接
      mqtt_client_->connect(conn_opts)->wait();

      // 订阅主题
      mqtt_client_->subscribe(mqtt_topic_, 1)->wait();

      RCLCPP_INFO(this->get_logger(), "MQTT Client connected to %s, subscribed to %s", mqtt_broker_.c_str(), mqtt_topic_.c_str());
    } catch (const mqtt::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start MQTT client: %s", e.what());
    }
  }

  void StopMqttClient() {
    if (mqtt_running_ && mqtt_client_) {
      mqtt_running_ = false;
      try {
        mqtt_client_->disconnect()->wait();
        RCLCPP_INFO(this->get_logger(), "MQTT Client stopped");
      } catch (const mqtt::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error disconnecting MQTT: %s", e.what());
      }
    }
  }

  void OnMqttMessage(const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "MQTT message received: %s", message.c_str());

    // 处理连接
    OnDogConnected(message);

    // 发布确认消息（可选）
    if (mqtt_client_ && mqtt_client_->is_connected()) {
      try {
        std::string reply_topic = mqtt_topic_ + "/ack";
        mqtt_client_->publish(reply_topic, "ACK", 3, 1, false);
      } catch (const mqtt::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to publish ACK: %s", e.what());
      }
    }
  }

  void OnDogConnected(const std::string& dog_id) { RCLCPP_INFO(this->get_logger(), "Dog with ID %s connected!", dog_id.c_str()); }

  void StartHttpServer() {
    http_server_ = std::make_shared<HttpServer>();

    // 注册WiFi连接接口
    http_server_->RegisterHandler("POST", "/wifi/connect", [this](const HttpRequest& req) { return HandleWifiConnect(req, default_ap_ssid_, default_ap_password_); });

    if (http_server_->Start(http_port_)) {
      RCLCPP_INFO(this->get_logger(), "HTTP Server started on port %d", http_port_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to start HTTP Server on port %d", http_port_);
    }
  }

  void StopHttpServer() {
    if (http_server_) {
      http_server_->Stop();
      RCLCPP_INFO(this->get_logger(), "HTTP Server stopped");
    }
  }

  void StartUdpServer() {
    udp_handler_ = std::make_shared<charge_control::UdpProtocolHandler>();

    using namespace std::placeholders;

    udp_handler_->RegisterHandler(charge_control::MSG_TYPE_EVSE_STATE_REQ, std::bind(&SerialSenderNode::OnEvseStateReq, this, _1, _2));

    if (udp_handler_->StartServer(udp_port_)) {
      RCLCPP_INFO(this->get_logger(), "UDP Protocol Server started on port %d", udp_port_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to start UDP Protocol Server on port %d", udp_port_);
    }
  }
  void StopUdpServer() {
    if (udp_handler_) {
      udp_handler_->StopServer();
      RCLCPP_INFO(this->get_logger(), "UDP Protocol Server stopped");
    }
  }
  // 充电桩状态请求回调
  void OnEvseStateReq(const charge_control::ProtocolMessage& msg, const charge_control::ClientAddr& client) {
    if (msg.header.source != charge_control::FROM_ROBOT) {
      RCLCPP_WARN(this->get_logger(), "EVSE_STATE_REQ: 非机器人来源");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "收到充电桩状态请求, seq=%d", msg.header.seq);

    charge_control::EvseStateAckPayload state{};
    state.light_intensity = 0;
    state.light_threshold = 0;
    state.neg_vol_1 = 0.0f;
    state.pos_vol_1 = 0.0f;
    state.neg_vol_2 = 0.0f;
    state.pos_vol_2 = 0.0f;
    state.fill_light_status = 0;
    state.mode = 0;
    state.pin_status_1 = 0;
    state.pin_status_2 = 0;
    state.neg_contact_state_1 = 0;
    state.pos_contact_state_1 = 0;
    state.neg_contact_state_2 = 0;
    state.pos_contact_state_2 = 0;

    udp_handler_->SendResponse(client, charge_control::MSG_TYPE_EVSE_STATE_ACK, state, msg.header.seq);
  }

  // WiFi连接处理函数
  HttpResponse HandleWifiConnect(const HttpRequest& req, const std::string& default_ap_ssid, const std::string& default_ap_password) {
    HttpResponse resp;
    resp.content_type = "application/json";

    std::string body = req.body;
    std::string ssid;
    std::string password;
    std::string mode;

    // 提取 mode
    size_t mode_pos = body.find("\"mode\"");
    if (mode_pos != std::string::npos) {
      size_t colon_pos = body.find(':', mode_pos);
      size_t quote_start = body.find('"', colon_pos);
      size_t quote_end = body.find('"', quote_start + 1);
      if (quote_start != std::string::npos && quote_end != std::string::npos) {
        mode = body.substr(quote_start + 1, quote_end - quote_start - 1);
      }
    }

    if (mode.empty()) {
      resp.status_code = 400;
      resp.body = R"({"success": false, "error": "Missing mode parameter"})";
      return resp;
    }

    std::string cmd;

    if (mode == "AP" || mode == "ap") {
      // AP模式：使用默认AP设置创建热点
      ssid = default_ap_ssid;
      password = default_ap_password;

      // 先断开当前WiFi连接，然后创建热点
      if (password.empty()) {
        cmd = "sudo nmcli dev wifi hotspot ifname wlan0 ssid \"" + ssid + "\" 2>&1";
      } else {
        cmd = "sudo nmcli dev wifi hotspot ifname wlan0 ssid \"" + ssid + "\" password \"" + password + "\" 2>&1";
      }
    } else if (mode == "STA" || mode == "sta") {
      // STA模式：使用JSON中的数据连接WiFi
      // 提取 username
      size_t username_pos = body.find("\"username\"");
      if (username_pos != std::string::npos) {
        size_t colon_pos = body.find(':', username_pos);
        size_t quote_start = body.find('"', colon_pos);
        size_t quote_end = body.find('"', quote_start + 1);
        if (quote_start != std::string::npos && quote_end != std::string::npos) {
          ssid = body.substr(quote_start + 1, quote_end - quote_start - 1);
        }
      }

      // 提取 password
      size_t pwd_pos = body.find("\"password\"");
      if (pwd_pos != std::string::npos) {
        size_t colon_pos = body.find(':', pwd_pos);
        size_t quote_start = body.find('"', colon_pos);
        size_t quote_end = body.find('"', quote_start + 1);
        if (quote_start != std::string::npos && quote_end != std::string::npos) {
          password = body.substr(quote_start + 1, quote_end - quote_start - 1);
        }
      }

      if (ssid.empty() || password.empty()) {
        resp.status_code = 400;
        resp.body = R"({"success": false, "error": "Missing username or password for STA mode"})";
        return resp;
      }

      cmd = "sudo nmcli dev wifi connect \"" + ssid + "\" password \"" + password + "\" 2>&1";
    } else {
      resp.status_code = 400;
      resp.body = R"({"success": false, "error": "Invalid mode, use 'STA' or 'AP'"})";
      return resp;
    }

    // 执行命令
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      resp.status_code = 500;
      resp.body = R"({"success": false, "error": "Failed to execute command"})";
      return resp;
    }

    char buffer[256];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      output += buffer;
    }

    int exit_code = pclose(pipe);

    if (exit_code == 0) {
      resp.status_code = 200;
      resp.body = R"({"success": true, "mode": ")" + mode + R"(", "ssid": ")" + ssid + R"("})";
    } else {
      output.erase(std::remove(output.begin(), output.end(), '\n'), output.end());
      output.erase(std::remove(output.begin(), output.end(), '\r'), output.end());
      resp.status_code = 500;
      resp.body = R"({"success": false, "error": ")" + output + R"("})";
    }

    return resp;
  }

  void MonitorGpioCallback() {
    if (!gpio_enable_ || !gpio_enable_->IsValid()) {
      return;
    }

    int current_value = gpio_enable_->ReadValue();
    if (current_value < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to read GPIO %d", gpio_num_enable_);
      return;
    }

    // 检测电平变化
    if (current_value != last_enable_value_) {
      RCLCPP_INFO(this->get_logger(), "GPIO %d changed: %d -> %d", gpio_num_enable_, last_enable_value_, current_value);

      // 调用电平变化处理函数
      OnEnableGpioChanged(last_enable_value_, current_value);

      last_enable_value_ = current_value;
    }
  }

  // 当 GPIO 381 电平变化时调用
  void OnEnableGpioChanged(int /*old_value*/, int new_value) {
    if (new_value == 1) {
      // 上升沿：从低电平变为高电平
      RCLCPP_INFO(this->get_logger(), "Enable signal RISING edge detected");
      // TODO: 在这里添加上升沿触发的操作

    } else {
      // 下降沿：从高电平变为低电平
      RCLCPP_INFO(this->get_logger(), "Enable signal FALLING edge detected");
      // TODO: 在这里添加下降沿触发的操作
    }
  }

  // 初始化GPIO引脚
  void InitializeAdditionalGpios() {
    for (int gpio_num : additional_gpio_pins_) {
      auto gpio_control = std::make_shared<GpioControl>(gpio_num);

      if (gpio_control->InitLow()) {
        additional_gpios_[gpio_num] = gpio_control;
        RCLCPP_INFO(this->get_logger(), "GPIO %d initialized as OUTPUT LOW", gpio_num);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO %d", gpio_num);
      }
    }
  }

  // 设置指定GPIO引脚为高电平
  bool SetGpioHigh(int gpio_num) {
    auto it = additional_gpios_.find(gpio_num);
    if (it != additional_gpios_.end() && it->second->IsValid()) {
      bool result = it->second->SetHigh();
      if (result) {
        RCLCPP_INFO(this->get_logger(), "GPIO %d set to HIGH", gpio_num);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO %d to HIGH", gpio_num);
      }
      return result;
    }
    RCLCPP_ERROR(this->get_logger(), "GPIO %d not found or invalid", gpio_num);
    return false;
  }

  // 设置指定GPIO引脚为低电平
  bool SetGpioLow(int gpio_num) {
    auto it = additional_gpios_.find(gpio_num);
    if (it != additional_gpios_.end() && it->second->IsValid()) {
      bool result = it->second->SetLow();
      if (result) {
        RCLCPP_INFO(this->get_logger(), "GPIO %d set to LOW", gpio_num);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO %d to LOW", gpio_num);
      }
      return result;
    }
    RCLCPP_ERROR(this->get_logger(), "GPIO %d not found or invalid", gpio_num);
    return false;
  }

  // 读取指定GPIO引脚的电平状态
  int ReadGpioValue_GPIO(int gpio_num) {
    auto it = additional_gpios_.find(gpio_num);
    if (it != additional_gpios_.end() && it->second->IsValid()) {
      int value = it->second->ReadValue();
      RCLCPP_INFO(this->get_logger(), "GPIO %d value: %d", gpio_num, value);
      return value;
    }
    RCLCPP_ERROR(this->get_logger(), "GPIO %d not found or invalid", gpio_num);
    return -1;
  }

 public:
  // 设置GPIO高电平
  void SetGpioHigh_Public(int gpio_num) { SetGpioHigh(gpio_num); }

  // 设置GPIO低电平
  void SetGpioLow_Public(int gpio_num) { SetGpioLow(gpio_num); }

  // 读取GPIO状态
  int ReadGpioValue_Public(int gpio_num) { return ReadGpioValue_GPIO(gpio_num); }
};

}  // namespace charge_control

int main(int argc, char** argv) {
  // 初始化日志系统
  initLogger();
  logger.info("程序启动");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<charge_control::SerialSenderNode>());
  rclcpp::shutdown();
  return 0;
}
