# 充电桩控制系统使用文档

## 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [环境依赖](#环境依赖)
- [工程编译与启动](#工程编译与启动)
- [配置文件说明](#配置文件说明)
- [HTTP API 接口](#http-api-接口)
- [MQTT 通信协议](#mqtt-通信协议)
- [GPIO 引脚说明](#gpio-引脚说明)
- [日志系统](#日志系统)
- [常见问题](#常见问题)

---

## 项目概述

本项目是一个基于 ROS2 Humble 的充电桩控制系统，主要用于控制充电桩与机器狗之间的充电交互。系统提供以下功能：

- **充电模式切换**：支持智能充电和快速充电两种模式
- **WiFi 配置**：支持 AP 热点模式和 STA 客户端模式切换
- **状态监控**：通过 MQTT 接收机器狗状态并控制 GPIO 指示灯
- **日志查询**：提供 HTTP 接口查询系统运行日志
- **UDP 协议通信**：与机器人进行状态交互

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    充电桩控制系统                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │ HTTP Server │  │ MQTT Client │  │ UDP Server  │     │
│  │   :8080     │  │             │  │   :53100    │     │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘     │
│         │                │                │             │
│         └────────────────┼────────────────┘             │
│                          │                              │
│                 ┌────────▼────────┐                     │
│                 │  SerialSenderNode│                    │
│                 │   (ROS2 Node)    │                    │
│                 └────────┬────────┘                     │
│                          │                              │
│         ┌────────────────┼────────────────┐             │
│         │                │                │             │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐     │
│  │ GPIO Control│  │ Serial Port │  │   Logger    │     │
│  └─────────────┘  └─────────────┘  └─────────────┘     │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## 环境依赖

### 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble

### 依赖库

```bash
# ROS2 依赖
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-ament-index-cpp

# Paho MQTT C/C++ 库
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev

# YAML-CPP
sudo apt install libyaml-cpp-dev

# nlohmann JSON
sudo apt install nlohmann-json3-dev

# Log4cpp
sudo apt install liblog4cpp5-dev
```

---

## 工程编译与启动

### 1. 克隆或拷贝项目

```bash
cd ~
mkdir -p charge/charge_ws/src
# 将 charge_control 包放入 src 目录
```

### 2. 编译工程

```bash
cd ~/charge/charge_ws

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译
colcon build

# 加载工作空间
source install/setup.bash
```

### 3. 运行程序

#### 手动运行

```bash
cd ~/charge/charge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run charge_control charge
```

#### 使用 systemd 开机自启

```bash
# 进入脚本目录
cd ~/charge/script

# 配置 GPIO 权限（首次运行需要）
sudo ./setup_gpio.sh

# 配置开机自启
sudo ./setup_autostart.sh

# 查看服务状态
sudo systemctl status charge_control.service

# 手动启动/停止服务
sudo systemctl start charge_control.service
sudo systemctl stop charge_control.service

# 查看实时日志
sudo journalctl -u charge_control.service -f
```

---

## 配置文件说明

配置文件路径：`charge_ws/src/charge_control/config/config.yaml`

```yaml
# 充电控制配置文件
charge:
  mode: 1  # 充电模式：0-智能充电，1-快速充电

# 串口配置
serial:
  port: "/dev/ttyS1"      # 串口设备路径
  baud_rate: 19200        # 波特率
  max_current: 10.0       # 最大电流 (A)

# GPIO 配置
gpio:
  gpio_num_enable: 380    # 使能信号监控引脚
  monitor_interval_ms: 100 # GPIO 监控间隔 (ms)
  gpio_pins: [382, 402, 397, 387, 395, 394, 379, 388, 390, 389, 420]

# MQTT 配置
mqtt:
  broker: "tcp://0.0.0.0:1883"  # MQTT Broker 地址
  client_id: "charge_control"   # 客户端 ID
  topic: "charge/+/dog_status"  # 订阅话题

# HTTP 服务器配置
http:
  port: 8080              # HTTP 服务端口
```

---

## HTTP API 接口

假设充电桩 IP 地址为 `192.168.127.11`，HTTP 端口为 `8080`。

### 1. 充电模式切换

切换充电桩的充电模式。

| 项目 | 说明 |
|------|------|
| URL | `/charge/switch` |
| 方法 | POST |
| 参数 | `mode` - 充电模式 (0: 智能充电, 1: 快速充电) |

#### 请求示例

**切换到智能充电模式：**

```bash
curl -X POST "http://192.168.127.11:8080/charge/switch?mode=0"
```

**切换到快速充电模式：**

```bash
curl -X POST "http://192.168.127.11:8080/charge/switch?mode=1"
```

#### 响应示例

**成功响应 (200):**

```json
{
  "success": true,
  "mode": "smart",
  "message": "Charge mode switched successfully"
}
```

**参数错误 (400):**

```json
{
  "success": false,
  "error": "Invalid mode value. Use 0 for smart mode or 1 for fast mode"
}
```

---

### 2. 日志查询

查询指定时间范围内的系统日志。

| 项目 | 说明 |
|------|------|
| URL | `/charge/loginfo` |
| 方法 | GET |
| 参数 | `date` - 日期 (格式: YYYY/M/D 或 YYYY-MM-DD，可选，默认当天) |
|      | `interval` - 时间间隔，分钟 (可选，默认30) |

#### 请求示例

**查询今天最近30分钟的日志：**

```bash
curl "http://192.168.127.11:8080/charge/loginfo?interval=30"
```

**查询指定日期最近60分钟的日志：**

```bash
curl "http://192.168.127.11:8080/charge/loginfo?date=2026/2/5&interval=60"
```

**查询指定日期（另一种格式）：**

```bash
curl "http://192.168.127.11:8080/charge/loginfo?date=2026-02-05&interval=30"
```

#### 响应示例

**成功响应 (200):**

```json
{
  "success": true,
  "date": "2026-02-05",
  "start_time": "2026-02-05 16:00:00",
  "end_time": "2026-02-05 16:30:00",
  "interval_minutes": 30,
  "log_count": 5,
  "logs": [
    "2026-02-05 16:05:23.123 [INFO] HTTP: Switched to Smart Charge Mode",
    "2026-02-05 16:10:45.456 [DEBUG] GPIO 379 set to LOW",
    "2026-02-05 16:10:45.789 [DEBUG] GPIO 388 set to HIGH"
  ]
}
```

**日志文件不存在 (404):**

```json
{
  "success": false,
  "error": "Log file not found for date: 2026-02-05"
}
```

---

### 3. WiFi 配置

配置充电桩的 WiFi 连接模式。

| 项目 | 说明 |
|------|------|
| URL | `/wifi/connect` |
| 方法 | POST |
| Content-Type | application/json |

#### AP 热点模式

创建 WiFi 热点供其他设备连接。

**请求示例：**

```bash
curl -X POST "http://192.168.127.11:8080/wifi/connect" \
  -H "Content-Type: application/json" \
  -d '{"mode": "AP"}'
```

#### STA 客户端模式

连接到指定的 WiFi 网络。

**请求示例：**

```bash
curl -X POST "http://192.168.127.11:8080/wifi/connect" \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "STA",
    "username": "MyWiFi_SSID",
    "password": "MyWiFi_Password"
  }'
```

#### 响应示例

**成功响应 (200):**

```json
{
  "success": true,
  "mode": "STA",
  "ssid": "MyWiFi_SSID"
}
```

**参数错误 (400):**

```json
{
  "success": false,
  "error": "Missing username or password for STA mode"
}
```

---

## MQTT 通信协议

### 订阅话题

系统订阅以下话题接收机器狗状态：

```
charge/+/dog_status
```

其中 `+` 为机器狗 ID 通配符，例如 `charge/dog01/dog_status`。

### 消息格式

```json
{
  "ID": "dog01",
  "fault_code": 0,
  "fault_level": 0,
  "fault_message": "",
  "charging_status": 1,
  "power1": 75.5,
  "voltage1": 48.2,
  "current1": 5.0,
  "temperature1": 35.0,
  "present1": true,
  "power_supply_status1": 1,
  "power2": 80.0,
  "voltage2": 48.5,
  "current2": 4.8,
  "temperature2": 34.5,
  "present2": true,
  "power_supply_status2": 1
}
```

### 发布话题

当检测到使能信号变化时，系统发布充电命令：

```
charge/{dog_id}/command
```

消息格式：

```json
{
  "charge_stop": false,
  "recharge": true
}
```

---

## GPIO 引脚说明

| GPIO 编号 | 功能 | 说明 |
|-----------|------|------|
| 380 | 使能信号监控 | 输入，监控使能信号电平变化 |
| 382 | 继电器控制 1 | 输出，控制继电器 |
| 402 | 继电器控制 2 | 输出，控制继电器 |
| 397 | 电量指示灯 | 电池电量 25%-50% |
| 387 | 电量指示灯 | 电池电量 50%-75% |
| 395 | 电量指示灯 | 电池电量 75%-100% |
| 394 | 电量指示灯 | 电池电量 100% |
| 379 | 快速充电模式指示 | 高电平表示快速充电模式 |
| 388 | 智能充电模式指示 | 高电平表示智能充电模式 |
| 389 | 充电中状态 | 高电平表示正在充电 |
| 390 | 充电桩故障 | 高电平表示故障 |
| 420 | 待机状态 | 高电平表示待机中 |

---

## 日志系统

### 日志存储位置

```
~/charge/charge_ws/logs/charge_control_YYYY-MM-DD.log
```

### 日志格式

```
2026-02-05 16:30:45.123 [INFO] 日志消息内容
```

### 日志保留策略

- 自动保留最近 30 天的日志
- 程序启动时自动清理超过 30 天的旧日志文件

---

## 常见问题

### Q1: 程序启动失败，提示串口打开失败

**解决方案：**

1. 检查串口设备是否存在：
   ```bash
   ls -l /dev/ttyS1
   ```

2. 检查串口权限：
   ```bash
   sudo chmod 666 /dev/ttyS1
   # 或将用户添加到 dialout 组
   sudo usermod -aG dialout $USER
   ```

### Q2: GPIO 操作提示权限不足

**解决方案：**

运行 GPIO 权限配置脚本：

```bash
cd ~/charge/script
sudo ./setup_gpio.sh
# 重启系统或重新登录
```

### Q3: MQTT 连接失败

**解决方案：**

1. 检查 MQTT Broker 是否运行：
   ```bash
   sudo systemctl status mosquitto
   ```

2. 检查配置文件中的 broker 地址是否正确

3. 检查防火墙是否允许 1883 端口

### Q4: HTTP 接口无法访问

**解决方案：**

1. 检查程序是否正常运行：
   ```bash
   sudo systemctl status charge_control.service
   ```

2. 检查端口是否被占用：
   ```bash
   sudo netstat -tlnp | grep 8080
   ```

3. 检查防火墙设置：
   ```bash
   sudo ufw allow 8080/tcp
   ```

---

## 版本信息

- **版本**: 1.0.0
- **ROS2 版本**: Humble
- **更新日期**: 2026-02-05

---

## 联系方式

如有问题，请联系征途开发团队。
