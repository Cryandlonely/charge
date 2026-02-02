#ifndef COMMON_HPP
#define COMMON_HPP

#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

enum MSG_SOURCE
{
    FROM_ROBOT = 0x7FFF,
    FROM_CHARGE_STATION = 0x8FFF
};

// 故障码枚举
enum class FaultCode : uint8_t
{
    Unknown = 0,                      // 异常未知
    ActuatorDisabled,                 // 执行器失能
    ActuatorEncoderError,             // 执行器编码器错误
    ActuatorOffline,                  // 执行器掉线
    ActuatorOverVoltage,              // 执行器过压
    ActuatorOverheat,                 // 执行器过温
    ActuatorTempWarn,                 // 执行器过温预警
    ActuatorTimeout,                  // 执行器控制超时
    ActuatorUndervolt,                // 执行器欠压
    PowerControlOverTemp,             // 单电池过温预警
    PowerControlPowerEmpty,           // 单电池电量低于10%
    PowerControlPowerLow,             // 单电池电量低于20%，高于10%
    PowerControlOffline,              // 电源控制板MCU连接失败
    CANBroken,                        // CAN通信异常
    RobotRemoteKeepAliveFailure,      // 遥控器断链
    SystemClockSanityError,           // 系统时间跳变
    SystemRobotStatusError,           // 机器人状态异常
    IMUConnectError,                  // IMU连接异常
    IMUDataNotUpdated                 // IMU数据不更新
};

// FaultCode 枚举的 JSON 序列化
NLOHMANN_JSON_SERIALIZE_ENUM(FaultCode, {
    {FaultCode::Unknown, "Unknown"},
    {FaultCode::ActuatorDisabled, "ActuatorDisabled"},
    {FaultCode::ActuatorEncoderError, "ActuatorEncoderError"},
    {FaultCode::ActuatorOffline, "ActuatorOffline"},
    {FaultCode::ActuatorOverVoltage, "ActuatorOverVoltage"},
    {FaultCode::ActuatorOverheat, "ActuatorOverheat"},
    {FaultCode::ActuatorTempWarn, "ActuatorTempWarn"},
    {FaultCode::ActuatorTimeout, "ActuatorTimeout"},
    {FaultCode::ActuatorUndervolt, "ActuatorUndervolt"},
    {FaultCode::PowerControlOverTemp, "PowerControlOverTemp"},
    {FaultCode::PowerControlPowerEmpty, "PowerControlPowerEmpty"},
    {FaultCode::PowerControlPowerLow, "PowerControlPowerLow"},
    {FaultCode::PowerControlOffline, "PowerControlOffline"},
    {FaultCode::CANBroken, "CANBroken"},
    {FaultCode::RobotRemoteKeepAliveFailure, "RobotRemoteKeepAliveFailure"},
    {FaultCode::SystemClockSanityError, "SystemClockSanityError"},
    {FaultCode::SystemRobotStatusError, "SystemRobotStatusError"},
    {FaultCode::IMUConnectError, "IMUConnectError"},
    {FaultCode::IMUDataNotUpdated, "IMUDataNotUpdated"}
})

// 电源状态枚举
enum class PowerSupplyStatus : uint8_t
{
    UNKNOWN = 0,      // 电源状态未知
    CHARGING = 1,     // 正在充电
    DISCHARGING = 2,  // 放电中
    FULL = 4          // 已满电
};

// PowerSupplyStatus 枚举的 JSON 序列化（使用字符串）
NLOHMANN_JSON_SERIALIZE_ENUM(PowerSupplyStatus, {
    {PowerSupplyStatus::UNKNOWN, "UNKNOWN"},
    {PowerSupplyStatus::CHARGING, "CHARGING"},
    {PowerSupplyStatus::DISCHARGING, "DISCHARGING"},
    {PowerSupplyStatus::FULL, "FULL"}
})

// 充电状态枚举
enum class ChargingStatus : uint8_t
{
    UNKNOWN = 0,      // 电源状态未知
    NOT_CHARGING = 1, // 未充电
    CHARGING = 2,     // 正在充电
    DISCHARGING = 3,  // 放电中
    FULL = 4          // 已满电
};

// ChargingStatus 枚举的 JSON 序列化（使用字符串）
NLOHMANN_JSON_SERIALIZE_ENUM(ChargingStatus, {
    {ChargingStatus::UNKNOWN, "Unknown"},
    {ChargingStatus::NOT_CHARGING, "NotCharging"},
    {ChargingStatus::CHARGING, "Charging"},
    {ChargingStatus::DISCHARGING, "Discharging"},
    {ChargingStatus::FULL, "Full"}
})

// 狗状态数据结构
struct DogStatus
{
    std::string ID = "";                                         // 狗的唯一ID（字符串类型）
    int fault_code = 0;                                          // 故障码
    int fault_level = 0;                                         // 故障等级
    std::string fault_message = "";                              // 故障信息文本
    ChargingStatus charging_status = ChargingStatus::UNKNOWN;    // 充电状态
    float power1 = 0.0f;                                         // 电池1 电量（百分比 0-100）
    float power2 = 0.0f;                                         // 电池2 电量（百分比 0-100）
    bool present1 = false;                                       // 电池1 是否在位
    bool present2 = false;                                       // 电池2 是否在位
    float voltage1 = 0.0f;                                       // 电池1 电压 (V)
    float voltage2 = 0.0f;                                       // 电池2 电压 (V)
    float temperature1 = 0.0f;                                   // 电池1 温度 (°C)
    float temperature2 = 0.0f;                                   // 电池2 温度 (°C)
    float current1 = 0.0f;                                       // 电池1 电流 (A)
    float current2 = 0.0f;                                       // 电池2 电流 (A)
    PowerSupplyStatus power_supply_status1 = PowerSupplyStatus::UNKNOWN;  // 电池1 电源状态
    PowerSupplyStatus power_supply_status2 = PowerSupplyStatus::UNKNOWN;  // 电池2 电源状态
};

// 自定义 JSON 转换，处理字段名称映射
inline void from_json(const nlohmann::json& j, DogStatus& s) {
    j.at("ID").get_to(s.ID);
    j.at("FaultCode").get_to(s.fault_code);
    j.at("FaultLevel").get_to(s.fault_level);
    j.at("FaultMessage").get_to(s.fault_message);
    j.at("ChargingStatus").get_to(s.charging_status);
    j.at("power1").get_to(s.power1);
    j.at("power2").get_to(s.power2);
    j.at("present1").get_to(s.present1);
    j.at("present2").get_to(s.present2);
    j.at("voltage1").get_to(s.voltage1);
    j.at("voltage2").get_to(s.voltage2);
    j.at("temperature1").get_to(s.temperature1);
    j.at("temperature2").get_to(s.temperature2);
    j.at("current1").get_to(s.current1);
    j.at("current2").get_to(s.current2);
    j.at("power_supply_status1").get_to(s.power_supply_status1);
    j.at("power_supply_status2").get_to(s.power_supply_status2);
}

inline void to_json(nlohmann::json& j, const DogStatus& s) {
    j = nlohmann::json{
        {"ID", s.ID},
        {"FaultCode", s.fault_code},
        {"FaultLevel", s.fault_level},
        {"FaultMessage", s.fault_message},
        {"ChargingStatus", s.charging_status},
        {"power1", s.power1},
        {"power2", s.power2},
        {"present1", s.present1},
        {"present2", s.present2},
        {"voltage1", s.voltage1},
        {"voltage2", s.voltage2},
        {"temperature1", s.temperature1},
        {"temperature2", s.temperature2},
        {"current1", s.current1},
        {"current2", s.current2},
        {"power_supply_status1", s.power_supply_status1},
        {"power_supply_status2", s.power_supply_status2}
    };
}

// 充电命令结构体
struct ChargeCommand
{
    bool charge_stop = false;   // 停止充电：true 默认 false (接收到停止命令，狗直接站立)
    bool recharge = false;      // 进行自动对准回充：true 默认 false

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ChargeCommand, charge_stop, recharge)
};

enum class MSG_TYPE : uint8_t
{
    NONE = 0,                                         // 无效
    HAND_SHAKE_REQ = 11,                              // 握手请求          
    HAND_SHAKE_ACK = 21,                              // 握手请求回复
    POWER_ON_CMD_REQ = 12,                            // 充电桩上电命令
    POWER_ON_CMD_ACK = 22,                            // 充电桩上电命令回复
    FILL_LIGHT_CMD_REQ = 13,                          // 充电桩补光灯命令
    FILL_LIGHT_CMD_ACK = 23,                           // 充电桩补光灯命令回复
    EVSE_STATE_REQ = 14,                               // 充电桩状态请求
    EVSE_STATE_ACK = 24,                               // 充电桩状态回复
    SWITCH_MODE_REQ = 15,                               // 充电桩模式切换请求
    SWITCH_MODE_ACK = 25,                               // 充电桩模式切换回复
    SET_LIGHT_THRESHOLD_REQ = 16,                       // 设置补光灯阈值请求
    SET_LIGHT_THRESHOLD_ACK = 26                        // 设置补光灯阈值回复
};

#endif // COMMON_HPP