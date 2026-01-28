#ifndef COMMON_HPP
#define COMMON_HPP

#include <cstdint>

enum MSG_SOURCE
{
    FROM_ROBOT = 0x7FFF,
    FROM_CHARGE_STATION = 0x8FFF
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