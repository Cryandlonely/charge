#ifndef CHARGE_CONTROL_UDP_PROTOCOL_HPP_
#define CHARGE_CONTROL_UDP_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <map>

namespace charge_control {

// 魔数
inline constexpr uint16_t PROTOCOL_MAGIC = 0x5A53;  // 'ZS'

// 消息来源枚举
enum MsgSource : uint16_t {
  FROM_ROBOT = 0x7FFF,
  FROM_CHARGE_STATION = 0x8FFF
};

// 消息类型枚举
enum MsgType : uint8_t {
  MSG_TYPE_NONE = 0,
  MSG_TYPE_HAND_SHAKE_REQ = 11,
  MSG_TYPE_HAND_SHAKE_ACK = 21,
  MSG_TYPE_POWER_ON_CMD_REQ = 12,
  MSG_TYPE_POWER_ON_CMD_ACK = 22,
  MSG_TYPE_FILL_LIGHT_CMD_REQ = 13,
  MSG_TYPE_FILL_LIGHT_CMD_ACK = 23,
  MSG_TYPE_EVSE_STATE_REQ = 14,
  MSG_TYPE_EVSE_STATE_ACK = 24,
  MSG_TYPE_SWITCH_MODE_REQ = 15,
  MSG_TYPE_SWITCH_MODE_ACK = 25,
  MSG_TYPE_SET_LIGHT_THRESHOLD_REQ = 16,
  MSG_TYPE_SET_LIGHT_THRESHOLD_ACK = 26
};

// 协议头结构 (12 bytes, 小端)
#pragma pack(push, 1)
struct ProtocolHeader {
  uint16_t magic;
  uint8_t  msg_type;
  uint16_t payload_len;
  uint16_t seq;
  uint16_t source;
  uint8_t  reserved[3];
};
#pragma pack(pop)

static_assert(sizeof(ProtocolHeader) == 12, "ProtocolHeader must be 12 bytes");

// ==================== Payload 结构定义 ====================

#pragma pack(push, 1)
struct HandShakeReqPayload {
  uint32_t proto_version;
};

struct HandShakeAckPayload {
  uint32_t sn;
  uint32_t version;
  uint16_t tag_id;
};

struct PowerOnCmdReqPayload {
  uint8_t power_on;
};

struct PowerOnCmdAckPayload {
  uint8_t response;
};

struct FillLightCmdReqPayload {
  uint8_t fill_light_switch;
};

struct FillLightCmdAckPayload {
  uint8_t response;
};

struct EvseStateAckPayload {
  uint32_t light_intensity;
  uint32_t light_threshold;
  float    neg_vol_1;
  float    pos_vol_1;
  float    neg_vol_2;
  float    pos_vol_2;
  uint8_t  fill_light_status;
  uint8_t  mode;
  uint8_t  pin_status_1;
  uint8_t  pin_status_2;
  uint8_t  neg_contact_state_1;
  uint8_t  pos_contact_state_1;
  uint8_t  neg_contact_state_2;
  uint8_t  pos_contact_state_2;
};

struct SwitchModeReqPayload {
  uint8_t mode;
};

struct SwitchModeAckPayload {
  uint8_t response;
  uint8_t current_mode;
};

struct SetLightThresholdReqPayload {
  uint32_t threshold;
};

struct SetLightThresholdAckPayload {
  uint8_t  response;
  uint32_t threshold;
};
#pragma pack(pop)

static_assert(sizeof(EvseStateAckPayload) == 32, "EvseStateAckPayload must be 32 bytes");

// 客户端地址信息
struct ClientAddr {
  uint32_t ip;
  uint16_t port;
};

// 完整消息结构
struct ProtocolMessage {
  ProtocolHeader header;
  std::vector<uint8_t> payload;
  uint16_t crc16;

  static ProtocolMessage Create(MsgType type, MsgSource source, 
                                 const std::vector<uint8_t>& payload, uint16_t seq = 0);

  template<typename T>
  static ProtocolMessage CreateFromStruct(MsgType type, MsgSource source, 
                                           const T& payload_struct, uint16_t seq = 0) {
    std::vector<uint8_t> payload_data(sizeof(T));
    std::memcpy(payload_data.data(), &payload_struct, sizeof(T));
    return Create(type, source, payload_data, seq);
  }

  std::vector<uint8_t> Serialize() const;
  static bool Parse(const uint8_t* data, size_t length, ProtocolMessage& msg);

  bool ValidateCrc() const;

  template<typename T>
  bool GetPayloadAs(T& out) const {
    if (payload.size() < sizeof(T)) return false;
    std::memcpy(&out, payload.data(), sizeof(T));
    return true;
  }
};

uint16_t CalculateCrc16(const uint8_t* data, size_t length);

// UDP协议处理器
class UdpProtocolHandler {
 public:
  using MessageCallback = std::function<void(const ProtocolMessage&, const ClientAddr&)>;

  UdpProtocolHandler();
  ~UdpProtocolHandler();

  UdpProtocolHandler(const UdpProtocolHandler&) = delete;
  UdpProtocolHandler& operator=(const UdpProtocolHandler&) = delete;

  bool StartServer(int port);
  void StopServer();
  bool IsRunning() const { return running_; }

  void RegisterHandler(MsgType type, MessageCallback callback);

  bool SendMessage(const ClientAddr& client, const ProtocolMessage& msg);
  bool SendResponse(const ClientAddr& client, MsgType type, const std::vector<uint8_t>& payload, uint16_t seq);

  template<typename T>
  bool SendResponse(const ClientAddr& client, MsgType type, const T& payload_struct, uint16_t seq) {
    std::vector<uint8_t> payload(sizeof(T));
    std::memcpy(payload.data(), &payload_struct, sizeof(T));
    return SendResponse(client, type, payload, seq);
  }

 private:
  void ServerLoop();

  int socket_fd_{-1};
  int port_{0};
  std::atomic<bool> running_{false};
  std::unique_ptr<std::thread> server_thread_;
  std::map<MsgType, MessageCallback> handlers_;
  std::atomic<uint16_t> seq_counter_{0};
};

}  // namespace charge_control

#endif  // CHARGE_CONTROL_UDP_PROTOCOL_HPP_