#include "udp_protocol.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace charge_control {

// CRC16-Modbus
uint16_t CalculateCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

ProtocolMessage ProtocolMessage::Create(MsgType type, MsgSource source, 
                                         const std::vector<uint8_t>& payload, uint16_t seq) {
  ProtocolMessage msg;
  msg.header.magic = PROTOCOL_MAGIC;
  msg.header.msg_type = type;
  msg.header.payload_len = static_cast<uint16_t>(payload.size());
  msg.header.seq = seq;
  msg.header.source = source;
  std::memset(msg.header.reserved, 0, sizeof(msg.header.reserved));
  msg.payload = payload;

  std::vector<uint8_t> crc_data;
  crc_data.resize(sizeof(ProtocolHeader) + payload.size());
  std::memcpy(crc_data.data(), &msg.header, sizeof(ProtocolHeader));
  if (!payload.empty()) {
    std::memcpy(crc_data.data() + sizeof(ProtocolHeader), payload.data(), payload.size());
  }
  msg.crc16 = CalculateCrc16(crc_data.data(), crc_data.size());

  return msg;
}

std::vector<uint8_t> ProtocolMessage::Serialize() const {
  std::vector<uint8_t> data;
  data.resize(sizeof(ProtocolHeader) + payload.size() + sizeof(uint16_t));

  std::memcpy(data.data(), &header, sizeof(ProtocolHeader));
  if (!payload.empty()) {
    std::memcpy(data.data() + sizeof(ProtocolHeader), payload.data(), payload.size());
  }

  data[data.size() - 2] = crc16 & 0xFF;
  data[data.size() - 1] = (crc16 >> 8) & 0xFF;

  return data;
}

bool ProtocolMessage::Parse(const uint8_t* data, size_t length, ProtocolMessage& msg) {
  if (length < sizeof(ProtocolHeader) + sizeof(uint16_t)) {
    return false;
  }

  std::memcpy(&msg.header, data, sizeof(ProtocolHeader));

  if (msg.header.magic != PROTOCOL_MAGIC) {
    std::cerr << "Invalid magic: 0x" << std::hex << msg.header.magic << std::endl;
    return false;
  }

  size_t expected_len = sizeof(ProtocolHeader) + msg.header.payload_len + sizeof(uint16_t);
  if (length < expected_len) {
    return false;
  }

  if (msg.header.payload_len > 0) {
    msg.payload.resize(msg.header.payload_len);
    std::memcpy(msg.payload.data(), data + sizeof(ProtocolHeader), msg.header.payload_len);
  }

  size_t crc_offset = sizeof(ProtocolHeader) + msg.header.payload_len;
  msg.crc16 = data[crc_offset] | (data[crc_offset + 1] << 8);

  return msg.ValidateCrc();
}

bool ProtocolMessage::ValidateCrc() const {
  std::vector<uint8_t> crc_data;
  crc_data.resize(sizeof(ProtocolHeader) + payload.size());
  std::memcpy(crc_data.data(), &header, sizeof(ProtocolHeader));
  if (!payload.empty()) {
    std::memcpy(crc_data.data() + sizeof(ProtocolHeader), payload.data(), payload.size());
  }
  uint16_t calculated_crc = CalculateCrc16(crc_data.data(), crc_data.size());
  return calculated_crc == crc16;
}

// UdpProtocolHandler 实现
UdpProtocolHandler::UdpProtocolHandler() = default;

UdpProtocolHandler::~UdpProtocolHandler() {
  StopServer();
}

bool UdpProtocolHandler::StartServer(int port) {
  if (running_) {
    StopServer();
  }

  port_ = port;

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "UdpProtocol: Failed to create socket" << std::endl;
    return false;
  }

  int opt = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(port);

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
    std::cerr << "UdpProtocol: Failed to bind port " << port << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  running_ = true;
  server_thread_ = std::make_unique<std::thread>(&UdpProtocolHandler::ServerLoop, this);

  std::cout << "UdpProtocol: Server started on port " << port << std::endl;
  return true;
}

void UdpProtocolHandler::StopServer() {
  running_ = false;

  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
    close(socket_fd_);
    socket_fd_ = -1;
  }

  if (server_thread_ && server_thread_->joinable()) {
    server_thread_->join();
    server_thread_.reset();
  }
}

void UdpProtocolHandler::RegisterHandler(MsgType type, MessageCallback callback) {
  handlers_[type] = std::move(callback);
}

bool UdpProtocolHandler::SendMessage(const ClientAddr& client, const ProtocolMessage& msg) {
  if (socket_fd_ < 0) {
    return false;
  }

  struct sockaddr_in client_addr;
  std::memset(&client_addr, 0, sizeof(client_addr));
  client_addr.sin_family = AF_INET;
  client_addr.sin_addr.s_addr = client.ip;
  client_addr.sin_port = htons(client.port);

  std::vector<uint8_t> data = msg.Serialize();
  ssize_t sent = sendto(socket_fd_, data.data(), data.size(), 0,
                        reinterpret_cast<struct sockaddr*>(&client_addr),
                        sizeof(client_addr));
  return sent == static_cast<ssize_t>(data.size());
}

bool UdpProtocolHandler::SendResponse(const ClientAddr& client, MsgType type, 
                                       const std::vector<uint8_t>& payload, uint16_t seq) {
  ProtocolMessage msg = ProtocolMessage::Create(type, FROM_CHARGE_STATION, payload, seq);
  return SendMessage(client, msg);
}

void UdpProtocolHandler::ServerLoop() {
  uint8_t buffer[2048];
  struct sockaddr_in client_addr;
  socklen_t client_len = sizeof(client_addr);

  while (running_) {
    ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                 reinterpret_cast<struct sockaddr*>(&client_addr),
                                 &client_len);

    if (recv_len <= 0) {
      if (running_) {
        continue;
      }
      break;
    }

    // 解析消息
    ProtocolMessage msg;
    if (ProtocolMessage::Parse(buffer, recv_len, msg)) {
      ClientAddr client;
      client.ip = client_addr.sin_addr.s_addr;
      client.port = ntohs(client_addr.sin_port);

      char ip_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
      std::cout << "UdpProtocol: Received from " << ip_str << ":" << client.port 
                << ", msg_type=" << static_cast<int>(msg.header.msg_type) << std::endl;

      auto it = handlers_.find(static_cast<MsgType>(msg.header.msg_type));
      if (it != handlers_.end() && it->second) {
        it->second(msg, client);
      } else {
        std::cout << "UdpProtocol: No handler for msg_type " 
                  << static_cast<int>(msg.header.msg_type) << std::endl;
      }
    } else {
      std::cerr << "UdpProtocol: Failed to parse message" << std::endl;
    }
  }
}

}  // namespace charge_control