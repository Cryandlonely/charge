#include "http_server.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <sstream>
#include <map>
#include <iostream>

namespace charge_control {

HttpServer::HttpServer() {
  // 设置默认的 404 处理器
  default_handler_ = [](const HttpRequest& req) {
    HttpResponse resp;
    resp.status_code = 404;
    resp.body = R"({"error": "Not Found", "path": ")" + req.path + R"("})";
    return resp;
  };
}

HttpServer::~HttpServer() {
  Stop();
}

bool HttpServer::Start(int port) {
  if (running_) {
    return false;
  }

  port_ = port;

  // 创建 socket
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  // 设置 socket 选项，允许地址重用
  int opt = 1;
  if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "Failed to set socket options" << std::endl;
    close(server_socket_);
    return false;
  }

  // 绑定地址
  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;  // 监听所有网卡
  server_addr.sin_port = htons(port_);

  if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Failed to bind to port " << port_ << std::endl;
    close(server_socket_);
    return false;
  }

  // 开始监听
  if (listen(server_socket_, 10) < 0) {
    std::cerr << "Failed to listen" << std::endl;
    close(server_socket_);
    return false;
  }

  running_ = true;
  server_thread_ = std::make_unique<std::thread>(&HttpServer::ServerLoop, this);

  std::cout << "HTTP Server started on port " << port_ << std::endl;
  return true;
}

void HttpServer::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;

  // 关闭服务器 socket，这会使 accept() 返回错误
  if (server_socket_ >= 0) {
    shutdown(server_socket_, SHUT_RDWR);
    close(server_socket_);
    server_socket_ = -1;
  }

  // 等待服务器线程结束
  if (server_thread_ && server_thread_->joinable()) {
    server_thread_->join();
  }

  std::cout << "HTTP Server stopped" << std::endl;
}

void HttpServer::RegisterHandler(const std::string& method, const std::string& path, HttpHandler handler) {
  std::string key = method + ":" + path;
  handlers_[key] = handler;
}

void HttpServer::SetDefaultHandler(HttpHandler handler) {
  default_handler_ = handler;
}

void HttpServer::ServerLoop() {
  while (running_) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
    if (client_socket < 0) {
      if (running_) {
        std::cerr << "Failed to accept connection" << std::endl;
      }
      continue;
    }

    // 处理客户端请求
    HandleClient(client_socket);
    close(client_socket);
  }
}

void HttpServer::HandleClient(int client_socket) {
  // 设置接收超时
  struct timeval timeout;
  timeout.tv_sec = 5;
  timeout.tv_usec = 0;
  setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  // 读取请求
  char buffer[4096];
  std::memset(buffer, 0, sizeof(buffer));
  ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
  
  if (bytes_read <= 0) {
    return;
  }

  std::string raw_request(buffer, bytes_read);

  // 解析请求
  HttpRequest request = ParseRequest(raw_request);

  // 查找处理器
  HttpHandler handler = FindHandler(request.method, request.path);

  // 调用处理器
  HttpResponse response = handler(request);

  // 构建并发送响应
  std::string response_str = BuildResponse(response);
  send(client_socket, response_str.c_str(), response_str.length(), 0);
}

HttpRequest HttpServer::ParseRequest(const std::string& raw_request) {
  HttpRequest request;

  std::istringstream stream(raw_request);
  std::string line;

  // 解析请求行
  if (std::getline(stream, line)) {
    // 去除 \r
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    std::istringstream line_stream(line);
    std::string version;
    line_stream >> request.method >> request.path >> version;

    // 解析查询字符串
    size_t query_pos = request.path.find('?');
    if (query_pos != std::string::npos) {
      request.query_string = request.path.substr(query_pos + 1);
      request.path = request.path.substr(0, query_pos);
    }
  }

  // 跳过头部，找到空行
  while (std::getline(stream, line)) {
    if (line == "\r" || line.empty()) {
      break;
    }
  }

  // 读取 body
  std::ostringstream body_stream;
  body_stream << stream.rdbuf();
  request.body = body_stream.str();

  return request;
}

std::string HttpServer::BuildResponse(const HttpResponse& response) {
  std::ostringstream oss;

  // 状态行
  std::string status_text;
  switch (response.status_code) {
    case 200: status_text = "OK"; break;
    case 201: status_text = "Created"; break;
    case 400: status_text = "Bad Request"; break;
    case 404: status_text = "Not Found"; break;
    case 500: status_text = "Internal Server Error"; break;
    default: status_text = "Unknown"; break;
  }

  oss << "HTTP/1.1 " << response.status_code << " " << status_text << "\r\n";
  oss << "Content-Type: " << response.content_type << "\r\n";
  oss << "Content-Length: " << response.body.length() << "\r\n";
  oss << "Access-Control-Allow-Origin: *\r\n";  // CORS 支持
  oss << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
  oss << "Access-Control-Allow-Headers: Content-Type\r\n";
  oss << "Connection: close\r\n";
  oss << "\r\n";
  oss << response.body;

  return oss.str();
}

HttpHandler HttpServer::FindHandler(const std::string& method, const std::string& path) {
  // OPTIONS 请求的特殊处理（CORS 预检）
  if (method == "OPTIONS") {
    return [](const HttpRequest&) {
      HttpResponse resp;
      resp.status_code = 200;
      resp.body = "";
      return resp;
    };
  }

  std::string key = method + ":" + path;
  auto it = handlers_.find(key);
  if (it != handlers_.end()) {
    return it->second;
  }

  return default_handler_;
}

}  // namespace charge_control
