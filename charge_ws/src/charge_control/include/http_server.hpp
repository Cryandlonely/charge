#ifndef CHARGE_CONTROL_HTTP_SERVER_HPP_
#define CHARGE_CONTROL_HTTP_SERVER_HPP_

#include <string>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <map>

// 简单的 HTTP 服务器类
// 使用 POSIX socket 实现，无外部依赖

namespace charge_control {

// HTTP 请求结构
struct HttpRequest {
  std::string method;
  std::string path;
  std::string body;
  std::string query_string;
};

// HTTP 响应结构
struct HttpResponse {
  int status_code{200};
  std::string content_type{"application/json"};
  std::string body;
};

// 请求处理回调类型
using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

class HttpServer {
 public:
  HttpServer();
  ~HttpServer();

  // 禁止拷贝
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  // 启动服务器
  // port: 监听端口
  // 返回: 成功返回 true
  bool Start(int port);

  // 停止服务器
  void Stop();

  // 检查服务器是否运行中
  bool IsRunning() const { return running_; }

  // 注册路由处理器
  // method: HTTP 方法 ("GET", "POST" 等)
  // path: URL 路径
  // handler: 处理函数
  void RegisterHandler(const std::string& method, const std::string& path, HttpHandler handler);

  // 设置默认处理器（404处理）
  void SetDefaultHandler(HttpHandler handler);

 private:
  void ServerLoop();
  void HandleClient(int client_socket);
  HttpRequest ParseRequest(const std::string& raw_request);
  std::string BuildResponse(const HttpResponse& response);
  HttpHandler FindHandler(const std::string& method, const std::string& path);

  int server_socket_{-1};
  int port_{8080};
  std::atomic<bool> running_{false};
  std::unique_ptr<std::thread> server_thread_;

  // 路由表: key = "METHOD:PATH"
  std::map<std::string, HttpHandler> handlers_;
  HttpHandler default_handler_;
};

}  // namespace charge_control

#endif  // CHARGE_CONTROL_HTTP_SERVER_HPP_
