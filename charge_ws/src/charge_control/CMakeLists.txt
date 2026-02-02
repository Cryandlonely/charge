cmake_minimum_required(VERSION 3.8)
project(charge_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)

# Find Paho MQTT C++ library
find_package(PahoMqttCpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3.10.5 REQUIRED)
pkg_check_modules(LOG4CPP REQUIRED log4cpp)

# Include directories
include_directories(include)
include_directories(${LOG4CPP_INCLUDE_DIRS})

# Serial port library
add_library(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# GPIO control library (using sysfs, no external dependencies)
add_library(gpio_control src/gpio_control.cpp)
target_include_directories(gpio_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# HTTP server library
add_library(http_server src/http_server.cpp)
target_include_directories(http_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# TCP protocol library
add_library(udp_protocol src/udp_protocol.cpp)
target_include_directories(udp_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Main executable
add_executable(charge src/main.cpp)
target_link_libraries(charge serial_port gpio_control http_server udp_protocol PahoMqttCpp::paho-mqttpp3 yaml-cpp nlohmann_json::nlohmann_json ${LOG4CPP_LIBRARIES})
ament_target_dependencies(charge rclcpp std_msgs ament_index_cpp)

# Install targets
install(TARGETS serial_port gpio_control http_server udp_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS charge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
