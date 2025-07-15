cmake_minimum_required(VERSION 3.5)
project(p4_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(p4_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  p4_interfaces
)

include_directories(include)

# Executables
add_executable(service_server src/node_server.cpp)
ament_target_dependencies(service_server ${dependencies})

add_executable(service_client src/node_client.cpp)
ament_target_dependencies(service_client ${dependencies})

# Installation
install(TARGETS
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()