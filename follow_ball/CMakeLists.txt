cmake_minimum_required(VERSION 3.8)
project(follow_ball)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    kobuki_ros_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/follow_ball/FollowNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follow src/main_follow_ball.cpp)
ament_target_dependencies(follow ${dependencies})
target_link_libraries(follow ${PROJECT_NAME})

install(TARGETS
  follow
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()

