cmake_minimum_required(VERSION 3.5)
project(cognitao_ros2_action_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_server_not_composable src/main.cpp)
ament_target_dependencies(action_server_not_composable
  "rclcpp"
  "rclcpp_action"
  "action_manager"
)

install(TARGETS action_server_not_composable
  DESTINATION lib/${PROJECT_NAME})

ament_package()
