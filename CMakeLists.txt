cmake_minimum_required(VERSION 3.5)
project(cognitao_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cognitao REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/KeyValue.msg"
  "msg/Goal.msg"
  "action/Action.action"
  DEPENDENCIES action_msgs
)

#
# Build
#

include_directories(
  include
)

add_library(cognitao_ros2_shared SHARED 
  src/runner/Ros2Runner.cpp
  src/runner/Ros2RunnerPlugin.cpp
  src/data_source/Ros2DataSource.cpp
  src/data_source/Ros2DataSource_pluggin.cpp
)

#
# Used to include generated messages/actions by this package
#
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(cognitao_ros2_shared
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()
#

ament_target_dependencies(cognitao_ros2_shared 
  rclcpp
  rclcpp_action
)

target_link_libraries(cognitao_ros2_shared
  CogniTao::cognitao_core
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  cognitao_ros2_shared
  DESTINATION /usr/lib/cognitao/plugins/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp_action)

ament_package()




