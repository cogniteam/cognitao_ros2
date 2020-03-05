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
find_package(action_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ros2_cognitao_msgs REQUIRED)


add_executable(minimal_action_client 
  src/Ros2RunnerPlugin.cpp
  src/Ros2Runner.cpp
)

ament_target_dependencies(minimal_action_client 
  rclcpp
  std_msgs
  "rclcpp_action"
  "action_manager"
)


add_executable(action_server_example
  src/server_example/server_example_main.cpp
  src/MinimalActionServer.cpp
  src/server_example/MinimalActionServerExample.cpp
)

ament_target_dependencies(action_server_example 
  rclcpp
  std_msgs
  "rclcpp_action"
  "action_manager"
)

add_executable(action_server_not_composable 
  src/server_main.cpp
  src/MinimalActionServer.cpp
)

ament_target_dependencies(action_server_not_composable
  "rclcpp"
  "rclcpp_action"
  "action_manager"
  "cognitao_ros2_action_server"
)



add_library(cognitao_ros2_shared SHARED 
   src/Ros2RunnerPlugin.cpp
   src/Ros2Runner.cpp )

ament_target_dependencies(cognitao_ros2_shared 
  rclcpp
  std_msgs
  "rclcpp_action"
  "action_manager"


)


install(TARGETS
    cognitao_ros2_shared
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})


# rclcpp


install(TARGETS minimal_action_client
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS action_server_not_composable
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS action_server_example
    DESTINATION lib/${PROJECT_NAME})



##make DESTDIR=/home/maytronics/dm_ros2_ws/build/cognitao_ros2 install

include_directories(
  include
  /usr/include/cognitao/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/EventMsg.msg"

)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}  
  "action/ActionMsg.action"
  "msg/KeyValMsg.msg"
  "msg/GoalMsg.msg"
  DEPENDENCIES ros2_cognitao_msgs
  DEPENDENCIES action_msgs 
  DEPENDENCIES builtin_interfaces 
 
)


ament_export_dependencies(
  rosidl_default_runtime
)
rosidl_target_interfaces(minimal_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")




ament_export_dependencies(rosidl_default_runtime)



ament_package()




