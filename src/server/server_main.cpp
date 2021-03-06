#include "../include/cognitao_ros2/server/Ros2ActionServer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = nullptr;
  node = rclcpp::Node::make_shared("server");

  auto action_server = std::make_shared<Ros2ActionServer>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}