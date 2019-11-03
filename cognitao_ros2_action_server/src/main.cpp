#include "../include/MinimalActionServer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = nullptr;
  node = rclcpp::Node::make_shared("bla");

  auto action_server = std::make_shared<MinimalActionServer>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}