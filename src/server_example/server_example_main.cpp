

#include <cognitao_ros2/server_example/Ros2ActionWaitServer.h>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = nullptr;
  node = rclcpp::Node::make_shared(argv[1]);

  auto action_server = std::make_shared<Ros2ActionWaitServer>(node, argv[1]);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}