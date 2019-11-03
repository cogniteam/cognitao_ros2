#include "../include/MinimalActionServerExample.h"
//#include "/home/maytronics/dm_ros2_ws/src/cognitao_ros2_action_server_example/include/MinimalActionServerExample.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = nullptr;
  node = rclcpp::Node::make_shared("bla");

  auto action_server = std::make_shared<MinimalActionServerExample>(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}