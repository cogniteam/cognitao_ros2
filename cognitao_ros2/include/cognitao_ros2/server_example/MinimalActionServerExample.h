#include <inttypes.h>
#include <memory>

#include "action_manager/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cognitao_ros2/server/MinimalActionServer.h>

using namespace std;

using actionType = action_manager::action::ActionMsg;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;

class MinimalActionServerExample : public MinimalActionServer
{

public:
  explicit MinimalActionServerExample(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::Server<actionType>::SharedPtr action_server_;

  virtual void execute(const std::shared_ptr<GoalHandleActionType> goal_handle) override;

  virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle) override;

private:
  rclcpp::Node::SharedPtr g_node_ = nullptr;
};
