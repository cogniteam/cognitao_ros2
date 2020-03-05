#include <inttypes.h>
#include <memory>
#include <action_manager/action/action_msg.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std;

class MinimalActionServer
{
public:
  using actionType = action_manager::action::ActionMsg;
  using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;

  explicit MinimalActionServer(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::Server<actionType>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const std::array<uint8_t, 16> &uuid,
      std::shared_ptr<const actionType::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleActionType> goal_handle);

  virtual void execute(const std::shared_ptr<GoalHandleActionType> goal_handle);

  virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle);

private:
  rclcpp::Node::SharedPtr g_node_ = nullptr;
};
