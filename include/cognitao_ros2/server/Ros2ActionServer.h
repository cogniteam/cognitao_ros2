#include <inttypes.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "Ros2ActionContext.h"


using namespace std;
using actionType=cognitao_ros2::action::ActionMsg;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;
using namespace std::placeholders;


class Ros2ActionServer
{
public: 

  Ros2ActionServer(rclcpp::Node::SharedPtr node, std::string action) {

    g_node_ = node;

    this->server_ = rclcpp_action::create_server<actionType>(
      g_node_,
      action,
      std::bind(&Ros2ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Ros2ActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Ros2ActionServer::onStart, this, std::placeholders::_1)
      );
      RCLCPP_INFO(g_node_->get_logger(), "init server");
  }

  virtual ~Ros2ActionServer(){};
  

private:

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid,
    std::shared_ptr<const actionType::Goal> goal) {
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleActionType> goal_handle)
  {
    RCLCPP_INFO(g_node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(Ros2ActionContext ros2ActionContext) = 0;
  
  virtual void onStart(const std::shared_ptr<GoalHandleActionType> goal_handle) = 0;

private:
    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Server<actionType>::SharedPtr server_;

};
