#include <inttypes.h>
#include <memory>
#include "action_manager/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std;


class MinimalActionServer
{
public:
  
  using actionType=action_manager::action::ActionMsg;
  using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;


  explicit MinimalActionServer(rclcpp::Node::SharedPtr node) {
    using namespace std::placeholders;

    g_node_ = node;

    this->action_server_ = rclcpp_action::create_server<actionType>(
      g_node_,
      "action_manager",
      std::bind(&MinimalActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MinimalActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MinimalActionServer::handle_accepted, this, std::placeholders::_1)
      );
      cout<<" constructor "<<endl;
  }

private:
  rclcpp_action::Server<actionType>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid,
    std::shared_ptr<const actionType::Goal> goal) {
    //RCLCPP_INFO(this->get_logger(), "Received goal request with actiontype %s", goal->actiontype);
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

  virtual void execute(const std::shared_ptr<GoalHandleActionType> goal_handle)
  {
    RCLCPP_INFO(g_node_->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    //auto feedback = std::make_shared<actionType::Feedback>();
    // auto & sequence = feedback->sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<actionType::Result>();

    cout<<"i got "<<goal->actiontype <<endl;


    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     //result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //     return;
    //   }
    //   // Update sequence
    //   //sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   //goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      //result->sequence = sequence;
      goal_handle->set_succeeded(result);
      RCLCPP_INFO(g_node_->get_logger(), "Goal Succeeded");
    }
  }

  virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
  }

private:
      rclcpp::Node::SharedPtr g_node_ = nullptr;
  
};