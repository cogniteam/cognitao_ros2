#include <inttypes.h>
#include <memory>

#include "cognitao_ros2/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "../server/MinimalActionServer.h"

using namespace std;


using actionType=cognitao_ros2::action::ActionMsg;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;

class MinimalActionServerExample: public MinimalActionServer {

public:

  explicit MinimalActionServerExample(rclcpp::Node::SharedPtr node)
    :MinimalActionServer(node){}

private:
  rclcpp_action::Server<actionType>::SharedPtr action_server_; 


  virtual void execute(const std::shared_ptr<GoalHandleActionType> goal_handle) override {

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<actionType::Feedback>();
    auto result = std::make_shared<actionType::Result>();
    

    string action_type = goal->goal.actiontype;

    cout<<" got action : "<<action_type<<endl;
    std::map<std::string, std::string> parameters;
    for (auto const& param : goal->goal.parameters) {
        parameters[param.key] = param.val;
        cout<<" val is "<<param.val<<endl;
    }

    if (action_type == "wait"){

      int totalLoop =  atoi(parameters["time"].c_str());
      cout<<" inside wait "<<endl;
      for(int i = 0; i <  totalLoop; i++){
        cout<<" executing "<<action_type<<" i "<<i<<endl;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
        if (goal_handle->is_canceling()) {    
          cout<<"Goal Canceled "<<endl;
          result->resultvalue = false;

          //
          // TODO Wrong usage, probably should be goal_handle->succeed(result)
          // 
          // goal_handle->set_succeeded(result);
          return;
        }
      }
    }


    // Check if goal is done
    if (rclcpp::ok()) {
      cout<<" set Goal Succeeded "<<endl;
      result->resultvalue = true;

      //
      // TODO Wrong usage, probably should be goal_handle->succeed(result)
      // 
      // goal_handle->set_succeeded(result);
    }   

   

    
  } 

  virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle) override {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServerExample::execute, this, _1), goal_handle}.detach();
  } 
  




private:
      rclcpp::Node::SharedPtr g_node_ = nullptr;
  
};
