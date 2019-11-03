#include <inttypes.h>
#include <memory>

#include "action_manager/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <MinimalActionServer.h>

using namespace std;

enum action_code {
    DriveForward,
    DriveBackward,
    defaultNum
};





using actionType=action_manager::action::ActionMsg;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;

class MinimalActionServerExample: public MinimalActionServer {

public:

  explicit MinimalActionServerExample(rclcpp::Node::SharedPtr node)
    :MinimalActionServer(node){}

private:
  rclcpp_action::Server<actionType>::SharedPtr action_server_; 


  virtual void execute(const std::shared_ptr<GoalHandleActionType> goal_handle) override {
     cout<<"enter to override exe 1 "<<endl;
    //RCLCPP_INFO(g_node_->get_logger(), "Executing goal");
    cout<<"enter to override exe 2 "<<endl;
  
    const auto goal = goal_handle->get_goal();
    cout<<"enter to override exe 3 "<<endl;

    action_code e = hashit(goal->actiontype);

    cout<<e<<endl;

    switch (e)
    {
    case DriveForward :
      cout<<" im driving foraward now"<<endl;
      break;
        case DriveBackward:
      cout<<" im driving backward now"<<endl;
      break;
    
      default:
      cout<<"nothing "<<endl;
      break;
    }
    
    

    
    // rclcpp::Rate loop_rate(1);
    // const auto goal = goal_handle->get_goal();
    // //auto feedback = std::make_shared<actionType::Feedback>();
    // // auto & sequence = feedback->sequence;
    // // sequence.push_back(0);
    // // sequence.push_back(1);
    // auto result = std::make_shared<actionType::Result>();

    // cout<<"i got "<<goal->actiontype <<endl;


    // // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // //   // Check if there is a cancel request
    // //   if (goal_handle->is_canceling()) {
    // //     //result->sequence = sequence;
    // //     goal_handle->canceled(result);
    // //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    // //     return;
    // //   }
    // //   // Update sequence
    // //   //sequence.push_back(sequence[i] + sequence[i - 1]);
    // //   // Publish feedback
    // //   //goal_handle->publish_feedback(feedback);
    // //   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    // //   loop_rate.sleep();
    // // }

    // // Check if goal is done
    // if (rclcpp::ok()) {
    //   //result->sequence = sequence;
    //   goal_handle->set_succeeded(result);
    //   RCLCPP_INFO(g_node_->get_logger(), "Goal Succeeded");
    // }
    } 

    virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle) override {
      cout<<"enter to override handle"<<endl;
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&MinimalActionServerExample::execute, this, _1), goal_handle}.detach();
    } 
    action_code hashit (std::string const& inString) {
      if (inString == "DriveForward") return DriveForward;
      if (inString == "DriveBackward") return DriveBackward;

      cout<<"bla"<<endl;
      return defaultNum;
    }
private:
      rclcpp::Node::SharedPtr g_node_ = nullptr;
  
};