#include <inttypes.h>
#include <memory>

#include "action_manager/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <MinimalActionServer.h>

using namespace std;

enum action_code {
    DriveForward_FORVER,
    DriveBackward_FORVER,
    DriveBackward_With_Timer,
    DriveForward_With_Timer,
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

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<actionType::Feedback>();
    auto result = std::make_shared<actionType::Result>();
    bool returnValue = true;

    action_code e = hashit(goal->actiontype);

    switch (e) {

      case DriveForward_FORVER :

        for(;;){
          cout<<" im driving foraward now"<<endl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();
          if (goal_handle->is_canceling()) {
      
            cout<<"DriveForward_FORVER --->Goal Canceled "<<endl;
            cout<<" set Goal Succeeded "<<endl;
            result->resultvalue = returnValue;
            goal_handle->set_succeeded(result);
            return;
          }
        }
        break;

      case DriveBackward_FORVER:
        
        for(;;){
          cout<<" im driving backward now"<<endl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();
          if (goal_handle->is_canceling()) {
      
            cout<<"DriveBackward_FORVER ---> Goal Canceled "<<endl;
            cout<<" set Goal Succeeded "<<endl;
            result->resultvalue = returnValue;
            goal_handle->set_succeeded(result);
            return;
          }
        }
        break;

      case DriveBackward_With_Timer:
        
        for(int i = 0; i < 20; i++){
          cout<<" im driving backward WITH_TIMER now"<<i<<endl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();
          if (goal_handle->is_canceling()) {      
            cout<<"Goal Canceled "<<endl;
            return;
          }
        }
        // set return value for BehaviourRosProxy
        break;  

       case DriveForward_With_Timer:
        
        for(int i = 0; i < 20; i++){
          cout<<" im driving forward WITH_TIMER now"<<i<<endl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();
          if (goal_handle->is_canceling()) {      
            cout<<"Goal Canceled "<<endl;
            return;
          }
        }
        // set return value for BehaviourRosProxy
        break;  



      default:
        cout<<"nothing "<<endl;
        break;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      cout<<" set Goal Succeeded "<<endl;
      result->resultvalue = returnValue;
      goal_handle->set_succeeded(result);
    }   

   

    
  } 

  virtual void handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle) override {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServerExample::execute, this, _1), goal_handle}.detach();
  } 
  action_code hashit (std::string const& inString) {
    if (inString == "DriveForward_FORVER") return DriveForward_FORVER;
    if (inString == "DriveBackward_FORVER") return DriveBackward_FORVER;
    if (inString == "DriveBackward_With_Timer") return DriveBackward_With_Timer;
    if (inString == "DriveForward_With_Timer") return DriveForward_With_Timer;
  }




private:
      rclcpp::Node::SharedPtr g_node_ = nullptr;
  
};