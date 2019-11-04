#include <iostream>

#include <CogniTAO.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager/action/action_msg.hpp"

using namespace std;
using actionType=action_manager::action::ActionMsg;

void BehaviourRosProxy_feedback_callback(
  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr,
  const std::shared_ptr<const actionType::Feedback> feedback)
{
    cout<<" get feedback "<<endl; 
}


class BehaviourRosProxy:  public Behaviour {

public:
    BehaviourRosProxy(string name):Behaviour(name){	
        
        g_node_ = rclcpp::Node::make_shared(name);
        action_client = rclcpp_action::create_client<actionType>(g_node_,"action_manager");    
        actionType_ =  name; 
        cout<<" constructor BehaviourRosProxy "<<endl;
          
    }

   

	virtual void onStart() override {

        // Populate a goal
        auto goal_msg =  actionType::Goal();
        goal_msg.actiontype = actionType_;

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg, BehaviourRosProxy_feedback_callback); 

        if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node_->get_logger(), "send goal call failed :(");
            return ;
        }        

        goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was rejected by server");
            return;
        } 

        run();            
	
	}

    virtual void run() override{
       
       Behaviour::run();
  
	}

     virtual bool action() override {

         // Wait for the server to be done with the goal
        auto result_future = goal_handle->async_result();

        RCLCPP_INFO(g_node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(g_node_, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS ) {
            return false;
        }              
        
        rclcpp_action::ClientGoalHandle<actionType>::Result result = result_future.get();

        
        return getResult(result); 

       
    }

    bool getResult(rclcpp_action::ClientGoalHandle<actionType>::Result result){

        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if(result.response->resultvalue == true ){
                    RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED WITH RETURN TRUE");
                    return true;
                } else {
                    RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED WITH RETURN FALSE");
                    return false;
                }

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(g_node_->get_logger(), "Goal was aborted");
                return false;
            
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(g_node_->get_logger(), "Goal was canceled");
                return false;
            
            default:
                RCLCPP_ERROR(g_node_->get_logger(), "Unknown result code");
                return false;
        }

    }

    virtual void onStop() override {

        
    } 

private:

    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<actionType>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = nullptr;
    string actionType_;

};


