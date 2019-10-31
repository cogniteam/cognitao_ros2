#include <iostream>

#include <CogniTAO.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager/action/action_msg.hpp"

using namespace std;
using actionType=action_manager::action::ActionMsg;

// void feedback_callback(
//   rclcpp_action::ClientGoalHandle<Sum>::SharedPtr,
//   const std::shared_ptr<const Sum::Feedback> feedback)
// {
// //   RCLCPP_INFO(
// //     g_node->get_logger(),
// //     "Next number in sequence received: %" PRId64,
// //     feedback->sequence.back());
//        cout<<" get feedback "<<endl; 
// }


class StateRosProxy:  public State {

public:
    StateRosProxy(string name, string action):State(name){	
        
        g_node_ = rclcpp::Node::make_shared(name);
        action_client = rclcpp_action::create_client<actionType>(g_node_,action);    
        actionType_ =  action;           
    }

	virtual void onStart() override {

        // Populate a goal
        auto goal_msg =  actionType::Goal();
        goal_msg.actiontype = actionType_;

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg/*, feedback_callback*/); 

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
       
        // Wait for the server to be done with the goal
        auto result_future = goal_handle->async_result();

        RCLCPP_INFO(g_node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(g_node_, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS ) {
            return;
        }              
        
        rclcpp_action::ClientGoalHandle<actionType>::Result result = result_future.get();

        printResult(result);

        RCLCPP_INFO(g_node_->get_logger(), "result received");
        
      
	}

    void printResult(rclcpp_action::ClientGoalHandle<actionType>::Result result){

        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED");
            break;
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was aborted");
            return ;
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was canceled");
            return ;
            default:
            RCLCPP_ERROR(g_node_->get_logger(), "Unknown result code");
            return;
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


