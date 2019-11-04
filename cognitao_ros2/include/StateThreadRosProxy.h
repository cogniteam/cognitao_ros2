#include <iostream>

#include <CogniTAO.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager/action/action_msg.hpp"

using namespace std;
using actionType=action_manager::action::ActionMsg;

void StateThreadRosProxy_feedback_callback(
  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr,
  const std::shared_ptr<const actionType::Feedback> feedback)
{
    cout<<" get feedback "<<endl; 
}


class StateThreadRosProxy:  public StateThread {

public:
    StateThreadRosProxy(string name):StateThread(name){	
        
        g_node_ = rclcpp::Node::make_shared(name);
        action_client = rclcpp_action::create_client<actionType>(g_node_,"action_manager");    
        actionType_ =  name;           
    }

	virtual void onStart() override {

        // Populate a goal
        auto goal_msg =  actionType::Goal();
        goal_msg.actiontype = actionType_;

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg, StateThreadRosProxy_feedback_callback); 

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

       
        StateThread::onStart();

	
	}

    virtual void run() override{
       
        rclcpp::Rate loop_rate(1);
        int count = 0;
        while(!stopRequested) {
            count++;
			std::cout<<count<<" stopRequested is "<<stopRequested<<endl;;
            loop_rate.sleep();
		}
        cout<<" stopRequested "<<endl;

  
	}

    virtual void onStop() override{

        StateThread::onStop();

        auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
        if (rclcpp::spin_until_future_complete(g_node_, cancel_result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
            //RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
            //rclcpp::shutdown();
            return ;
        }
        RCLCPP_INFO(g_node_->get_logger(), "goal is being canceled");

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

  
private:

    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<actionType>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = nullptr;
    string actionType_;


};


