/*
 * Ros2Runner.h
 *
 *  Created on: Dec 23, 2019
 *      Author: yakir huri
 */

#ifndef INCLUDE_ROS2_RUNNER_H_
#define INCLUDE_ROS2_RUNNER_H_

#include <string>
#include <map>
#include <future>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager/action/action_msg.hpp"

#include <CogniTAO.h>


using actionType=action_manager::action::ActionMsg;
using namespace std;

void Ros2Runner_feedback_callback(
  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr,
  const std::shared_ptr<const actionType::Feedback> feedback)
{
    cout<<" got feedback "<<endl; 
}



class Ros2Runner : public Runner{

	atomic<bool> stopRequested;
public:

	Ros2Runner(string action, std::map<std::string, std::string> parameters) : Runner(action,parameters){

		g_node_ = rclcpp::Node::make_shared(action);
        action_client = rclcpp_action::create_client<actionType>(g_node_,"action_manager");

		stopRequested = false;	

		 // Populate a goal
        auto goal_msg =  actionType::Goal();
        goal_msg.actiontype = action_;

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg, Ros2Runner_feedback_callback); 

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

		stopReqThread_ = std::thread(&Ros2Runner::loopStopReq, this);
		stopReqThread_.detach();	     

 
    
       
        cout<<" constructor Ros2Runner "<<endl;
	}

	virtual bool run() override{

		//Wait for the server to be done with the goal
        auto result_future = goal_handle->async_result();

        while (true) {

            auto wait_result = rclcpp::spin_until_future_complete(
            g_node_,
            result_future,
            std::chrono::seconds(1));
            
            if( rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result){
                if (stopRequested == true){
                    cout<<"finished ----> send cancelllll "<<endl;
                    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
                    
					stop();
        			return false;
                }
            }

            if ( wait_result == rclcpp::executor::FutureReturnCode::SUCCESS){ 
                cout<<"finished ----> server SUCCESS "<<endl;

				stop();
				return true;
            }              
        
        }

		stop();
        return false;    

	}

	void loopStopReq() {

        rclcpp::Rate loop_rate(1);

        for (int i = 0 ; i < 100; i++ ){

            loop_rate.sleep();

        }

        stopRequested = true;


        
    }

	virtual void stop() {
		stopRequested = true;
	}
	virtual std::string getType() { return "Ros2Runner";};



private:

    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<actionType>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = nullptr;
    std::thread stopReqThread_;




};


#endif /* INCLUDE_ROS2_RUNNER_H_ */
