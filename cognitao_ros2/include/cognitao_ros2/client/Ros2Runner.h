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

#include <CogniTao.h>

using actionType = action_manager::action::ActionMsg;

using namespace std;

/*
void Ros2Runner_feedback_callback(
  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr,
  const std::shared_ptr<const actionType::Feedback> feedback)
{
    cout<<" got feedback "<<endl; 
}
*/


class Ros2Runner : public Runner
{

    atomic<bool> stopRequested;
    bool success_;

public:
    Ros2Runner();

    Ros2Runner(const string &action, map<string, string> parameters);


    virtual bool run() override;

    virtual void stop();

    virtual std::string getType(); 

private:
    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<actionType>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = nullptr;
    // std::thread stopReqThread_;
};

#endif /* INCLUDE_ROS2_RUNNER_H_ */
