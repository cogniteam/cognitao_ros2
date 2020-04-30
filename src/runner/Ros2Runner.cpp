/*
 * Ros1Runner.cpp
 * 
 * @author Lin Azan (lin@cogniteam.com)
 * @date 2020-03-15
 * @copyright Cogniteam (c) 2020
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2010-2020 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <cognitao_ros2/runner/Ros2Runner.h>

Ros2Runner::Ros2Runner(){
  stopRequested = false;
}

Ros2Runner::~Ros2Runner(){
  
}

void Ros2Runner::setAction(const std::string &action){

   action_ = action;
   g_node_ = rclcpp::Node::make_shared(action_);
   client_ = rclcpp_action::create_client<actionType>(g_node_,action_);
}

 bool Ros2Runner::run(){ 

  if (!this->client_) {
    return false;
  }

  if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
    return false;
  }

  // Populate a goal
  auto goal_msg = actionType::Goal();        

  goal_msg.goal.actiontype = action_;
  for (auto const &x : parameters_) {
      cognitao_ros2::msg::KeyValue param;
      param.key = x.first;
      param.val = x.second;
      goal_msg.goal.parameters.push_back(param);
  }

  // Ask server to achieve some goal and wait until it's accepted
  
  auto goal_handle_future = client_->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
      stop(); 
      return false;
  }

  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
      stop(); 
      return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_->async_get_result(goal_handle);

  while(rclcpp::ok()){

    auto wait_result = rclcpp::spin_until_future_complete(g_node_, 
        result_future, std::chrono::seconds(1));

    // canceled or ctrl+c
    if (stopRequested == true){
        auto cancel_result_future = client_->async_cancel_goal(goal_handle);
        if (rclcpp::spin_until_future_complete(g_node_, cancel_result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node_->get_logger(), "failed to cancel goal");
            stop();
            return false;
        }
        stop();
        return false;
    }  

    //server FINISHED
    if (wait_result == rclcpp::executor::FutureReturnCode::SUCCESS){
      ///get true or false from the server
      rclcpp_action::ClientGoalHandle<actionType>::WrappedResult wrapped_result
           = result_future.get();
      stop();
      return wrapped_result.result->resultvalue;     
    }      
  }

  stop(); 
  return false;

 }


void Ros2Runner::stop(){
  stopRequested = true; 
}

std::string Ros2Runner::getType(){    
  return "ros2";
}


