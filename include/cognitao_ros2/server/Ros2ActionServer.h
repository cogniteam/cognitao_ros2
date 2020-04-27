/**
 * @brief ROS 2 Action Server
 * 
 * @file Ros2ActionServer.h
 * 
 * @author Lin Azan (lin@cogniteam.com)
 * @date 2020-03-15
 * @copyright Cogniteam (c) 2020
 *    
 * MIT License
 *   
 * Permission is hereby granted, free of charge, to any person obtaining a  copy
 * of this software and associated documentation files (the 'Software'), to deal
 * in the Software without restriction, including without  limitation the rights
 * to use, copy, modify, merge,  publish,  distribute,  sublicense,  and/or sell
 * copies of the Software, and  to  permit  persons  to  whom  the  Software  is 
 * furnished to do so, subject to the following conditions:
 *   
 * The   above   copyright   notice   and  this   permission   notice  shall  be
 * included in all copies or substantial portions of the Software.
 *   
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY  KIND,  EXPRESS  OR
 * IMPLIED, INCLUDING BUT NOT LIMITED  TO  THE  WARRANTIES  OF  MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN  NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE  LIABLE  FOR  ANY  CLAIM,  DAMAGES  OR  OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */


#ifndef INCLUDE_ROS2_ACTION_SERVER_H_
#define INCLUDE_ROS2_ACTION_SERVER_H_

#include <inttypes.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "Ros2ActionContext.h"


using namespace std;
using actionType=cognitao_ros2::action::Action;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;
using namespace std::placeholders;


class Ros2ActionServer
{
public: 

  Ros2ActionServer(rclcpp::Node::SharedPtr node, std::string action) {

    g_node_ = node;

    this->server_ = rclcpp_action::create_server<actionType>(
      g_node_,
      action,
      std::bind(&Ros2ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Ros2ActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Ros2ActionServer::onStart, this, std::placeholders::_1)
      );
      RCLCPP_INFO(g_node_->get_logger(), "init server");
  }

  virtual ~Ros2ActionServer(){};
  

private:

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid,
    std::shared_ptr<const actionType::Goal> goal) {
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

  virtual void execute(Ros2ActionContext ros2ActionContext) = 0;
  
  virtual void onStart(const std::shared_ptr<GoalHandleActionType> goal_handle) = 0;

private:
    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Server<actionType>::SharedPtr server_;

};

#endif /* INCLUDE_ROS2_ACTION_SERVER_H_ */

