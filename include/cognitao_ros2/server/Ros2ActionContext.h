/**
 * @brief ROS 2Action Wait Server
 * 
 * @file Ros2ActionContext.h
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


#ifndef INCLUDE_ROS2_ACTION_CONTEXT_H_
#define INCLUDE_ROS2_ACTION_CONTEXT_H_


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;
using actionType=cognitao_ros2::action::Action;
using GoalHandleActionType = rclcpp_action::ServerGoalHandle<actionType>;
using namespace std::placeholders;


class Ros2ActionContext {
  
public:

  Ros2ActionContext (const std::shared_ptr<GoalHandleActionType> goal_handle){
    goal_handle_ = goal_handle; 

    const auto goal = goal_handle_->get_goal();   

    for (auto const& param : goal->goal.parameters){
      parameters_[param.key] = param.val;      
    }

  }

  std::map<std::string, std::string> getParameters() const{
      return parameters_;
  }

  std::shared_ptr<GoalHandleActionType> getGoalHandle() const{
    return goal_handle_;
  }

  void setResult(bool resultVal) {
    auto result = std::make_shared<actionType::Result>(); 
    result->resultvalue = resultVal;
    goal_handle_->succeed(result);

  }

private:

  std::shared_ptr<GoalHandleActionType> goal_handle_;
  std::map<std::string, std::string> parameters_;

};

#endif /* INCLUDE_ROS2_ACTION_CONTEXT_H_ */

