/**
 * @brief ROS 2 cognitao runner 
 * 
 * @file Ros2Runner.h
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


#ifndef INCLUDE_ROS2_RUNNER_H_
#define INCLUDE_ROS2_RUNNER_H_


#include <string>
#include <map>
#include <future>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cognitao/CogniTao.h>

#include <cognitao_ros2/action/action.hpp>
#include <cognitao_ros2/msg/event.hpp>


using actionType =cognitao_ros2:: action::Action;
using GoalHandleActionType = rclcpp_action::ClientGoalHandle<actionType>;

using namespace std::placeholders;
using namespace std;


/**
 * Allows to receive data, manage callback and publish da
 */
class Ros2Runner : public Runner {

public:

    /**
     * @brief Construct a new Ros 2 Runner object
     */
    Ros2Runner();

    /**
     * @brief Destroys the Ros 2 Runner object
     */
    ~Ros2Runner();

public:

    /**
     * @brief execute task
     * @return bool 
     */
    virtual bool run() override;

    /**
     * @brief Gets the Type
     * @return std::string 
     */
    virtual std::string getType() override; 

    /**
     * @brief stop executing task 
     */
    virtual void stop() override; 

private:

    atomic<bool> stopRequested_;    

    rclcpp::Node::SharedPtr nodeHandle_;

    rclcpp_action::Client<actionType>::SharedPtr client_;

    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goalHandle_;

};


#endif /* INCLUDE_ROS2_RUNNER_H_ */
