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


#include <cognitao_ros2/client/Ros2Runner.h>


void h_sig_sigint(int signum){

    std::cout << "Receive signum: " << signum << std::endl;
    rclcpp::shutdown();
}

Ros2Runner::Ros2Runner(){

    g_node_ = rclcpp::Node::make_shared("action");

    stopRequested = false;
    success_ = false;
}

Ros2Runner::Ros2Runner(const string &action, map<string, string> parameters) : Runner(action, parameters){   
    
    g_node_ = rclcpp::Node::make_shared("action");
    stopRequested = false;
    success_ = false;
}

bool Ros2Runner::run(){

    stopRequested = false;
    success_=false;
    
    signal(SIGINT, h_sig_sigint); //to handle with ctrl+c button    
    

    // Populate a goal
    auto goal_msg = actionType::Goal();
        

    goal_msg.goal.actiontype = action_;
            cout<<" action_"<<action_<<endl;

    for (auto const &x : parameters_)
    {
        cognitao_ros2::msg::KeyValMsg param;
        param.key = x.first;
        param.val = x.second;
        goal_msg.goal.parameters.push_back(param);

    }

    action_client = rclcpp_action::create_client<actionType>(g_node_, "action_manager");


    RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS){

        RCLCPP_ERROR(g_node_->get_logger(), "send goal call failed :(");
        return false;
    }

    goal_handle = goal_handle_future.get();

    if (!goal_handle){

        RCLCPP_ERROR(g_node_->get_logger(), "Goal was rejected by server");

        return false;
    }

    //Wait for the server to be done with the goal
    auto result_future = goal_handle->async_result();

    while (true){

        auto wait_result = rclcpp::spin_until_future_complete(
            g_node_,
            result_future,
            std::chrono::seconds(1));

        if (rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result){

            if (stopRequested == true){

                cout << "finished ----> send cancelllll " << endl;
                auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

                stop();
                success_ = false;
                return success_;
                ;
            }
        }

        if (wait_result == rclcpp::executor::FutureReturnCode::SUCCESS){

            cout << "finished - server SUCCESS " << endl;

            stop();
            success_ = true;

            return success_;
            ;
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
};
