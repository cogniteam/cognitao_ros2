#include <cognitao_ros2/client/Ros2Runner.h>

void h_sig_sigint(int signum)
{
    std::cout << "Receive signum: " << signum << std::endl;
    rclcpp::shutdown();
}

Ros2Runner::Ros2Runner()
{
    g_node_ = rclcpp::Node::make_shared("action");

    stopRequested = false;
    success_ = false;
}

Ros2Runner::Ros2Runner(const string &action, map<string, string> parameters) : Runner(action, parameters)
{   

    cout<<" yakir 1 "<<endl;
    g_node_ = rclcpp::Node::make_shared("action");
        cout<<" yakir 2 "<<endl;

    stopRequested = false;
    success_ = false;
}
bool Ros2Runner::run()
{
    stopRequested = false;
    
    signal(SIGINT, h_sig_sigint); //to handle with ctrl+c button
    
        cout<<" yakir 3 "<<endl;

    // Populate a goal
    auto goal_msg = actionType::Goal();
         cout<<" yakir 4 "<<endl;

    goal_msg.goal.actiontype = action_;

    for (auto const &x : parameters_)
    {

        ros2_cognitao_msgs::msg::KeyValMsg param;
        param.key = x.first;
        param.val = x.second;
        goal_msg.goal.parameters.push_back(param);
    }

    action_client = rclcpp_action::create_client<actionType>(g_node_, "action_manager");
        cout<<" yakir 5 "<<endl;

    RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node_->get_logger(), "send goal call failed :(");
        return false;
    }

    goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(g_node_->get_logger(), "Goal was rejected by server");
        return false;
    }

    //Wait for the server to be done with the goal
    auto result_future = goal_handle->async_result();

    while (true)
    {

        auto wait_result = rclcpp::spin_until_future_complete(
            g_node_,
            result_future,
            std::chrono::seconds(1));

        if (rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result)
        {

            if (stopRequested == true)
            {
                cout << "finished ----> send cancelllll " << endl;
                auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

                stop();
                success_ = false;
                return success_;
                ;
            }
        }

        if (wait_result == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            cout << "finished ----> server SUCCESS " << endl;

            stop();
            success_ = true;
            return success_;
            ;
        }
    }

    stop();
    return false;
}

void Ros2Runner::stop() { stopRequested = true; }

std::string Ros2Runner::getType() { return "ros2"; };
