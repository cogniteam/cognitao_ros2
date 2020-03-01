#include <cognitao_ros2/server/MinimalActionServer.h>

MinimalActionServer::MinimalActionServer(rclcpp::Node::SharedPtr node)
{
    using namespace std::placeholders;

    g_node_ = node;

    this->action_server_ = rclcpp_action::create_server<actionType>(
        g_node_,
        "action_manager",
        std::bind(&MinimalActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MinimalActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MinimalActionServer::handle_accepted, this, std::placeholders::_1));
    cout << " constructor " << endl;
}

rclcpp_action::GoalResponse MinimalActionServer::handle_goal(
    const std::array<uint8_t, 16> &uuid,
    std::shared_ptr<const actionType::Goal> goal)
{
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MinimalActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleActionType> goal_handle)
{
    cout << " cancel " << endl;
    RCLCPP_INFO(g_node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MinimalActionServer::execute(const std::shared_ptr<GoalHandleActionType> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<actionType::Feedback>();
    auto result = std::make_shared<actionType::Result>();

    // Check if goal is done
    if (rclcpp::ok())
    {
        goal_handle->set_succeeded(result);
    }
}

void MinimalActionServer::handle_accepted(const std::shared_ptr<GoalHandleActionType> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
}