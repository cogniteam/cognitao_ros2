
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;
using actionType=cognitao_ros2::action::ActionMsg;
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
