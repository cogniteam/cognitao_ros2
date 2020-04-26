#include <inttypes.h>
#include <memory>

#include "cognitao_ros2/action/action_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "../server/Ros2ActionServer.h"

using namespace std;

class Ros2ActionWaitServer: public Ros2ActionServer {

public:

  Ros2ActionWaitServer(rclcpp::Node::SharedPtr node, std::string action)
    :Ros2ActionServer(node, action){    
  }

  ~Ros2ActionWaitServer(){}   

private:

  virtual void onStart(const std::shared_ptr<GoalHandleActionType> goal_handle) override {   
    
    Ros2ActionContext ros2ActionContext(goal_handle);
    
    std::thread{std::bind(&Ros2ActionWaitServer::execute, this, _1), 
        ros2ActionContext}.detach();
  } 

  virtual void execute(Ros2ActionContext ros2ActionContext) override {

    rclcpp::Rate loop_rate(1);

    int totalLoop =  atoi(ros2ActionContext.getParameters()["time"].c_str());

    for(int i = 0; i <  totalLoop; i++){
      cout<<i<<endl;
      loop_rate.sleep();

      //cancel goal
      if (ros2ActionContext.getGoalHandle()->is_canceling()) {             
        ros2ActionContext.setResult(false);
        return;
      }
    }  

    // Check if goal is done
    if (rclcpp::ok()) {
      ros2ActionContext.setResult(true);
    }     
  }   
  
};
