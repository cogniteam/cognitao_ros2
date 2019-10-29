#include <iostream>

#include <CogniTAO.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_manager/action/sum.hpp"

using Sum = actions_manager::action::Sum;

using namespace std;


void feedback_callback(
  rclcpp_action::ClientGoalHandle<Sum>::SharedPtr,
  const std::shared_ptr<const Sum::Feedback> feedback)
{
//   RCLCPP_INFO(
//     g_node->get_logger(),
//     "Next number in sequence received: %" PRId64,
//     feedback->sequence.back());
       cout<<" get feedback "<<endl; 
}

class StateRosProxy:  public State {

public:
    StateRosProxy(string name):State(name){	
        
        g_node_ = rclcpp::Node::make_shared("minimal_action_client");
        action_client = rclcpp_action::create_client<Sum>(g_node_,"Sum"); 

        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            RCLCPP_ERROR(g_node_->get_logger(), "Action server not available after waiting");
            return ;
        }

       
        
    }

	// virtual void onStart() override {

       

              
	
	// }

    virtual void run() override{
        
        globalSum_ = 0;
        // Populate a goal
        auto goal_msg = Sum::Goal();
        int num1 = 6;
        int num2 = 5;
        goal_msg.numbers.push_back(num1);
        goal_msg.numbers.push_back(num2);

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg, feedback_callback); 

        if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(g_node_->get_logger(), "send goal call failed :(");
            return ;
        }
        

        goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was rejected by server");
            return;
        }

        // spinTHread_ = std::thread(&StateRosProxy::doSpin, this);
		// spinTHread_.detach();	

        // Wait for the server to be done with the goal
        auto result_future = goal_handle->async_result();

        RCLCPP_INFO(g_node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(g_node_, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS )
        {
            
            return ;
        }
              
        
        rclcpp_action::ClientGoalHandle<Sum>::Result result = result_future.get(); ;



        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED");
            break;
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was aborted");
            return ;
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was canceled");
            return ;
            default:
            RCLCPP_ERROR(g_node_->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(g_node_->get_logger(), "result received");
        
        globalSum_ = result.response->sum;    
        cout<<" the sum is "<<globalSum_<<endl;;
    

        // action_client.reset();
        // g_node_.reset();
	}

    virtual void onStop() override {

        // std::cout<<"1111111111111 "<<endl;
        // // Cancel the goal since it is taking too long
        // auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
        // std::cout<<"222222222 "<<endl;

        // if (rclcpp::spin_until_future_complete(g_node_, cancel_result_future) !=
        // rclcpp::executor::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_ERROR(g_node_->get_logger(), "failed to cancel goal");
        //     rclcpp::shutdown();
        //     return;
        // }
        // RCLCPP_INFO(g_node_->get_logger(), "goal is being canceled");
        // std::cout<<"333333333333 "<<endl;


    }

    // void doSpin(){
		 

    //     cout<<" start thread "<<endl; 
    //     for(;;){
    //         cout<<" stopRequested " <<stopRequested<<"globalSum_ "<<globalSum_<<endl;
    //         if( stopRequested == true && globalSum_ == 0){
    //             cout<<" yakir asked to stop "<<endl;
    //             auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

    //             if (rclcpp::spin_until_future_complete(g_node_, cancel_result_future) !=
    //                 rclcpp::executor::FutureReturnCode::SUCCESS)
    //             {
    //                 RCLCPP_ERROR(g_node_->get_logger(), "failed to cancel goal");
    //                 rclcpp::shutdown();
    //                 return;
    //             }
    //             cout<<" SUCCESS cancel"<<endl;
    //             RCLCPP_INFO(g_node_->get_logger(), "goal is being canceled");
    //             RCLCPP_ERROR(g_node_->get_logger(), "get result call failed :(");

    //             return;
    //         }

    //         if ( globalSum_ != 0 ){
    //             cout<<" return from spiiiiin "<<endl;
    //             return;
    //         }
          
    //     }

	// }
 


 

private:


    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<Sum>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<Sum>::SharedPtr goal_handle = nullptr;
	std::thread spinTHread_;
    int globalSum_ = 0;
  	
	

};


