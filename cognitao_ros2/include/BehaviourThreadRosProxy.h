#include <iostream>

#include <CogniTAO.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_manager/action/action_msg.hpp"

using namespace std;
using actionType=action_manager::action::ActionMsg;

void BehaviourThreadRosProxy_feedback_callback(
  rclcpp_action::ClientGoalHandle<actionType>::SharedPtr,
  const std::shared_ptr<const actionType::Feedback> feedback)
{
    cout<<" get feedback "<<endl; 
}


class BehaviourThreadRosProxy:  public BehaviourThread {

public:
    BehaviourThreadRosProxy(string name):BehaviourThread(name){	
        
        g_node_ = rclcpp::Node::make_shared(name);
        action_client = rclcpp_action::create_client<actionType>(g_node_,"action_manager");    
        actionType_ =  name; 
        cout<<" constructor BehaviourThreadRosProxy "<<endl;
          
    }

   

	virtual void onStart() override {

        // Populate a goal
        auto goal_msg =  actionType::Goal();
        goal_msg.actiontype = actionType_;

        RCLCPP_INFO(g_node_->get_logger(), "Sending goal");

        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg, BehaviourThreadRosProxy_feedback_callback); 

        if (rclcpp::spin_until_future_complete(g_node_, goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node_->get_logger(), "send goal call failed :(");
            return ;
        }        

        goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(g_node_->get_logger(), "Goal was rejected by server");
            return;
        } 

        BehaviourThread::onStart(); 

        //// stop req
        stopReqThread_ = std::thread(&BehaviourThreadRosProxy::loopStopReq, this);
		stopReqThread_.detach();	     
	
	}

    void loopStopReq() {

        rclcpp::Rate loop_rate(1);

        for (int i = 0 ; i < 10; i++ ){

            loop_rate.sleep();

        }

        stopRequested = true;


        
    }



     virtual bool action() override {

        //Wait for the server to be done with the goal
        auto result_future = goal_handle->async_result();

        while (true) {
            cout<<" stopRequested "<<stopRequested<<endl;
            auto wait_result = rclcpp::spin_until_future_complete(
            g_node_,
            result_future,
            std::chrono::seconds(1));
            
            if( rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result){
                if (stopRequested == true){
                    cout<<"finished ----> send cancelllll "<<endl;
                    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
                    break;
                }
            }

            if ( wait_result == rclcpp::executor::FutureReturnCode::SUCCESS){ 
                cout<<"finished ----> server SUCCESS "<<endl;
                break;
            }              
        
        }


        cout<<" rrrr "<<endl;
        return getServerResult(result_future.get()); 
    } 

    bool getServerResult(rclcpp_action::ClientGoalHandle<actionType>::Result result){

        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if(result.response->resultvalue == true ){
                    RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED WITH RETURN TRUE");
                    return true;
                } else {
                    RCLCPP_INFO(g_node_->get_logger(), "SUCCEEDED WITH RETURN FALSE");
                    return false;
                }

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(g_node_->get_logger(), "Goal was aborted");
                return false;
            
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(g_node_->get_logger(), "Goal was canceled");
                return false;
            
            default:
                RCLCPP_ERROR(g_node_->get_logger(), "Unknown result code");
                return false;
        }

    }


private:

    rclcpp::Node::SharedPtr g_node_ = nullptr;
    rclcpp_action::Client<actionType>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<actionType>::SharedPtr goal_handle = nullptr;
    string actionType_;
    std::thread stopReqThread_;


};


