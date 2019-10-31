#include <iostream>

#include <CogniTAO.h>
#include "rclcpp/rclcpp.hpp"
#include "cognitao_ros2/msg/event_msg.hpp"


void callback(const cognitao_ros2::msg::EventMsg::SharedPtr msg) {
	cout<<" inside callback "<<endl; 
	WM::setVar(msg->key, msg->value);
}

class RosDataSource:  public MapThreadSafeDataSource {

public:
  RosDataSource(int argc, char **argv){	

		rclcpp::init(argc, argv);
	
		g_node = rclcpp::Node::make_shared("dm_ros_2");
	
		event_pub_ 
			= g_node->create_publisher<cognitao_ros2::msg::EventMsg>("/wme/out", 1000);
		
		event_sub_ = g_node->create_subscription<cognitao_ros2::msg::EventMsg>
		("/wme/in", callback);	

		spinTHread_ = std::thread(&RosDataSource::doSpin, this);
		spinTHread_.detach();	
    }

   ~RosDataSource(){
	    rclcpp::shutdown();
    }

    
    virtual void setVar(std::string variable,std::string value) override{
		sl.lock();
		wm_[variable]=value;
		publishEvent(variable,value);
		sl.unlock();

	}
	virtual std::string getVar(std::string variable) override{
		std::string ret="";
		sl.lock_shared();
		ret = wm_[variable];
		sl.unlock_shared();
		return ret;
	}

	virtual std::string toString() override {
		std::ostringstream strs;
		strs  << wm_;
		std::string str = strs.str();
		return str;
	}

	void publishEvent(std::string variable,std::string value) {

		cognitao_ros2::msg::EventMsg eventMsg; 
		eventMsg.key = variable;
		eventMsg.value = value;
		cout<<"publish msg "<<variable<<", "<<value<<endl;
		event_pub_->publish(eventMsg);
	}
	

 	void doSpin(){
		 
		rclcpp::spin(g_node);

	}

private:
  	
	rclcpp::Subscription<cognitao_ros2::msg::EventMsg>::SharedPtr event_sub_;

	rclcpp::Publisher<cognitao_ros2::msg::EventMsg>::SharedPtr event_pub_;

	std::thread spinTHread_;

	rclcpp::Node::SharedPtr g_node = nullptr;

};


