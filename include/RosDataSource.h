#include <iostream>

#include <CogniTAO.h>
#include "rclcpp/rclcpp.hpp"
#include "dm_ros2/msg/event_msg.hpp"

class RosDataSource: public rclcpp::Node, public MapThreadSafeDataSource {

public:
  RosDataSource()
  : Node("dm_ros_2"){


		event_pub_ 
		 	= this->create_publisher<dm_ros2::msg::EventMsg>("/wme/out", 1000);		

		event_sub_ = this->create_subscription<dm_ros2::msg::EventMsg>(
			"/wme/in", std::bind(&RosDataSource::callback, this, std::placeholders::_1));

		spinTHread_ = std::thread(&RosDataSource::doSpin, this);
		spinTHread_.detach();		

  }

  ~RosDataSource(){
  }

  void callback(const dm_ros2::msg::EventMsg::SharedPtr msg) {
	  cout<<"inside callllllback "<<endl;
	  WM::setVar(msg->key, msg->value);
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

		dm_ros2::msg::EventMsg eventMsg; 
		eventMsg.key = variable;
		eventMsg.value = value;
		cout<<"publish msg "<<variable<<", "<<value<<endl;
		event_pub_->publish(eventMsg);

	}
	

 	void doSpin(){
	
		auto node = rclcpp::Node::SharedPtr(this);
		rclcpp::spin(node);
	}

private:
  	
	rclcpp::Subscription<dm_ros2::msg::EventMsg>::SharedPtr event_sub_;

	rclcpp::Publisher<dm_ros2::msg::EventMsg>::SharedPtr event_pub_;

	std::thread spinTHread_;

};

