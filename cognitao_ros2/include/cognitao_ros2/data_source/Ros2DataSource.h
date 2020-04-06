

#ifndef ROS2_DATA_SOURCE_H_
#define ROS2_DATA_SOURCE_H_


//#include <cognitao/data_sources/DataSource.h>
#include <cognitao/CogniTao.h>
#include "rclcpp/rclcpp.hpp"
#include "cognitao_ros2/msg/event_msg.hpp"
#include <cognitao/data_sources/DataSource.h>
// #include <cognitao/CogniTao.h>

using namespace std;



class Ros2DataSource :public DataSource{

public:

   
    /*static DataSource::Ptr create() {
        return DataSource::Ptr(new Ros2DataSource());
    }*/

public:

    Ros2DataSource();

    virtual ~Ros2DataSource(){};

public:

    static DataSource::Ptr create(){
        return DataSource::Ptr(new Ros2DataSource());
    }

protected:

    /**
     * @brief publish events
     * @return bool
     */
    virtual bool publishUpdateEvent(const string &name,
                                    const string &value);

    /**
     * @brief updates the world model
     * @return bool
     */
    void  onDataSourceEvent(const cognitao_ros2::msg::EventMsg::SharedPtr msg);

private:

    void doSpin(){		 
		rclcpp::spin(g_node_);
	}


private:

    rclcpp::Subscription<cognitao_ros2::msg::EventMsg>::SharedPtr event_sub_;

	rclcpp::Publisher<cognitao_ros2::msg::EventMsg>::SharedPtr event_pub_;

	std::thread spinThread_;

	rclcpp::Node::SharedPtr g_node_ = nullptr;       

};



#endif /* ROS2_DATA_SOURCE_H_ */

