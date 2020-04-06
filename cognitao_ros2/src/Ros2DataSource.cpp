
#include <cognitao_ros2/data_source/Ros2DataSource.h>

using std::placeholders::_1;

using namespace std;


Ros2DataSource::Ros2DataSource()
{

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    cout<<" yakir "<<endl;
    g_node_ = rclcpp::Node::make_shared("cognitao_ros2");

    event_pub_ =
        g_node_->create_publisher<cognitao_ros2::msg::EventMsg>("/wme/in", 1000);

    event_sub_ = g_node_->create_subscription<cognitao_ros2::msg::EventMsg>("/wme/in"
        , std::bind(&Ros2DataSource::onDataSourceEvent, this,_1));


    spinThread_ = std::thread(&Ros2DataSource::doSpin, this);
    spinThread_.detach();
}

bool Ros2DataSource::publishUpdateEvent(const string &name, const string &value)
{
    
    cognitao_ros2::msg::EventMsg eventMsg;
    // ros_data_source::msg::EventMsg eventMsg;

    eventMsg.key = name;

    eventMsg.value = value;

    cout << "publish msg " << eventMsg.key << ", " << eventMsg.value << endl;

    event_pub_->publish(eventMsg);

    return true;
}
void Ros2DataSource::onDataSourceEvent(const cognitao_ros2::msg::EventMsg::SharedPtr msg)
{
    //     cout << " inside callback " << msg->key << " " << msg->value << endl;


    DataSource::variableUpdated(msg->key, msg->value);
}
