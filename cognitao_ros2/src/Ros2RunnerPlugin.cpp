

#include <iostream>
#include <cognitao_ros2/client/Ros2Runner.h>
#include <atomic>
#include <CogniTao.h>

static std::atomic<int> counter(0);

extern "C" Runner *create_runner()
{
  if (!rclcpp::ok())
  {
    cout<<" init "<<endl;
    rclcpp::init(0, nullptr);
  }
  counter++;
  return new Ros2Runner();
}

extern "C" void destroy_runner(Runner *object)
{
  counter --;
  delete object;
  if(counter==0) rclcpp::shutdown();

}

extern "C" const char *get_runner_type()
{
  return "ros2_runner";
}

int main()
{
  // Runner *r1 = create_runner();
  // Runner *r2 = create_runner();
  // map<string, string> map_;
  // map_["time"] = "10.0";
  // r1->setAction("wait");
  // r1->setParameters(map_);
  // r1->run();
  // r2->setAction("wait");
  // r2->setParameters(map_);
  // r2->run();

  // // Runner *r1 = create_runner();
  // //  map<string, string> map_;
  // // map_["time"] = "10.0";
  // // r1->setAction("wait");
  // // r1->setParameters(map_);
  // // r1->run();
  return 0;
}
