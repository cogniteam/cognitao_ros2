
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <CogniTAO.h>
#include "../include/RosDataSource.h"


using namespace std;


int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  std::cout<<"node running"<<std::endl;
    
	WM::init(new RosDataSource());

	UILink link_("/home/maytronics/dm_ros2_ws/src/dm/www");
	link_.start();

	auto s1 = new State("locked");
	auto s2 = new State("unlocked");

	Machine m;
	auto E1 = new ProtocolTransition ({"PUSH"});
	auto E2 = new ProtocolTransition ({"COIN"});

	auto E3 = new ProtocolTransition ({"PUSH_BACK"});
	auto E4 = new ProtocolTransition ({"COIN_BACK"});
	m.setInitialTask(s1);
	
	m.addLink(s2,s1,E1);
	
	m.addLink(s2,s2,E4);
	m.addLink(s1,s2,E2);
	m.addLink(s1,s1,E3);


	Task * stateS1 =TaskFactory::createTask("state","root");
	stateS1->setMachine(&m);
	WM::setVar("GRAPH", StateJSONWriter::toString(stateS1)  );




	//cout<<StateJSONWriter::toString(stateS1)<<endl;



	m.start();  
	
  std::this_thread::sleep_for(std::chrono::seconds(120));
	cout<<" stop everything "<<endl;
	m.stop();
	link_.stop();
  rclcpp::shutdown();
  return 0;
}

