
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <CogniTAO.h>
#include "../include/RosDataSource.h"
#include "../include/StateRosProxy.h"
#include "actions_manager/action/sum.hpp"





using namespace std;


int main(int argc, char **argv) {


  	std::cout<<"node running"<<std::endl;
    
	WM::init(new RosDataSource(argc, argv));


	UILink link_("/home/maytronics/dm_ros2_ws/src/cognitao.git/www","127.0.0.1",1234);
	link_.start();

	auto s1 = new StateRosProxy<actions_manager::action::Sum>("sum1","Sum");
	auto s2 = new StateRosProxy<actions_manager::action::Sum>("sum2","Sum");

	Machine m;
	auto E1 = new ProtocolTransition ({"TO_FIB"});
	auto E2 = new ProtocolTransition ({"TO_SUM"});

	auto E3 = new ProtocolTransition ({"FIB_AGAIN"});
	auto E4 = new ProtocolTransition ({"SUM_AGAIN"});
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

	
  	std::this_thread::sleep_for(std::chrono::seconds(200));
	m.stop();
	link_.stop();

  
  	return 0;
}

