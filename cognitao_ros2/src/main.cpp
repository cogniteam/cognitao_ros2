#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <CogniTAO.h>
#include "../include/RosDataSource.h"

#include "../include/StateRosProxy.h"
#include "../include/StateThreadRosProxy.h"
#include "../include/BehaviourRosProxy.h"

using namespace std;


int main(int argc, char **argv) {

  	std::cout<<"node running"<<std::endl;
    
	WM::init(new RosDataSource(argc, argv));


	UILink link_("/home/maytronics/dm_ros2_ws/src/desicion_making/cognitao.git/www","127.0.0.1",1234);
	link_.start();

	bool USE_STATE = false;
	bool USE_STATE_THREAD = true;
	bool USE_BEHAVIOUR = true;
	bool USE_BEHAVIOUR_THREAD = false;

	if (USE_STATE ){
		auto s1 = new StateRosProxy("DriveForward");
		auto s2 = new StateRosProxy("DriveBackward");

		Machine m;
		auto E1 = new ProtocolTransition ({"TO_DriveForward"});
		auto E2 = new ProtocolTransition ({"TO_DriveBackward"});

		m.setInitialTask(s1);
		
		m.addLink(s2,s1,E1);
		m.addLink(s1,s2,E2);


		State * stateS1 = (State*) TaskFactory::createTask("state","root");
		stateS1->setMachine(&m);
		WM::setVar("GRAPH", StateJSONWriter::toString(stateS1)  );

		m.start(); 

		std::this_thread::sleep_for(std::chrono::seconds(1000));
		m.stop();
		link_.stop();
	}


	if (USE_STATE_THREAD ){
		auto s1 = new StateThreadRosProxy("DriveForward_With_Timer");
		auto s2 = new StateThreadRosProxy("DriveBackward_With_Timer");

		Machine m;
		auto E1 = new ProtocolTransition ({"TO_DriveForward"});
		auto E2 = new ProtocolTransition ({"TO_DriveBackward"});

		m.setInitialTask(s1);
		
		m.addLink(s2,s1,E1);
		m.addLink(s1,s2,E2);


		State * stateS1 = (State*) TaskFactory::createTask("state","root");
		stateS1->setMachine(&m);
		WM::setVar("GRAPH", StateJSONWriter::toString(stateS1)  );

		m.start(); 

		std::this_thread::sleep_for(std::chrono::seconds(1000));
		m.stop();
		link_.stop();
	}

	if (USE_BEHAVIOUR){
		auto s1 = new BehaviourRosProxy("DriveForward_With_Timer");
		auto s2 = new BehaviourRosProxy("DriveBackward_With_Timer");

		Behaviour * BehaviourS1 = (Behaviour*) TaskFactory::createTask("seq","root");
		BehaviourS1->addChild(s1);
		BehaviourS1->addChild(s2);

		WM::setVar("GRAPH", BehaviourJSONWriter::toString(BehaviourS1)  );

		BehaviourS1->start();

		link_.stop();
	}

	

	

  
  	return 0;
}
