#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <CogniTAO.h>
#include "../include/RosDataSource.h"

#include "../include/StateRosProxy.h"
#include "../include/StateThreadRosProxy.h"
#include "../include/BehaviourRosProxy.h"
#include "../include/BehaviourThreadRosProxy.h"


using namespace std;


int main(int argc, char **argv) {

    
	WM::init(new RosDataSource(argc, argv));

	const char* homeDir = getenv("HOME");
	
	string path = string(homeDir)+ "/dm_ros2_ws/src/dm_ros2/cognitao.git/www";
	UILink link_(path.data(),"127.0.0.1",1234);
	cout<<path<<endl;
	link_.start();

	bool USE_STATE = false;
	bool USE_STATE_THREAD = false;
	bool USE_BEHAVIOUR = false;
	bool USE_BEHAVIOUR_THREAD = true;

	if (USE_STATE ){
		auto s1 = new StateRosProxy("DriveForward_With_Timer");
		auto s2 = new StateRosProxy("DriveBackward_With_Timer");

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
		auto s1 = new StateThreadRosProxy("DriveForward_FORVER");
		auto s2 = new StateThreadRosProxy("DriveBackward_FORVER");

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
		Machine m;

		auto s1 = new BehaviourThreadRosProxy("DriveForward_With_Timer");
		auto s2 = new BehaviourThreadRosProxy("DriveBackward_With_Timer");

		Behaviour * BehaviourS1 = (Behaviour*) TaskFactory::createTask("seq","root");
		BehaviourS1->addChild(s1);
		BehaviourS1->addChild(s2);

		WM::setVar("GRAPH", BehaviourJSONWriter::toString(BehaviourS1)  );

		m.setInitialTask(BehaviourS1);
		m.start();
		while(!BehaviourS1->isFinished())
			std::this_thread::sleep_for(std::chrono::seconds(1000));
		link_.stop();
	}

	if (USE_BEHAVIOUR_THREAD){
		
		Machine m;

		auto s1 = new BehaviourThreadRosProxy("DriveForward_FORVER");
		auto s2 = new BehaviourThreadRosProxy("DriveBackward_FORVER");

		BehaviourThread * BehaviourS1 = (BehaviourThread*) TaskFactory::createTask("par","root");
		BehaviourS1->addChild(s1);
		BehaviourS1->addChild(s2);

		WM::setVar("GRAPH", BehaviourJSONWriter::toString(BehaviourS1)  );

		m.setInitialTask(BehaviourS1);
		m.start();
		while(!BehaviourS1->isFinished())
			std::this_thread::sleep_for(std::chrono::seconds(1000));
		link_.stop();
	}
  
  	return 0;
}

