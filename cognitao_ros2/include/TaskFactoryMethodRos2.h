

#include <CogniTAO.h>

#include "../include/Ros2Runner.h"


class TaskFactoryMethodRos2 :  public TaskFactoryMethod{
public:
	TaskFactoryMethodRos2(){}
	virtual ~TaskFactoryMethodRos2(){}

	virtual Task* createTask(
			std::string type,
			std::string name,
			std::string description,
			std::string variable,
			std::string operation,
			std::string value,
			std::string runner,
			std::map<std::string, std::string> params
	){	
		std::map<std::string, std::string> parameters;

		cerr<<" yakiur "<<endl;
		Ros2Runner* runner_ = new Ros2Runner(name,params);	
		runner_->run();

		cout<< "Building " << name << " " << type << " " << runner << endl;
		Task *tRet = nullptr;

		if(std::strcmp(type.c_str(), "sequencer") == 0) {
			tRet = new BehaviourSequencer(name);
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "selector") == 0) {
			tRet = new BehaviourSelector(name);
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "parallel") == 0) {
			tRet = new BehaviourParallel(name);
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "condition") == 0) {
			tRet = new BehaviourCondition(name,variable,operation,value);
		}
		if(std::strcmp(type.c_str(), "wait") == 0) {
			tRet = new BehaviourWait(name,stoi(value));
			tRet->setDescritpion(description);
		}
		if(std::strcmp(type.c_str(), "inverter") == 0) {
			tRet = new BehaviourInverter(name);
			tRet->setDescritpion(description);
		}
		if(std::strcmp(type.c_str(), "loop") == 0) {
			tRet = new BehaviourLoop(name,atoi(variable.c_str()));
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "set") == 0) {
			tRet = new BehaviourSet(name,variable,value);
		}

		if(std::strcmp(type.c_str(), "task") == 0)
		{
			if(runner=="ros2")
			{	
				cerr<<"1111111111111111111 "<<endl;
				tRet=new BehaviourTask(name);
				cerr<<"222222222 "<<endl;
				//Ros2Runner* runner_ = new Ros2Runner(name,params);	
				// tRet->setRunner(new Ros2Runner(name,params));
				// cerr<<"333333333 "<<endl;
				// tRet->setDescritpion(description);
				// cerr<<"4444444 "<<endl;
				

			}

			else
			{
				tRet = new BehaviourWait(name,2,params);
				tRet->setDescritpion(description);
			}
		}


		if(std::strcmp(type.c_str(), "succeeder") == 0)
		{
			tRet=new BehaviourSucceder();
			tRet->setName(name);
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "failer") == 0)
		{
			tRet=new BehaviourFailer();
			tRet->setName(name);
			tRet->setDescritpion(description);
		}

		if(std::strcmp(type.c_str(), "state") == 0)
		{
			tRet=new State(name.c_str());
		}

		if(std::strcmp(type.c_str(), "trigger") == 0)
		{
			tRet = new BehaviourTrigger(name,variable,operation,value);
		}

		if(tRet == nullptr)
		{
			cout << "ERROR IN XML TYPE " << type << " UNKNOWN EXITING";
			exit(0);
		}
		return tRet;
	}

};
