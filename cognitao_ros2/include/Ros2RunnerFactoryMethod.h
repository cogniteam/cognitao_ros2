#include <CogniTAO.h>

class Ros2RunnerFactoryMethod : public RunnerFactoryMethod
{
public:
    Ros2RunnerFactoryMethod() {}
    virtual ~Ros2RunnerFactoryMethod() {}

    virtual Runner *createRunner(
        std::string action,
        BehaviourTask *t)
    {

        Runner *tRet = nullptr;
        tRet = new Ros2Runner(action, t->getParameters());
        cout << "USER RUNNER CREATED" << endl;
        return tRet;
    }
};
