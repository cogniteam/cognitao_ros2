<div align="center">
  <img src="/ros.jpeg"><br><br>
</div>

-----------------

# COGNITAO_ROS
Decision making system implementation interface and proxy for ROS.

For more information about CogniTAO decision making system see-[https://github.com/cogniteam/cognitao] 

- [Getting Started](#getting-started)
- [Integration](#integration)
- [Prerequisites](#prerequisites)
- [RosDataSource](#send\geT-events)
- [Action manager](#actionmanger)
- [Client side](#clientside)
    - [StateRosproxy](#staterosproxy)
    - [StateThreadRosProxy](#statethreadrosproxy)
-[Server side](#serverside)
- [Contributing](#contributing)
## Getting atarted
First, create your own workspace.
The next step is to download the project into the src directory.
To get a copy of the project up and running on your local machine use Download button.

Clone using :
```
git clone "https://github.com/cogniteam/cognitao_ros.git"
```

Install the CogniTAO library using install.sh script located in[yourWorkspace/src/cognitao_ros].
```
cd yourWorkspace/src/cognitao_ros

//run the install.sh script
./install.sh
```
The install.sh script:
- Downloads the CogniTAO library.
- Compiles the project.

After the script is ran the project will be placed in [yourWorkspace/src/cognitao.git].

## Integration

To use the CogniTAO library in your workspace add to your project file:
```
#include <CogniTAO.h>
```

**CMake**
After writing yout node, dont forget to add to your CMake file the executable file.
```
# CMakeLists.txt
add_executable(dm_ros_node
	  src/main.cpp		
)  
```
## Prerequisites
C++11 support, ROS.

## RosDataSource 
For the worldModel ROS interface- use the RosDataSource when initializing the World Model.
```
WM::init(new RosDataSource(argc, argv));
```

The RosDataSource object listens to a topic called "/wme/out".
When someone publishes an event to this topic, the RosDataSource gets the message and sets the key-value parameters.
To publish via terminal:
```
rostopic pub /wme/in dm_ros/EventMsg '{key: COIN, value: ENT}'
```
** Publish COIN event
The setVar() method publishes to a topic called "/wme/in".
The kind of the messages is EventMsg.

EventMsg:
This message has two fields- both sting type.
The first field called- key, the second- value.
```
string key
string value
```
## Action manager
The action_manager directory contains ActionMsg.h file (located at [action_manager/action]).

The action conatains three fields-

**Goal**- the desirable action that the user want the server to execute.
The goal is expressed by a string which called action type.

**Result**- boolean value that represents whether the operation was successful or not.

**FeedBack** -string that gives feedback about the action's status.
```
# Goal
string actiontype 
---
# Result
bool resultvalue
---
# Feedback
string params
```
## Client side
The client side based on CogniTAO library and supports all the structures it includes.
The structures located in [cognitao_ros2] directory.

### StateRosproxy
The StateProxy is a State.
When creating a new StateRosProxy, the user chooses a name which expresses the state's action(the goal he send to the server).

```
auto s1 = new StateRosProxy("DriveForward_With_Timer");
```
For example, the state's above action is to drive forward for a while.
Each StateProxy has its own action type that starts when the state starts
(by asking the  server to achieve some goal and wait until it's accepted).

Note that the StateRosproxy inherits from a state(not StateThread) so cancelling the task not really stop it's until it done.
Beyond that, the StateRosProxy has the same functionality as State has.

### StateThreadRosProxy
The StateThreadRosProxy is a StateThread.
Similar to StateRosproxy When creating a new StateThreadRosProxy, the user chooses a name which expresses the state's action.
```
auto s2 = new StateRosProxy("DriveForward_With_Timer");

```
The state above operates an actionof driving forward in a separate thread.
If the user decide to cancel the action, its stops.

## Server side
To create the server side for the user's machine create a new package inside [yourWS/src].

Add to the CMake file:
find_package(ament_cmake REQUIRED)
find_package(action_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

ament_target_dependencies(your_action_server_node_name
  "rclcpp"
  "rclcpp_action"
  "action_manager"
  "cognitao_ros2_action_server"
)


include_directories(
	include
	../cognitao_ros2_action_server/include/
)

install(TARGETS your_action_server_node_name
  DESTINATION lib/${PROJECT_NAME})
```
Add to package.xml:
```
```

Create your own server-
## Contributing

Feel free to contact us at info@cogniteam.com if you wish to contribute code to the library

