# armadillo_hl_interface package
## General description
This package includes separate interfaces for different robot actuators and modules (e.g. arm, driving, torso...), wrapped by c++ classes.
In addition, it contains a basic framework for creating more complex actions using a finite state machine.  
### Execution modes
As a rule of thumb, every basic action can execute in 3 different 'modes'. For example, the *TorsoInterface::move(...)* methods:

1. `void move(double height);`  
Move torso to the given height.

2. `bool move_block(double height);`  
Move torso to the given height, block until execution is done.  
Return *true* upon success, *false* otherwise.

3. `void move(const CallbackBool callback, double height);`  
Move torso to a given height.  
Execute callback with *true* value upon success, *false* otherwise.

The *CallbackBool* parameter is a function of type `void callback(bool result){...}`, which will be called when the action finish executing. A simple code example:

```
void cb(bool result){
    ROS_INFO("torso moved!");
}

void main(int argc, char **argv){
    ...
    TorsoInterface ti;
    ti.move(&cb, 0.2);
    ...
}

```
### Actions with coordinate parameters
Actions which get coordinates as parameters comes in 2 flavors:  
1. Get coordinates as separate parameters of type *double* (one for each dimension).  
In this case, coordinates are relative to robot.
2. Get coordinates as one packed parameter of type *geometry_msgs::Pose*.  
In this case, coordinates are relative to map.

For example:
```
ObjectHandler oh;
ArmInterface ai;
geometry_msgs::Pose pose;

// get the can's position relative to map
if(oh.find_object(pose, "can"){
    // move to can's position relative to map
    ai.move_block(pose);
}

// move relative to robot
ai.move(0.3, 0.0, 0.3);
```

## Interface classes

### HeadInterface (head_interface.h)

Controls the Armadillo's head motion.  
Methods:
```
// point relative to map
bool point_head_block(const geometry_msgs::Pose &pose, double vel=1.0);
void point_head(const geometry_msgs::Pose &pose, double vel=1.0);
void point_head(const CallbackBool callback, const geometry_msgs::Pose &pose, double vel=1.0);

// point relative to robot
bool point_head_block(double x, double y, double z, double vel=1.0);
void point_head(double x, double y, double z, double vel=1.0);
void point_head(const CallbackBool callback, double x, double y, double z, double vel=1.0);

// cancel head motion
void stop();
```

### TorsoInterface (torso_interface.h)
Controls the Armadillo's torso elevator.  
Methods:
```
bool move_block(double height);
void move(double height);
void move(const CallbackBool callback, double height);
```
Constants:  
```
static const float MIN_HEIGHT = MIN_HEIGHT_TORSO;
static const float MAX_HEIGHT = MAX_HEIGHT_TORSO;
```

### DriverInterface (driver_interface.h)
DriverInterface currently offers 2 types of navigation:  
Simple navigation is done relative to the robot, and doesn't use move_base (hence- no collision checking, use with care!).  
Methods for simple navigation:  
```
bool drive_block(double dist, double z, double vel=0.2);
void drive(double dist, double z, double vel=0.2);
void drive(const CallbackBool callback, double dist, double z, double vel);
```
Normal navigation is done using move-base, and takes an extra parameter- *radius*. For *radius*>0, move_base will try to navigate to the closest available point in the given radius from the given position.  
Methods for normal navigation:  
```
bool drive_block(geometry_msgs::Pose &pose, double radius=0);
void drive(geometry_msgs::Pose &object, double radius=0);
void drive(const CallbackBool callback, geometry_msgs::Pose &object, double radius=0);
```
Other methods:
```
void stop();
```

### ArmInterface (arm_interface.h)
**To be uploaded to git soon!**

### ObjectHandler (object_handler.h)
Used to communicate with moveit PlanningSceneInterface, and in the future will hold different methods for identifying and handling objects, as well as objects database.  
Note that currently both robotican's *find_object* and *object_handler* nodes needs to be initiated in-order to use this class.  
Current available methods:
```
bool find_object(geometry_msgs::Pose &target, const std::string &name);
```

## FSM framework
The FSM framework allows one to build simple finite state machines by assembling different types of nodes together.  
A few classes are used here:
- **FSMNode** := The base class for all FSM nodes. Declares a simple interface. All nodes should inherite from this class.
- **FSM** := Represent a single FSM (and can be used as a node in other FSMs). Allows a user to register nodes and to run the machine.
- **FuncFSMNode** := When a simple node with no inner state is needed, this class allows to quickly wrap a given function.
- **DisjFSMNode** := A parallel node. Given a vecto of nodes, runs all of them and returns the answer of the first node to finish.
- **ConjFSMNode** := A parallel node. Given a vector of nodes, runs all of them and returns a vector of answers.  

**A few other nodes will be added soon!**  

A simple use example:
```
#include <ros/ros.h>
#include <armadillo_hl_interface/head_interface.h>
#include <armadillo_hl_interface/driver_interface.h>
#include <armadillo_hl_interface/torso_interface.h>
#include <armadillo_hl_interface/fsm.h>

HeadInterface *hi;
DriverInterface *di;
TorsoInterface *ti;

int head(){
    // head up
    hi->point_head_block(1.0, 0.3, 0.3);
    // head down
    hi->point_head_block(1.0, 0.0, 0.0);
    // head up
    hi->point_head_block(1.0, 0.3, 0.3);
    // head down
    hi->point_head_block(1.0, 0.0, 0.0);
    // next node
    return 3;
}

int torso(){
    // torso up
    ti->move_block(ti->MAX_HEIGHT);
    // torso down
    ti->move_block(ti->MIN_HEIGHT);
    // next node
    return 3;
}

int drive(){
    // driving 2m forward
    di->drive_block(2.0, 0.0, 0.4);
    // driving 2m backward
    di->drive_block(2.0, 0.0, -0.4);
    // returning 1 means end FSM with success
    return 1;
}

int main(int argc, char **argv){
    // init node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // init interfaces
    HeadInterface l_hi;
    DriverInterface l_di;
    TorsoInterface l_ti;

    // make them global
    hi = &l_hi;
    di = &l_di;
    ti = &l_ti;

    // some time to setup eveything
    ros::Duration w(2);
    w.sleep();

    // build nodes
    // a small shortcut: note that we can also build a vector of functions, and ConjFSMNode will wrap them with FuncFSMNode for us.
    FSM fsm;
    FuncFSMNode a(&drive);
    FuncFSMNode b(&head);
    FuncFSMNode c(&torso);

    std::vector<FSMNode*> nodes;
    nodes.push_back(&b);
    nodes.push_back(&c);

    ConjFSMNode d(nodes);

    // register nodes to fsm
    // 0 and 1 are resrved to failure and success, respectively. FSM starts from node 2, unless other node is specified.
    fsm.add_node(2, &d);
    fsm.add_node(3, &a);

    // run FSM
    if(fsm.run())
        ROS_INFO("Success!");

    ros::spin();
    return 0;
}
```
