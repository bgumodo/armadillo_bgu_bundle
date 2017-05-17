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