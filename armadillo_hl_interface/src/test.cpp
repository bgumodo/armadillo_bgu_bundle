#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <armadillo_hl_interface/head_interface.h>
#include <armadillo_hl_interface/driver_interface.h>
#include <armadillo_hl_interface/torso_interface.h>
#include <armadillo_hl_interface/speech_interface.h>
#include <armadillo_hl_interface/arm_interface.h>
#include <armadillo_hl_interface/object_handler.h>
#include <armadillo_hl_interface/fsm.h>

HeadInterface *hi;
DriverInterface *di;
TorsoInterface *ti;
ArmInterface *ai;
SpeechInterface *si;

int head(){
    // head up
    hi->move_head_block(1.0, 0.3, 0.3);
    // head down
    hi->move_head_block(1.0, 0.0, 0.0);
    // head up
    hi->move_head_block(1.0, 0.3, 0.3);
    // head down
    hi->move_head_block(1.0, 0.0, 0.0);
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

void cb(bool success, std::string text){
    if(success)
        ROS_INFO_STREAM("I got: " << text);
    else
        ROS_INFO("got nothing!");
}

int main(int argc, char **argv){
    // init node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // init interfaces
    HeadInterface l_hi;
    DriverInterface l_di;
    TorsoInterface l_ti;
    ArmInterface l_ai;
    SpeechInterface l_si;

    ROS_INFO("ready!\n");
    // make them global
    hi = &l_hi;
    di = &l_di;
    ti = &l_ti;
    ai = &l_ai;
    si = &l_si;

    // ObjectHandler oh;
    // some time to setup eveything
    ros::Duration w(2);
    w.sleep();

    // l_ai.move_block("pre_grasp3");
    // geometry_msgs::Pose p;

    // if(oh.find_object(p, "can")){
    //     ROS_INFO("Found can, picking up...");
    //     l_ai.pickup_block("can", p);
    //     ROS_INFO("Placing back...");
    //     l_ai.place("can", p);
    //     ROS_INFO("done!");
    // }
    // else
    //     ROS_INFO("Can't find object!");

    // // build nodes
    // // a small shortcut: note that we can also build a vector of functions, and ConjFSMNode will wrap them with FuncFSMNode for us.
    // FSM fsm;
    // FuncFSMNode a(&drive);
    // FuncFSMNode b(&head);
    // FuncFSMNode c(&torso);

    // std::vector<FSMNode*> nodes;
    // nodes.push_back(&b);
    // nodes.push_back(&c);

    // ConjFSMNode d(nodes);

    // // register nodes to fsm
    // // 0 and 1 are resrved to failure and success, respectively. FSM starts from node 2, unless other node is specified.
    // fsm.add_node(2, &d);
    // fsm.add_node(3, &a);

    // // run FSM
    // if(fsm.run())
    //     ROS_INFO("Success!");

    ROS_INFO("listening...");
    l_si.speech_to_text(10, cb);
    
    // ROS_INFO("driving to door...");
    // l_di.drive_block("coffee_room_door");
    // ROS_INFO("driving back...");
    // l_di.drive_block("table_room");
    // ROS_INFO("done!");

    // geometry_msgs::Pose p;
    // while(ros::ok()){
        // if(oh.find_object(p, "can")){
        //     // l_ai.move_block("pre_grasp1");
        //     // ros::Duration(2.0).sleep();
        //     ROS_INFO("found button, pushing...");
        //     l_ai.push_button(p);
        //     ROS_INFO("done!");
        // }
        // else
        //     ROS_INFO("can't find object!");
    // }

    ros::spin();
    return 0;
}