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
ObjectHandler *oh;

bool lookup_and_drive(std::string object){
    move_head_block(-1.0, 0.0);
    move_head_block(0.0, 0.0);
    move_head_block(1.0, 0.0);
    return false;
}

bool run_script(){
    // wait for a coffee request
    std::string talk;
    do{
        ROS_INFO("listening...");
        speech_to_text_block(10, talk);
    } while(talk != "get me coffee")
    
    // drive to elevator door
    if(!di->drive_block("coffee_room_door"))
        return false;

    // find button and drive
    if(!lookup_and_drive("can"))
        return false;

    // push button

    // enter coffee room
    
    // ask for cofee

    // find can

    // pickup can

    // drive back

    return true;
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
    ObjectHandler l_oh;

    // make them global
    hi = &l_hi;
    di = &l_di;
    ti = &l_ti;
    ai = &l_ai;
    si = &l_si;
    oh = &l_oh;
    
    // some time to setup eveything
    ros::Duration w(2);
    w.sleep();

    // start script
    ROS_INFO("all ready! starting script...");
    if(run_script())
        ROS_INFO("success!");
    else
        ROS_INFO("fail!");

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
    
    // ROS_INFO("driving to door...");
    // l_di.drive_block("coffee_room_door");
    // ROS_INFO("driving back...");
    // l_di.drive_block("table_room");
    // ROS_INFO("done!");

    // geometry_msgs::Pose p;
    // while(ros::ok()){
    //     if(l_oh.find_object_block(p, "button")){
    //     //     // l_ai.move_block("pre_grasp1");
    //     //     // ros::Duration(2.0).sleep();
    //         ROS_INFO("found button, pushing...");
    //     //     l_ai.push_button(p);
    //     //     ROS_INFO("done!");
    //     }
    //     else
    //         ROS_INFO("can't find object!");
    //     ros::Duration(5.0).sleep();
    // }

    ros::spin();
    return 0;
}