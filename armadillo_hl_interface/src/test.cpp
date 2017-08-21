#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <armadillo_hl_interface/head_interface.h>
#include <armadillo_hl_interface/driver_interface.h>
#include <armadillo_hl_interface/torso_interface.h>
#include <armadillo_hl_interface/speech_interface.h>
#include <armadillo_hl_interface/arm_interface.h>
#include <armadillo_hl_interface/object_handler.h>
#include <armadillo_hl_interface/fsm.h>

#include <iostream>
#include <sstream>
#include <fstream>

ros::ServiceClient add_obj;
ros::ServiceClient rem_obj;

HeadInterface *hi;
DriverInterface *di;
TorsoInterface *ti;
ArmInterface *ai;
SpeechInterface *si;
ObjectHandler *oh;

void add_model(const std::string &name, const geometry_msgs::Pose pose, const std::string xml_path){
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = name;

    std::ifstream file(xml_path.c_str());
    std::stringstream sstr;
    while(file >> sstr.rdbuf());

    srv.request.model_xml = sstr.str();
    srv.request.initial_pose = pose;
    srv.request.reference_frame = "map";
    add_obj.call(srv);
}

void remove_object(const std::string &name){
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = name;
    rem_obj.call(srv);
}

void open_door(){
    remove_object("unit_box_2");
    ros::Duration(1.0);
}

void place_coffee(){
    geometry_msgs::Pose pose;
    pose.position.x = -10.729576;
    pose.position.y = 7.307851;
    pose.position.z = 0.735001;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    add_model("can", pose, "/home/bgumodo2/catkin_ws/src/robotican/robotican_common/models/coke_can_slim/coke_can_slim.sdf");
    ros::Duration(1.0);
}

bool lookup(geometry_msgs::Pose &pose, std::string object){
    hi->move_head_block(1.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, "button"))
        return true;

    hi->move_head_block(0.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, "button"))
        return true;

    hi->move_head_block(-1.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, "button"))
        return true;

    return false;
}

bool run_script(){
    // // wait for a coffee request
    // std::string talk;
    // do{
    //     ROS_INFO("listening...");
    //     si->speech_to_text_block(10, talk);
    //     ROS_INFO_STREAM("got: '" << talk << "'");
    // } while(talk != "Get me coffee.");
    
    // // drive to elevator door
    // if(!di->drive_block("coffee_room_door"))
    //     return false;

    // // find button
    geometry_msgs::Pose p;
    // if(!lookup(p, "button"))
    //     return false;
    // hi->move_head(0.0, 0.0); // head back to place

    // // drive to button
    // di->drive_block(p, 0.6);

    // // TODO: add option to move infront of the button
    // // push button
    // ai->push_button(p);
    // ai->move("pre_grasp1"); // move arm back to driving position
    open_door();

    // enter coffee room
    di->drive_block("coffee_room");
    
    // ask for coffee
    // TODO: implement!
    place_coffee();

    // find can
    if(!oh->find_object_block(p, "can"))
        return false;
    hi->move_head(0.0, 0.0); // head back to place

    // drive to can
    ai->move("pre_grasp2");
    hi->move_head(0.0, 0.3);
    di->drive_block(p, 0.6);
    ros::Duration(4.0).sleep();

    // look again to refine location (TODO: remove)
    if(!oh->find_object_block(p, "can", ObjectHandler::ARM_CAM))
        return false;

    // pickup can
    while(!ai->pickup_block("can", p) && ros::ok()){
        ROS_INFO("failed.");
        ros::Duration(4.0).sleep();
        ROS_INFO("trying to grasp again...");
    }

    // // drive back
    // di->drive_block("table_room");

    return true;
}

int main(int argc, char **argv){
    // init node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // used to add and remove parts of the simulation
    rem_obj = nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    add_obj = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_gazebo_model");
    while(!rem_obj.waitForExistence(ros::Duration(5.0)) || !add_obj.waitForExistence(ros::Duration(5.0))){
        ROS_INFO("waiting for gazebo services...");
    }

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
    // NOW START SPEECH NODE WITH PYTHON!
    ros::Duration w(6.0);
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