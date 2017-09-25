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

// place cofee for the begining of simulation 2
void init_coffee(){
    geometry_msgs::Pose pose;
    pose.position.x = 0.717489;
    pose.position.y = 0.123294;
    pose.position.z = 0.442722;
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

bool run_script1(){
    // wait for a coffee request

    std::string talk;
    do{
        ROS_INFO("listening...");
        si->text_to_speech_block("What should I get you?");
        si->speech_to_text_block(10, talk);
        ROS_INFO_STREAM("got: '" << talk << "'");
    } while(talk != "Get me coffee.");
    si->text_to_speech_block("I'm going to get you coffee.");
    
    // drive to elevator door
    if(!di->drive_block("coffee_room_door"))
        return false;

    // find button
    geometry_msgs::Pose p;
    if(!lookup(p, "button"))
        return false;
    hi->move_head(0.0, 0.0); // head back to place

    // drive to button
    di->drive_block(p, 0.6, DriverInterface::ANGLE_FRONT);

    // push button
    ai->push_button(p);
    ai->move("pre_grasp1"); // move arm back to driving position
    open_door();

    // enter coffee room
    di->drive_block("coffee_room");
    
    // ask for coffee
    si->text_to_speech_block("Can I have some coffee?");
    
    place_coffee();
    ros::Duration(2.0).sleep();

    // find can
    if(!oh->find_object_block(p, "can"))
        return false;

    // drive to can
    ai->move("pre_grasp3");
    hi->move_head(0.0, 0.3);
    di->drive_block(p, 0.55);
    ti->move_block(MAX_HEIGHT_TORSO-0.05); // TODO: automate?
    ros::Duration(2.0).sleep();

    // look again to refine location
    if(!oh->find_object_block(p, "can", ObjectHandler::ARM_CAM))
        return false;

    // pickup can
    if(!ai->pickup_block("can", p))
        return false;

    // drive back
    ai->move("pre_grasp3");
    di->drive_block("table_room");
    ti->move_block(MIN_HEIGHT_TORSO);

    return true;
}

bool run_script2(){
    // place cofee in place for script2
    init_coffee();

    // find can
    hi->move_head_block(0.0, 0.5); // tilt head
    ros::Duration(2.0).sleep();
    geometry_msgs::Pose p;
    if(!oh->find_object_block(p, "can"))
        return false;
    hi->move_head(0.0, 0.0); // head back to place

    // drive to can
    ai->move("pre_grasp3");
    hi->move_head(0.0, 0.3);
    di->drive_block(p, 0.55);
    ros::Duration(1.0).sleep();

    // look again to refine location
    if(!oh->find_object_block(p, "can", ObjectHandler::ARM_CAM))
        return false;

    // pickup can
    if(!ai->pickup_block("can", p))
        return false;

    // drive to elevator door
    if(!di->drive_block("coffee_room_door"))
        return false;
    
    // ran into a door...
    si->text_to_speech_block("oh no!");

    // turn around and look for a place put the can
    di->drive_block(5.0, 0.3, 0.2);
    di->drive_block(5.0, 0.3, -0.2);
    di->drive_block(5.0, 0.3, 0.2);
    di->drive_block(5.0, 0.3, -0.2);

    if(!oh->find_object_block(p, "bench"))
        return false;
    
    di->drive_block(p, 0.55, DriverInterface::ANGLE_FRONT);

    ai->place_block("can", p);

}

void torso_callback(bool succ){
    ROS_INFO("DONE GOING DOWN");
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

    // // make them global
    // hi = &l_hi;
    // di = &l_di;
    // ti = &l_ti;
    // ai = &l_ai;
    // si = &l_si;
    // oh = &l_oh;
    
    // some time to setup eveything
    // NOW START SPEECH NODE WITH PYTHON!
    ros::Duration(5.0).sleep();

    // // start script
    // ROS_INFO("all ready! starting script...");

    // // here you can set the script number
    // if(run_script2())
    //     ROS_INFO("success!");
    // else
    //     ROS_INFO("fail!");

    // ros::spin();

    l_ti.move_block(0.3);
    ROS_INFO("DONE GOING UP");
    l_ti.move(torso_callback, 0.05);
    ROS_INFO("GOING DOWN");
    ros::spin();
    

    return 0;
}