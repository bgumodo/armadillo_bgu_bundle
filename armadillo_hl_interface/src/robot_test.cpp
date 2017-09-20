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

bool lookup(geometry_msgs::Pose &pose, std::string object){
    hi->move_head_block(1.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, object))
        return true;

    hi->move_head_block(0.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, object))
        return true;

    hi->move_head_block(-1.0, 0.0);
    ros::Duration(1.0).sleep(); // wait for head to stop moving
    if(oh->find_object_block(pose, object))
        return true;

    return false;
}

bool run_script(){
    // wait for a coffee request
    std::string talk;
    do{
        ROS_INFO("listening...");
        si->text_to_speech_block("What should I do?");
        si->speech_to_text_block(10, talk);
        ROS_INFO_STREAM("got: '" << talk << "'");
        si->text_to_speech_block("I got " + talk);
    } while(talk != "Get me coke.");

    geometry_msgs::Pose pose;
    if(!lookup(pose, "coke")){
        si->text_to_speech_block("I can't find the coke.");
        return false;
    }

    si->text_to_speech_block("I've found the coke.");

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
    // NOW START SPEECH NODE WITH PYTHON!
    ros::Duration(5.0).sleep();

    // start script
    ROS_INFO("all ready! starting script...");

    // here you can set the script number
    if(run_script())
        ROS_INFO("success!");
    else
        ROS_INFO("fail!");

    ros::spin();
    return 0;
}