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

bool move_to_obj(){
    
 geometry_msgs::Pose p;
    if(!lookup(p, "button"))
        return false;
    hi->move_head(0.0, 0.0); // head back to place


}

int main(int argc, char **argv){
    // init node
    ros::init(argc, argv, "move_and_pick_node");
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

    // start script
    ROS_INFO("all ready! starting script...");

    //add feedback succeed or fail
    ros::spin();

    return 0;
}