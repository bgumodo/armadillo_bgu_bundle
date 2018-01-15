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



bool lookup(geometry_msgs::Pose &pose, std::string object){
    return false;
}

bool run_script(){
	//tf::TransformListener::lookupTransform("map","base_footprint");
	ROS_INFO("in script");
	geometry_msgs::Pose pose;
	pose.position.x = 2;
	pose.position.y = 1;
	pose.position.z = 0;

    pose.orientation.x = 1;
    pose.orientation.y = 1;	
	pose.orientation.z = 90;
	pose.orientation.w = 3;

	ROS_INFO("driving");
	di->drive(pose);
	ros::Duration(5.0).sleep();
	
	di->stop();	
	
	ROS_INFO("looking");
	hi->move_head_block(0.5,1);
	ros::Duration(5.0).sleep();

	ROS_INFO("moving up");
	ti->move_block(0.36);
	ros::Duration(5.0).sleep();

	ROS_INFO("moving down");
	ti->move_block(0.1);
	ros::Duration(5.0).sleep();
	



	

    return true;
}

int main(int argc, char **argv){
    // init node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
	ROS_INFO("initing");
    // init interfaces
    HeadInterface l_hi;
	ROS_INFO("after HI");
    DriverInterface l_di;
	ROS_INFO("after DI");
    TorsoInterface l_ti;
	ROS_INFO("after TI");
    ArmInterface l_ai;
	ROS_INFO("after AI");
    SpeechInterface l_si;
	ROS_INFO("after SI");
    // make them global
    hi = &l_hi;
    di = &l_di;
    ti = &l_ti;
    ai = &l_ai;
    si = &l_si;
	ROS_INFO("after global");
    
    // some time to setup everything
    // NOW START SPEECH NODE WITH PYTHON!
    ros::Duration(3.0).sleep();

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
