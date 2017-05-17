#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include <armadillo_hl_interface/head_interface.h>

HeadInterface::HeadInterface():
    _ready(false),
    _point_head_client("/pan_tilt_trajectory_controller/point_head_action", true)
{
    ros::Duration w(1.0);
    while(!_point_head_client.waitForServer() && ros::ok()){
        ROS_INFO("Waiting for action servers...");
        w.sleep();
    }
    _ready = true;
}

HeadInterface::HIGoal HeadInterface::xyz_to_goal(std::string ref_frame, double x, double y, double z, double vel){
    HIGoal goal;

    geometry_msgs::PointStamped point;
    point.header.frame_id = ref_frame;
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;

    goal.target = point;
    goal.pointing_frame = "kinect2_depth_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    goal.min_duration = ros::Duration(0.5);
    goal.max_velocity = vel;

    return goal;
}

bool HeadInterface::point_head_block(double x, double y, double z, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return false;
    }
    HIGoal goal = xyz_to_goal("head_link", x, y, z, vel);
    _point_head_client.sendGoalAndWait(goal);
    return _point_head_client.getState() == GoalState::SUCCEEDED;
}

bool HeadInterface::point_head_block(const geometry_msgs::Pose &pose, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return false;
    }
    HIGoal goal = xyz_to_goal("map", pose.position.x, pose.position.y, pose.position.z, vel);
    _point_head_client.sendGoalAndWait(goal);
    return _point_head_client.getState() == GoalState::SUCCEEDED;
}

void HeadInterface::point_head(double x, double y, double z, double vel){
    if(!_ready)
        ROS_ERROR("HeadInterface is not ready!");
    else{
        HIGoal goal = xyz_to_goal("head_link", x, y, z, vel);
        _point_head_client.sendGoal(goal);
    }
}

void HeadInterface::point_head(const geometry_msgs::Pose &pose, double vel){
    if(!_ready)
        ROS_ERROR("HeadInterface is not ready!");
    HIGoal goal = xyz_to_goal("map", pose.position.x, pose.position.y, pose.position.z, vel);
    _point_head_client.sendGoal(goal);
}

void HeadInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void HeadInterface::point_head(const CallbackBool callback, double x, double y, double z, double vel){
    if(!_ready)
        ROS_ERROR("HeadInterface is not ready!");
    else{
        HIGoal goal = xyz_to_goal("head_link", x, y, z, vel);
        _point_head_client.sendGoal(goal, boost::bind(&HeadInterface::generic_done_callback, boost::ref(this), callback, _1));
    }
}

void HeadInterface::point_head(const CallbackBool callback, const geometry_msgs::Pose &pose, double vel){
    if(!_ready)
        ROS_ERROR("HeadInterface is not ready!");
    else{
        HIGoal goal = xyz_to_goal("map", pose.position.x, pose.position.y, pose.position.z, vel);
        _point_head_client.sendGoal(goal, boost::bind(&HeadInterface::generic_done_callback, boost::ref(this), callback, _1));
    }
}

void HeadInterface::stop(){
    _point_head_client.cancelAllGoals();
}

HeadInterface::~HeadInterface(){

}