#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <armadillo_hl_interface/head_interface.h>


HeadInterface::HeadInterface():
    _ready(false),
    _point_head_client("/pan_tilt_trajectory_controller/point_head_action", true),
    _pan_tilt_client("/pan_tilt_trajectory_controller/follow_joint_trajectory", true)
{
	ROS_INFO("Starting Head Interface.");	
    // wait for action serveres
    ros::Duration w(1.0);
	ros::Duration w1(4.0);
	
    while(!_point_head_client.waitForServer(w1) && !_pan_tilt_client.waitForServer(w1) && ros::ok()){
        ROS_INFO("waiting for action servers...");
        w.sleep();
    }
	ROS_INFO("1");
    _ready = true;
}

HeadInterface::PTGoal HeadInterface::pan_tilt_to_goal(double pan, double tilt, double vel){
    PTGoal goal;

    std::vector<double> pan_tilt;
    pan_tilt.push_back(pan);
    pan_tilt.push_back(tilt);

    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
    goal.trajectory.points[0].positions = pan_tilt;
    goal.trajectory.points[0].velocities.push_back(vel);
    goal.trajectory.points[0].velocities.push_back(vel);

    goal.trajectory.header.stamp = ros::Time::now();

    return goal;
}

HeadInterface::HIGoal HeadInterface::pose_to_goal(const geometry_msgs::Pose &pose, double vel){
    HIGoal goal;

    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.point.x = pose.position.x;
    point.point.y = pose.position.y;
    point.point.z = pose.position.z;

    goal.target = point;
    goal.pointing_frame = "kinect2_depth_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    goal.min_duration = ros::Duration(0.5);
    goal.max_velocity = vel;

    return goal;
}

bool HeadInterface::move_head_block(double pan, double tilt, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return false;
    }
    
    PTGoal goal = pan_tilt_to_goal(pan, tilt, vel);
    _pan_tilt_client.sendGoal(goal);

    return _pan_tilt_client.getState() == GoalState::SUCCEEDED;
}

bool HeadInterface::move_head_block(const geometry_msgs::Pose &pose, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return false;
    }

    HIGoal goal = pose_to_goal(pose, vel);
    _point_head_client.sendGoalAndWait(goal);
    return _point_head_client.getState() == GoalState::SUCCEEDED;
}

void HeadInterface::move_head(double pan, double tilt, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return;
    }

    PTGoal goal = pan_tilt_to_goal(pan, tilt, vel);
    _pan_tilt_client.sendGoal(goal);
}

void HeadInterface::move_head(const geometry_msgs::Pose &pose, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return;
    }

    HIGoal goal = pose_to_goal(pose, vel);
    _point_head_client.sendGoal(goal);
}

void HeadInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void HeadInterface::move_head(const CallbackBool callback, double pan, double tilt, double vel){
    if(!_ready){
        ROS_ERROR("HeadInterface is not ready!");
        return;
    }

    PTGoal goal = pan_tilt_to_goal(pan, tilt, vel);
    _pan_tilt_client.sendGoal(goal, boost::bind(&HeadInterface::generic_done_callback,this, callback, _1));
}

void HeadInterface::move_head(const CallbackBool callback, const geometry_msgs::Pose &pose, double vel){
    if(!_ready)
        ROS_ERROR("HeadInterface is not ready!");
    else{
        HIGoal goal = pose_to_goal(pose, vel);
        _point_head_client.sendGoal(goal, boost::bind(&HeadInterface::generic_done_callback,this, callback, _1));
    }
}

void HeadInterface::stop(){
    _point_head_client.cancelAllGoals();
    _pan_tilt_client.cancelAllGoals();
}

HeadInterface::~HeadInterface(){

}
