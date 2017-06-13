#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <armadillo_hl_interface/SimpleDriverAction.h>
#include <armadillo_hl_interface/SimpleDriverFeedback.h>

#include <armadillo_hl_interface/driver_interface.h>

// SimpleDriverServer implementation

SimpleDriverServer::SimpleDriverServer():
    _nh(),
    _cbq(),
    _action_server(0),
    _tf_listener(),
    _active(false)
{
    _nh.setCallbackQueue(&_cbq);
    _pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    _tf_listener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(5.0));
    _action_server = new ActionServer(_nh, "simple_driver", boost::bind(&SimpleDriverServer::callback, boost::ref(this), _1), false);
    _action_server->start();

    // spin local queue
    _active = true;
    while(ros::ok() && _active){
        _cbq.callAvailable(ros::WallDuration(0));
    }  
}

void SimpleDriverServer::stop_server(){
    _active = false;
}

void SimpleDriverServer::callback(const armadillo_hl_interface::SimpleDriverGoalConstPtr &goal){
    tf::StampedTransform last_tf, curr_tf;
    armadillo_hl_interface::SimpleDriverFeedback feedback;
    double dist=0;

    // record starting position
    _tf_listener.lookupTransform("base_footprint", "odom", ros::Time(0), last_tf);
    ros::Rate r(1);

    bool success = false;
    while(ros::ok() && !success && !_action_server->isPreemptRequested()){
        // move
        _pub.publish(goal->cmd);
        r.sleep();

        // update distance traveled so-far
        _tf_listener.lookupTransform("base_footprint", "odom", ros::Time(0), curr_tf);
        tf::Transform relative_tf = last_tf.inverse() * curr_tf;
        dist += relative_tf.getOrigin().length();
        feedback.distance = dist;
        _action_server->publishFeedback(feedback);

        // update last position to be current position
        last_tf = curr_tf;
        
        // check if done
        success = dist >= goal->distance;
    }
    
    if(success){
        armadillo_hl_interface::SimpleDriverResult result;
        result.distance = dist;
        _action_server->setSucceeded(result);
    }
}

SimpleDriverServer::~SimpleDriverServer(){
    delete _action_server;
}

// DriverInterface implementation

DriverInterface::DriverInterface():
    _ready(false),
    _mb_client("move_base", true),
    _sd_client("simple_driver", true),
    _tf_listener(),
    _cm_interface("global_costmap", _tf_listener),
    _cm_model(*_cm_interface.getCostmap()),
    _dest_map()
    {
    // start simple-driver server
    boost::thread sd_thread(&DriverInterface::start_sd_server, this);

    // wait for both servers to come-up
    ros::Duration w(1.0);
    while(!(_mb_client.waitForServer() && _sd_client.waitForServer()) && ros::ok()){
		ROS_INFO("Waiting for action servers...");
        w.sleep();
	}

	// fill destination map
	import_dest_map("dest_map.xml");
	
    _ready = true;
}

void DriverInterface::import_dest_map(const std::string filename){
	// TODO: implement not hard-coded
    geometry_msgs::Pose table_room, coffee_room_door, coffee_room;
    tf::Quaternion ori;
    ori.setRPY(0.0, 0.0, 0.0);

    table_room.position.x = -0.5;
    table_room.position.y = -0.4;
    table_room.position.z = 0.0;
    table_room.orientation.x = ori.x();
    table_room.orientation.y = ori.y();
    table_room.orientation.z = ori.z();
    table_room.orientation.w = ori.w();

    coffee_room_door.position.x = -9.0;
    coffee_room_door.position.y = 0.0;
    coffee_room_door.position.z = 0.0;
    coffee_room_door.orientation.x = ori.x();
    coffee_room_door.orientation.y = ori.y();
    coffee_room_door.orientation.z = ori.z();
    coffee_room_door.orientation.w = ori.w();

    coffee_room.position.x = -9.0;
    coffee_room.position.y = 6.5;
    coffee_room.position.z = 0.0;
    coffee_room.orientation.x = ori.x();
    coffee_room.orientation.y = ori.y();
    coffee_room.orientation.z = ori.z();
    coffee_room.orientation.w = ori.w();

    _dest_map.insert(dest_dict_item("table_room", table_room));
    _dest_map.insert(dest_dict_item("cofee_room_door", coffee_room_door));
    _dest_map.insert(dest_dict_item("coffee_room", coffee_room));
}

void DriverInterface::start_sd_server(){
    ROS_INFO("starting simple-driver server...");
    SimpleDriverServer sd_server;
}

bool DriverInterface::pose_blocked(const geometry_msgs::Pose &pose){
    // TODO: update to real inscribed_radius, circumscribed_radius
    double cost = _cm_model.footprintCost(pose.position, _cm_interface.getRobotFootprint(), 0.2, 0.3);
    return cost >= 0;
}

bool DriverInterface::point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2){
    double d1 = pow(base.position.x - point1.position.x, 2) + pow(base.position.y - point1.position.y, 2);
    double d2 = pow(base.position.x - point2.position.x, 2) + pow(base.position.y - point2.position.y, 2);
    return d1 <= d2;
}

bool DriverInterface::get_best_pose_in_rad(geometry_msgs::Pose &target, const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius){
    // create sampling points
    std::vector<geometry_msgs::Pose> poses;
    double theta, rel_x, rel_y, yaw;

    double theta_unit = (2 * M_PI)/NUM_OF_SAMPLING_POINTS;
    for(int i=0; i<NUM_OF_SAMPLING_POINTS; i++){
        geometry_msgs::Pose n_pose;
        
        // calc relative x, y, yaw
        theta = theta_unit * i;
        rel_x = radius * cos(theta);
        rel_y = radius * sin(theta);
        yaw = fmod(theta+M_PI, 2*M_PI);

        // transform to absolute vals
        n_pose.position.x = object.position.x + rel_x;
        n_pose.position.y = object.position.y + rel_y;
        n_pose.position.z = 0.0;

        tf::Quaternion ori;
        ori.setRPY(0.0, 0.0, yaw);

        n_pose.orientation.x = ori.getX();
        n_pose.orientation.y = ori.getY();
        n_pose.orientation.z = ori.getZ();
        n_pose.orientation.w = ori.getW();

        poses.push_back(n_pose);
    }

    // sort according to distance
    std::sort(poses.begin(), poses.end(),  boost::bind(&DriverInterface::point_dist_comperator, robot, _1, _2));

    // find first first accessible point and return
    for(std::vector<geometry_msgs::Pose>::iterator it=poses.begin(); it!=poses.end(); ++it){
        if(!pose_blocked(*it)){
            target = *it;
            return true;
        }
    }

    // in case no such point
    return false;
}

bool DriverInterface::build_digoal(DIGoal &target, geometry_msgs::Pose &object, double radius){
    // get robot position
    tf::StampedTransform robot_transform;
    _tf_listener.lookupTransform("map", "base_link", ros::Time(0), robot_transform);
    geometry_msgs::Pose robot;
    robot.position.x = robot_transform.getOrigin().x();
    robot.position.y = robot_transform.getOrigin().y();
    robot.position.z = 0.0;

    // if needed, get robot's new pose in radius from goal
    geometry_msgs::Pose dest;
    if(radius)
        if(!get_best_pose_in_rad(dest, robot, object, radius)) // get new goal on radius
            return false; // return false if no reachable goal on radius
    else
        dest = object; // if no radius is given, just use original position

    // build goal and return
    target.target_pose.header.frame_id = "map";
    target.target_pose.header.stamp = ros::Time::now();

    target.target_pose.pose.position.x = dest.position.x;
    target.target_pose.pose.position.y = dest.position.y;
    target.target_pose.pose.position.z = 0.0;

    target.target_pose.pose.orientation.x = dest.orientation.x;
    target.target_pose.pose.orientation.y = dest.orientation.y;
    target.target_pose.pose.orientation.z = dest.orientation.z;
    target.target_pose.pose.orientation.w = dest.orientation.w;
    
    return true;
}

bool DriverInterface::drive_block(geometry_msgs::Pose &object, double radius){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }

    DIGoal goal;

    if(!build_digoal(goal, object, radius))
        return false;

    _mb_client.sendGoalAndWait(goal);

	return _mb_client.getState() == GoalState::SUCCEEDED;
}


void DriverInterface::drive(geometry_msgs::Pose &object, double radius){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return;
    }

    DIGoal goal;

    if(build_digoal(goal, object, radius))
        _mb_client.sendGoal(goal);
}

void DriverInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void DriverInterface::drive(const CallbackBool callback, geometry_msgs::Pose &object, double radius){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return;
    }

    DIGoal goal;
    if(!build_digoal(goal, object, radius)){
        callback(false);
        return;
    }

    _mb_client.sendGoal(goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
}

bool DriverInterface::drive_block(double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }

    SDGoal goal;
    goal.cmd.linear.x = vel;
    goal.cmd.angular.z = z;
    goal.distance = dist;
    _sd_client.sendGoalAndWait(goal);
    return _sd_client.getState() == GoalState::SUCCEEDED;
}

void DriverInterface::drive(double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return;
    }

    SDGoal goal;
    goal.cmd.linear.x = vel;
    goal.cmd.angular.z = z;
    goal.distance = dist;
    _sd_client.sendGoal(goal);
}

void DriverInterface::drive(const CallbackBool callback, double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        callback(false);
    }

    SDGoal goal;
    goal.cmd.linear.x = vel;
    goal.cmd.angular.z = z;
    goal.distance = dist;
    _sd_client.sendGoal(goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
}

bool DriverInterface::drive_block(const std::string &destination){
	if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }

	std::map<std::string, geometry_msgs::Pose>::iterator pose_iter = _dest_map.find(destination);
	if(pose_iter == _dest_map.end())
	{
		ROS_ERROR("DriverInterface couldn't find destination!");
       return false;
	}
	 
	geometry_msgs::Pose obj = pose_iter->second;
	DIGoal goal;
    build_digoal(goal, obj, 0);
    
    _mb_client.sendGoalAndWait(goal);

	return _mb_client.getState() == GoalState::SUCCEEDED;
}

void DriverInterface::drive(const std::string &destination){
	if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return;
    }

	std::map<std::string, geometry_msgs::Pose>::iterator pose_iter = _dest_map.find(destination);
	if(pose_iter == _dest_map.end())
	{
		ROS_ERROR("DriverInterface couldn't find destination!");
		return;
	}
	
	geometry_msgs::Pose obj = pose_iter->second;
	DIGoal goal;
    build_digoal(goal, obj, 0);
	 
    _mb_client.sendGoal(goal);
}

void DriverInterface::drive(const CallbackBool callback, const std::string &destination){
	if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return;
    }

    std::map<std::string, geometry_msgs::Pose>::iterator pose_iter = _dest_map.find(destination);
	if(pose_iter == _dest_map.end())
	{
		ROS_ERROR("DriverInterface couldn't find destination!");
    	return;
	}
	
	geometry_msgs::Pose obj = pose_iter->second;
	DIGoal goal;
    build_digoal(goal, obj, 0);

    _mb_client.sendGoal(goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
}

void DriverInterface::stop(){
    _mb_client.cancelAllGoals();
    _sd_client.cancelAllGoals();
}

bool DriverInterface::is_ready(){
    return _ready == true;
}

DriverInterface::~DriverInterface(){

}
