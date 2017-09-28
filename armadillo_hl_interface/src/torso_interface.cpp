#include <ros/ros.h>
#include <armadillo_hl_interface/torso_interface.h>
#include <armadillo_hl_interface/TorsoResult.h>
#include <armadillo_hl_interface/TorsoFeedback.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>

// Torso Action Server implemantation

TorsoServer::TorsoServer():
    _cbq(),
    _nh(),
    _torso_server(0),
    _tf_listener(),
    _active(true)
{
    // init local NodeHandle
    _nh.setCallbackQueue(&_cbq);

    // init publiser
    _pub = _nh.advertise<std_msgs::Float64>("/torso_controller/command", 100);

    // start server
    _torso_server = new ActionServer(_nh, "torso", boost::bind(&TorsoServer::callback, boost::ref(this), _1), false);
    _torso_server->start();

    // spin server
    while(_active && ros::ok()){
        _cbq.callAvailable(ros::WallDuration(0));
    }
}

void TorsoServer::callback(const armadillo_hl_interface::TorsoGoalConstPtr &goal){
    // publish command
    std_msgs::Float64 wgoal;
    wgoal.data = goal->height;
    _pub.publish(wgoal);

    // update periodicaly
    ros::Duration w(0.1);
    double org_space=MAX_HEIGHT_TORSO-MIN_HEIGHT_TORSO,  tf_space=MAX_HEIGHT_TORSO_LINK-MIN_HEIGHT_TORSO_LINK;
    double curr_tf_height, curr_org_height, target_tf_height=MIN_HEIGHT_TORSO_LINK+((goal->height-MIN_HEIGHT_TORSO)/org_space)*tf_space;

    tf::StampedTransform transform;
    armadillo_hl_interface::TorsoFeedback feedback;
    armadillo_hl_interface::TorsoResult result;
    while(ros::ok()){
        // get current height in tf and transform to org
        _tf_listener.lookupTransform("base_link", "torso_link", ros::Time(0), transform);
        curr_tf_height = transform.getOrigin().length();
        curr_org_height = MIN_HEIGHT_TORSO + ((curr_tf_height-MIN_HEIGHT_TORSO_LINK)/tf_space)*org_space;
        
        if(curr_tf_height < target_tf_height+EPSILON_TORSO && curr_tf_height > target_tf_height-EPSILON_TORSO){ // check if done
            result.height = curr_org_height;
            _torso_server->setSucceeded(result);
            return;
        }

        feedback.height = curr_org_height;
        _torso_server->publishFeedback(feedback);

        w.sleep();
    }
    
    // action failed
    result.height = curr_org_height;
    _torso_server->setPreempted(result);
}

void TorsoServer::stop_server(){
    _active = false;
}

TorsoServer::~TorsoServer(){
    delete _torso_server;
}

// Torso Interface Implemantation

TorsoInterface::TorsoInterface():
    _torso_client("torso", true),
    _ready(false),
    _server_thread(0),
    _server(0)
{
    // start server thread
    _server_thread = new boost::thread(&TorsoInterface::start_server, this);

    // wait for server to come-up
    ros::Duration w(1.0);
    while(!_torso_client.waitForServer() && ros::ok()){
		ROS_INFO("waiting for torso action server...");
        w.sleep();
	}

    // interface is ready
    _ready = true;
}

void TorsoInterface::start_server(){
    ROS_INFO("starting torso action-server...");
    _server = new TorsoServer();
}

double TorsoInterface::height_validation(double height){
    if(height > MAX_HEIGHT){
        ROS_WARN("%f is out of torso range. Moving to %f.", height, MAX_HEIGHT);
        return MAX_HEIGHT;
    }

    if(height < MIN_HEIGHT){
        ROS_WARN("%f is out of torso range. Moving to %f.", height, MIN_HEIGHT);
        return MIN_HEIGHT;
    }

    return height;
}

void TorsoInterface::move(double height){
    // ensure everything is ready
    if(!_ready){
        ROS_ERROR("TorsoInterface is not ready!");
        return;
    }

    // make sure height is in range
    height = height_validation(height);

    // send goal to action server
    TGoal goal;
    goal.height = height;
    _torso_client.sendGoal(goal);
}

bool TorsoInterface::move_block(double height){
    // ensure everything is ready
    if(!_ready){
        ROS_ERROR("TorsoInterface is not ready!");
        return false;
    }

    // make sure height is in range
    height = height_validation(height);

    // send goal to action server
    TGoal goal;
    goal.height = height;
    _torso_client.sendGoalAndWait(goal);

    // return result
    return _torso_client.getState() == GoalState::SUCCEEDED;
}

void TorsoInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void TorsoInterface::move(const CallbackBool callback, double height){
    // ensure everything is ready
    if(!_ready){
        ROS_ERROR("TorsoInterface is not ready!");
        return;
    }

    // make sure height is in range
    height = height_validation(height);

    // send goal to action server
    TGoal goal;
    goal.height = height;
    _torso_client.sendGoal(goal, boost::bind(&TorsoInterface::generic_done_callback, boost::ref(this), callback, _1));
}

TorsoInterface::~TorsoInterface(){
    // stop server thread
    _server->stop_server();
    _server_thread->join();

    delete _server_thread;
    delete _server;
}