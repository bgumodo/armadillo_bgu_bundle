#include <ros/ros.h>
#include <armadillo_hl_interface/arm_interface.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/PlaceGoal.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

ArmInterface::ArmInterface():
    _cbq(),
    _nh(),
    _tf_listener(),
    _move_group(0),
    _gripper_client("/gripper_controller/gripper_cmd", true),
    _pickup_client("/pickup", true),
    _place_client("/place", true),
    _rnd_gen(),
    _spinner_thread(0),
    _ready(false),
    _spin(true)
{
    // init local NodeHandle
    _nh.setCallbackQueue(&_cbq);

    /**opt stucture.
	*@param: create an options "opt" with Group: "arm"
	*@param: "robot_description" is fixed, which says the location of URDF
	*@param: and a nodehandler.
	**/
    moveit::planning_interface::MoveGroupInterface::Options opt("arm", "robot_description", _nh);

    //MoveGroupInterface instance, init with opt that was created above ^
    _move_group = new moveit::planning_interface::MoveGroupInterface(opt);

    //set the move_group reference as base_link.
    _move_group->setPoseReferenceFrame("base_link");

    //create a new thread, with start_spinner function, so its running in the same time that this code is running,
    _spinner_thread = new boost::thread(&ArmInterface::start_spinner, this);

    //if one of the servers didnt come up yet, wait for it. (this doesnt happen usually)
    while(ros::ok() && !_gripper_client.waitForServer() && !_pickup_client.waitForServer() && !_place_client.waitForServer()){
        ROS_INFO("waiting for arm control servers...");
        ros::Duration w(1.0);
    }
	//indicate that ArmInterface is ready and all the servers are up.
    _ready = true;
}
/**
*call all the callback in the callbackqueue.
*this spins in a seperated thread, while the code is running.
**/
void ArmInterface::start_spinner(){
    while(_spin && ros::ok)
        _cbq.callAvailable(ros::WallDuration(0));
}
/**
* stops the spinner and the other thread
* only when distrctor is called
**/
void ArmInterface::stop_spinner(){
    _spin = false;
}

/**
*
*@param: an empty plan to plan too.
*@param: x,y,z to send to moveit to create a plan.
*@return: true if succeeded, false otherwise.
**/
bool ArmInterface::plan_to_xyz(moveit::planning_interface::MoveGroupInterface::Plan &plan, double x, double y, double z){
    // build goal
    geometry_msgs::Pose p;

    // set position
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    
   	//general orientation- in front of the robot
    tf::Quaternion orientation;
	//calculate quaternion from the x,y position.
    orientation.setRPY(atan2(y, x), 0.0, 0.0);
	//set the calculated orientation to the pose.
    p.orientation.x = orientation.x();
    p.orientation.y = orientation.y();
    p.orientation.z = orientation.z();
    p.orientation.w = orientation.w();

    //plan to goal (_move_group is moveit::planning_interface::MoveGroupInterface)
    _move_group->setPoseTarget(p);
    _move_group->plan(plan);
	return true;
}

/**
*recives a pose and an empty plan. with moveit! plan to that pose and put it in the plan.
*@param plan an empty plan.  
*@param pose
**/
bool ArmInterface::plan_to_pose(moveit::planning_interface::MoveGroupInterface::Plan &plan, const geometry_msgs::Pose &pose){
    // transform from map to robot axis
    tf::StampedTransform arm_tf;
	//put the transform from map to robot axis in arm_tf
    _tf_listener.lookupTransform("base_link", "map", ros::Time(0), arm_tf);
    tf::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    position = arm_tf * position;
    // general orientation- in front of the robot
    tf::Quaternion orientation;
    //calculate quaternion from the x,y position.
    orientation.setRPY(atan2(position.y(), position.x()), 0.0, 0.0);

    // build goal
    geometry_msgs::Pose p;
    
    p.position.x = position.x();
    p.position.y = position.y();
    p.position.z = position.z();

    p.orientation.x = orientation.x();
    p.orientation.y = orientation.y();
    p.orientation.z = orientation.z();
    p.orientation.w = orientation.w();
    
    // plan to goal
    _move_group->setPoseTarget(p);
    _move_group->plan(plan);
	return true;
}

void ArmInterface::move(double x, double y, double z){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    ROS_INFO("planning to pose...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(plan_to_xyz(plan, x, y, z))
        _move_group->asyncExecute(plan);
    else
        ROS_WARN("can't plan to pose!");
}

bool ArmInterface::move_block(double x, double y, double z){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    ROS_INFO("planning to pose...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(plan_to_xyz(plan, x, y, z)){
       _move_group->execute(plan);
		return true;
	}
    else
        ROS_WARN("can't plan to pose!");
    return false;
}

void ArmInterface::move(const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ROS_INFO("planning to pose...");
    if(plan_to_pose(plan, pose))
        _move_group->asyncExecute(plan);
    else
        ROS_WARN("can't plan to pose!");
}

bool ArmInterface::move_block(const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ROS_INFO("planning to pose...");
    if(plan_to_pose(plan, pose)){
        _move_group->execute(plan);
		return  true;
}
    else
        ROS_WARN("can't plan to pose!");
    return false;
}

void ArmInterface::move(const std::string &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ROS_INFO("planning to pose...");
    _move_group->setNamedTarget(pose);
    if(_move_group->plan(plan))
        _move_group->asyncExecute(plan);
    else
        ROS_WARN("can't plan to pose!");
}

bool ArmInterface::move_block(const std::string &pose){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ROS_INFO("planning to pose...");
    _move_group->setNamedTarget(pose);
    if(_move_group->plan(plan)){
        _move_group->execute(plan);
		return true;
	}
    else
        ROS_WARN("can't plan to pose!");
    return false;
}

double ArmInterface::gripper_validation(double position){
    if(position > OPEN_GRIPPER_POSITION){
        ROS_WARN("%f is out of gripper range. Moving to %f.", position, OPEN_GRIPPER_POSITION);
        return OPEN_GRIPPER_POSITION;
    }

    if(position < CLOSE_GRIPPER_POSITION){
        ROS_WARN("%f is out of gripper range. Moving to %f.", position, CLOSE_GRIPPER_POSITION);
        return CLOSE_GRIPPER_POSITION;
    }

    return position;
}

void ArmInterface::move_gripper(double position, double force){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    position = gripper_validation(position);

    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gripper_client.sendGoal(goal);
}

bool ArmInterface::move_gripper_block(double position, double force){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    position = gripper_validation(position);

    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gripper_client.sendGoalAndWait(goal);

    return _gripper_client.getState() == GoalState::SUCCEEDED;
}

void ArmInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void ArmInterface::move_gripper(CallbackBool callback, double position, double force){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    position = gripper_validation(position);

    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gripper_client.sendGoal(goal, boost::bind(&ArmInterface::generic_done_callback, this, callback, _1));
}

void ArmInterface::open_gripper(double force){
    move_gripper(OPEN_GRIPPER_POSITION, force);
}

bool ArmInterface::open_gripper_block(double force){
    move_gripper_block(OPEN_GRIPPER_POSITION, force);
}

void ArmInterface::open_gripper(CallbackBool callback, double force){
    move_gripper(callback, OPEN_GRIPPER_POSITION, force);
}

void ArmInterface::close_gripper(double force){
    move_gripper(CLOSE_GRIPPER_POSITION, force);
}

bool ArmInterface::close_gripper_block(double force){
    move_gripper_block(CLOSE_GRIPPER_POSITION, force);
}

void ArmInterface::close_gripper(CallbackBool callback, double force){
    move_gripper(callback, CLOSE_GRIPPER_POSITION, force);
}

moveit_msgs::PickupGoal ArmInterface::build_pickup_goal(const std::string &object, const geometry_msgs::Pose &pose, double sigma){
    // build goal
    moveit_msgs::PickupGoal goal;
    
    // generate random vars
    double d_x=0, d_y=0, d_z=0, d_Y=0;

    if(sigma){
        boost::normal_distribution<double> nd(0, sigma);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gen(_rnd_gen, nd);

        d_x = gen();
        d_y = gen();
        d_z = gen();
        d_Y = gen();
    }

    // basic information regarding pick action
    goal.target_name = object;
    goal.group_name = "arm";
    goal.end_effector = "eef";
    goal.allow_gripper_support_collision = false;
    goal.minimize_object_distance = true;
    // goal.attached_object_touch_links.resize(4);
    // goal.attached_object_touch_links[0] = "gripper_link";
    // goal.attached_object_touch_links[1] = "wrist_link";
    // goal.attached_object_touch_links[2] = "left_finger_link";
    // goal.attached_object_touch_links[3] = "right_finger_link";

    // planning stuff
    goal.allowed_planning_time = 5.0;
    goal.planning_options.replan_delay = 0.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.replan = true;
    goal.planning_options.replan_attempts = 10;
    goal.planner_id = "RRTConnectkConfigDefault";

    moveit_msgs::Grasp g;
    // position of gripper before grasp
    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = OPEN_GRIPPER_POSITION;
    
    // position of gripper during grasp (+ grasp force)
    g.grasp_posture.joint_names.push_back("left_finger_joint");
    g.grasp_posture.joint_names.push_back("right_finger_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = CLOSE_GRIPPER_POSITION;
    g.grasp_posture.points[0].effort.resize(1);
    g.grasp_posture.points[0].effort[0] = DEFAULT_GRIPPER_FORCE;

    // position+orientation of end-effector during grasp
    tf::StampedTransform robot_tf;
    _tf_listener.lookupTransform("base_link", "map", ros::Time(0), robot_tf);
    
    tf::Vector3 v_pos(pose.position.x, pose.position.y, pose.position.z);
    tf::Vector3 tf_pos = robot_tf * v_pos;
    tf::Quaternion tf_ori;
    
    g.grasp_pose.header.frame_id = "base_link";
    g.grasp_pose.pose.position.x = tf_pos.getX()-0.04 + d_x;
    g.grasp_pose.pose.position.y = tf_pos.getY() + d_y;
    g.grasp_pose.pose.position.z = tf_pos.getZ();

    double yaw = atan2(tf_pos.getY(), tf_pos.getX());
    tf_ori.setRPY(0.0, 0.0, yaw + d_Y);
    
    g.grasp_pose.pose.orientation.x = tf_ori.getX();
    g.grasp_pose.pose.orientation.y = tf_ori.getY();
    g.grasp_pose.pose.orientation.z = tf_ori.getZ();
    g.grasp_pose.pose.orientation.w = tf_ori.getW();

    // location of end-effector before grasp
    g.pre_grasp_approach.direction.header.frame_id = "base_footprint";
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.1;
    g.pre_grasp_approach.desired_distance = 0.2;

    // location of end-effector after grasp
    g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.allowed_touch_objects.resize(1);
    g.allowed_touch_objects[0] = object;
    g.max_contact_force = 1.0;

    goal.possible_grasps.push_back(g);

    return goal;
}

void ArmInterface::pickup(const std::string object, const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    for(int i=0; i<PLAN_ATTEMPTS; i++){
        moveit_msgs::PickupGoal goal = build_pickup_goal(object, pose, 0.02);
        _pickup_client.sendGoal(goal);

        if(_pickup_client.getState() == GoalState::SUCCEEDED){
            return;
        }
    }
}

bool ArmInterface::pickup_block(const std::string object, const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    for(int i=0; i<PLAN_ATTEMPTS; i++){
        moveit_msgs::PickupGoal goal = build_pickup_goal(object, pose, 0.02);
        _pickup_client.sendGoalAndWait(goal);

        if(_pickup_client.getState() == GoalState::SUCCEEDED){
            return true;
        }
    }
    return false;
}

moveit_msgs::PlaceGoal ArmInterface::build_place_goal(const std::string &object, const geometry_msgs::Pose &pose){
    // build goal
    moveit_msgs::PlaceGoal goal;

    // basic config
    goal.group_name = "arm";
    goal.attached_object_name = object;
    goal.place_eef = true;
    goal.allow_gripper_support_collision = false;
    
    // planning stuff
    goal.planner_id = "RRTConnectkConfigDefault";
    goal.allowed_planning_time = 5.0;
    goal.planning_options.replan = true;
    goal.planning_options.replan_attempts = 10;
    goal.planning_options.replan_delay = 0.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // building place location
    std::vector<moveit_msgs::PlaceLocation> pls;
    moveit_msgs::PlaceLocation pl;

    pl.place_pose.pose = pose;
    pl.place_pose.header.frame_id = "map";

    pl.pre_place_approach.direction.header.frame_id = "base_footprint";
    pl.pre_place_approach.direction.vector.z = -1.0;
    pl.pre_place_approach.min_distance = 0.1;
    pl.pre_place_approach.desired_distance = 0.2;

    pl.post_place_retreat.direction.header.frame_id = "gripper_link";
    pl.post_place_retreat.direction.vector.x = -1.0;
    pl.post_place_retreat.min_distance = 0.0;
    pl.post_place_retreat.desired_distance = 0.2;
    
    pls.push_back(pl);
    goal.place_locations = pls;

    return goal;
}

void ArmInterface::place(const std::string object, const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    moveit_msgs::PlaceGoal goal = build_place_goal(object, pose);
    _place_client.sendGoal(goal);
}

bool ArmInterface::place_block(const std::string object, const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return false;
    }

    moveit_msgs::PlaceGoal goal = build_place_goal(object, pose);
    _place_client.sendGoalAndWait(goal);
    
    return _place_client.getState() == GoalState::SUCCEEDED;
}

bool ArmInterface::move_to_cartesian(const geometry_msgs::Pose &pose){
    std::vector<geometry_msgs::Pose> points;
    points.push_back(pose);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = _move_group->computeCartesianPath(points, 0.01, 0.0, trajectory, true);

    if(fraction < 0)
        return false;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    _move_group->setStartStateToCurrentState();
    _move_group->execute(plan);
    return true;
}

// TODO: fix transform (no publish), improve path
void ArmInterface::push_button(const geometry_msgs::Pose &pose){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    // publish a single frame with the button as its' origin
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform obj_origin;
    
    obj_origin.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    obj_origin.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    tf_broadcaster.sendTransform(tf::StampedTransform(obj_origin, ros::Time::now(), "map", "button"));

    // get transform for this frame
    _tf_listener.waitForTransform("base_link", "button", ros::Time(0), ros::Duration(2.0));

    // find points relative to transforms
    geometry_msgs::PoseStamped org_approch, org_click, tf_approch, tf_click;

    org_approch.pose.position.x = 0.3;
    org_approch.pose.orientation.z = 1.0;
    org_approch.header.stamp = ros::Time(0);
    org_approch.header.frame_id = "button";

    org_click.pose.position.x = -0.2;
    org_click.pose.orientation.z = 1.0;
    org_click.header.stamp = ros::Time(0);
    org_click.header.frame_id = "button";

    _tf_listener.transformPose("base_link", org_approch, tf_approch);
    _tf_listener.transformPose("base_link", org_click, tf_click);

    // move_block(tf_approch.pose);
    
    // plan to cartesian path, don't avoid colisions (to allow button press)
    close_gripper_block();
    move_to_cartesian(tf_approch.pose);
    move_to_cartesian(tf_click.pose);
    move_to_cartesian(tf_approch.pose);
}

ArmInterface::~ArmInterface()
{
    // stop spinner thread
    stop_spinner();
    _spinner_thread->join();

    delete _spinner_thread;
    delete _move_group;
}
