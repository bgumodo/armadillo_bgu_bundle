#include <ros/ros.h>
#include <armadillo_hl_interface/arm_interface.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group.h>
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
    _ready(false)
{
    // init local NodeHandle
    _nh.setCallbackQueue(&_cbq);
    moveit::planning_interface::MoveGroup::Options opt("arm", "robot_description", _nh);
    _move_group = new moveit::planning_interface::MoveGroup(opt);
    boost::thread server_thread(&ArmInterface::spinner_thread, this);

    // init gripper client
    while(ros::ok() && !_gripper_client.waitForServer() && !_pickup_client.waitForServer() && !_place_client.waitForServer()){
        ROS_INFO("waiting for arm control servers...");
        ros::Duration w(1.0);
    }
    _ready = true;
}

void ArmInterface::spinner_thread(){
    while(ros::ok)
        _cbq.callAvailable(ros::WallDuration(0));
}

bool ArmInterface::plan_to_xyz(moveit::planning_interface::MoveGroup::Plan &plan, double x, double y, double z){
    // build goal
    geometry_msgs::Pose p;

    // set position
    p.position.x = x;
    p.position.y=  y;
    p.position.z = z;
    
    // set orientation
    tf::Quaternion q;
    q.setRPY(atan2(y, x), 0.0, 0.0);
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    // plan to goal
    _move_group->setPoseTarget(p);
    return _move_group->plan(plan);
}

bool ArmInterface::plan_to_pose(moveit::planning_interface::MoveGroup::Plan &plan, const geometry_msgs::Pose &pose){
    // transform from map to robot axis
    tf::StampedTransform arm_tf;
    _tf_listener.lookupTransform("base_link", "map", ros::Time(0), arm_tf);
    tf::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    position = arm_tf * position;

    // general orientation- in front of the robot
    tf::Quaternion orientation;
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
    return _move_group->plan(plan);
}

void ArmInterface::move(double x, double y, double z){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    ROS_INFO("planning to pose...");
    moveit::planning_interface::MoveGroup::Plan plan;
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
    moveit::planning_interface::MoveGroup::Plan plan;
    if(plan_to_xyz(plan, x, y, z))
        return _move_group->execute(plan);
    else
        ROS_WARN("can't plan to pose!");
    return false;
}

void ArmInterface::move(const geometry_msgs::Pose &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    moveit::planning_interface::MoveGroup::Plan plan;
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

    moveit::planning_interface::MoveGroup::Plan plan;
    ROS_INFO("planning to pose...");
    if(plan_to_pose(plan, pose))
        return _move_group->execute(plan);
    else
        ROS_WARN("can't plan to pose!");
    return false;
}

void ArmInterface::move(const std::string &pose){
     if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    moveit::planning_interface::MoveGroup::Plan plan;
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

    moveit::planning_interface::MoveGroup::Plan plan;
    ROS_INFO("planning to pose...");
    _move_group->setNamedTarget(pose);
    if(_move_group->plan(plan))
        return _move_group->execute(plan);
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
    _gripper_client.sendGoal(goal, boost::bind(&ArmInterface::generic_done_callback, boost::ref(this), callback, _1));
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
    g.grasp_pose.header.frame_id = "base_link";
    tf::StampedTransform robot_tf;
    _tf_listener.lookupTransform("base_link", "map", ros::Time(0), robot_tf);
    
    tf::Vector3 v_pos(pose.position.x, pose.position.y, pose.position.z);
    tf::Vector3 tf_pos = robot_tf * v_pos;
    tf::Quaternion tf_ori;
    
    g.grasp_pose.pose.position.x = tf_pos.getX()-0.04 + d_x;
    g.grasp_pose.pose.position.y = tf_pos.getY() + d_y;
    g.grasp_pose.pose.position.z = tf_pos.getZ();

    double yaw = atan2(tf_pos.getY(),  tf_pos.getX());
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

void ArmInterface::push_button(const geometry_msgs::Pose &pose){
    if(!_ready){
        ROS_ERROR("ArmInterface is not ready!");
        return;
    }

    // publish a single frame with the button as its' origin
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform obj_origin;
    tf::StampedTransform obj_tf;
    
    obj_origin.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    obj_origin.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    tf_broadcaster.sendTransform(tf::StampedTransform(obj_origin, ros::Time::now(), "map", "button"));

    // get transform for this frame
    _tf_listener.lookupTransform("base_link", "button", ros::Time(0), obj_tf);

    // find points relative to transforms
    geometry_msgs::PoseStamped org_approch, org_click, tf_approch, tf_click;
    org_approch.pose.position.x = 0.2;
    org_approch.header.stamp = ros::Time(0);
    org_approch.header.frame_id = "button";
    org_click.pose.position.x = -0.005;
    org_click.header.stamp = ros::Time(0);
    org_click.header.frame_id = "button";

    _tf_listener.transformPose("base_link", org_approch, tf_approch);
    _tf_listener.transformPose("base_link", org_click, tf_click);

    // plan to cartesian path, don't avoid colisions (to allow button press)
    std::vector<geometry_msgs::Pose> points;
    points.push_back(tf_approch.pose);
    points.push_back(tf_click.pose);
    points.push_back(org_approch.pose);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = _move_group->computeCartesianPath(points, 0.01, 0.0, trajectory, false);
}

ArmInterface::~ArmInterface()
{
    delete _move_group;
}