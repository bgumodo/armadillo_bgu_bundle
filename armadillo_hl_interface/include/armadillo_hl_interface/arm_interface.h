#ifndef ARM_INTERFACE_H_
#define ARM_INTERFACE_H_

#include <ros/ros.h>
#include <boost/atomic.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/PlaceGoal.h>
#include <tf/transform_listener.h>

#define DEFAULT_GRIPPER_FORCE 0.2
#define OPEN_GRIPPER_POSITION 0.14
#define CLOSE_GRIPPER_POSITION 0.01

#define PLAN_ATTEMPTS 25

class ArmInterface{
    private:
        typedef void (*CallbackBool)(bool success);
        typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
        typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;
        typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> PlaceClient;
        typedef actionlib::SimpleClientGoalState GoalState;

        ros::CallbackQueue _cbq;
        ros::NodeHandle _nh;
        tf::TransformListener _tf_listener;
        moveit::planning_interface::MoveGroup *_move_group;
        GripperClient _gripper_client;
        PickupClient _pickup_client;
        PlaceClient _place_client;
        boost::mt19937 _rnd_gen;
        boost::thread *_spinner_thread;
        boost::atomic<bool> _ready;
        boost::atomic<bool> _spin;

        void generic_done_callback(const CallbackBool f, const GoalState &state);
        void start_spinner();
        void stop_spinner();
        double gripper_validation(double val);

        bool plan_to_xyz(moveit::planning_interface::MoveGroup::Plan &plan, double x, double y, double z);
        bool plan_to_pose(moveit::planning_interface::MoveGroup::Plan &plan, const geometry_msgs::Pose &pose);

        moveit_msgs::PickupGoal build_pickup_goal(const std::string &object, const geometry_msgs::Pose &pose, double sigma=0);
        moveit_msgs::PlaceGoal build_place_goal(const std::string &object, const geometry_msgs::Pose &pose);

        bool move_to_cartesian(const geometry_msgs::Pose &pose);

    public:
        ArmInterface();

        // arm control (relative to robot)
        void move(double x, double y, double z);
        bool move_block(double x, double y, double z);

        // arm control (relative to map)
        void move(const geometry_msgs::Pose &pose);
        bool move_block(const geometry_msgs::Pose &pose);

        // arm control (predefined positions)
        void move(const std::string &pose);
        bool move_block(const std::string &pose);

        // gripper control
        void move_gripper(double position, double force=DEFAULT_GRIPPER_FORCE);
        bool move_gripper_block(double position, double force=DEFAULT_GRIPPER_FORCE);
        void move_gripper(CallbackBool callback, double position, double force=DEFAULT_GRIPPER_FORCE);

        void open_gripper(double force=DEFAULT_GRIPPER_FORCE);
        bool open_gripper_block(double force=DEFAULT_GRIPPER_FORCE);
        void open_gripper(CallbackBool callback, double force=DEFAULT_GRIPPER_FORCE);

        void close_gripper(double force=DEFAULT_GRIPPER_FORCE);
        bool close_gripper_block(double force=DEFAULT_GRIPPER_FORCE);
        void close_gripper(CallbackBool callback, double force=DEFAULT_GRIPPER_FORCE);

        // batch high-level control
        void pickup(const std::string object, const geometry_msgs::Pose &pose);
        bool pickup_block(const std::string object, const geometry_msgs::Pose &pose);

        void place(const std::string object, const geometry_msgs::Pose &pose);
        bool place_block(const std::string object, const geometry_msgs::Pose &pose);

        // TODO: implement push_button(...)
        // assuming button position AND orientation are known
        void push_button(const geometry_msgs::Pose &pose);

        ~ArmInterface();
};

#endif