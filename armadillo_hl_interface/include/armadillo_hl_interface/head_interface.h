#ifndef HEAD_INTERFACE_H_
#define HEAD_INTERFACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/atomic.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

// this class is used for moving the robot's head (kinect2 cam)
class HeadInterface{
    private:
        typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
        typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> PanTiltClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef pr2_controllers_msgs::PointHeadGoal HIGoal;
        typedef control_msgs::FollowJointTrajectoryGoal PTGoal;
        typedef void (*CallbackBool)(bool success);

        boost::atomic<bool> _ready;
        PointHeadClient _point_head_client;
        PanTiltClient _pan_tilt_client;

        HIGoal pose_to_goal(const geometry_msgs::Pose &pose, double vel);
        PTGoal pan_tilt_to_goal(double pan, double tilt, double vel);
        void generic_done_callback(const CallbackBool f, const GoalState &state);
        
    public:
        HeadInterface();

        // point relative to map
        // pose := object pose for robot to look at
        // vel := motion velocity (speed)
        bool move_head_block(const geometry_msgs::Pose &pose, double vel=1.0);
        void move_head(const geometry_msgs::Pose &pose, double vel=1.0);
        void move_head(const CallbackBool callback, const geometry_msgs::Pose &pose, double vel=1.0); // void callback(bool success){...}

        // point relative to robot
        // pan/tilt := head orientation relative to the robot's body
        // vel := motion velocity (speed)
        bool move_head_block(double pan, double tilt, double vel=1.0);
        void move_head(double pan, double tilt, double vel=1.0);
        void move_head(const CallbackBool callback, double pan, double tilt, double vel=1.0); // void callback(bool success){...}

        // cancel head motion
        void stop();
        
        ~HeadInterface();
};

#endif