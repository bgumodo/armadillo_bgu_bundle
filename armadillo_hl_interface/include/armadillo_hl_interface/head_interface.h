#ifndef HEAD_INTERFACE_H_
#define HEAD_INTERFACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/atomic.hpp>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>
#include <geometry_msgs/Pose.h>

class HeadInterface{
    private:
        typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef pr2_controllers_msgs::PointHeadGoal HIGoal;
        typedef void (*CallbackBool)(bool success);

        boost::atomic<bool> _ready;
        PointHeadClient _point_head_client;

        HIGoal xyz_to_goal(std::string ref_frame, double x, double y, double z, double vel);
        void generic_done_callback(const CallbackBool f, const GoalState &state);

    public:
        HeadInterface();

        // point relative to map
        bool point_head_block(const geometry_msgs::Pose &pose, double vel=1.0);
        void point_head(const geometry_msgs::Pose &pose, double vel=1.0);
        void point_head(const CallbackBool callback, const geometry_msgs::Pose &pose, double vel=1.0);

        // point relative to robot
        bool point_head_block(double x, double y, double z, double vel=1.0);
        void point_head(double x, double y, double z, double vel=1.0);
        void point_head(const CallbackBool callback, double x, double y, double z, double vel=1.0);

        // cancel head motion
        void stop();
        
        ~HeadInterface();
};

#endif