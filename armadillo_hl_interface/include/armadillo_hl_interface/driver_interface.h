#ifndef DRIVER_INTERFACE_H_
#define DRIVER_INTERFACE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <actionlib/server/simple_action_server.h>
#include <armadillo_hl_interface/SimpleDriverAction.h>
#include <boost/atomic.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#define NUM_OF_SAMPLING_POINTS 32

class SimpleDriverServer{
    private:
        typedef actionlib::SimpleActionServer<armadillo_hl_interface::SimpleDriverAction> ActionServer;

        ros::NodeHandle _nh;
        ros::CallbackQueue _cbq;
        ActionServer *_action_server;
        tf::TransformListener _tf_listener;
        ros::Publisher _pub;        
        boost::atomic<bool> _active;
        
        void callback(const armadillo_hl_interface::SimpleDriverGoalConstPtr &goal);

    public:
        SimpleDriverServer();
        void stop_server();
        ~SimpleDriverServer();
};

class DriverInterface{
     private:
        typedef move_base_msgs::MoveBaseGoal DIGoal;
        typedef armadillo_hl_interface::SimpleDriverGoal SDGoal;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;
        typedef actionlib::SimpleActionClient<armadillo_hl_interface::SimpleDriverAction> SDClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef void (*CallbackBool)(bool success);

        typedef std::pair<std::string, geometry_msgs::Pose> dest_dict_item;
    
        boost::atomic<bool> _ready;
        MBClient _mb_client;
        SDClient _sd_client;
        tf::TransformListener _tf_listener;
        costmap_2d::Costmap2DROS _cm_interface;
        base_local_planner::CostmapModel _cm_model;
        std::map<std::string, geometry_msgs::Pose> _dest_map;

        // used to wrap the user's callback function'
        void generic_done_callback(const CallbackBool f, const GoalState &state);
        // returns true if point is blocked by some object, i.e. not reachable
        bool pose_blocked(const geometry_msgs::Pose &point);
        // returns true if point1 is closer to base then point2
        static bool point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2);
        // returns closest available point in radius
        bool get_best_pose_in_rad(geometry_msgs::Pose &target, const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius);
        // build DIGoal and handle radius
        bool build_digoal(DIGoal &target, geometry_msgs::Pose &object, double radius);
        // runs the simple-driver server on a seperate thread
        void start_sd_server();
        // load the destination dictionary
        void import_dest_map(const std::string filename);


    public:
        DriverInterface();

        // drive relative to map- using move_base
        bool drive_block(geometry_msgs::Pose &pose, double radius=0);
        void drive(geometry_msgs::Pose &object, double radius=0);
        void drive(const CallbackBool callback, geometry_msgs::Pose &object, double radius=0);
        
        // drive using premade dict
        bool drive_block(const std::string &destination);
        void drive(const std::string &destination);
        void drive(const CallbackBool callback, const std::string &destination);

        // drive relative to robot- *not* using move_base
        bool drive_block(double dist, double z, double vel=0.2);
        void drive(double dist, double z, double vel=0.2);
        void drive(const CallbackBool callback, double dist, double z, double vel);

        void stop();
        bool is_ready();

        ~DriverInterface();
};

#endif
