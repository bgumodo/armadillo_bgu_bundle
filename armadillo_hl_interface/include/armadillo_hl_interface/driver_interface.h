#ifndef DRIVER_INTERFACE_H_
#define DRIVER_INTERFACE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <actionlib/server/simple_action_server.h>
#include <armadillo_hl_interface/SimpleDriverAction.h>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#define NUM_OF_SAMPLING_POINTS 32

// a wrapper for a SimpleActionServer which is used for simple driving commands (that is- without move_base)
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

// this class is used for driving the robot
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
        tf::TransformBroadcaster _tf_broadcaster;
        costmap_2d::Costmap2DROS _cm_interface;
        base_local_planner::CostmapModel _cm_model;
        std::map<std::string, geometry_msgs::Pose> _dest_map;
        SimpleDriverServer *_sd_server;
        boost::thread *_sd_thread;

        // used to wrap the user's callback function'
        void generic_done_callback(const CallbackBool f, const GoalState &state);
        // returns true if point is blocked by some object, i.e. not reachable
        bool pose_blocked(const geometry_msgs::Pose &point);
        // returns true if point1 is closer to base then point2
        static bool point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2);
        // returns a new pose in a given radius and angle from a given pose
        geometry_msgs::Pose get_position_from_radius_angle(const geometry_msgs::Pose &object, double radius, double angle);
        // returns closest available point in radius
        bool get_best_pose_in_rad(geometry_msgs::Pose &target, const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius);
        // build DIGoal and handle radius
        bool build_digoal(DIGoal &target, geometry_msgs::Pose &object, double radius, double theta);
        // runs the simple-driver server on a seperate thread
        void start_sd_server();
        // load the destination dictionary
        void import_dest_map(const std::string filename);


    public:
        static const double ANGLE_NEAREST = -1.0;
        static const double ANGLE_FRONT = 0.0;
        static const double ANGLE_BACK = M_PI;

        DriverInterface();

        // drive relative to map- using move_base
        bool drive_block(geometry_msgs::Pose &pose, double radius=0, double theta=ANGLE_NEAREST);
        void drive(geometry_msgs::Pose &object, double radius=0, double theta=ANGLE_NEAREST);
        void drive(const CallbackBool callback, geometry_msgs::Pose &object, double radius=0, double theta=ANGLE_NEAREST);
        
        // drive to a fixed location loaded from the dictionary
        bool drive_block(const std::string &destination);
        void drive(const std::string &destination);
        void drive(const CallbackBool callback, const std::string &destination);

        // drive relative to robot- *not* using move_base
        bool drive_block(double dist, double z, double vel=0.2);
        void drive(double dist, double z, double vel=0.2);
        void drive(const CallbackBool callback, double dist, double z, double vel);

        // stop robot motion
        void stop();

        // TODO: implement these!
        geometry_msgs::Pose get_current_location();
        double distance_from(const geometry_msgs::Pose &other); 

        ~DriverInterface();
};

#endif
