#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <armadillo_hl_interface/TorsoGoal.h>
#include <armadillo_hl_interface/TorsoAction.h>
#include <tf/transform_listener.h>
#include <boost/atomic.hpp>

#define MIN_HEIGHT_TORSO 0.05
#define MAX_HEIGHT_TORSO 0.37
#define MIN_HEIGHT_TORSO_LINK 0.561577
#define MAX_HEIGHT_TORSO_LINK 0.825775
#define EPSILON_TORSO 0.02

// an action server which wraps the basic torso controller
// runs on a local callback queue, so that it can run on a different thread
class TorsoServer{
    private:
        typedef actionlib::SimpleActionServer<armadillo_hl_interface::TorsoAction> ActionServer;

        ros::CallbackQueue _cbq;
        ros::NodeHandle _nh;
        ActionServer *_torso_server;
        ros::Publisher _pub;
        tf::TransformListener _tf_listener;
        boost::atomic<bool> _active;

        void callback(const armadillo_hl_interface::TorsoGoalConstPtr &goal);

    public:
        TorsoServer();

        void stop_server();        

        ~TorsoServer();
};

// TorsoInterface implamentation
class TorsoInterface{
    private:
        typedef void (*CallbackBool)(bool success);
        typedef actionlib::SimpleActionClient<armadillo_hl_interface::TorsoAction> TorsoClient;
        typedef armadillo_hl_interface::TorsoGoal TGoal;
        typedef actionlib::SimpleClientGoalState GoalState;

        TorsoClient _torso_client;
        boost::atomic<bool> _ready;
        boost::thread *_server_thread;
        TorsoServer *_server;

        double height_validation(double height);
        void start_server();
        void generic_done_callback(const CallbackBool f, const GoalState &state);

    public:
        static const float MIN_HEIGHT = MIN_HEIGHT_TORSO;
        static const float MAX_HEIGHT = MAX_HEIGHT_TORSO;

        TorsoInterface();

        bool move_block(double height);
        void move(double height);
        void move(const CallbackBool callback, double height);

        ~TorsoInterface();
};