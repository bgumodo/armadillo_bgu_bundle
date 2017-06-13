#include <ros/ros.h>
#include <std_msgs/String.h>
#include <armadillo_hl_interface/SpeechToTextGoal.h>
#include <armadillo_hl_interface/SpeechToTextAction.h>

class SpeechServer{
    private:
        typedef actionlib::SimpleActionServer<armadillo_hl_interface::SpeechToTextAction> ActionServer;

        ros::CallbackQueue _cbq;
        ros::NodeHandle _nh;
        ActionServer *_s2t_server;
        boost::atomic<bool> _active;
        boost::atomic<bool> _msg_accepted;

        void s2t_callback(const armadillo_hl_interface::SpeechToTextGoalConstPtr &goal);
        void s2t_listener(const std_msgs::String::ConstPtr &msg);

    public:
        SpeechServer();

        void stop_server();        

        ~SpeechServer();
};

class SpeechInterface{
    private:
        typedef actionlib::SimpleActionClient<armadillo_hl_interface::SpeechToTextAction> S2TClient;
        typedef void (*CallbackSpeech)(bool success, std::string speech);
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef armadillo_hl_interface::SpeechToTextGoal S2TGoal;

        S2TClient _s2t_client;
        boost::atomic<bool> _ready;

        void generic_done_callback(const CallbackSpeech callback, const GoalState &state, armadillo_hl_interface::SpeechToTextResult::ConstPtr res);
        void start_server();
        
    public:
        SpeechInterface();

        bool speech_to_text_block(int timeout, std::string &text);
        void speech_to_text(int timeout, CallbackSpeech callback);

        // TODO: implement text-to-speech   

        ~SpeechInterface();

};