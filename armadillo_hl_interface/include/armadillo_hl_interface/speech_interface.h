#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <armadillo_hl_interface/SpeechToTextGoal.h>
#include <armadillo_hl_interface/SpeechToTextAction.h>
#include <armadillo_hl_interface/TextToSpeechGoal.h>
#include <armadillo_hl_interface/TextToSpeechAction.h>

class S2TServer{
    private:
        typedef actionlib::SimpleActionServer<armadillo_hl_interface::SpeechToTextAction> S2TActionServer;

        ros::CallbackQueue _cbq;
        ros::NodeHandle _nh;
        S2TActionServer *_s2t_server;
        boost::atomic<bool> _active;
        boost::atomic<bool> _msg_accepted;

        void s2t_callback(const armadillo_hl_interface::SpeechToTextGoalConstPtr &goal);
        void s2t_listener(const std_msgs::String::ConstPtr &msg);

    public:
        S2TServer();

        void stop_server();        

        ~S2TServer();
};

class T2SServer{
    private:
        typedef actionlib::SimpleActionServer<armadillo_hl_interface::TextToSpeechAction> T2SActionServer;

        ros::CallbackQueue _cbq;
        ros::NodeHandle _nh;
        T2SActionServer *_t2s_server;
        boost::atomic<bool> _active;

        void t2s_callback(const armadillo_hl_interface::TextToSpeechGoalConstPtr &goal);

    public:
        T2SServer();

        void stop_server();        

        ~T2SServer();
};


class SpeechInterface{
    private:
        typedef actionlib::SimpleActionClient<armadillo_hl_interface::SpeechToTextAction> S2TClient;
        typedef actionlib::SimpleActionClient<armadillo_hl_interface::TextToSpeechAction> T2SClient;
        typedef void (*CallbackBool)(bool success);
        typedef void (*CallbackSpeech)(bool success, std::string speech);
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef armadillo_hl_interface::SpeechToTextGoal S2TGoal;
        typedef armadillo_hl_interface::TextToSpeechGoal T2SGoal;

        S2TClient _s2t_client;
        T2SClient _t2s_client;
        boost::thread *_s2t_thread;
        boost::thread *_t2s_thread;
        S2TServer *_s2t_server;
        T2SServer *_t2s_server;
        boost::atomic<bool> _ready;

        void generic_done_callback(const CallbackBool f, const GoalState &state);
        void generic_speech_callback(const CallbackSpeech callback, const GoalState &state, armadillo_hl_interface::SpeechToTextResult::ConstPtr res);
        void start_s2t_server();
        void start_t2s_server();
        
    public:
        SpeechInterface();

        bool speech_to_text_block(int timeout, std::string &text);
        void speech_to_text(int timeout, CallbackSpeech callback);

        // TODO: resolve issues with text-to-speech
        bool text_to_speech_block(const std::string &text);
        void text_to_speech(const std::string &text);
        void text_to_speech(CallbackBool callback, const std::string &text);

        ~SpeechInterface();

};