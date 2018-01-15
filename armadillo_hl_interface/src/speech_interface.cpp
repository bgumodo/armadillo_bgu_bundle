#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Trigger.h>
#include <armadillo_hl_interface/speech_interface.h>
#include <armadillo_hl_interface/SpeechToTextResult.h>
#include <armadillo_hl_interface/SpeechToTextFeedback.h>
#include <armadillo_hl_interface/TextToSpeech.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>

// S2TServer implamentation

S2TServer::S2TServer():
    _cbq(),
    _nh(),
    _s2t_server(0),
    _active(false),
    _msg_accepted(false)
{
    // set callback queue
    _nh.setCallbackQueue(&_cbq);

    // subscribe to speech node topics
    ros::Subscriber s2t_pub = _nh.subscribe("speech_text", 10, &S2TServer::s2t_listener, this);

    // start server
    _s2t_server = new S2TActionServer(_nh, "speech_to_text_action", boost::bind(&S2TServer::s2t_callback, this, _1), false);
    _s2t_server->start();
    
    // spin server
    _active = true;
    while(_active && ros::ok()){
        _cbq.callAvailable(ros::WallDuration(0));
    }
}

void S2TServer::s2t_listener(const std_msgs::String::ConstPtr &msg){
    if(!_msg_accepted){
        _msg_accepted = true;
        armadillo_hl_interface::SpeechToTextResult result;
        result.res = msg->data;
        _s2t_server->setSucceeded(result);
    }
}

void S2TServer::s2t_callback(const armadillo_hl_interface::SpeechToTextGoalConstPtr &goal){
    // make sure input is legal
    if(goal->timeout<0){
        ROS_ERROR("timeout must be 0 or longer!");
        armadillo_hl_interface::SpeechToTextResult result;
        _s2t_server->setPreempted(result);
    }

    // trigger speech-to-text service
    ROS_INFO("calling service...");
    ros::ServiceClient srv = _nh.serviceClient<std_srvs::Trigger>("speech_to_text");
    std_srvs::Trigger trg;
    srv.call(trg);

    // wait for answer until timeout
    _msg_accepted = false;
    ros::Duration d(1.0);
    armadillo_hl_interface::SpeechToTextFeedback feedback;
    for(int i=goal->timeout; i>0 && !_msg_accepted; i--){
        feedback.countdown = i;
        _s2t_server->publishFeedback(feedback);
        d.sleep();
    }
    
    // if no answer before the timeout
    if(!_msg_accepted){
        armadillo_hl_interface::SpeechToTextResult result;
        _s2t_server->setPreempted(result);
    }
}

void S2TServer::stop_server(){
    _active = false;
}

S2TServer::~S2TServer(){
    delete _s2t_server;
}

// T2SServer implamentation

T2SServer::T2SServer():
    _cbq(),
    _nh(),
    _t2s_server(0),
    _active(false)
{
    // set callback queue
    _nh.setCallbackQueue(&_cbq);

    // start server
    _t2s_server = new T2SActionServer(_nh, "text_to_speech_action", boost::bind(&T2SServer::t2s_callback,this, _1), false);
    _t2s_server->start();
    
    // spin server
    _active = true;
    while(_active && ros::ok()){
        _cbq.callAvailable(ros::WallDuration(0));
    }
}

// TODO: make speech node return before speech itself (and indicate when it is done);
void T2SServer::t2s_callback(const armadillo_hl_interface::TextToSpeechGoalConstPtr &goal){
    // trigger text-to-speech service
    ros::ServiceClient srv = _nh.serviceClient<armadillo_hl_interface::TextToSpeech>("text_to_speech");
    armadillo_hl_interface::TextToSpeech trg;
    trg.request.text = goal->text;
    armadillo_hl_interface::TextToSpeechResult result;
    if(!srv.call(trg)){ // failed to call service
        ROS_ERROR("TextToSpeech service is not available!");
        result.success = false;
        _t2s_server->setPreempted(result);
    }
    else{
        result.success = trg.response.success;
        _t2s_server->setSucceeded(result);
    }
}

void T2SServer::stop_server(){
    _active = false;
}

T2SServer::~T2SServer(){
    delete _t2s_server;
}

// SpeechInterface implamentation

SpeechInterface::SpeechInterface():
    _s2t_client("speech_to_text_action", true),
    _t2s_client("text_to_speech_action", true),
    _s2t_thread(0),
    _t2s_thread(0),
    _s2t_server(0),
    _t2s_server(0),
    _ready(false)
{
    // start server thread
    _s2t_thread = new boost::thread(&SpeechInterface::start_s2t_server, this);
    _t2s_thread = new boost::thread(&SpeechInterface::start_t2s_server, this);

    // wait for server to come-up
    ros::Duration w(1.0);
    while(!_s2t_client.waitForServer() && !_t2s_client.waitForServer() && ros::ok()){
        ROS_INFO("waiting for speech action-server...");
        w.sleep();
    }

    // interface is ready
    _ready = true;
}

bool SpeechInterface::speech_to_text_block(int timeout, std::string &text){
    // make sure interface is ready
    if(!_ready){
        ROS_ERROR("SpeechInterface is not ready!");
        return false;
    }
    
    // send goal to action serevr (and wait for answer)
    S2TGoal goal;
    goal.timeout = timeout;
    _s2t_client.sendGoalAndWait(goal);
    
    // get answer and return
    text = _s2t_client.getResult()->res;
    return _s2t_client.getState() == GoalState::SUCCEEDED;
}

void SpeechInterface::generic_speech_callback(const CallbackSpeech callback, const GoalState &state, armadillo_hl_interface::SpeechToTextResult::ConstPtr res){
    callback(state == GoalState::SUCCEEDED, res->res);
}

void SpeechInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

void SpeechInterface::speech_to_text(int timeout, CallbackSpeech callback){
    // make sure interface is ready
    if(!_ready){
        ROS_ERROR("SpeechInterface is not readnony!");
        return;
    }

    // send goal to action serevr
    S2TGoal goal;
    goal.timeout = timeout;
    _s2t_client.sendGoal(goal, boost::bind(&SpeechInterface::generic_speech_callback,this, callback, _1, _2));
}

bool SpeechInterface::text_to_speech_block(const std::string &text){
    // make sure interface is ready
    if(!_ready){
        ROS_ERROR("SpeechInterface is not ready!");
        return false;
    }

    // send goal to action serevr (and wait for answer)
    T2SGoal goal;
    goal.text = text;
    _t2s_client.sendGoalAndWait(goal);

    // return true for success
    return _t2s_client.getState() == GoalState::SUCCEEDED;
}

void SpeechInterface::text_to_speech(const std::string &text){
    // make sure interface is ready
    if(!_ready){
        ROS_ERROR("SpeechInterface is not ready!");
        return;
    }

    // send goal to action serevr
    T2SGoal goal;
    goal.text = text;
    _t2s_client.sendGoal(goal);
}

void SpeechInterface::text_to_speech(CallbackBool callback, const std::string &text){
    // make sure interface is ready
    if(!_ready){
        ROS_ERROR("SpeechInterface is not ready!");
        return;
    }

    // send goal to action serevr
    T2SGoal goal;
    goal.text = text;
    _t2s_client.sendGoal(goal, boost::bind(&SpeechInterface::generic_done_callback, this, callback, _1));
}

void SpeechInterface::start_s2t_server(){
    ROS_INFO("starting speech action-server...");
    _s2t_server = new S2TServer();
}

void SpeechInterface::start_t2s_server(){
    ROS_INFO("starting speech action-server...");
    _t2s_server = new T2SServer();
}

SpeechInterface::~SpeechInterface(){
    // stop servers and wait for server threads
    _s2t_server->stop_server();
    _t2s_server->stop_server();
    _s2t_thread->join();
    _t2s_thread->join();

    delete _s2t_server;
    delete _t2s_server;
    delete _s2t_thread;
    delete _t2s_thread;
}
