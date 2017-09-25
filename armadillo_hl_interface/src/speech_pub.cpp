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

#include "std_msgs/String.h"




int main(int argc, char **argv){

    ros::init(argc, argv, "speech_pub");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("/speech_text2", 1000);

    ros::Rate loop_rate(10);

    SpeechInterface si;
    std::string text = "";
    si.text_to_speech_block("what do you need?");
    si.speech_to_text_block(10, text);

    while(text!="Bring me coffee."){
       si.text_to_speech_block("i did not understand, please try again, do you want coffee?");
       si.speech_to_text_block(10, text);
       if(text == "Yes."){
            text = "Bring me coffee.";
            break;
       }
    }
  
      std_msgs::String msg;
      std::stringstream ss;
      ss << text;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
     


       return 0;
}
