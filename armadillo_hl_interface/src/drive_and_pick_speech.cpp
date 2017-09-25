#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <robotican_common/switch_topic.h>

void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Message: [%s]", msg->data.c_str());
  std::string inputString = msg->data.c_str();

  if(inputString=="Bring me coffee.") {
  	ROS_INFO("entering");
   	ros::NodeHandle nh;
    ros::ServiceClient drive_client = nh.serviceClient<std_srvs::Trigger>("drive2object_go");
    ros::ServiceClient pick_client = nh.serviceClient<std_srvs::Trigger>("pick_go");
    ros::ServiceClient sw_client = nh.serviceClient<robotican_common::switch_topic>("switch_pcl_topic");
    // ros::ServiceClient uc_client = nh.serviceClient<std_srvs::SetBool>("update_collision_objects");

    ROS_INFO("Waiting for services...");
    drive_client.waitForExistence();
    pick_client.waitForExistence();
    sw_client.waitForExistence();
    // uc_client.waitForExistence();

    robotican_common::switch_topic sw_srv;
    sw_srv.request.num=1;
    sw_client.call(sw_srv);

    ros::Duration(5).sleep();
    ROS_INFO("Ready to drive!");
    std_srvs::Trigger drive_srv;
    if (drive_client.call(drive_srv))
    {
        ROS_INFO("drive2object response: %s", drive_srv.response.message.c_str());
        if (drive_srv.response.success) {
            sw_srv.request.num=2;
            sw_client.call(sw_srv);
            ros::Duration(5).sleep();
            std_srvs::Trigger pick_srv;
            if (pick_client.call(pick_srv)) {
                ROS_INFO("pick response: %s", pick_srv.response.message.c_str());
                if (pick_srv.response.success)  ROS_INFO("Done!");
            }
            else ROS_ERROR("Failed to call pick service");
        }
    }
    else ROS_ERROR("Failed to call drive2object service");
  }
  ROS_INFO("Done");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_speech");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/speech_text2", 1000, Callback);
  ros::spin();

  return 0;
}


