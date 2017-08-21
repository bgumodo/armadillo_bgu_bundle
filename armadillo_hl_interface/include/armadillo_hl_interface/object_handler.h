#ifndef OBJECT_HANDLER_H_
#define OBJECT_HANDLER_H_

// for temp mockup find-object node
// TODO: add a non-blocking version

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class ObjectHandler{
    private:
        ros::NodeHandle _nh;
        ros::ServiceClient _find_obj;
        ros::Subscriber _res_listener;

    public:
        enum CamType {HEAD_CAM, ARM_CAM};
        
        ObjectHandler();

        bool find_object_block(geometry_msgs::Pose &target, const std::string &name, CamType camera=HEAD_CAM);

        ~ObjectHandler();
};

#endif