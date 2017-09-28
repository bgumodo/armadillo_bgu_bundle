#ifndef OBJECT_HANDLER_H_
#define OBJECT_HANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// this class is used for detecting objects using kinect2/softkinetic cams
// it requires an external node for handling the vision part (current: temp_find_object)
// TODO: add a non-blocking version
class ObjectHandler{
    private:
        ros::NodeHandle _nh;
        ros::ServiceClient _find_obj;
        ros::Subscriber _res_listener;

    public:
        enum CamType {HEAD_CAM, ARM_CAM};
        
        ObjectHandler();

        // finds an object using a selected depth cam.
        // @RETUEN true if the object is detected, false otherwise
        // @PARAM target: a pose object which will be updated to the object's location
        // @PARAM name: object name to be detected
        // @PARAM camera: HEAD_CAM for kinect2, ARM_CAM for softkinetic
        bool find_object_block(geometry_msgs::Pose &target, const std::string &name, CamType camera=HEAD_CAM);

        ~ObjectHandler();
};

#endif