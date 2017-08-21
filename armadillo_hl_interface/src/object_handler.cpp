#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>
#include <boost/shared_ptr.hpp>
#include <armadillo_hl_interface/object_handler.h>

ObjectHandler::ObjectHandler():
    _nh(),
    _find_obj(_nh.serviceClient<object_identification::find_obj>("find_object"))
{
    ros::Duration d(2.0);
    while(!_find_obj.waitForExistence(d))
        ROS_INFO("waiting for temp find_object service...");

    ROS_INFO("temp object-handler ready.");
}

bool ObjectHandler::find_object_block(geometry_msgs::Pose &target, const std::string &name, CamType camera){
    // call service with given params
    object_identification::find_obj srv;
    srv.request.name = name;

    switch(camera){
        case HEAD_CAM:
            srv.request.camera = object_identification::find_obj::Request::HEAD_CAM;
            break;
        case ARM_CAM:
            srv.request.camera = object_identification::find_obj::Request::ARM_CAM;
            break;
    }

    _find_obj.call(srv);
    
    // wait for answer
    boost::shared_ptr<const object_identification::find_results> res = ros::topic::waitForMessage<object_identification::find_results>("/find_results", ros::Duration(5.0));
    if(!res || !res->success){
        return false;
    }
    
    // return answer
    target = res->target_pose;
    return true;
}

ObjectHandler::~ObjectHandler(){
    
}

