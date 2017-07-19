#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>

#include <armadillo_hl_interface/object_handler.h>

#include <boost/shared_ptr.hpp>

ObjectHandler::ObjectHandler():
    _nh(),
    _find_object(_nh.serviceClient<object_identification::find_obj>("find_object")),
    _ps_interface()
{
    ROS_INFO("waiting for find_object service...");
    _find_object.waitForExistence();
    ROS_INFO("object handler ready.");
}

// TODO: complete this, check if I can get 2d pixek location of object
// asumming object is currently visible to kinect
// bool ObjectHandler::get_orientation_from_wall(geometry_msgs::Quaternion &orientation, const geometry_msgs::Pose &pose, double w=STD_OBJ_W, double h=STD_OBJ_H){
//     sensor_msgs::Image *img = ros::topic::waitForMessage<sensor_msgs::Image>("kinect2/qhd/image_depth_rect", ros::Duration(2.0));
//     sensor_msgs::PointCloud2 *pcl = ros::topic::waitForMessage<sensor_msgs::Image>("kinect2/qhd/points", ros::Duration(2.0));

//     if(!img || !pcl){
//         ROS_ERROR("Can't get object orientation from wall: kinect2 is not available!");
//         return false;
//     }
    
// }

bool ObjectHandler::find_object(geometry_msgs::Pose &target, const std::string &name, int h, int s, int v, int tolerance){
    // call service with given params
    object_identification::find_obj srv;
    srv.request.name = name;
    srv.request.h = h;
    srv.request.s = s;
    srv.request.v = v;
    srv.request.tolerance = tolerance;
    _find_object.call(srv);
    
    // wait for answer
    boost::shared_ptr<const object_identification::find_results> res = ros::topic::waitForMessage<object_identification::find_results>("find_results", ros::Duration(10.0));
    if(!res || !res->success)
        return false;
    
    target = res->target_pose;
    return true;
}

// TODO: build dict and test
bool ObjectHandler::find_object(geometry_msgs::Pose &target, const std::string &name){
    
    return true;
}

ObjectHandler::~ObjectHandler(){
    
}

