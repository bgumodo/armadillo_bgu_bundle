#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>

#include <armadillo_hl_interface/object_handler.h>

ObjectHandler::ObjectHandler():
    _nh(),
    _update_collision(_nh.serviceClient<std_srvs::SetBool>("update_collision_objects")),
    _ps_interface(),
    _w(1.0)
{
    ROS_INFO("waiting for uc service...");
    _update_collision.waitForExistence();
    ROS_INFO("object handler ready.");
}

void ObjectHandler::set_uc(bool val){
    std_srvs::SetBool srv;
    srv.request.data = val;
    _update_collision.call(srv);
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

bool ObjectHandler::find_object(geometry_msgs::Pose &target, const std::string &name){
    // set uc_service
    set_uc(true);

    // some time to update collision service
    _w.sleep();

    // get objects pose map
    std::vector<std::string> ids;
    ids.push_back(name);
    std::map<std::string, geometry_msgs::Pose> poses = _ps_interface.getObjectPoses(ids);

    // return false if map is empty
    if(poses.empty())
        return false;

    // found a suitable object
    target = poses[name];
    // temp for button
    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 0;
    target.orientation.w = 1;
    set_uc(false);
    return true;
}

ObjectHandler::~ObjectHandler(){
    
}

