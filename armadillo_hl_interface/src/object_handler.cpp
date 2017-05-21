#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
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
    set_uc(false);
    return true;
}

ObjectHandler::~ObjectHandler(){
    
}

