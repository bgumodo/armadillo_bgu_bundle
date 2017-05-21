#ifndef OBJECT_HANDLER_H_
#define OBJECT_HANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ObjectHandler{
    private:
        ros::NodeHandle _nh;
        ros::ServiceClient _update_collision;
        moveit::planning_interface::PlanningSceneInterface _ps_interface;
        ros::Duration _w;

        void set_uc(bool val); // set state of update_collision_service

    public:
        ObjectHandler();

        bool find_object(geometry_msgs::Pose &target, const std::string &name);

        ~ObjectHandler();
};

#endif