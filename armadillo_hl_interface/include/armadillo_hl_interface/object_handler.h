#ifndef OBJECT_HANDLER_H_
#define OBJECT_HANDLER_H_

#define STD_OBJ_W = 0.4;
#define STD_OBJ_H = 0.4;

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ObjectHandler{
    private:
        ros::NodeHandle _nh;
        ros::ServiceClient _update_collision;
        moveit::planning_interface::PlanningSceneInterface _ps_interface;
        ros::Duration _w;

        void set_uc(bool val); // set state of update_collision_service
        // bool get_orientation_from_wall(geometry_msgs::Quaternion &orientation, const geometry_msgs::Pose &pose, double w=STD_OBJ_W, double h=STD_OBJ_H); // returns wall oreintation at object position pose

    public:
        ObjectHandler();

        bool find_object(geometry_msgs::Pose &target, const std::string &name);

        ~ObjectHandler();
};

#endif