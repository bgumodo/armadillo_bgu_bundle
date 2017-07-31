#include <ros/ros.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>

#include <boost/shared_ptr.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void build_obj_dict(){

}

void fail(){
    ROS_INFO("Can't find object!");
    // TODO: publish a fail message
}

bool srv_trigger(object_identification::find_obj::Request &req, object_identification::find_obj::Response &res){
    // get image and point-cloud
    ros::Duration d(10.0);
    
    // no message filter here, assuming robot dosn't move!
    boost::shared_ptr<const sensor_msgs::Image> rgb = ros::topic::waitForMessage<sensor_msgs::Image>("/kinect2/qhd/image_color", d);
    boost::shared_ptr<const sensor_msgs::PointCloud2> pcl = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2/qhd/points", d);
    if(!rgb || !pcl){
        fail();
        return true;
    }

    // find object in rgb image
    

    // publish location to topic and ps_interface
    std::string name = req.name;

	return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "find_object");
    ros::NodeHandle nh;

    // init stuff

    // advertise service and results topic
    ros::ServiceServer srv = nh.advertiseService("find_object", srv_trigger);
    pub = nh.advertise<object_identification::find_results>("find_results", 10);
    
    ros::spin();

    return 0;
}