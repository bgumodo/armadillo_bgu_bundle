#include <ros/ros.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

typedef struct objrec{
    int h_low;
    int s_low;
    int v_low;
    int h_high;
    int s_high;
    int v_high;
    shape_msgs::SolidPrimitive shape;
} props;

typedef std::pair<std::string, objrec> dict_item;

ros::Publisher res_pub;
ros::Publisher psi_pub;
std::map<std::string, objrec> obj_dict;

void build_obj_dict(){
    // button
    objrec button;
    button.h_low = 120;
    button.s_low = 100;
    button.v_low = 100;
    button.h_high = 255;
    button.s_high = 255;
    button.v_high = 255;
    button.shape.type = button.shape.BOX;
    button.shape.dimensions.resize(3);
    button.shape.dimensions[0] = 0.02;
    button.shape.dimensions[1] = 0.1;
    button.shape.dimensions[2] = 0.1;
    obj_dict.insert(dict_item("button", button));
    
    // can
    objrec can;
    can.h_low = 100;
    can.s_low = 110;
    can.v_low = 40;
    can.h_high = 180;
    can.s_high = 200;
    can.v_high = 255;
    can.shape.type = button.shape.CYLINDER;
    can.shape.dimensions.resize(2);
    can.shape.dimensions[0] = 0.17;
    can.shape.dimensions[1] = 0.03;
    obj_dict.insert(dict_item("can", can));
}

void fail(){
    ROS_INFO("Can't find object!");
    object_identification::find_results res_msg;
    res_msg.success = false;
    res_pub.publish(res_msg);
}

void find(std::string name, objrec rec, const unsigned char camera){
    std::string rgb_topic, pcl_topic, cam_frame;

    // set cam topic and tf frame
    switch(camera){
        case object_identification::find_obj::Request::HEAD_CAM:
            rgb_topic = "/kinect2/qhd/image_color";
            pcl_topic = "/kinect2/qhd/points";
            cam_frame = "kinect2_depth_optical_frame";
            break;
        case object_identification::find_obj::Request::ARM_CAM:
            rgb_topic = "/softkinetic_camera/rgb/image_raw";
            pcl_topic = "/softkinetic_camera/depth/points";
            cam_frame = "softkinetic_camera_depth_optical_frame";
            break;
    }

    // get image and point-cloud
    ros::Duration d(10.0);
    
    // no message filter here, assuming robot dosn't move!
    boost::shared_ptr<const sensor_msgs::Image> rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic, d);
    boost::shared_ptr<const sensor_msgs::PointCloud2> pcl_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, d);
    if(!rgb_msg || !pcl_msg){
        fail();
        return;
    }

    // find object in rgb image
    cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg)->image;

    // process image
    cv::GaussianBlur(rgb_img, rgb_img, cv::Size(5, 5), 1.0); // blur
    cv::Mat hsv_img;
    cv::cvtColor(rgb_img, hsv_img, CV_RGB2HSV); // move to hsv
    int lower_arr[] = {rec.h_low, rec.s_low, rec.v_low};
    std::vector<int> lower(lower_arr, lower_arr+3);
    int upper_arr[] = {rec.h_high, rec.s_high, rec.v_high};
    std::vector<int> upper(upper_arr, upper_arr+3);
    cv::inRange(hsv_img, lower, upper, hsv_img); // filter just reds
    cv::morphologyEx(hsv_img, hsv_img, CV_MOP_OPEN, cv::Mat::ones(5, 5, hsv_img.type())); // do some cleaning

    // find mean
    int mean_x=0, mean_y=0, sum=0;
    for(int i=0; i<hsv_img.rows; i++){
        for(int j=0; j<hsv_img.cols; j++){
            if(hsv_img.at<unsigned char>(i, j)){
                sum++;
                mean_x += j;
                mean_y += i;
            }
        }
    }

    // // for debug
    // cv::imshow("sample", hsv_img);
    // cv::waitKey(0);

    if(sum <= 10){
        fail();
        return;
    }

    mean_x /= sum;
    mean_y /= sum;

    // get location from mean
    pcl::PointCloud<pcl::PointXYZRGB> pcl_conv;
    pcl::fromROSMsg(*pcl_msg, pcl_conv);
    pcl::PointXYZRGB point = pcl_conv[(mean_y * pcl_conv.width) + mean_x];
    
    // convert to global position (relative to map)
    tf::TransformListener tf_listener;
    if(!tf_listener.waitForTransform("map", cam_frame, ros::Time(0), d)){
        fail();
        return;
    }
    
    tf::StampedTransform transform;
    tf_listener.lookupTransform("map", cam_frame, ros::Time(0), transform);
    tf::Vector3 pose_vec(point.x, point.y, point.z);
    tf::Vector3 trans_vec = transform * pose_vec;

    // publish location to topic and ps_interface
    ROS_INFO_STREAM("'" << name << "'' located at (map coordinates) > X: " << trans_vec.x() << ", Y: " << trans_vec.y() << ", Z: " << trans_vec.z());
    
    // TODO: move this to dict-building function!
    moveit_msgs::CollisionObject col_obj;
    col_obj.header.frame_id = "map";
    col_obj.id = name;

    // object pose
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0; // default orientation
    pose.position.x = trans_vec.x();
    pose.position.y = trans_vec.y();
    pose.position.z = trans_vec.z();

    // build object msg
    col_obj.primitives.push_back(rec.shape);
    col_obj.primitive_poses.push_back(pose);
    col_obj.operation = col_obj.ADD;

    // build planning_scene msg and publish
    moveit_msgs::PlanningScene ps_msg;
    ps_msg.world.collision_objects.push_back(col_obj);
    ps_msg.is_diff = true;
    psi_pub.publish(ps_msg);

    // publish results
    object_identification::find_results res_msg;
    res_msg.target_pose = pose;
    res_msg.success = true;
    res_pub.publish(res_msg);
}

bool srv_trigger(object_identification::find_obj::Request &req, object_identification::find_obj::Response &res){
    ROS_INFO("temp find_object triggered!");
    std::string name = req.name;

    // check that item is in dict
    std::map<std::string, objrec>::iterator obj_iter = obj_dict.find(name);
	if(obj_iter == obj_dict.end()){
	    ROS_ERROR_STREAM("temp find_obj got an unkown object ('" << name << "')!");
        return false;
	}
	objrec rec = obj_iter->second;

    // start working thread
    boost::thread work_thread(find, name, rec, req.camera);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "find_object");
    ros::NodeHandle nh;

    // init stuff
    build_obj_dict();

    // advertise service and results topic
    ros::ServiceServer srv = nh.advertiseService("find_object", srv_trigger);
    res_pub = nh.advertise<object_identification::find_results>("find_results", 10);
    psi_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    while(ros::ok() && psi_pub.getNumSubscribers() < 1){
        ros::Duration(2.0).sleep();
        ROS_INFO("temp find_object is waiting for planning_scene...");
    }
    
    ROS_INFO("temp find_object is ready.");
    ros::spin();

    return 0;
}