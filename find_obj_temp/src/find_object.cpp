#include <ros/ros.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>

#include <boost/shared_ptr.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;

void build_obj_dict(){

}

void fail(){
    ROS_INFO("Can't find object!");
    // TODO: publish a fail message
}

bool srv_trigger(object_identification::find_obj::Request &req, object_identification::find_obj::Response &res){
    ROS_INFO("Triggered!");
    // get image and point-cloud
    ros::Duration d(10.0);
    
    // no message filter here, assuming robot dosn't move!
    boost::shared_ptr<const sensor_msgs::Image> rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/kinect2/qhd/image_color", d);
    boost::shared_ptr<const sensor_msgs::PointCloud2> pcl_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2/qhd/points", d);
    if(!rgb_msg || !pcl_msg){
        fail();
        return true;
    }

    // find object in rgb image
    cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg)->image;

    // process image
    cv::GaussianBlur(rgb_img, rgb_img, cv::Size(5, 5), 1.0); // blur
    cv::Mat hsv_img;
    cv::cvtColor(rgb_img, hsv_img, CV_RGB2HSV); // move to hsv
    int lower_arr[] = {120, 100, 100};
    std::vector<int> lower(lower_arr, lower_arr+3);
    int upper_arr[] = {255, 255, 255};
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
    mean_x /= sum;
    mean_y /= sum;
    
    // // for debug
    // cv::imshow("sample", hsv_img);
    // cv::waitKey(0);

    // get location from mean
    pcl::PointCloud<pcl::PointXYZRGB> pcl_conv;
    pcl::fromROSMsg(*pcl_msg, pcl_conv);
    pcl::PointXYZRGB point = pcl_conv[(mean_y * pcl_conv.width) + mean_x];
    
    // convert to global position (relative to map)


    ROS_INFO_STREAM("X img: " << mean_x << ", Y img: " << mean_y);
    ROS_INFO_STREAM("X: " << point.x << ", Y: " << point.y);

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
    
    ROS_INFO("temp find_object is ready.");
    ros::spin();

    return 0;
}