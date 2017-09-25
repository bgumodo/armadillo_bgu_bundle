#include <ros/ros.h>
#include <object_identification/find_obj.h>
#include <object_identification/find_results.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
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

struct objrec{
    int h_low;
    int s_low;
    int v_low;
    int h_high;
    int s_high;
    int v_high;
    shape_msgs::SolidPrimitive shape;
    enum {DEFAULT, SURFACE} ori_type;
};

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
    button.ori_type = objrec::DEFAULT; // should be objrec::SURFACE
    obj_dict.insert(dict_item("button", button));
    
    // can
    objrec can;
    // can.h_low = 115;
    // can.s_low = 110;
    // can.v_low = 30;
    can.h_low = 115;
    can.s_low = 110;
    can.v_low = 50;
    can.h_high = 180;
    can.s_high = 255;
    can.v_high = 255;
    can.shape.type = button.shape.CYLINDER;
    can.shape.dimensions.resize(2);
    can.shape.dimensions[0] = 0.17;
    can.shape.dimensions[1] = 0.03;
    can.ori_type = objrec::DEFAULT;
    obj_dict.insert(dict_item("can", can));

    // bench
    objrec bench;
    // bench.h_low = 0;
    // bench.s_low = 40;
    // bench.v_low = 50;
    // bench.h_high = 25;
    // bench.s_high = 255;
    // bench.v_high = 200;
    bench.h_low = 100;
    bench.s_low = 100;
    bench.v_low = 100;
    bench.h_high = 255;
    bench.s_high = 255;
    bench.v_high = 255;
    bench.shape.type = button.shape.BOX;
    bench.shape.dimensions.resize(3);
    bench.shape.dimensions[0] = 0.3;
    bench.shape.dimensions[1] = 1.1;
    bench.shape.dimensions[2] = 0.02;
    bench.ori_type = objrec::DEFAULT; // should be objrec::SURFACE
    obj_dict.insert(dict_item("bench", bench));

    // can for robot sim
    objrec can_real;
    can.h_low = 115;
    can.s_low = 110;
    can.v_low = 30;
    can_real.h_high = 180;
    can_real.s_high = 255;
    can_real.v_high = 255;
    can_real.shape.type = button.shape.CYLINDER;
    can_real.shape.dimensions.resize(2);
    can_real.shape.dimensions[0] = 0.17;
    can_real.shape.dimensions[1] = 0.03;
    can_real.ori_type = objrec::DEFAULT;
    obj_dict.insert(dict_item("can_real", can_real));
}

void fail(){
    ROS_INFO("Can't find object!");
    object_identification::find_results res_msg;
    res_msg.success = false;
    res_pub.publish(res_msg);
}

// tf::Vector3 get_cross_product(const tf::Vector3 &u, const tf::Vector3 &v){
//     tf::Vector3 ans;
//     ans.setX(u.y * v.z - u.z * v.y);
//     ans.setY(u.z * v.x - u.x * v.z);
//     ans.setZ(u.x * v.y - u.y * v.x);
//     return ans;
// }

void get_object_orientation(geometry_msgs::Pose &pose, int ori_type, int x, int y, const cv::Mat &bin_image, const pcl::PointCloud<pcl::PointXYZRGB> &pcl, tf::StampedTransform transform){
    switch(ori_type){
        case objrec::DEFAULT:
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            break;
        case objrec::SURFACE: // get orientation from the surface around the object (for example, a wall where a button is placed)
            // find 3 anchor points
            // TODO: handle edge cases (one of the sides is missing)
            int x_right=x, x_left=x, y_top=y;
            while(bin_image.at<char>(y, x_right))
                x_right++;
            while(bin_image.at<char>(y, x_left))
                x_left--;
            while(bin_image.at<char>(y_top, x))
                y_top--;

            ROS_INFO_STREAM("width: " << pcl.width << ", left: " << x_left << ", right: " << x_right);

            // cv::imshow("hello", bin_image);
            // cv::waitKey(0);
            
            // find points in pcl and transform
            pcl::PointXYZRGB right_pcl = pcl[(y * pcl.width) + x_right];
            pcl::PointXYZRGB left_pcl = pcl[(y * pcl.width) + x_left];
            pcl::PointXYZRGB top_pcl = pcl[(y_top * pcl.width) + x];

            // find angles
            double m = (right_pcl.x - left_pcl.x) / (right_pcl.y - left_pcl.y); // slope
            double op_angle = atan(-1/m);
            double yaw = (1.5*M_PI) - op_angle;

            ROS_INFO_STREAM("m: " << m << ", yaw: " << yaw);

            // transform yaw
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            tf::Quaternion tras_q = transform * q;

            // get transformed yaw
            double t_roll, t_pitch, t_yaw;
            tf::Matrix3x3 mat_yaw(tras_q);
            mat_yaw.getRPY(t_roll, t_pitch, t_yaw);

            // set orientation
            tf::Quaternion q_final;
            q_final.setRPY(0.0, 0.0, t_yaw); 
            
            pose.orientation.x = q_final.x();
            pose.orientation.y = q_final.y();
            pose.orientation.z = q_final.z();
            pose.orientation.w = q_final.w();
            ROS_INFO_STREAM("x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", z: " << pose.orientation.z << ", w: " << pose.orientation.w);
            break;
    };
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
    ros::Duration d(2.0);
    
    // no message filter here, assuming robot dosn't move!
    boost::shared_ptr<const sensor_msgs::Image> rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic, d);
    boost::shared_ptr<const sensor_msgs::PointCloud2> pcl_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, d);
    // waiting for the second time, otherwise old messages are returned...
    // TODO: find out why and fix this!
    rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic, d);
    pcl_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, d);
    if(!rgb_msg || !pcl_msg){
        fail();
        return;
    }

    // find object in rgb image
    cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg)->image;

    // process image
    cv::GaussianBlur(rgb_img, rgb_img, cv::Size(5, 5), 1.0); // blur
    cv::Mat hsv_img, bin_img;
    cv::cvtColor(rgb_img, hsv_img, CV_RGB2HSV); // move to hsv
    int lower_arr[] = {rec.h_low, rec.s_low, rec.v_low};
    std::vector<int> lower(lower_arr, lower_arr+3);
    int upper_arr[] = {rec.h_high, rec.s_high, rec.v_high};
    std::vector<int> upper(upper_arr, upper_arr+3);
    cv::inRange(hsv_img, lower, upper, bin_img); // filter just reds
    cv::erode(bin_img, bin_img, cv::Mat::ones(5, 5, bin_img.type())); // clean noise
    cv::dilate(bin_img, bin_img, cv::Mat::ones(10, 10, bin_img.type())); // connect objects

    // find all 'blobs'
    cv::SimpleBlobDetector::Params bd_params;
    bd_params.filterByColor = true;
    bd_params.blobColor = 255;
    bd_params.filterByArea = true;
    bd_params.minArea = 100;
    bd_params.filterByCircularity = false;
    bd_params.filterByConvexity = false;
    bd_params.filterByInertia = false;
    cv::SimpleBlobDetector detector(bd_params);
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(bin_img, keypoints);

    // // for debug
    // cv::drawKeypoints(bin_img, keypoints, bin_img, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow("sample", bin_img);
    // cv::waitKey(0);

    // no objects found
    if(keypoints.size() == 0){
        fail();
        return;
    }

    // find biggest 'blob'
    double blob_size = 0;
    int centroid_x, centroid_y;
    for(int i=0; i<keypoints.size(); i++){
        if(keypoints[i].size > blob_size){
            blob_size = keypoints[i].size;
            centroid_x = (int)keypoints[i].pt.x;
            centroid_y = (int)keypoints[i].pt.y;
        }
    }

    // get location from keypoint
    pcl::PointCloud<pcl::PointXYZRGB> pcl_conv;
    pcl::fromROSMsg(*pcl_msg, pcl_conv);
    pcl::PointXYZRGB point = pcl_conv[(centroid_y * pcl_conv.width) + centroid_x];
    
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
    
    moveit_msgs::CollisionObject col_obj;
    col_obj.header.frame_id = "map";
    col_obj.id = name;

    // object pose
    geometry_msgs::Pose pose;
    get_object_orientation(pose, rec.ori_type, centroid_x, centroid_y, bin_img, pcl_conv, transform);
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