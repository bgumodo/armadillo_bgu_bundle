
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <object_identification/find_results.h>
#include <object_identification/find_obj.h>
#include <boost/atomic.hpp>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
using namespace cv;

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#endif
#include "detector.h"
#define OBJ_POS_PATH  "/home/juvy/catkin_ws/src/armadillo_bgu_bundle/object_identification/src/"

bool debug_vision = false;

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
int ifThatColourIsThere(Mat input, labeld_squares* detections, int detect_count);  

ros::Publisher *found_pub;

tf::TransformListener *listener_ptr;

int object_id;
int view_field_angle;

std::string depth_topic1, depth_topic2, depth_topic;
bool have_object = false;

ros::Publisher object_pub;
image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;
ros::Publisher cmd_vel_pub_;
ros::Publisher *pose_pub;

int minH, maxH;
int minS, maxS;
int minV, maxV;
std::string obj_name;
boost::atomic<bool> working;

int minA = 200,	maxA = 50000;
int gaussian_ksize = 0;
int gaussian_sigma = 0;
int morph_size = 0;
int inv_H = 1;

enum head_locations_e {
	HEAD_FRONT,
	HEAD_LEFT,
	HEAD_RIGHT
};

PointHeadClient* point_head_client_;

head_locations_e curr_location = HEAD_FRONT;


/** 
  * Sending Position Command... (Error)
  * We are now ready to send "position" commands using the Action interface to the Head Trajectory controller
  * Points the high-def camera frame at a point in a given frame. 
  */	
void lookAt( double x, double y, double z) 
{
    // The goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;
    
    // The target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.point.x = x; 
    point.point.y = y; 
    point.point.z = z;
    goal.target = point;    
   
    // Pointing to high-def cam frame
    goal.pointing_frame = "kinect2_depth_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    // Take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    // And go no faster than 1 rad/s
    goal.max_velocity = 1.0;	
}

float linearMotion() {
	return 0.0;
	// return (float)(rand()%100 + 50)/1000;
}

float angularMotionOnly() {
	return 0.0;
	// return (rand()%100 + 10)*(((float)22/7)/180); 	 	
}	

void moveBody(const sensor_msgs::PointCloud2ConstPtr& input) {   
    geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
    base_cmd.angular.z = angularMotionOnly(); 
    base_cmd.linear.x = linearMotion();    
}

// Not able to move its head, currently working on it.
void moveHead(const sensor_msgs::PointCloud2ConstPtr& input) {	
   	switch(curr_location) {
   		case HEAD_FRONT:
	   		lookAt(2.0, 1.0, 1.2);
   			break;		
   	}
}

void rotateNext(const sensor_msgs::PointCloud2ConstPtr& input) {
	moveBody(input);
	if (curr_location == HEAD_RIGHT) {
		curr_location = HEAD_FRONT;
	} 
}
	
/* As long object was not found and we havn't completed a cicle. */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) 
{
	ROS_INFO("got a pointcloud...");
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::fromROSMsg (*input, cloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

	if (cloudp->empty()) {
	    ROS_WARN("empty cloud");
	    return;
	}

	sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
	pcl::toROSMsg (*input, *image_msg);
	image_msg->header.stamp = input->header.stamp;
	image_msg->header.frame_id = input->header.frame_id;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	Mat result=cv_ptr->image;

	Point3d obj;

	// this function returns a bool to the callback function as per if it finds the specified object  
	have_object = find_object(result, cloudp, &obj, input->header.frame_id);
	
    if (!have_object) {
    	// rotateNext(input);    	
    	// the above call purpously commented - don't remove the dependent functions. 
    }

    waitKey(1);
    
    object_identification::find_results res;
    
    if (have_object) {
		res.success = true;
        res.target_pose.position.x = obj.x;
        res.target_pose.position.y = obj.y;
        res.target_pose.position.z = obj.z;
    }
    else{
    	res.success = false;
    }
    
    pose_pub->publish(res);
    working = false;
}

/** 
  * This function gets an image (input) and detects all objects it finds in it, using a deep neural network (DNN). 
  * You will see that it does some extra bit of work like identifying areas and looking for a particulr colour even 
  * before DNN comes into the play. The reason is that the identification via DNN is a slower process, so, we cannot 
  * send all image that Kinect sees. Hence only selective frames are to be sent. */
bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp, Point3d *pr, std::string frame) 
{	
    Mat hsv,filtered,bw,mask;
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=  frame;

    cvtColor(input, hsv, CV_BGR2HSV);

    if (inv_H) 
    {
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else 
    {
      	inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
    }
    
    hsv.copyTo(filtered, mask);
    cvtColor(filtered, filtered, CV_HSV2BGR);

    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());

    mask.copyTo(bw);
    if ( gaussian_ksize > 0 ) {
        if (gaussian_ksize % 2 == 0) 
        	gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }

    if ( morph_size > 0 ) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }

    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());

    vector < vector < Point > > contours;
    vector < Vec4i > hierarchy;

    findContours(bw, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double largest_area = 0;
    int largest_contour_index = 0;
    for (int i = 0; i< contours.size(); i++) 
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0 > largest_area) {
            largest_area = area0;
            largest_contour_index = i;
        }
    }
    
    bool ok = false;
    
    IplImage* image_iplimage = new IplImage(input);   
    if ((largest_area > minA) && (largest_area < maxA)) 
    {
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00, (mu.m01/mu.m00) );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );       

	    
		// Code asking YOLO to detect the objects in images taken from kinect
		const char* data_cfg = OBJ_POS_PATH "cfg/coco.data";
		const char* cfg_file = OBJ_POS_PATH "cfg/yolo.cfg";
		const char* weight_file = OBJ_POS_PATH "yolo.weights";

		// this threshold value defines the confidence we want from the DNN for a particular object
		float thresh = 0.1f; 
		labeld_squares* detections;
		
		int detect_count = predict_scuares(OBJ_POS_PATH, data_cfg, cfg_file, weight_file, image_iplimage, thresh, &detections);			
		int found_idx = ifThatColourIsThere(input, detections, detect_count);  
		if(found_idx > 0) {
			have_object = true;
		}
				
		int pcl_index;			
		if (have_object) {
			float x_1, y_1;
			y_1 = 0.5 * (detections[found_idx].bot + detections[found_idx].top);
			x_1 = 0.5 * (detections[found_idx].right + detections[found_idx].left);		
			
			if (detections) {
				free(detections);
			}		
		
			pcl_index = ((int)(y_1)*input.cols) + (int)(x_1);
			circle( input, mc, 8, Scalar(0, 255, 0), -1, 8, 0 );

			pr->x = cloudp->points[pcl_index].x;
			pr->y = cloudp->points[pcl_index].y;
			pr->z = cloudp->points[pcl_index].z;			
					
			char str[100];
			if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
			    sprintf(str,"NaN");
			    ok = false;
			}
			else {
				ok = true;
			}
			putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(255,255,255), 1, 8);				
		}			
    }
    cvWaitKey(1);
    sleep(5);
    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());
	
	delete image_iplimage;
	input.release();
	
    return ok;
}

/**
  * Once DNN identifies the particular object, it looks for its colour. 
  * Returns TRUE if it finds the actual coloured object. */
int ifThatColourIsThere(Mat input, labeld_squares* detections, int detect_count) {
	int found_idx = -1;	
	int count = 0;	
	for (int i = 0; i < detect_count; ++i) {	
		if ((strcmp("apple", detections[i].label) == 0 || strcmp("cup", detections[i].label) == 0)  && detections[i].prob > 0.2) 
		{			 
			std::cout << "\nFound "<<detections[i].label << " with prob "<<detections[i].prob * 100 << "\n";			
			std::cout << detections[i].top  << " " << detections[i].bot << " " << detections[i].left << " " <<  detections[i].right << " " <<
						(int)(detections[i].top + detections[i].bot)/2 << " " << (int)(detections[i].left + detections[i].right)/2 << "\n";
	
			int count_temp = 0;
			int found_idx_temp = -1;
			int t = detections[i].top;
			int b = detections[i].bot;
			int l = detections[i].left;
			int r = detections[i].right;
			rectangle(input, cv::Point(l,t), cv::Point(r,b), Scalar(0,0,255));
			
			for (int p=0; p<40; p++) {
				int k1 = p+1;
				int k2 = 40-p;	
				int cor_y = (int) (k1*b + k2*t)/(k1+k2);
				int cor_x = (int) (k1*r + k2*l)/(k1+k2);
				int xb = input.at<cv::Vec3b>(cor_y, cor_x)[0];
				int xg = input.at<cv::Vec3b>(cor_y, cor_x)[1];
				int xr = input.at<cv::Vec3b>(cor_y, cor_x)[2];
				
				/* 
				 * Hard coded for red colour, for some other colour we need to write HSV2RGB converter to use upper and lower limits accordingly.
				 **/ 
				if ( (xb<80 && xg<80 && xr>130) || (xb<10 && xg<10 && xr>50))
					count_temp++;											
				
				circle( input, cv::Point(cor_x, cor_y), 2, Scalar(0,0,255), -1, 8, 0 ); 		    					
			}
			for (int p=0; p<40; p++) {
				int k1 = p+1;
				int k2 = 40-p;	
				int cor_y = (int) (k1*b + k2*t)/(k1+k2);
				int cor_x = (int) (k1*l + k2*r)/(k1+k2);
				int xb = input.at<cv::Vec3b>(cor_y, cor_x)[0];
				int xg = input.at<cv::Vec3b>(cor_y, cor_x)[1];
				int xr = input.at<cv::Vec3b>(cor_y, cor_x)[2];
				
				/* 
				 * Hard coded for red colour, for some other colour we need to write HSV2RGB converter to use upper and lower limits accordingly.
				 **/ 
				if ( (xb<80 && xg<80 && xr>130) || (xb<10 && xg<10 && xr>50))
					count_temp++;
				
				circle( input, cv::Point(cor_x, cor_y), 2, Scalar(0,0,255), -1, 8, 0 ); 
			}				
			
			if (count < count_temp) {
				count = count_temp;
				found_idx = found_idx_temp;
			}			
		}
	}	

	if(count > 0) {
		return found_idx;
	}
	return -1;
}

void RobotHead() {
    point_head_client_ = new PointHeadClient("/pan_tilt_trajectory_controller/point_head_action", true);
    while(!point_head_client_->waitForServer(ros::Duration(5.0))) {
    	ROS_INFO("Waiting for the point_head_action server to come up");
    }
}

ros::NodeHandle *n;

// TODO: change from global vars to boost::bind

bool trigger_search(object_identification::find_obj::Request &req, object_identification::find_obj::Response &res) 
{
	if(!working) {
		working = true;
		minH	=	req.h - req.tolerance,	maxH	=	req.h + req.tolerance;
		minS	=	req.s - req.tolerance, 	maxS	=	req.s + req.tolerance;
		minV	=	req.v - req.tolerance, 	maxV	=	req.v + req.tolerance;
		
		/* hard coded in respect of a 'CUP' */
		minA	=	100,					maxA	=	50000; 	
		
		obj_name = req.name;
		ROS_INFO("Starting search...");
		ros::Subscriber pcl_sub = n->subscribe("/kinect2/qhd/points", 10, cloud_cb);
		ros::spin();
	}
	return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_objects_node");
    n = new ros::NodeHandle();
    working = false;
    // RobotHead();
    
    ros::Publisher pub = n->advertise<object_identification::find_results>("find_results", 10);
	ros::ServiceServer srv = n->advertiseService("find_object", trigger_search);

	pose_pub = &pub;
  
	ros::spin();	
    return 0;
}

