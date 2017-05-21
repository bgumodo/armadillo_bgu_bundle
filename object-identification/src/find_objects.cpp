#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <robotican_common/FindObjectDynParamConfig.h>
#include <dynamic_reconfigure/server.h>

#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <robotican_common/switch_topic.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

using namespace cv;

/* YOLO Detector Code */
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "opencv2/highgui/highgui_c.h"
#include <detector.h>
#define OBJ_POS_PATH  "/opt/object_positions/"

bool debug_vision=false;

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level);
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
int ifThatColourIsThere(Mat input, labeld_squares* detections, int detect_count);  

tf::TransformListener *listener_ptr;
int object_id;

// ros::Time detect_t;
std::string depth_topic1,depth_topic2,depth_topic;
bool have_object=false;

ros::Publisher object_pub;
image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;
ros::Publisher pose_pub;
ros::Publisher cmd_vel_pub_;

// set-up for the red colour
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;

enum head_locations_e {
	HEAD_FRONT,
	HEAD_LEFT,
	HEAD_RIGHT
};

PointHeadClient* point_head_client;

head_locations_e curr_location = HEAD_FRONT;

/* Sending Position Command... 
We are now ready to send "position" commands using the Action interface to the Head Trajectory controller
Points the high-def camera frame at a point in a given frame. */	
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
	
	// Send the goal
    // point_head_client->sendGoal(goal);

    // Wait for it to get there (abort after 2 secs to prevent getting stuck)
    // point_head_client->waitForResult(ros::Duration(2));
	
}

float linearMotion() {
	return 0.0;
	// return (float)(rand()%100 + 50)/1000;
}

float angularMotionOnly() {
	return 0.0;
	return (rand()%100 + 10)*(((float)22/7)/180); 	 	
}	

void moveBody(const sensor_msgs::PointCloud2ConstPtr& input) {   
    geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
    base_cmd.angular.z = angularMotionOnly(); 
    base_cmd.linear.x = linearMotion();
    cmd_vel_pub_.publish(base_cmd);
}

void moveHead(const sensor_msgs::PointCloud2ConstPtr& input) {	
   	switch(curr_location) {
   		case HEAD_FRONT:
	   		// lookAt(0, 0, 0);
	   		ROS_INFO("lookAt(2.0, 1.0, 1.2)");
	   		lookAt(2.0, 1.0, 1.2);
   			break;
		/*
   		case HEAD_LEFT:
	   		//lookAt(2.0, 1.0, 1.2);
   			break;
   		case HEAD_RIGHT:
	   		//lookAt(2.0, -1.0, 1.2);
   			break;
		*/
   	}
}

// (Order: front --> left --> right)	
void rotateNext(const sensor_msgs::PointCloud2ConstPtr& input) {
	moveBody(input);
	if (curr_location == HEAD_RIGHT) {
		curr_location = HEAD_FRONT;
		// moveHead(input);
		
	} 
	/*
	else {
		curr_location = (head_locations_e)(curr_location + 1);
		moveHead(input);
	}
	*/
}
	
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) 
{	
	// pcl - point clound library with lot of algorithms
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    
    // Convert ros point cloud msg to pcl point cloud 
    pcl::fromROSMsg (*input, cloud); 
    
    // Create projection image from p.c.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {
        ROS_WARN("empty cloud");
        return;
    }
    
	// Creating new ros sensor msg - picture is relative to depth camera tf
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;
	
	
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    Mat result=cv_ptr->image;

    Point3d obj;
    // Find object
    have_object = find_object(result, cloudp, &obj, input->header.frame_id);
	
	if (!have_object) {
    	rotateNext(input);    	
    }

    waitKey(1);
    if (have_object) {
		// Publish geometry msg containing object's location
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id=input->header.frame_id;
        target_pose.header.stamp=ros::Time::now();
        target_pose.pose.position.x =obj.x;
        target_pose.pose.position.y = obj.y;
        target_pose.pose.position.z = obj.z;
        target_pose.pose.orientation.w=1;
        pose_pub.publish(target_pose);
	}
}

void obj_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
	// Not found -> rotate to next position.
    if (!have_object) {
    	// Rotate to next position.
    	// ROS_INFO("\nInside cloud_cb(), NO object found --> calling rotate_to_next_position()");
    	// rotateNext(input);    	
    }
	//get object location msg
    try {
        geometry_msgs::PoseStamped base_object_pose;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

		//simulate alvar msgs, to get tf
        ar_track_alvar_msgs::AlvarMarkers msg;
        msg.header.stamp=base_object_pose.header.stamp;
        msg.header.frame_id="base_footprint";

        ar_track_alvar_msgs::AlvarMarker m;
        m.id=object_id;
        m.header.stamp=base_object_pose.header.stamp;
        m.header.frame_id="base_footprint";
      	m.pose=base_object_pose;
        msg.markers.push_back(m);

        m.pose.pose.position.z-=0.1;
        m.id=2;
        msg.markers.push_back(m);

        object_pub.publish(msg);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

// Gets an images and detects objects present in it using DNN. 
bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp, Point3d *pr, std::string frame) 
{	
    Mat hsv,filtered,bw,mask;
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=  frame;

    cvtColor(input, hsv, CV_BGR2HSV);

    if (inv_H) {
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else {
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
    }
    hsv.copyTo(filtered, mask);
    cvtColor(filtered, filtered, CV_HSV2BGR);

    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());

    mask.copyTo(bw);
    if ( gaussian_ksize > 0 ) 
    {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }

    if ( morph_size > 0 ) 
    {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }

    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double largest_area=0;
    int largest_contour_index=0;
    for(int i = 0; i< contours.size(); i++) 
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0 > largest_area) 
        {
            largest_area = area0;
            largest_contour_index = i;
        }
    }
    
    bool ok = false;
    
    IplImage* image_iplimage = new IplImage(input);   
    imwrite("./image-1.jpg", input);
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

		// const char* file_name = OBJ_POS_PATH "data/dog.jpg";		 
		float thresh = 0.1f;
		labeld_squares* detections;
		
		int detect_count = predict_scuares(OBJ_POS_PATH, data_cfg, cfg_file, weight_file, image_iplimage, thresh, &detections);			
		int found_idx = ifThatColourIsThere(input, detections, detect_count);  
		if(found_idx > 0) {
			have_object = true;
		}
				
		int pcl_index;			
		if (have_object) 
		{
			float x_1, y_1;
			y_1 = 0.5 * (detections[found_idx].bot + detections[found_idx].top);
			x_1 = 0.5 * (detections[found_idx].right + detections[found_idx].left);		
			
			if (detections) {
				free(detections);
			}		
		
			pcl_index = ((int)(y_1)*input.cols) + (int)(x_1);
			// pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
			circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

			pr->x = cloudp->points[pcl_index].x;
			pr->y = cloudp->points[pcl_index].y;
			pr->z = cloudp->points[pcl_index].z;			
					
			char str[100];
			if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
			    sprintf(str,"NaN");
			    ok = false;
			}
			else {
				std::cout << "The actual coordinates are (X, Y, Z) : (" <<pr->x<<", "<<pr->y<<", "<<pr->z<<")\n";
			    sprintf(str,"[%.3f,%.3f,%.3f] A = %lf", pr->x, pr->y, pr->z, largest_area);
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

int ifThatColourIsThere(Mat input, labeld_squares* detections, int detect_count)  
{
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
					// std::cout << xb << " " << xg << " " << xr <<"\n";
					if ( (xb<80 && xg<80 && xr>130) || (xb<10 && xg<10 && xr>50))
						count_temp++;											
					circle( input, cv::Point(cor_x, cor_y), 2, Scalar(0,0,255), -1, 8, 0 ); 
				    imwrite("./image-2.jpg", input);
					
				}
				for (int p=0; p<40; p++) {
					int k1 = p+1;
					int k2 = 40-p;	
					int cor_y = (int) (k1*b + k2*t)/(k1+k2);
					int cor_x = (int) (k1*l + k2*r)/(k1+k2);
					int xb = input.at<cv::Vec3b>(cor_y, cor_x)[0];
					int xg = input.at<cv::Vec3b>(cor_y, cor_x)[1];
					int xr = input.at<cv::Vec3b>(cor_y, cor_x)[2];
					// std::cout << xb << " " << xg << " " << xr <<"\n";
					if ( (xb<80 && xg<80 && xr>130) || (xb<10 && xg<10 && xr>50))
						count_temp++;
					circle( input, cv::Point(cor_x, cor_y), 2, Scalar(0,0,255), -1, 8, 0 ); 
				    imwrite("./image-2.jpg", input);
				}				
				if (count < count_temp) {
					count = count_temp;
					found_idx = found_idx_temp;
				}					
			}
		}	
		// std::cout << "count = " << count << "\n";		
		if(count > 0) {
			// have_object = true;					
			return found_idx;
		}
		return -1;
}
 
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level) {
    minH = config.H_min;
    maxH = config.H_max;

    minS = config.S_min;
    maxS = config.S_max;

    minV = config.V_min;
    maxV = config.V_max;

    minA = config.A_min;
    maxA = config.A_max;

    gaussian_ksize = config.gaussian_ksize;
    gaussian_sigma = config.gaussian_sigma;

    morph_size = config.morph_size;

    inv_H = config.invert_Hue;
}


void on_trackbar( int, void* ){}

bool switch_pcl_topic(robotican_common::switch_topic::Request &req, robotican_common::switch_topic::Response &res) {

    if (req.num==1) depth_topic=depth_topic1;
    else if (req.num==2) depth_topic=depth_topic2;
    res.success=true;
return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
	
    pn.param<int>("object_id", object_id, 1);
    pn.param<std::string>("depth_topic1", depth_topic1, "/kinect2/qhd/points");
    pn.param<std::string>("depth_topic2", depth_topic2, "/kinect2/qhd/points");
    depth_topic=depth_topic1;

    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig> dynamicServer;
    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig>::CallbackType callbackFunction;

    callbackFunction = boost::bind(&dynamicParamCallback, _1, _2);
    dynamicServer.setCallback(callbackFunction);

    image_transport::ImageTransport it_(pn);

    result_image_pub = it_.advertise("result", 1);
    object_image_pub = it_.advertise("hsv_filterd", 1);
    bw_image_pub = it_.advertise("bw", 1);
    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
    ROS_INFO_STREAM(depth_topic);

    object_pub=n.advertise<ar_track_alvar_msgs::AlvarMarkers>("detected_objects", 2, true);

    pose_pub=pn.advertise<geometry_msgs::PoseStamped>("object_pose",10);

	//convert depth cam tf to base foot print tf (moveit work with base foot print tf)
    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(pn, "object_pose", 10);

    tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    tf_filter.registerCallback( boost::bind(obj_msgCallback, _1) );


	// ros::ServiceServer switch_sub = n.advertiseService("switch_pcl_topic", &switch_pcl_topic);

	ros::ServiceServer switch_sub = n.advertiseService("switch_pcl_topic", &switch_pcl_topic);
	cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate r(10);
    ROS_INFO("Ready to find objects!");
    while (ros::ok()) {
    if (pcl_sub.getTopic()!=depth_topic) {
        pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
         ROS_INFO("switching pcl topic");
    }
     ros::spinOnce();
     r.sleep();
    }

    return 0;
}

