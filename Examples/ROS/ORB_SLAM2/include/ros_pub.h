// The header file of orb_slam2 node publication
#include<iostream>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "../../../../include/System.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ROS_PUB{
public:	
	// Publisher
	ros::Publisher pub_cloud;
	ros::Publisher pub_pts_and_pose;
	ros::Publisher pub_all_kf_and_pts;
	ros::Publisher pub_cam_pose;

	//Param
	std::string map_frame_id_param_ = "map";
	std::string camera_frame_id_param_ = "camera_link";
	std::string map_file_name_param_;
	int all_pts_pub_gap = 0;
	bool pub_all_pts = false;
	int pub_count = 0;
	bool bUseViewer_ =true;

	// Function declaration
	ROS_PUB(int argc, char **argv);
	~ROS_PUB();
	void init_publisher();
	void publish(ORB_SLAM2::System &SLAM);
	bool getRosParams(ros::NodeHandle &node_handle_);
	vector<float> QuaternionDotProduct(vector<float> q1,vector<float> q2);
	geometry_msgs::Pose TransformtoWorld(geometry_msgs::Pose camera_pose);
	ros::NodeHandle nodeHandler;
};