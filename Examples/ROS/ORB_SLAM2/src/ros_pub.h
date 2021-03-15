#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "../../../../include/System.h"
#include "../"

// Publisher
ros::Publisher pub_cloud;
ros::Publisher pub_pts_and_pose;
ros::Publisher pub_all_kf_and_pts;
ros::Publisher pub_cam_pose;

void init_publisher(ros::NodeHandle nodeHandler){
        pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/ros_cloud", 1000);
        pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
        pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
        pub_cam_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("/cam_pose", 1000);
}

void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cam_pose, int frame_id);