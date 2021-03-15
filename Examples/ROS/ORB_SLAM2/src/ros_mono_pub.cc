/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <time.h>
#include <iomanip>

#include "../lib/ros_pub.h"
// #include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud.h"
// #include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>
// #include "../../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>




//! parameters
bool read_from_topic = true, read_from_camera = false;
// std::string name_of_node_;
// std::string map_frame_id_param_ = "map";
// std::string camera_frame_id_param_ = "camera_link";
// std::string map_file_name_param_;
// bool bUseViewer_ =true;

//Publish
// ros::Publisher pub_cloud;
// ros::Publisher pub_cloud_;
// ros::Publisher pub_cam_pose_;
bool save_to_results = false;
std::string image_topic = "/camera/image_raw";
// int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

// bool pub_all_pts = false;
// int pub_count = 0;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);
void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cam_pose, int frame_id);
bool parseParams(int argc, char **argv);
bool getRosParams(ros::NodeHandle &node_handle_);
vector<float> QuaternionDotProduct(vector<float> q1,vector<float> q2);
geometry_msgs::Pose TransformtoWorld(geometry_msgs::Pose camera_pose);

class ImageGrabber{
public:
	ImageGrabber(ORB_SLAM2::System &_SLAM, ros::Publisher &_pub_pts_and_pose,
		ros::Publisher &_pub_all_kf_and_pts, ros::Publisher &_pub_cam_pose) :
		SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose),
		pub_all_kf_and_pts(_pub_all_kf_and_pts), pub_cam_pose(_pub_cam_pose), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System &SLAM;
	ros::Publisher &pub_pts_and_pose;
	ros::Publisher &pub_all_kf_and_pts;
	ros::Publisher &pub_cam_pose;
	int frame_id;
};

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	cout << "Monopub" << endl;
	int n_images = vstrImageFilenames.size();
	ros::NodeHandle nodeHandler;
	// getRosParams(nodeHandler);
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, bUseViewer_);

	init_publisher(nodeHandler);
	// pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud>("/ros_cloud", 1000);
	// pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/ros_cloud_rotated", 1000);
	// pub_cloud_ = nodeHandler.advertise<sensor_msgs::PointCloud2>("/ros_cloud", 1000);
	// ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	// ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	// ros::Publisher pub_cam_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("/cam_pose_rotated", 1000);
	// pub_cam_pose_ = nodeHandler.advertise<geometry_msgs::PoseStamped>("/cam_pose", 1000);

	if (read_from_topic) {
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cam_pose);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
		ros::Rate loop_rate(5);
		cv::Mat im;
		double tframe = 0;
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		for (int frame_id = 0; read_from_camera || frame_id < n_images; ++frame_id){
			if (read_from_camera) {
				cap_obj.read(im);
#ifdef COMPILEDWITHC11
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
				// std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
				tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
				//printf("fps: %f\n", 1.0 / tframe);
			}
			else {
				// Read image from file
				im = cv::imread(vstrImageFilenames[frame_id], CV_LOAD_IMAGE_UNCHANGED);
				tframe = vTimestamps[frame_id];
			}
			if (im.empty()){
				cerr << endl << "Failed to load image at: " << vstrImageFilenames[frame_id] << endl;
				return 1;
			}
			// Pass the image to the SLAM system
			cv::Mat curr_pose = SLAM.TrackMonocular(im, tframe);

			publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cam_pose, frame_id);

			//cv::imshow("Press escape to exit", im);
			//if (cv::waitKey(1) == 27) {
			//	break;
			//}
			ros::spinOnce();
			loop_rate.sleep();
			if (!ros::ok()){ break; }
		}
	}
	//ros::spin();
	// if(save_to_results){
	// 	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	// 	SLAM.getMap()->Save("results//map_pts_out.obj");
	// 	SLAM.getMap()->SaveWithTimestamps("results//map_pts_and_keyframes.txt");
	// 	// Save camera trajectory
	// 	SLAM.SaveKeyFrameTrajectoryTUM("results//key_frame_trajectory.txt");

	// 	cout << "Press 'q' in the Frame Window to quit!" << endl;
	// 	while (cv::waitKey(0) != 'q') { }
	// }
	// Stop all threads
	SLAM.Shutdown();
	//geometry_msgs::PoseArray pt_array;
	//pt_array.header.seq = 0;
	//pub_pts_and_pose.publish(pt_array);
	ros::shutdown();
	return 0;
}

// void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
// 			 ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cam_pose, int frame_id) {
// 	if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
// 		pub_all_pts = true;
// 		pub_count = 0;
// 	}

// 	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {
// 		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
// 		geometry_msgs::PoseArray kf_pt_array;
// 		vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
// 		//! placeholder for number of keyframes
// 		kf_pt_array.poses.push_back(geometry_msgs::Pose());
// 		sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
// 		unsigned int n_kf = 0;
// 		for (auto key_frame : key_frames) {
// 			// pKF->SetPose(pKF->GetPose()*Two);

// 			if (key_frame->isBad())
// 				continue;

// 			cv::Mat R = key_frame->GetRotation().t();
// 			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
// 			cv::Mat twc = key_frame->GetCameraCenter();
// 			geometry_msgs::Pose kf_pose;

// 			kf_pose.position.x = twc.at<float>(0);
// 			kf_pose.position.y = twc.at<float>(1);
// 			kf_pose.position.z = twc.at<float>(2);
// 			kf_pose.orientation.x = q[0];
// 			kf_pose.orientation.y = q[1];
// 			kf_pose.orientation.z = q[2];
// 			kf_pose.orientation.w = q[3];
// 			kf_pt_array.poses.push_back(kf_pose);

// 			unsigned int n_pts_id = kf_pt_array.poses.size();
// 			//! placeholder for number of points
// 			kf_pt_array.poses.push_back(geometry_msgs::Pose());
// 			std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
// 			unsigned int n_pts = 0;
// 			for (auto map_pt : map_points) {
// 				if (!map_pt || map_pt->isBad()) {
// 					// printf("Point %d is bad\n", pt_id);
// 					continue;
// 				}
// 				cv::Mat pt_pose = map_pt->GetWorldPos();
// 				if (pt_pose.empty()) {
// 					// printf("World position for point %d is empty\n", pt_id);
// 					continue;
// 				}
// 				geometry_msgs::Pose curr_pt;
// 				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
// 				// pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
// 				curr_pt.position.x = pt_pose.at<float>(0);
// 				curr_pt.position.y = pt_pose.at<float>(1);
// 				curr_pt.position.z = pt_pose.at<float>(2);
// 				kf_pt_array.poses.push_back(curr_pt);
// 				++n_pts;
// 			}
// 			geometry_msgs::Pose n_pts_msg;
// 			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
// 			kf_pt_array.poses[n_pts_id] = n_pts_msg;
// 			++n_kf;
// 		}
// 		geometry_msgs::Pose n_kf_msg;
// 		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
// 		kf_pt_array.poses[0] = n_kf_msg;
// 		kf_pt_array.header.frame_id = map_frame_id_param_;// "map"
// 		// kf_pt_array.header.seq = frame_id + 1;
// 		printf("Publishing data for %u keyfranmes\n", n_kf);
// 		pub_all_kf_and_pts.publish(kf_pt_array);
// 	}
// 	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
// 		++pub_count;
// 		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
// 		ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

// 		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

// 		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
// 		//while (pKF->isBad())
// 		//{
// 		//	Trw = Trw*pKF->mTcp;
// 		//	pKF = pKF->GetParent();
// 		//}

// 		vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
// 		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

// 		// Transform all keyframes so that the first keyframe is at the origin.
// 		// After a loop closure the first keyframe might not be at the origin.
// 		cv::Mat Two = vpKFs[0]->GetPoseInverse();

// 		Trw = Trw*pKF->GetPose()*Two;
// 		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
// 		cv::Mat Tcw = lit*Trw;
// 		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
// 		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

// 		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
// 		//geometry_msgs::Pose camera_pose;

// 		std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
// 		int n_map_pts = map_points.size();
// 		printf("all_map_pts: %d\n", n_map_pts);

// 		// std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
// 		// int n_map_pts = map_points.size();
// 		// printf("tracked_map_pts: %d\n", n_map_pts);
		

// 		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// 		geometry_msgs::PoseArray pt_array;
// 		//pt_array.poses.resize(n_map_pts + 1);

// 		geometry_msgs::Pose camera_pose;

// 		camera_pose.position.x = twc.at<float>(0);
// 		camera_pose.position.y = twc.at<float>(1);
// 		camera_pose.position.z = twc.at<float>(2);

// 		camera_pose.orientation.x = q[0];
// 		camera_pose.orientation.y = q[1];
// 		camera_pose.orientation.z = q[2];
// 		camera_pose.orientation.w = q[3];

// 		pt_array.poses.push_back(camera_pose);

// 		//printf("Done getting camera pose\n");
// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
// 		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

// 			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
// 				//printf("Point %d is bad\n", pt_id);
// 				continue;
// 			}
// 			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

// 			if (wp.empty()) {
// 				//printf("World position for point %d is empty\n", pt_id);
// 				continue;
// 			}
// 			geometry_msgs::Pose curr_pt;
// 			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
// 			pcl_cloud_->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
// 			// transform cloud from camera coordinate to world coordinate.
// 			pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(2), -wp.at<float>(0), -wp.at<float>(1)));			
// 			curr_pt.position.x = wp.at<float>(0);
// 			curr_pt.position.y = wp.at<float>(1);
// 			curr_pt.position.z = wp.at<float>(2);
// 			pt_array.poses.push_back(curr_pt);
// 			//printf("Done getting map point %d\n", pt_id);
// 		}

// 		sensor_msgs::PointCloud2 ros_cloud;
// 		pcl::toROSMsg(*pcl_cloud, ros_cloud);
// 		ros_cloud.header.frame_id = map_frame_id_param_; // "map"
// 		// ros_cloud.header.seq = ni;
// 		pub_cloud.publish(ros_cloud);

// 		// sensor_msgs::PointCloud2 ros_cloud_;
// 		// pcl::toROSMsg(*pcl_cloud_, ros_cloud_);
// 		// ros_cloud_.header.frame_id = map_frame_id_param_; // "map"
// 		// // ros_cloud.header.seq = ni;
// 		// pub_cloud_.publish(ros_cloud_);

// 		// sensor_msgs::PointCloud ros_cloud1;
// 		// sensor_msgs::convertPointCloud2ToPointCloud(ros_cloud,ros_cloud1);
// 		// pub_cloud.publish(ros_cloud1);

// 		pt_array.header.frame_id = map_frame_id_param_;// "map"
// 		pt_array.header.seq = frame_id + 1;
// 		pub_pts_and_pose.publish(pt_array);
// 		// pub_kf.publish(camera_pose);
// 	}
// 	// Publish current camera pose
// 	geometry_msgs::Pose camera_pose;
// 	if (!SLAM.getTracker()->mCurrentFrame.mTcw.empty())
// 	{
// 		cv::Mat Tcw = SLAM.getTracker()->mCurrentFrame.mTcw; 
// 		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
// 		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

// 		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

// 		camera_pose.orientation.x = q[0];
// 		camera_pose.orientation.y = q[1];
// 		camera_pose.orientation.z = q[2];
// 		camera_pose.orientation.w = q[3];		
				
// 		camera_pose.position.x = twc.at<float>(0);
// 		camera_pose.position.y = twc.at<float>(1);
// 		camera_pose.position.z = twc.at<float>(2);
// 		// transform camera_pose from camera coordinate to world coordinate.
// 		// camera_pose.position.x = twc.at<float>(2);
// 		// camera_pose.position.y = -twc.at<float>(0);
// 		// camera_pose.position.z = -twc.at<float>(1);
// 		// cout << "before rotation" << camera_pose.position.x << " " << camera_pose.position.y << " " << camera_pose.position.z << " " << endl;

// 		// geometry_msgs::PoseStamped camera_posestamped_;
// 		// camera_posestamped_.pose = camera_pose;
// 		// camera_posestamped_.header.frame_id = map_frame_id_param_; //"map""
// 		// camera_posestamped_.header.stamp = ros::Time::now();
// 		// pub_cam_pose_.publish(camera_posestamped_);
// /*
// 		Eigen::Quaterniond quat(q[3], q[0], q[1], q[2]);
// 		Eigen::Matrix3d rx = quat.toRotationMatrix();
// 		Eigen::Vector3d ea = rx.eulerAngles(2,1,0);
// 		cout << "EulerAngel  x: " << ea[0]/3.14*180 << ", y: " << ea[1]/3.14*180 << ", z: "<< ea[2]/3.14*180 << endl;


// 		// ::Eigen::Matrix3d R;
// 		// R = ::Eigen::AngleAxisd(ea[0], ::Eigen::Vector3d::UnitZ())
// 		// 	* ::Eigen::AngleAxisd(ea[1], ::Eigen::Vector3d::UnitY())
// 		// 	* ::Eigen::AngleAxisd(ea[2], ::Eigen::Vector3d::UnitX());
// 		::Eigen::Quaterniond quaternion = Eigen::AngleAxisd(ea[0], ::Eigen::Vector3d::UnitZ()) *
// 											Eigen::AngleAxisd(ea[2], ::Eigen::Vector3d::UnitY()) *
// 											Eigen::AngleAxisd(ea[1], ::Eigen::Vector3d::UnitX());

// 		vector<float> quat_world2cam{  0,0,0,1  };
// 		vector<float> quat_rotated = QuaternionDotProduct(vector<float>{-q[0],-q[1],-q[2],q[3]},quat_world2cam);

// 		camera_pose.orientation.x = quaternion.x(); //quat_rotated[0];
// 		camera_pose.orientation.y = quaternion.y(); //quat_rotated[1];
// 		camera_pose.orientation.z = quaternion.z(); //quat_rotated[2];
// 		camera_pose.orientation.w = quaternion.w(); //quat_rotated[3];
// */		
// 		geometry_msgs::Pose pose = TransformtoWorld(camera_pose);
// 		// cout << "after rotation" << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << endl;
// 		geometry_msgs::PoseStamped camera_posestamped;
// 		camera_posestamped.pose = pose;//camera_pose;
// 		camera_posestamped.header.frame_id = map_frame_id_param_; //"map""
// 		camera_posestamped.header.stamp = ros::Time::now();
// 		pub_cam_pose.publish(camera_posestamped);
// 		//Modify Map points nearby camear by the way
// 		// FilterNearbyPoint(SLAM.getMap(),std::vector<float>{twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)});
// 	}
// 	// mainProcess(SLAM,pub_cur_view_cloud,camera_pose);
// }


inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg){
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cam_pose, frame_id);
	++frame_id;
}

bool parseParams(int argc, char **argv) {
	if (argc < 4){
		cerr << endl << "Usage: rosrun ORB_SLAM2 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id/-1 <image_topic>" << endl;
		return 1;
	}
	if (isInteger(std::string(argv[3]))) {
		int camera_id = atoi(argv[3]);
		if (camera_id >= 0){
			read_from_topic = false;
			read_from_camera = true;
			printf("Reading images from camera with id %d\n", camera_id);
			cap_obj.open(camera_id);
			if (!(cap_obj.isOpened())) {
				printf("Camera stream could not be initialized successfully\n");
				ros::shutdown();
				return 0;
			}
			int img_height = cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
			int img_width = cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);
			printf("Images are of size: %d x %d\n", img_width, img_height);
		}
		else {
			read_from_topic = true;
			if (argc > 4){
				image_topic = std::string(argv[4]);
			}
			printf("Reading images from topic %s\n", image_topic.c_str());
		}
	}
	else {
		LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
		read_from_topic = false;
		read_from_camera = false;
	}
	if (argc >= 5) {
		all_pts_pub_gap = atoi(argv[4]);
	}
	printf("all_pts_pub_gap: %d\n", all_pts_pub_gap);
	return 1;
}


