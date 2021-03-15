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
// #include "sensor_msgs/PointCloud.h"
// #include "sensor_msgs/PointCloud2.h"
// #include <sensor_msgs/point_cloud_conversion.h>
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseArray.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>
// #include "../../../../include/System.h"

#include "MapPoint.h"
// #include <opencv2/highgui/highgui_c.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <Converter.h>


//! parameters
bool read_from_topic = true, read_from_camera = false;
bool save_to_results = false;
std::string image_topic = "/camera/image_raw";

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);
bool parseParams(int argc, char **argv);


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

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, bUseViewer_);

	init_publisher(nodeHandler);

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

			// publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cam_pose, frame_id);
			publish(SLAM, frame_id);

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
	// publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cam_pose, frame_id);
	publish(SLAM, frame_id);
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


