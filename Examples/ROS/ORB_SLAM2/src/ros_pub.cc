#include"../include/ros_pub.h"

ROS_PUB::ROS_PUB(int argc, char **argv){
        ros::init(argc,argv,"rospub");
	ros::NodeHandle nodeHandler;
        init_publisher(nodeHandler);
        cout << "Initialize node rospub inside class Done.";
}

ROS_PUB::~ROS_PUB(){
        ros::shutdown();
        cout << "Node rospub has been shutdown!";
}

void ROS_PUB::init_publisher(ros::NodeHandle nodeHandler){	
        pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/ros_cloud", 1000);
        pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
        pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
        pub_cam_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("/cam_pose", 1000);
}

void ROS_PUB::publish(ORB_SLAM2::System &SLAM){	
	if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
		pub_all_pts = true;
		pub_count = 0;
	}

	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {
		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (key_frame->isBad())
				continue;

			cv::Mat R = key_frame->GetRotation().t();
			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
			cv::Mat twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			unsigned int n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					// printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = map_pt->GetWorldPos();
				if (pt_pose.empty()) {
					// printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				// pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			geometry_msgs::Pose n_pts_msg;
			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
			kf_pt_array.poses[n_pts_id] = n_pts_msg;
			++n_kf;
		}
		geometry_msgs::Pose n_kf_msg;
		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
		kf_pt_array.poses[0] = n_kf_msg;
		kf_pt_array.header.frame_id = map_frame_id_param_;// "map"
		// kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyfranmes\n", n_kf);
		pub_all_kf_and_pts.publish(kf_pt_array);
	}
	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
		++pub_count;
		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
		ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		//while (pKF->isBad())
		//{
		//	Trw = Trw*pKF->mTcp;
		//	pKF = pKF->GetParent();
		//}

		vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		Trw = Trw*pKF->GetPose()*Two;
		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
		cv::Mat Tcw = lit*Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
		//geometry_msgs::Pose camera_pose;

		std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
		int n_map_pts = map_points.size();
		printf("all_map_pts: %d\n", n_map_pts);

		// std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		// int n_map_pts = map_points.size();
		// printf("tracked_map_pts: %d\n", n_map_pts);		

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		//printf("Done getting camera pose\n");
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

			if (wp.empty()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;
			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
			// transform cloud from camera coordinate to world coordinate.
			pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(2), -wp.at<float>(0), -wp.at<float>(1)));			
			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		ros_cloud.header.frame_id = map_frame_id_param_; // "map"
		// ros_cloud.header.seq = ni;
		pub_cloud.publish(ros_cloud);

		pt_array.header.frame_id = map_frame_id_param_;// "map"
		// pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose.publish(pt_array);
		// pub_kf.publish(camera_pose);
	}
	// Publish current camera pose
	geometry_msgs::Pose camera_pose;
	if (!SLAM.getTracker()->mCurrentFrame.mTcw.empty())
	{
		cv::Mat Tcw = SLAM.getTracker()->mCurrentFrame.mTcw; 
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];		
				
		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);
	
		geometry_msgs::Pose pose = TransformtoWorld(camera_pose);
		// cout << "after rotation" << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << endl;
		geometry_msgs::PoseStamped camera_posestamped;
		camera_posestamped.pose = pose;//camera_pose;
		camera_posestamped.header.frame_id = map_frame_id_param_; //"map""
		camera_posestamped.header.stamp = ros::Time::now();
		pub_cam_pose.publish(camera_posestamped);
	}
}

bool ROS_PUB::getRosParams(ros::NodeHandle &node_handle_) {
	std::string name_of_node_ = ros::this_node::getName();
	node_handle_.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
	node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
	node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
	node_handle_.param(name_of_node_ + "/bUseViewer", bUseViewer_, true);
	// node_handle_.param(name_of_node_ + "/read_from_topic", read_from_topic, true);
}

vector<float> ROS_PUB::QuaternionDotProduct(vector<float> q1,vector<float> q2){
	// quaternion format is [x,y,z,w]
	float x1=q1[0];
	float y1=q1[1];
	float z1=q1[2];
	float w1=q1[3];
	float x2=q2[0];
	float y2=q2[1];
	float z2=q2[2];
	float w2=q2[3];
	return vector<float>{		
		w1*x2 + x1*w2 + y1*z2 - z1*y2,
		w1*y2 - x1*z2 + y1*w2 + z1*x2,
		w1*z2 + x1*y2 - y1*x2 + z1*w2,
		w1*w2 - x1*x2 - y1*y2 - z1*z2
	};
}

geometry_msgs::Pose ROS_PUB::TransformtoWorld(geometry_msgs::Pose camera_pose){
	Eigen::Quaterniond quat_cam(camera_pose.orientation.w, camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z);
	Eigen::Matrix3d mat_cam = quat_cam.matrix();
	Eigen::Matrix<double,1,3> vec_cam(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
	Eigen::Matrix4d trans_cam = Eigen::Matrix4d::Zero();
	trans_cam.topLeftCorner<3,3>() = mat_cam;
	trans_cam.topRightCorner<3,1>() = vec_cam;
	trans_cam(-1,-1) = 1;
	
	float angles[3]{-90,0,-90}; // xyz
	Eigen::Vector3d eulerAngle(angles[0] * M_PI / 180 , angles[1] * M_PI / 180 , angles[2] * M_PI / 180);
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d mat_rotate;
	mat_rotate = yawAngle * pitchAngle * rollAngle;
	Eigen::Matrix4d trans_rotate = Eigen::Matrix4d::Zero();
	trans_rotate.topLeftCorner<3,3>() = mat_rotate;
	trans_rotate(3,3) = 1;

	// cout << "mat_rotate: \n" << mat_rotate << endl;
	// cout << "trans_rotate: \n" << trans_rotate << endl;

	Eigen::Matrix4d trans_rectified = trans_rotate * trans_cam;
	Eigen::Matrix3d mat_rectified = trans_rectified.topLeftCorner<3,3>();
	Eigen::Quaterniond quat_rectified(mat_rectified);
	geometry_msgs::Pose pose;
	pose.orientation.w = quat_rectified.w();
	pose.orientation.x = quat_rectified.x();
	pose.orientation.y = quat_rectified.y();
	pose.orientation.z = quat_rectified.z();
	pose.position.x = trans_rectified(0,3);
	pose.position.y = trans_rectified(1,3);
	pose.position.z = trans_rectified(2,3);
	return pose;
}