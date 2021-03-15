#  rosrun ORB_SLAM2 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id/-1 <image_topic>
source build/devel/setup.zsh
rosrun ORB_SLAM2 Monopub /home/gxx/WorkSpace/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/gxx/WorkSpace/ORB_SLAM2/Examples/Monocular/KITTI00-02.yaml /home/gxx/WorkSpace/1Dataset/data_odometry_gray/dataset/sequences/00
# rosrun ORB_SLAM2 Monopub /home/gxx/WorkSpace/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/gxx/WorkSpace/ORB_SLAM2/Examples/Monocular/KITTI00-02.yaml -1 /camera/rgb/image_color