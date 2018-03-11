#ros_seg
This package conbines yolo and SegmenterLight together using ROS.File registered.cpp is used to register pointcloud instead of using inaccurate topic. /camera/depth_registered/points.
#How to run
1.See "how to run" in ros_yolo package, do it again.<br>
2.In SegmenterLight, run the following commands in a terminal.<br>
cd SegmenterLight<br>
mkdir build<br>
cd build<br>
cmake ..<br>
make<br>
sudo make install<br>
3.catkin_make it in the ros workshopl<br>
4.rosrun ros_seg pcl_create or rosrun ros_seg registered(change the topic name to "registered_cloud" on line 244 in pcl_create.cpp before catkin_make, then rosrun ros_seg pcl_create after rosrun ros_seg registered)<br>
