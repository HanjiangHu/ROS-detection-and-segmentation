#ros_seg
This package conbines yolo and SegmenterLight together using ROS.
#How to run
1.See "how to run" in ros_yolo package, do it again.<br>
2.In SegmenterLight, run the following commands in a terminal.<br>
cd SegmenterLight<br>
mkdir build<br>
cd build<br>
cmake ..<br>
make<br>
sudo make install<br>
3.catkin_make it in the ros workshop
4.rosrun ros_seg pcl_create
