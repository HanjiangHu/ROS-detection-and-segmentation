# ros detection and segmentation module
This is a ROS node for object detection and segmentation from 2D image to 3D point cloud. The detection is based on [YOLO-v2](https://pjreddie.com/darknet/yolov2/). The segmentation is based on [SegmenterLight](https://github.com/kanster/RGBD-Segmenter).
## Environment
The RGBD camera is Astra of ORBBEC and the ROS SDK is supposed to be installed first.

## Compile and prepare
### YOLO-darknet
Download the the pretrained model from [YOLO-v2](https://pjreddie.com/darknet/yolov2/) under the path `./darknet` and `./data`.

Change the path on line 231 in the file `image.c` in `./darknet/src` to your own path.
Replace all `XXX/catkin_ws` path in the file `ros_yolo_test.cpp` and `test_detector.h` with your own ROS workspace path.

Compile the darknet module.
- `cd darknet`
- `make`

### SegmenterLight
Run the following commands to compile and install. Change the topic name to `registered_cloud` on line 244 in `pcl_create.cpp`.
- `cd SegmenterLight`
- `mkdir build`
- `cd build`
- `cmake ..`
- `make`
- `sudo make install`


Compile the `ros_yolo` node and `ros_seg` node:  `catkin_make`.



### How to run
- `roscore`

Run Astra ROS SDK.
- `roslaunch astra_launch  astra.launch`

Run the YOLO node.
- `rosrun ros_yolo ros_yolo_test`

Check the published YOLO message.
- `rostopic echo /yolo_ret`

Register the point cloud to image and create the segmented object point cloud.
- `rosrun ros_seg registered`
- `rosrun ros_seg pcl_create`

