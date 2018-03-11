#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<vector>
#include<list>
#include <time.h>
#include<stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include<Eigen/Dense>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_cloud = nh.subscribe("/camera/depth/points", 1, &cloudHandler::cloudCB, this);
        pcl_sub_rgb = nh.subscribe("/camera/rgb/image_raw", 1, &cloudHandler::rgbCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("registered_cloud", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input){
        pcl::fromROSMsg(input, pcl_cloud);
    }
    void rgbCB(const sensor_msgs::ImageConstPtr &input){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input, "bgr8");
        srcimg = cv_ptr -> image;
        if(pcl_cloud.points.size() == 307200) cloudHandler::register_cloud(srcimg,pcl_cloud);
        else std::cout << "点云数量不足，无法生成！" << std::endl;
    }

    void register_cloud(cv::Mat &srcimg, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud){
        unsigned pWidth = pcl_cloud.width;
        unsigned pHeight = pcl_cloud.height;
        unsigned position = 0;
        for (unsigned row = 0; row < pHeight; row++) {
            for (unsigned col = 0; col < pWidth; col++) {
                position = row * pWidth + col;
                pcl_cloud.points[position].r = srcimg.at<cv::Vec3b>(row, col)[2];
                pcl_cloud.points[position].g = srcimg.at<cv::Vec3b>(row, col)[1];
                pcl_cloud.points[position].b = srcimg.at<cv::Vec3b>(row, col)[0];
            }
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pcl_cloud, output);
        output.header.frame_id = "map";
        pcl_pub.publish(output);
    }
    

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_cloud;
    ros::Subscriber pcl_sub_rgb;
    ros::Publisher pcl_pub;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    cv::Mat srcimg;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "registered");

    cloudHandler handler;//点云处理器对象

    ros::spin();

    return 0;
}

/*ros::Publisher pcl_pub ;

Eigen::Matrix4f Tr ;
Eigen::Vector4f originalVector;//ir下
Eigen::Vector4f newVector;//rgb下


main (int argc, char **argv)
{
    ros::init (argc, argv, "registered");

    ros::NodeHandle nh;
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_registered", 1);
    //ros::Subscriber bat_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCB);//pcl_output
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/huhanjiang/test.pcd", *pcl_cloud);
    std::cout << "hhh"<<std::endl;

    cv::Mat srcimg = cv::imread("/home/huhanjiang/test.png",1);

    std::cout << "hhh"<<std::endl;
    Tr <<     0.9998,   -0.0021,    0.0211,  -21.3212,
    0.0016,   0.9998,    0.0216,  -41.3857,
   -0.0211,   -0.0215,    0.9995,   18.5756,
         0,         0,         0,    1.0000;
    
    unsigned pWidth = pcl_cloud->width;
    unsigned pHeight = pcl_cloud->height;
    unsigned position = 0;
    for (unsigned row = 0; row < pHeight; row++) {
      for (unsigned col = 0; col < pWidth; col++) {
        position = row * pWidth + col;
        //pcl::PointXYZRGB &pt = pcl_cloud->points[position];
        originalVector(0,0) = pcl_cloud->points[position].x;
        originalVector(1,0) = pcl_cloud->points[position].y;
        originalVector(2,0) = pcl_cloud->points[position].z;
        originalVector(3,0) = 1;
        newVector = Tr * originalVector;//.inverse()
        //pcl_cloud->points[position].x = newVector(0,0);
        //pcl_cloud->points[position].y = newVector(1,0);
        //pcl_cloud->points[position].z = newVector(2,0);
        pcl_cloud->points[position].r = srcimg.at<cv::Vec3b>(row, col)[2];
        pcl_cloud->points[position].g = srcimg.at<cv::Vec3b>(row, col)[1];
        pcl_cloud->points[position].b = srcimg.at<cv::Vec3b>(row, col)[0];
      }
    }
    pcl::io::savePCDFileASCII ("/home/huhanjiang/test_NO_registered.pcd", *pcl_cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pcl_cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}*/


