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
#include <SegmenterLight.h>
#include "test_detector.h"


#define WIDTH 640
#define HEIGHT 480


void PCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pcl_cloud, cv::Mat_<cv::Vec3b> &image){
    unsigned pcWidth = pcl_cloud->width;
    unsigned pcHeight = pcl_cloud->height;
    unsigned position = 0;

    image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

    for (unsigned row = 0; row < pcHeight; row++) {
      for (unsigned col = 0; col < pcWidth; col++) {
        cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
        position = row * pcWidth + col;
        const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

        cvp[2] = pt.r;
        cvp[1] = pt.g;
        cvp[0] = pt.b;
      }
    }

}
ros::Publisher pcl_pub ;
int counter = 0;
char * datacfg = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/coco.data"));//voc.data
char * cfgfile = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/yolo.cfg"));//tiny-yolo-voc yolo
char * weightfile = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/yolo.weights"));//tiny-yolo-voc yolo
char * filename = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/rgb.png"));// /home/huhanjiang/catkin_ws/src/ros_seg/data/rgb0.png

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(input, cloud);
	//cloud.setKeepOrganized(true);
    pcl::io::savePCDFileASCII ("/home/huhanjiang/segment/SegmenterLight/bin/seg_yolo.pcd", cloud);
	
	 //get point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/huhanjiang/segment/SegmenterLight/bin/seg_yolo.pcd", *pcl_cloud);
    //get labeled
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl_cloud_labeled.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);
    std::string modelPath = "/home/huhanjiang/catkin_ws/src/ros_seg/model/";
    segment::SegmenterLight seg(modelPath);
    seg.setFast(true);
    seg.setDetail(2);
    //cout << "h" <<endl;
    pcl_cloud_labeled = seg.processPointCloud(pcl_cloud);
    //cout << "hh" <<endl;
    
    sensor_msgs::PointCloud2 output;

    //get rgb img
    cv::Mat_<cv::Vec3b> src;
    PCLCloud2Image(pcl_cloud, src);
    //cv::imshow("saf", src);
    //cv::waitKey(0);
    cv::imwrite("/home/huhanjiang/catkin_ws/src/ros_seg/data/rgb.png", src);
    //yolo
    unsigned centerRow ,row_begin=89 ,row_end=150 ;//35/89  89/150
    unsigned centerCol ,col_begin=389 ,col_end=442 ;//385/433   389/442
   
    detection test = detector1(datacfg,cfgfile,weightfile ,filename,0.24,0.5,0,0);
    std::cout << "yolo finished successfully!" << std:: endl;
    for(int i = 0; i < 10; i++){
            if(test.selected[i][0] != -1){
              std::cout << test.names[test.selected[i][1]] << std::endl;
                if(0 == strcmp(test.names[test.selected[i][1]], "cup")) {
                    row_begin = HEIGHT * (test.pos[test.selected[i][0]].y - test.pos[test.selected[i][0]].h/2);
                    row_end = HEIGHT * (test.pos[test.selected[i][0]].y + test.pos[test.selected[i][0]].h/2);
                    col_begin = WIDTH * (test.pos[test.selected[i][0]].x - test.pos[test.selected[i][0]].w/2);
                    col_end = WIDTH * (test.pos[test.selected[i][0]].x + test.pos[test.selected[i][0]].w/2);
                    break;
                }
            }
        else {
            std::cout << "没有检测到杯子诶，再试一下吧" << std:: endl;
            break;
            }
        }

    
    //segment of labeled point cloud
  std::cout << row_begin <<std::endl;
  std::cout << row_end <<std::endl;
  std::cout << col_begin <<std::endl;
  std::cout << col_end <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
  unsigned pcWidth = pcl_cloud_labeled -> width ;
  unsigned pcHeight = pcl_cloud_labeled -> height;
  unsigned position = 0;
  //unsigned centerRow ,row_begin ,row_end ;//35/89  89/150
  //unsigned centerCol ,col_begin ,col_end ;//385/433   389/442
  std::vector<pcl::PointXYZRGBL> sameLabel;//相同标签的点存在顺序表里面
  sameLabel.push_back(pcl_cloud_labeled -> points[(row_begin + 10) * pcWidth + (col_begin + 15)]);
  std::list< std::vector<pcl::PointXYZRGBL> > difLabel;//不同标签的点云 链表
  difLabel.push_back(sameLabel);
  std::list< std::vector<pcl::PointXYZRGBL> >::iterator itr = difLabel.begin(),itre;
  int flag = 0;
    for (unsigned row = row_begin ; row < row_end +20; row++) {//+5
      for (unsigned col = col_begin ; col < col_end+15 ; col++) {//+25
        position = row * pcWidth + col;
        //pcl_cloud_labeled -> points[centerRow * pcWidth + centerCol].label
        //target_cloud -> points.push_back(pcl_cloud_labeled -> points[position]);
        if(pcl_cloud_labeled -> points[position].label == (itr -> back()).label){
          itr -> push_back(pcl_cloud_labeled -> points[position]);//label和上一个一样
        }
        else{
          itr = difLabel.begin();//从头开始
          flag = 0;
          while(pcl_cloud_labeled -> points[position].label != (itr -> back()).label){
            itre = difLabel.end();//itre是最后一个元素的下一个元素！！！
            if(++itr == itre) {//新的一个label
              std::vector<pcl::PointXYZRGBL> newLabel;
              newLabel.push_back(pcl_cloud_labeled -> points[position]);
              difLabel.push_back(newLabel);
              flag = 1;
              --itr;
            }
          }
          if(flag == 0)
            itr -> push_back(pcl_cloud_labeled -> points[position]);
        } 
      }
    }
    
    int max = 0;
  while(target_cloud -> points.size() < 5000){
    if(max != 0) {
      difLabel.erase(itre);
      max = 0;
      itre = difLabel.begin();
    }
    for(itr = difLabel.begin();itr != difLabel.end(); itr++){
    if(itr -> size() > max){//itr -> size() > max
      max = itr -> size();
      itre = itr;
      }
    }
    
    std::cout << "这次有："<<max <<std::endl;  
    std::cout <<" label:" <<(*itre)[0].label <<std::endl;
    for(int i = 0; i < max; i++)
      if((itre -> back()).label != 255 ) //   remove the invalid points
        target_cloud -> points.push_back((*itre)[i]);
      else std::cout << "但是这次不算。。。："<<max <<std::endl;
    std::cout << "一共：" <<target_cloud -> points.size() << std::endl<< std::endl;
  }
  char *thisTarget;
  char target_pc[20] ="target";
  char buff[5];
  sprintf(buff,"%d",counter);
 
  thisTarget = strcat(target_pc,buff);
  
  pcl::io::savePCDFileBinary(strcat(thisTarget,".pcd"), *target_cloud );//setup dataset
  //pcl::io::savePCDFileASCII ("tar_rgbl.pcd", *target_cloud);

   
    //Convert the cloud to ROS message
    pcl::toROSMsg(*target_cloud, output);
    output.header.frame_id = "map";
	pcl_pub.publish(output);
  counter++;
    for(int i=0;i<10;i++)  
        free(test.selected[i]);  
    free(test.selected); 
	
}

/*main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_create");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    //get point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/huhanjiang/catkin_ws/src/ros_seg/pcd/seg_yolo.pcd", *pcl_cloud);
    //get labeled
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl_cloud_labeled.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);
    std::string modelPath = "/home/huhanjiang/catkin_ws/src/ros_seg/model/";
    segment::SegmenterLight seg(modelPath);
    seg.setFast(true);
    seg.setDetail(2);
    //cout << "h" <<endl;
    pcl_cloud_labeled = seg.processPointCloud(pcl_cloud);
    //cout << "hh" <<endl;
    
    sensor_msgs::PointCloud2 output;

    //get rgb img
    cv::Mat_<cv::Vec3b> src;
    PCLCloud2Image(pcl_cloud, src);
    //cv::imshow("saf", src);
    //cv::waitKey(0);
    cv::imwrite("/home/huhanjiang/catkin_ws/src/ros_seg/data/rgb.png", src);
    //yolo
    unsigned centerRow ,row_begin=89 ,row_end=150 ;//35/89  89/150
    unsigned centerCol ,col_begin=389 ,col_end=442 ;//385/433   389/442
    char * datacfg = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/voc.data"));
    char * cfgfile = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/yolo.cfg"));//tiny-yolo-voc
    char * weightfile = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/yolo.weights"));//tiny-yolo-voc
    char * filename = const_cast<char*>(("/home/huhanjiang/catkin_ws/src/ros_seg/data/dog.jpg"));
    detection test = detector1(datacfg,cfgfile,weightfile ,filename,0.24,0.5,0,0);
    std::cout << "hhh" << std:: endl;
    for(int i = 0; i < 10; i++){
            if(test.selected[i][0] != -1){
                if(test.names[test.selected[i][1]] == "cup") {
                    row_begin = HEIGHT * (test.pos[test.selected[i][0]].y - test.pos[test.selected[i][0]].h/2);
                    row_end = HEIGHT * (test.pos[test.selected[i][0]].y + test.pos[test.selected[i][0]].h/2);
                    col_begin = WIDTH * (test.pos[test.selected[i][0]].x - test.pos[test.selected[i][0]].w/2);
                    col_end = WIDTH * (test.pos[test.selected[i][0]].x + test.pos[test.selected[i][0]].w/2);
                    break;
                }
            }
        else {
            std::cout << "没有检测到杯子诶，再试一下吧" << std:: endl;
            break;
            }
        }

    
    //segment of labeled point cloud
  std::cout << pcl_cloud_labeled -> width <<std::endl;
  std::cout << pcl_cloud_labeled -> height <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
  unsigned pcWidth = pcl_cloud_labeled -> width ;
  unsigned pcHeight = pcl_cloud_labeled -> height;
  unsigned position = 0;
  //unsigned centerRow ,row_begin ,row_end ;//35/89  89/150
  //unsigned centerCol ,col_begin ,col_end ;//385/433   389/442
  std::vector<pcl::PointXYZRGBL> sameLabel;
  sameLabel.push_back(pcl_cloud_labeled -> points[36 * pcWidth + 405]);
  std::list< std::vector<pcl::PointXYZRGBL> > difLabel;
  difLabel.push_back(sameLabel);
  std::list< std::vector<pcl::PointXYZRGBL> >::iterator itr = difLabel.begin(),itre;
  int flag = 0;
    for (unsigned row = row_begin + 5 ; row < row_end + 5; row++) {
      for (unsigned col = col_begin + 25 ; col < col_end + 25; col++) {
        position = row * pcWidth + col;
        //pcl_cloud_labeled -> points[centerRow * pcWidth + centerCol].label
        //target_cloud -> points.push_back(pcl_cloud_labeled -> points[position]);
        if(pcl_cloud_labeled -> points[position].label == (itr -> back()).label){
          itr -> push_back(pcl_cloud_labeled -> points[position]);//label和上一个一样
        }
        else{
          itr = difLabel.begin();//从头开始
          flag = 0;
          while(pcl_cloud_labeled -> points[position].label != (itr -> back()).label){
            itre = difLabel.end();//itre是最后一个元素的下一个元素！！！
            if(++itr == itre) {//新的一个label
              std::vector<pcl::PointXYZRGBL> newLabel;
              newLabel.push_back(pcl_cloud_labeled -> points[position]);
              difLabel.push_back(newLabel);
              flag = 1;
              --itr;
            }
          }
          if(flag == 0)
            itr -> push_back(pcl_cloud_labeled -> points[position]);
        } 
      }
    }
    int max = 0;
  while(target_cloud -> points.size() < 2000){
    if(max != 0) {
      difLabel.erase(itre);
      max = 0;
      itre = difLabel.begin();
    }
    for(itr = difLabel.begin();itr != difLabel.end(); itr++){
    if(itr -> size() > max){//itr -> size() > max
      max = itr -> size();
      itre = itr;
      }
    }
    std::cout << max <<std::endl;  
    std::cout << (*itre)[0].label <<std::endl;
    std::cout << (itre -> back()).label <<std::endl;
    for(int i = 0; i < max; i++)
      target_cloud -> points.push_back((*itre)[i]);
  }
  
  //pcl::io::savePCDFileBinary("tar_rgbl2.pcd", *target_cloud );
  //pcl::io::savePCDFileASCII ("tar_rgbl.pcd", *target_cloud);

   
    //Convert the cloud to ROS message
    pcl::toROSMsg(*target_cloud, output);
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


main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_create");

    ros::NodeHandle nh;
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Subscriber bat_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCB);//pcl_output
	
    ros::spin();

    return 0;
}


