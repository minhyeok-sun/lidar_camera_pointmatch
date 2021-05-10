#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <vector>

ros::Publisher pub;
cv::Mat img(720,1280,CV_8UC3, cv::Scalar(255,255,255));

void image_raw(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img = cv_ptr->image;
}


void projection(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::vector<cv::Point3f> points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptc (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg (*cloud_msg, *pcl_ptc);
  
  pcl::PassThrough<pcl::PointXYZI> passz;
  passz.setInputCloud (pcl_ptc);         
  passz.setFilterFieldName ("x");            
  passz.setFilterLimits (1.5, 100); 
  passz.filter (*pcl_ptc);             


  for (const auto& point : *pcl_ptc) {
  points.push_back(cv::Point3f(point.x, point.y, point.z));}

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat rt(4,4,cv::DataType<double>::type);  cv::Mat r(4,4,cv::DataType<double>::type);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 카메라 projection matrix
  p.at<double>(0,0) = 693.5944213867188;    p.at<double>(0,1) = 0.00000000;    p.at<double>(0,2) = 636.1531982421875;    p.at<double>(0,3) = 0.00000000;  
  p.at<double>(1,0) = 0.00000000;    p.at<double>(1,1) = 693.5944213867188;    p.at<double>(1,2) = 394.94525146484375;    p.at<double>(1,3) = 0.00000000;  
  p.at<double>(2,0) = 0.00000000;    p.at<double>(2,1) = 0.00000000;    p.at<double>(2,2) = 1.00000000;    p.at<double>(2,3) = 0.00000000;  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Rotation matrix
  rt.at<double>(0,0) = 1.00;    rt.at<double>(0,1) = 0.00;    rt.at<double>(0,2) = -0.0000;    rt.at<double>(0,3) = -0.38;  // x축 (좌측이동시 -)
  rt.at<double>(1,0) = 0.000;    rt.at<double>(1,1) = 1.00;    rt.at<double>(1,2) = 0.00;    rt.at<double>(1,3) = -0.15;  // y축 이동(위로 이동시 -)
  rt.at<double>(2,0) = 0.00;    rt.at<double>(2,1) = 0.000;    rt.at<double>(2,2) = 1.00;    rt.at<double>(2,3) = -0;  //확대 축소 (줌인(-))
  rt.at<double>(3,0) = 0.00000000;    rt.at<double>(3,1) = 0.00000000;    rt.at<double>(3,2) = 0.00000000;    rt.at<double>(3,3) = 1.00000000;

  double c = 0; //roll (x축기준 회전) 좌우 기우리짐
  double b = -3.201592/2;  //pitch (y축기준 회전) 앞뒤 쏠림 아래 : 양수
  double a = 3.141592/2;  //yaw (xy좌표계 변환!!)

  rt.at<double>(0,0) = cos(a)*cos(b);
  rt.at<double>(0,1) = cos(a)*sin(b)*sin(c) - sin(a)*cos(c);
  rt.at<double>(0,2) = cos(a)*sin(b)*cos(c) + sin(a)*sin(c);

  rt.at<double>(1,0) = sin(a)*cos(b);
  rt.at<double>(1,1) = sin(a)*sin(b)*sin(c) + cos(a)*cos(c);
  rt.at<double>(1,2) = sin(a)*sin(b)*cos(c) - cos(a)*sin(c);

  rt.at<double>(2,0) = -sin(b);
  rt.at<double>(2,1) = cos(b)*sin(c);
  rt.at<double>(2,2) = cos(b)*cos(c); 

  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat visImg = img.clone();
  cv::Mat overlay = visImg.clone();

// 단순히 x, y 픽셀 평행 이동 
  double offset_x = 0.00;
  double offset_y = 0.0;
  int number = 0;
  for(auto it=points.begin(); it!=points.end(); ++it) {
    X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
    Y = p * rt * X;
    cv::Point pt;
    pt.x = Y.at<double>(0,0) / Y.at<double>(0,2) + offset_x;
    pt.y = Y.at<double>(1,0) / Y.at<double>(0,2) + offset_y;
    

    float val = it->x;
    float maxval = 50;
    int red = std::min(255, (int)(255*abs((val - maxval)/maxval)));
    int green = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));

    cv::circle(overlay, pt, 3, cv::Scalar(0,green,red), -1);
    number += 1;
  }

  std::cout << number << std::endl;

  float opacity = 0.7;
  cv::addWeighted(overlay, opacity, visImg, 1-opacity, 0, visImg);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg();
  pub.publish(msg);
}




int
main (int argc, char** argv)
{
  ros::init (argc, argv, "projection");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe ("/zed/zed_node/rgb/image_rect_color", 1, image_raw);
  ros::Subscriber sub2 = nh.subscribe ("merge", 1, projection);

  pub = nh.advertise<sensor_msgs::Image> ("projected", 1);

  ros::spin ();
}
