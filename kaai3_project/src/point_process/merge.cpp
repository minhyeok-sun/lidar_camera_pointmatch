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


ros::Publisher pub;


void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::fromROSMsg (*cloud_msg, *source_cloud);

  Eigen::Matrix4f m;
  m<<     1,   0,  0,  0.0,	// x

          0,   1,  0,  0,	// y

          0,   0,  1,  0,	// z

          0,   0,  0,  1;

  double a, b, c, d;
  c = -0.0; //roll (x축기준 회전) 좌우 기우리짐
  b = 0;  //pitch (y축기준 회전) 앞뒤 쏠림
  a = 16 * 3.14 / 180;  //yaw (xy좌표계 변환!!)


  m(0,0) = cos(a)*cos(b);
  m(0,1) = cos(a)*sin(b)*sin(c) - sin(a)*cos(c);
  m(0,2) = cos(a)*sin(b)*cos(c) + sin(a)*sin(c);

  m(1,0) = sin(a)*cos(b);
  m(1,1) = sin(a)*sin(b)*sin(c) + cos(a)*cos(c);
  m(1,2) = sin(a)*sin(b)*cos(c) - cos(a)*sin(c);

  m(2,0) = -sin(b);
  m(2,1) = cos(b)*sin(c);
  m(2,2) = cos(b)*cos(c);

  pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
  pcl::copyPointCloud(*transformed_cloud, *cloud1);
  delete source_cloud;
  delete transformed_cloud;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*cloud1, *cloud1_pc);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud1_pc, output);
  pub.publish(output); 
}



int
main (int argc, char** argv)
{
  ros::init (argc, argv, "final_merge");
  ros::NodeHandle nh;
  ros::Subscriber sub2 = nh.subscribe ("/point_process/livox_plane", 1, cloud_cb2);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/merge", 1);
 
  ros::spin ();
}
