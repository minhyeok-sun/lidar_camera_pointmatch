#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI> ());


///////////////////////////////////////////////////////////////////////////////////////////////

void // livox/lidar_1HDDH3200106141(left)
cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_3)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud_3 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_3 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg_3, *source_cloud_3);

  Eigen::Matrix4f m_3;
  m_3<<     1,   0,  0,  -0.1,	// x

          0,   1,  0,  0.18,	// y

          0,   0,  1,  -0.0,	// z

          0,   0,  0,  1;

  double a_3, b_3, c_3;
  c_3 = 0 * 3.14 / 180;  //roll (x축기준 회전) 좌우 기우리짐
  b_3 = 0 * 3.14 / 180;  //pitch (y축기준 회전) 앞뒤 쏠림
  a_3 = 56 * 3.14 / 180;  //yaw (xy좌표계 변환!!) 단순 회전  (- 우측 이동)

  m_3(0,0) = cos(a_3)*cos(b_3);
  m_3(0,1) = cos(a_3)*sin(b_3)*sin(c_3) - sin(a_3)*cos(c_3);
  m_3(0,2) = cos(a_3)*sin(b_3)*cos(c_3) + sin(a_3)*sin(c_3);

  m_3(1,0) = sin(a_3)*cos(b_3);
  m_3(1,1) = sin(a_3)*sin(b_3)*sin(c_3) + cos(a_3)*cos(c_3);
  m_3(1,2) = sin(a_3)*sin(b_3)*cos(c_3) - cos(a_3)*sin(c_3);

  m_3(2,0) = -sin(b_3);
  m_3(2,1) = cos(b_3)*sin(c_3);
  m_3(2,2) = cos(b_3)*cos(c_3);

  pcl::transformPointCloud (*source_cloud_3, *transformed_cloud_3, m_3);
  pcl::copyPointCloud(*transformed_cloud_3, *cloud3);

  delete source_cloud_3;
  delete transformed_cloud_3;

}

///////////////////////////////////////////////////////////////////////////////////////////////

void // livox/1H-10418(right)
cloud_cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_4)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud_4 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_4 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg_4, *source_cloud_4);

  Eigen::Matrix4f m_4;
  m_4<<     1,   0,  0,  -0.19,	// x

          0,   1,  0,  -0.26,	// y

          0,   0,  1,  -0.0,	// z

          0,   0,  0,  1;

  double a_4, b_4, c_4;
  c_4 = 0 * 3.14 / 180;  //roll (x축기준 회전) 좌우 기우리짐
  b_4 = 0 * 3.14 / 180;  //pitch (y축기준 회전) 앞뒤 쏠림
  a_4 = -89.7 * 3.14 / 180;  //yaw (xy좌표계 변환!!) 단순 회전  (- 우측 이동)

  m_4(0,0) = cos(a_4)*cos(b_4);
  m_4(0,1) = cos(a_4)*sin(b_4)*sin(c_4) - sin(a_4)*cos(c_4);
  m_4(0,2) = cos(a_4)*sin(b_4)*cos(c_4) + sin(a_4)*sin(c_4);

  m_4(1,0) = sin(a_4)*cos(b_4);
  m_4(1,1) = sin(a_4)*sin(b_4)*sin(c_4) + cos(a_4)*cos(c_4);
  m_4(1,2) = sin(a_4)*sin(b_4)*cos(c_4) - cos(a_4)*sin(c_4);

  m_4(2,0) = -sin(b_4);
  m_4(2,1) = cos(b_4)*sin(c_4);
  m_4(2,2) = cos(b_4)*cos(c_4);

  pcl::transformPointCloud (*source_cloud_4, *transformed_cloud_4, m_4);
  pcl::copyPointCloud(*transformed_cloud_4, *cloud4);

  delete source_cloud_4;
  delete transformed_cloud_4;

}


//////////////////////////////////////////////////////////////////////////////////////////////

void 
cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  pcl::fromROSMsg (*cloud_msg2, *cloud1);
  pcl::PointCloud<pcl::PointXYZI>* merge_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud3_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud4_pc = new pcl::PCLPointCloud2;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_f = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* merge_cloud_pc = new pcl::PCLPointCloud2;

  pcl::toPCLPointCloud2(*cloud1, *cloud1_pc);
  pcl::toPCLPointCloud2(*cloud3, *cloud3_pc);
  pcl::toPCLPointCloud2(*cloud4, *cloud4_pc);
  pcl::toPCLPointCloud2(*merge_cloud, *merge_cloud_pc);

  pcl::concatenatePointCloud(*cloud1_pc, *cloud3_pc, *merge_cloud_pc);
  pcl::concatenatePointCloud(*merge_cloud_pc, *cloud4_pc, *merge_cloud_pc);


  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(*merge_cloud_pc, output);
  pub.publish(output);
  delete merge_cloud;
  delete cloud1_pc;
  delete cloud3_pc;
  delete cloud4_pc;
  delete merge_cloud_pc;

}


///////////////////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "merge_final");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/point_process/livox_merge_first", 1, cloud_cb1);
//  ros::Subscriber sub2 = nh.subscribe ("/livox/lidar_3WEDH5900101801", 1, cloud_cb2);
  ros::Subscriber sub3 = nh.subscribe ("/livox/lidar_1HDDH3200106141", 1, cloud_cb3);
  ros::Subscriber sub4 = nh.subscribe ("/livox/lidar_1HDDH1200104181", 1, cloud_cb4);
//  ros::Subscriber sub5 = nh.subscribe ("/livox/lidar_3WEDH7600101761", 1, cloud_cb5);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/livox_merge", 1);
 
  // Spin
  ros::spin ();
}
