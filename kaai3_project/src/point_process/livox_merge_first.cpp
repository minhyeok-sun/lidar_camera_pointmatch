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
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI> ());

void ///livox/lidar_3WEDH5900101801  front right
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud_2 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_2 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg_2, *source_cloud_2);

  Eigen::Matrix4f m_2;
  m_2<<     1,   0,  0,  -0.05,	// x

          0,   1,  0,  -0.2,	// y

          0,   0,  1,  -0.1,	// z

          0,   0,  0,  1;

  double a_2, b_2, c_2;
  c_2 = 0 * 3.14 / 180;  //roll (x축기준 회전) 좌우 기우리짐
  b_2 = 0 * 3.14 / 180;  //pitch (y축기준 회전) 앞뒤 쏠림
  a_2 = -32 * 3.14 / 180;  //yaw (xy좌표계 변환!!) 단순 회전  (- 우측 이동)

  m_2(0,0) = cos(a_2)*cos(b_2);
  m_2(0,1) = cos(a_2)*sin(b_2)*sin(c_2) - sin(a_2)*cos(c_2);
  m_2(0,2) = cos(a_2)*sin(b_2)*cos(c_2) + sin(a_2)*sin(c_2);

  m_2(1,0) = sin(a_2)*cos(b_2);
  m_2(1,1) = sin(a_2)*sin(b_2)*sin(c_2) + cos(a_2)*cos(c_2);
  m_2(1,2) = sin(a_2)*sin(b_2)*cos(c_2) - cos(a_2)*sin(c_2);

  m_2(2,0) = -sin(b_2);
  m_2(2,1) = cos(b_2)*sin(c_2);
  m_2(2,2) = cos(b_2)*cos(c_2);

  pcl::transformPointCloud (*source_cloud_2, *transformed_cloud_2, m_2);
  pcl::copyPointCloud(*transformed_cloud_2, *cloud2);

  delete source_cloud_2;
  delete transformed_cloud_2;

}

///////////////////////////////////////////////////////////////////////////////////////////////


void // livox/(back)
cloud_cb5 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_5)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud_5 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_5 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg_5, *source_cloud_5);

  Eigen::Matrix4f m_5;
  m_5<<     1,   0,  0,  -0.40,	// x

          0,   1,  0,  0.05,	// y

          0,   0,  1,  -0.0,	// z

          0,   0,  0,  1;

  double a_5, b_5, c_5;
  c_5 = 0 * 3.14 / 180;  //roll (x축기준 회전) 좌우 기우리짐
  b_5 = 0 * 3.14 / 180;  //pitch (y축기준 회전) 앞뒤 쏠림
  a_5 = 164* 3.14 / 180;  //yaw (xy좌표계 변환!!) 단순 회전  (- 우측 이동)

  m_5(0,0) = cos(a_5)*cos(b_5);
  m_5(0,1) = cos(a_5)*sin(b_5)*sin(c_5) - sin(a_5)*cos(c_5);
  m_5(0,2) = cos(a_5)*sin(b_5)*cos(c_5) + sin(a_5)*sin(c_5);

  m_5(1,0) = sin(a_5)*cos(b_5);
  m_5(1,1) = sin(a_5)*sin(b_5)*sin(c_5) + cos(a_5)*cos(c_5);
  m_5(1,2) = sin(a_5)*sin(b_5)*cos(c_5) - cos(a_5)*sin(c_5);

  m_5(2,0) = -sin(b_5);
  m_5(2,1) = cos(b_5)*sin(c_5);
  m_5(2,2) = cos(b_5)*cos(c_5);

  pcl::transformPointCloud (*source_cloud_5, *transformed_cloud_5, m_5);
  pcl::copyPointCloud(*transformed_cloud_5, *cloud5);

  delete source_cloud_5;
  delete transformed_cloud_5;

}

//////////////////////////////////////////////////////////////////////////////////////////////

void 
cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  pcl::fromROSMsg (*cloud_msg2, *cloud1);
  pcl::PointCloud<pcl::PointXYZI>* merge_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud2_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud5_pc = new pcl::PCLPointCloud2;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_f = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* merge_cloud_pc = new pcl::PCLPointCloud2;

  pcl::toPCLPointCloud2(*cloud1, *cloud1_pc);
  pcl::toPCLPointCloud2(*cloud2, *cloud2_pc);
  pcl::toPCLPointCloud2(*cloud5, *cloud5_pc);
  pcl::toPCLPointCloud2(*merge_cloud, *merge_cloud_pc);

  pcl::concatenatePointCloud(*cloud1_pc, *cloud2_pc, *merge_cloud_pc);
  pcl::concatenatePointCloud(*merge_cloud_pc, *cloud5_pc, *merge_cloud_pc);


  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(*merge_cloud_pc, output);
  pub.publish(output);
  delete merge_cloud;
  delete cloud1_pc;
  delete cloud2_pc;
  delete cloud5_pc;
  delete merge_cloud_pc;

}


///////////////////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "merge");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/livox/lidar_3WEDH7600101631", 1, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/livox/lidar_3WEDH5900101801", 1, cloud_cb2);
  ros::Subscriber sub5 = nh.subscribe ("/livox/lidar_3WEDH7600101761", 1, cloud_cb5);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/livox_merge_first", 1);
 
  // Spin
  ros::spin ();
}
