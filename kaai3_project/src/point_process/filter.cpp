#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
typedef pcl::PointXYZI PointT;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud);



// filter
  pcl::PassThrough<pcl::PointXYZI> passz;
  passz.setInputCloud (cloud);                //입력 
  passz.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
  passz.setFilterLimits (-10, 2); 
  passz.filter (*cloud_filtered);             //필터 적용

  pcl::PassThrough<pcl::PointXYZI> passy;   //두번째 필터
  passy.setInputCloud (cloud_filtered);
  passy.setFilterFieldName ("y");
  passy.setFilterLimits (-15, 15);
  passy.filter (*cloud_filtered);

  pcl::PassThrough<pcl::PointXYZI> passx;  //세번째 필터
  passx.setInputCloud (cloud_filtered);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (-20, 100);
  passx.filter (*cloud_filtered);

// voxelize
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud (cloud_filtered);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";   
  pub.publish(output); 

}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "filter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/point_process/merge", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/filter", 1);
  ros::spin ();
}
