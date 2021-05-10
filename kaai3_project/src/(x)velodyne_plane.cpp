#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

void
cloud_cb_1 (const sensor_msgs::PointCloud2ConstPtr& input) //for ns1 plane_seg
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg (*input, *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

// plane dividing (upper) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_upper (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass1; 
  pass1.setInputCloud (cloud);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (-1.5, 3);
  pass1.filter (*cloud_copy_upper);
// plane dividing (lower) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_lower (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass2.setInputCloud (cloud);           
  pass2.setFilterFieldName ("z");        
  pass2.setFilterLimits (-5, -1.5); 
  pass2.filter (*cloud_copy_lower); 

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);
  seg.setInputCloud (cloud_copy_lower);  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);   
  seg.setMaxIterations (1000);  
  seg.setDistanceThreshold (0.2);
  seg.segment (*inliers, *coefficients);
  std::cerr << "velodne " << coefficients->values[0] << " "  << coefficients->values[3] << std::endl;
  pcl::copyPointCloud<pcl::PointXYZI>(*cloud_copy_lower, *inliers, *inlierPoints);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_copy_lower);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*cloud_copy_lower);

  pcl::PassThrough<pcl::PointXYZI> passz1; 
  passz1.setInputCloud (cloud_copy_lower);
  passz1.setFilterFieldName ("z");
  passz1.setFilterLimits (-1.5, 3);
  passz1.filter (*cloud_copy_lower);

  *cloud_filtered = *cloud_copy_lower + *cloud_copy_upper;
  pcl::PCLPointCloud2 cloud_f;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_f);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_f, output);
  output.header.frame_id = "velodyne";   
  pub1.publish(output);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
cloud_cb_2 (const sensor_msgs::PointCloud2ConstPtr& input)  //for ns2 plane_seg, same with above
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg (*input, *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

// plane dividing (upper) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_upper (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass1; 
  pass1.setInputCloud (cloud);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (-1.5, 3);
  pass1.filter (*cloud_copy_upper);
// plane dividing (lower) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_lower (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass2.setInputCloud (cloud);           
  pass2.setFilterFieldName ("z");        
  pass2.setFilterLimits (-5, -1.5); 
  pass2.filter (*cloud_copy_lower); 

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);
  seg.setInputCloud (cloud_copy_lower);  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);   
  seg.setMaxIterations (1000);  
  seg.setDistanceThreshold (0.2);
  seg.segment (*inliers, *coefficients);
  std::cerr << "velodne " << coefficients->values[0] << " "  << coefficients->values[3] << std::endl;
  pcl::copyPointCloud<pcl::PointXYZI>(*cloud_copy_lower, *inliers, *inlierPoints);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_copy_lower);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*cloud_copy_lower);


  pcl::PassThrough<pcl::PointXYZI> passz2; 
  passz2.setInputCloud (cloud_copy_lower);
  passz2.setFilterFieldName ("z");
  passz2.setFilterLimits (-1.5, 3);
  passz2.filter (*cloud_copy_lower);


  *cloud_filtered = *cloud_copy_lower + *cloud_copy_upper;
  pcl::PCLPointCloud2 cloud_f;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_f);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_f, output);
  output.header.frame_id = "velodyne";   
  pub2.publish(output);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
cloud_cb_3 (const sensor_msgs::PointCloud2ConstPtr& input)  //for ns3 plane_seg, same with above
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg (*input, *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

// plane dividing (upper) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_upper (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass1; 
  pass1.setInputCloud (cloud);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (-1.5, 3);
  pass1.filter (*cloud_copy_upper);
// plane dividing (lower) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_lower (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass2.setInputCloud (cloud);           
  pass2.setFilterFieldName ("z");        
  pass2.setFilterLimits (-5, -1.5); 
  pass2.filter (*cloud_copy_lower); 

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);
  seg.setInputCloud (cloud_copy_lower);  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);   
  seg.setMaxIterations (1000);  
  seg.setDistanceThreshold (0.2);
  seg.segment (*inliers, *coefficients);
  std::cerr << "velodne " << coefficients->values[0] << " "  << coefficients->values[3] << std::endl;
  pcl::copyPointCloud<pcl::PointXYZI>(*cloud_copy_lower, *inliers, *inlierPoints);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_copy_lower);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*cloud_copy_lower);


  pcl::PassThrough<pcl::PointXYZI> passz3; 
  passz3.setInputCloud (cloud_copy_lower);
  passz3.setFilterFieldName ("z");
  passz3.setFilterLimits (-1.5, 3);
  passz3.filter (*cloud_copy_lower);


  *cloud_filtered = *cloud_copy_lower + *cloud_copy_upper;
  pcl::PCLPointCloud2 cloud_f;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_f);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_f, output);
  output.header.frame_id = "velodyne";   
  pub3.publish(output);
}


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "velodyne_plane_seg");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe ("/ns1/velodyne_points", 1, cloud_cb_1);
  ros::Subscriber sub2 = nh.subscribe ("/ns2/velodyne_points", 1, cloud_cb_2);
  ros::Subscriber sub3 = nh.subscribe ("/ns3/velodyne_points", 1, cloud_cb_3);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/ns1/velodyne_points/plane_seg", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/ns2/velodyne_points/plane_seg", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/ns3/velodyne_points/plane_seg", 1);
  ros::spin ();
}
