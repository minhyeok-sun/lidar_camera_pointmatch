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


ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// read message
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud);

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_1_near (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass3;
  pass3.setInputCloud (cloud_copy_lower);           
  pass3.setFilterFieldName ("x");        
  pass3.setFilterLimits (-10, 40); 
  pass3.filter (*cloud_copy_1_near); 

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_1_far (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass5;
  pass5.setInputCloud (cloud_copy_lower);           
  pass5.setFilterFieldName ("x");        
  pass5.setFilterLimits (40, 100); 
  pass5.filter (*cloud_copy_1_far);
////////////////////////////////////////////////////////////////////////////////////////////////


  //cloud_copy_1_near plane seg
  pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZI> seg_1;
  seg_1.setOptimizeCoefficients (true); 
  seg_1.setInputCloud (cloud_copy_1_near);     
  seg_1.setModelType (pcl::SACMODEL_PLANE); 
  seg_1.setMethodType (pcl::SAC_RANSAC);   
  seg_1.setMaxIterations (500);    
  seg_1.setDistanceThreshold (0.20); 
  seg_1.segment (*inliers_1, *coefficients_1);  
  std::cerr << "Model coefficients: " << coefficients_1->values[0] << " " 
                                      << coefficients_1->values[1] << " "
                                      << coefficients_1->values[2] << " " 
                                      << coefficients_1->values[3] << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints_1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints_neg_1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*cloud_copy_1_near, *inliers_1, *inlierPoints_1);

  pcl::ExtractIndices<pcl::PointXYZI> extract_1;
  extract_1.setInputCloud (cloud_copy_1_near);
  extract_1.setIndices (inliers_1);
  extract_1.setNegative (true);//false
  extract_1.filter (*cloud_copy_1_near);

  pcl::PassThrough<pcl::PointXYZI> passz1; 
  passz1.setInputCloud (cloud_copy_1_near);
  passz1.setFilterFieldName ("z");
  passz1.setFilterLimits (-1.8, 3);
  passz1.filter (*cloud_copy_1_near);


  // merge the filtered pointclouds
  *cloud_copy_lower = *cloud_copy_1_far + *cloud_copy_1_near;
  *cloud_filtered = *cloud_copy_lower  + *cloud_copy_upper;
  // convert back to ros message
  pcl::PCLPointCloud2 cloud_f;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_f);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_f, output);
  output.header.frame_id = "velodyne";   
  pub.publish(output);

}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "livox_plane");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/point_process/livox_merge", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/livox_plane", 1);
  ros::spin ();
}
