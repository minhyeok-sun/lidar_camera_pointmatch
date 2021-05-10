#include <ros/ros.h>
// PCL specific includes
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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub;
ros::Publisher pub2;

typedef pcl::PointXYZ PointT;
 
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
 
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud.makeShared());
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.6); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (500000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
 


 
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;


  int j = 0;
  float minx, miny, minz;
  float maxx, maxy, maxz;
  float xsize, ysize, zsize, volume, x_location, y_location, z_location;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    float a = cluster_indices.size();
    std::cout << a << std::endl;


    minx = 9999.99;
    miny = 9999.99;
    minz = 9999.99;
    maxx = -9999.99;
    maxy = -9999.99;
    maxz = -9999.99;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {

        pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            minx = std::min(minx, pt2.x);
            miny = std::min(miny, pt2.y);
            minz = std::min(minz, pt2.z); 
            maxx = std::max(maxx, pt2.x);
            maxy = std::max(maxy, pt2.y);
            maxz = std::max(maxz, pt2.z);

            pt2.intensity = (float)(j + 1);
            TotalCloud.push_back(pt2);
    }
    std::cout << j <<std::endl;
    std::cout << xsize <<std::endl;
    std::cout << ysize <<std::endl;
    std::cout << zsize <<std::endl;
    xsize = std::abs(maxx-minx);
    ysize = std::abs(maxy-miny);
    zsize = std::abs(maxz-minz);
    x_location = (minx+maxx)/2;
    y_location = (miny+maxy)/2;
    z_location = (minz+maxz)/2;
    volume = xsize * ysize * zsize;


    if (0 < xsize && xsize < 10 && 1 < ysize && ysize < 3 && 0.2<zsize && zsize<3 && volume > 0 && z_location < 0 && x_location > 1) 
	{
	    marker.header.frame_id = "velodyne";
	    marker.header.stamp = ros::Time();
   	    marker.ns = a;
 	    marker.id = j;
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = (minx+maxx)/2;
	    marker.pose.position.y = (miny+maxy)/2;
	    marker.pose.position.z = (minz+maxz)/2;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = -(minx-maxx);
	    marker.scale.y = -(miny-maxy);
	    marker.scale.z = -(minz-maxz);
	    marker.color.a = 1.0;
	    marker.color.r = 0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 1.0;
	    marker.lifetime = ros::Duration(0.1);
	    marker_array.markers.push_back(marker);
	}
    j++;
  }
  pub2.publish(marker_array);
    // Convert To ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";   
  pub.publish(output); 
}


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/point_process/filter", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/clustering", 1);
  pub2 = nh.advertise<visualization_msgs::MarkerArray> ("/point_process/boundingbox", 1);
 
  ros::spin ();
}
