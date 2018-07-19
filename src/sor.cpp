#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
// #include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


ros::Publisher cloud_pub_before_filtered;
ros::Publisher cloud_pub_after_filtered;

laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header = message->header;

  // project the scan into a point cloud msg
  projector_.projectLaser(*message, cloud_msg);

  // printf("cloud_msg: height, width = %d, %d\n", cloud_msg.height, cloud_msg.width);

  cloud_pub_before_filtered.publish(cloud_msg);

  // Convert to PCL data type
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // SOR
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setMeanK(10);
  sor.setStddevMulThresh (1.0);
  sor.filter(cloud_filtered);

  // convert to PCL_ros
  sensor_msgs::PointCloud2 cloud_filtered_msg;
  pcl_conversions::fromPCL(cloud_filtered, cloud_filtered_msg);

  // Publish the data
  cloud_pub_after_filtered.publish(cloud_filtered_msg);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sor");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("scan", 1000, laserScanCallback);

  cloud_pub_before_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cloud_before_filtered", 1);

  cloud_pub_after_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cloud_after_filtered", 1);

  ros::spin();

  return 0;
}
