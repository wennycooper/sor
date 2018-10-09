#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
// #include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>


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

  ros::Time begin = ros::Time::now();

  // Convert to PCL data type
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // ROR
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
  ror.setInputCloud(cloudPtr);
  ror.setRadiusSearch(0.8);
  ror.setMinNeighborsInRadius(2);
  ror.filter(cloud_filtered);

  // convert to PCL_ros
  sensor_msgs::PointCloud2 cloud_filtered_msg;
  pcl_conversions::fromPCL(cloud_filtered, cloud_filtered_msg);


  // ros::Time end = ros::Time::now();

  // Publish the data
  cloud_pub_after_filtered.publish(cloud_filtered_msg);
  
  // ros::Duration duration = end - begin;
  // int t;
  // t = duration.toNSec();
  // printf("%d\n", t);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ror");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("scan", 1000, laserScanCallback);

  cloud_pub_before_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cloud_before_filtered", 1);

  cloud_pub_after_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cloud_after_filtered", 1);

  ros::spin();

  return 0;
}
