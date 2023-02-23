#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the incoming ROS message to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);  // Change the limits based on your requirements

  // Extract the ground points
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*filtered_cloud);

  // Convert the filtered point cloud to ROS message format
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);

  // Publish the filtered point cloud
  pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "remove_ground_points");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("velodyne_points", 100, cloudCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 100);

  ros::spin();

  return 0;
}