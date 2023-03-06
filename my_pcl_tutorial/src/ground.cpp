#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub_filtered_cloud;

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert ROS point cloud message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Filter out points with z-coordinate below a threshold (e.g. ground points)
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-100.0, 0.0); // adjust threshold as needed
  pass.setFilterLimitsNegative(true); // set to true to keep points above the threshold
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*cloud_filtered);

  // Convert filtered PCL point cloud back to ROS message
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);
  filtered_cloud_msg.header = cloud_msg->header;

  // Publish filtered point cloud message to output topic
  pub_filtered_cloud.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
  // Initialize ROS node and create node handle
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle nh;

  // Create subscribers and publishers
  ros::Subscriber sub_cloud = nh.subscribe("velodyne_points", 100, cloud_callback);
  pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("ground", 100);

  // Spin the node and process callbacks
  ros::spin();

  return 0;
}
