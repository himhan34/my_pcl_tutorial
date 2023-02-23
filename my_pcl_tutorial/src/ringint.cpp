#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub[16];

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert the incoming ROS message to PCL format
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Split the cloud into separate channels
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> channels(16);
  for (int i = 0; i < 16; i++) {
    channels[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < cloud->points.size(); i++) {
    pcl::PointXYZI p = cloud->points[i];
    channels[(int)p.intensity % 16]->push_back(p);
  }

  // Publish each channel
  for (int i = 0; i < 16; i++) {
    sensor_msgs::PointCloud2 channel_msg;
    pcl::toROSMsg(*(channels[i]), channel_msg);
    channel_msg.header.stamp = cloud_msg->header.stamp;
    channel_msg.header.frame_id = cloud_msg->header.frame_id;
    pub[i].publish(channel_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_pointcloud");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("velodyne_points", 100, cloudCallback);
  for (int i = 0; i < 16; i++) {
    std::string topic_name = "channel_" + std::to_string(i);
    pub[i] = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 100);
  }

  ros::spin();

  return 0;
} 