#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *cloud1);
}

void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *cloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_pointcloud");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("vlp_1/velodyne_points", 100, cloudCallback1);
  ros::Subscriber sub2 = nh.subscribe("vlp_2/velodyne_points", 100, cloudCallback2);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("merged_cloud", 100);

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (cloud1->points.size() > 0 && cloud2->points.size() > 0)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
      *cloud_merged = *cloud1 + *cloud2;

      sensor_msgs::PointCloud2 cloud_merged_msg;
      pcl::toROSMsg(*cloud_merged, cloud_merged_msg);
      cloud_merged_msg.header.stamp = ros::Time::now();
      cloud_merged_msg.header.frame_id = cloud1->header.frame_id;

      pub.publish(cloud_merged_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}