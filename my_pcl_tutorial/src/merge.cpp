#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

ros::Publisher pub;

using PointXYZIRT = VelodynePointXYZIRT; // this code is saying that the using this name  PointXYZIRT as VelodynePointXYZIRT
pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud1(new pcl::PointCloud<VelodynePointXYZIRT>());
pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud2(new pcl::PointCloud<VelodynePointXYZIRT>());




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

  ros::Subscriber sub1 = nh.subscribe("vlp_2/velodyne_points", 100, cloudCallback1);
  ros::Subscriber sub2 = nh.subscribe("vlp203", 100, cloudCallback2);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("vlp202", 100);

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (cloud1->points.size() > 0 && cloud2->points.size() > 0)
    {
      pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud3(new pcl::PointCloud<VelodynePointXYZIRT>());
      *cloud3 = *cloud1 + *cloud2;

      sensor_msgs::PointCloud2 cloud3_msg;
      pcl::toROSMsg(*cloud3, cloud3_msg);
      cloud3_msg.header.stamp = ros::Time::now();
      cloud3_msg.header.frame_id = "velodyne";

      pub.publish(cloud3_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}