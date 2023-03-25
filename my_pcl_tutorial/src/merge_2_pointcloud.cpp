#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define PI 3.14159265359

using namespace std;

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

using PointXYZIRT = VelodynePointXYZIRT;

ros::Publisher pub1;

void input1(const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2)
{
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<PointXYZIRT>
    pcl::PointCloud<PointXYZIRT>::Ptr cloud1(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg1, *cloud1);
    pcl::PointCloud<PointXYZIRT>::Ptr cloud2(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg2, *cloud2);

    // Concatenate point clouds
    pcl::PointCloud<PointXYZIRT>::Ptr merged_cloud(new pcl::PointCloud<PointXYZIRT>());
    *merged_cloud = *cloud1 + *cloud2;

    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*merged_cloud, output);
    output.header.frame_id = "vlp201";

    // Publish merged point cloud
    pub1.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "input");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> ("input1", 100, boost::bind(input1, _1, _2));
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("input2", 100, boost::bind(input2, _1, _2));
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output", 100);
    ros::spin();
}
