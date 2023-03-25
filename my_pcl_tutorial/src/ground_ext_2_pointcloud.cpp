#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

using PointXYZIRT = VelodynePointXYZIRT;

ros::Publisher pub1;

void input(const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2)
{
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<PointXYZIRT>
    pcl::PointCloud<PointXYZIRT>::Ptr cloud1(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg1, *cloud1);
    pcl::PointCloud<PointXYZIRT>::Ptr cloud2(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg2, *cloud2);

    // Concatenate point clouds
    pcl::PointCloud<PointXYZIRT>::Ptr merged_cloud(new pcl::PointCloud<PointXYZIRT>());
    *merged_cloud = *cloud1 + *cloud2;

    // Perform ground plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointXYZIRT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(merged_cloud);
    seg.segment(*inliers, *coefficients);

    // Extract ground points
    pcl::PointCloud<PointXYZIRT>::Ptr ground_cloud(new pcl::PointCloud<PointXYZIRT>);
    pcl::ExtractIndices<PointXYZIRT> extract;
    extract.setInputCloud(merged_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground_cloud);

    // Extract non-ground points
    pcl::PointCloud<PointXYZIRT>::Ptr obstacle_cloud(new pcl::PointCloud<PointXYZIRT>);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*obstacle_cloud, output);
    output.header.frame_id = "vlp201";
      
    // Publish extracted point cloud
    pub1.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "input");
    ros::NodeHandle nh;
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);  // pub1 초기화
    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> ("input1", 1000, boost::bind(input, _1, _2));
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("input2", 1000, boost::bind(input, _1, _2));
    ros::spin();
}
