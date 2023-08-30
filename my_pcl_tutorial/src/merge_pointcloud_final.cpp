#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#define PI 3.14159265359

using namespace std;
using namespace message_filters;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)

using PointXYZIRT = VelodynePointXYZIRT;
ros::Publisher pub1;

float theta_r180 = 180 * PI / 180;
float theta_r90 = 90 * PI / 180;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
{
    pcl::PointCloud<PointXYZIRT>::Ptr cloud1(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<PointXYZIRT>::Ptr cloud2(new pcl::PointCloud<PointXYZIRT>());

    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_4 = Eigen::Matrix4f::Identity();

    // Define your transformations for cloud1
    transform_1(0,0) = std::cos(theta_r180);
    transform_1(0,1) = -sin(theta_r180);
    transform_1(1,0) = std::sin(theta_r180);
    transform_1(1,1) = std::cos(theta_r180);

    transform_3(0,0) = std::cos(-theta_r90);
    transform_3(0,2) = std::sin(-theta_r90);
    transform_3(2,0) = -sin(-theta_r90);
    transform_3(2,2) = std::cos(-theta_r90);

    transform_4 = transform_1 * transform_3;

    pcl::PointCloud<PointXYZIRT>::Ptr transformed_cloud(new pcl::PointCloud<PointXYZIRT>());

    // Apply transformations to cloud1
    pcl::transformPointCloud(*cloud1, *transformed_cloud, transform_4);

    // Define translation for cloud2
    Eigen::Matrix4f translation_for_cloud2 = Eigen::Matrix4f::Identity();
    translation_for_cloud2(0,3) = -0.18;  // x-axis translation
    translation_for_cloud2(2,3) = 0.14;   // z-axis translation

    // Apply translation to cloud2
    pcl::PointCloud<PointXYZIRT>::Ptr translated_cloud2(new pcl::PointCloud<PointXYZIRT>());
    pcl::transformPointCloud(*cloud2, *translated_cloud2, translation_for_cloud2);

    // Combine the transformed cloud1 with translated cloud2
    pcl::PointCloud<PointXYZIRT>::Ptr combined_cloud(new pcl::PointCloud<PointXYZIRT>());
    *combined_cloud = *transformed_cloud + *translated_cloud2;

    // Z-axis 180-degree rotation matrix
    Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();
    rotation_z(0,0) = -1;
    rotation_z(1,1) = -1;
   
    // Apply Z-axis rotation to the combined point cloud
    pcl::transformPointCloud(*combined_cloud, *combined_cloud, rotation_z);

    // Y-axis 90-degree rotation matrix
    Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();
    rotation_y(0,0) = std::cos(theta_r90);
    rotation_y(0,2) = std::sin(theta_r90);
    rotation_y(2,0) = -std::sin(theta_r90);
    rotation_y(2,2) = std::cos(theta_r90);

    // Apply Y-axis rotation to the combined point cloud
    pcl::transformPointCloud(*combined_cloud, *combined_cloud, rotation_y);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*combined_cloud, output);
    output.header.frame_id = "vlp201";
    pub1.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_merger");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "vlp_1/velodyne_points", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "vlp_2/velodyne_points", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub1 = nh.advertise<sensor_msgs::PointCloud2>("vlp203", 10);

    ros::spin();
    return 0;
}
