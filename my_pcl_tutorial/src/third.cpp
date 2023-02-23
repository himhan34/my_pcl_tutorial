#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>


#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZI
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

float theta_r15 =  15* PI/ 180; // 라디안 각도로 회전 (15도 회전)
float theta_r45 =  45* PI/ 180; // 라디안 각도로 회전 (45도 회전)   
float theta_r60 =  60* PI/ 180; // 라디안 각도로 회전 (45도 회전)
float theta_r75 =  75* PI/ 180; // 라디안 각도로 회전 (75도 회전)
float theta_r90 =  90* PI/ 180; // 라디안 각도로 회전 (90도 회전)
float theta_r105 =  105* PI/ 180; // 라디안 각도로 회전 (105도 회전)
float theta_r165 =  165* PI/ 180; // 라디안 각도로 회전 (165도 회전)
float theta_r180 =  180* PI/ 180; // 라디안 각도로 회전 (180도 회전)


void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{

    // Msg to pointcloud
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud(new pcl::PointCloud<VelodynePointXYZIRT>());
    pcl::fromROSMsg(*scan,*cloud); // ros msg 에서 pcl cloud 데이터로 변환

    //회전변환행렬
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_4 = Eigen::Matrix4f::Identity();



    transform_1 (0,0) = std::cos (theta_r180);
    transform_1 (0,1) = -sin(theta_r180);
    transform_1 (1,0) = std::sin (theta_r180);
    transform_1 (1,1) = std::cos (theta_r180);

    transform_2 (0,0) = std::cos (-theta_r90);
    transform_2 (0,2) = std::sin(-theta_r90);
    transform_2 (2,0) = -sin (-theta_r90);
    transform_2 (2,1) = std::cos (-theta_r90);



    transform_4 = transform_2;

    
    //    (row, column)

    // Executing the transformation
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr transformed_cloud (new pcl::PointCloud<PointXYZIRT>());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_4);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_p); 

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "vlp201";
    pub1.publish(output);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "input");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("vlp_2/velodyne_points", 100, input);
	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("vlp203", 100);
	ros::spin();
}


    

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)

/*
    <rotation of x axis>
    transform_1 (1,1) = std::cos (theta_r90);
    transform_1 (1,2) = -sin(theta_r90);
    transform_1 (2,1) = std::sin (theta_r90);
    transform_1 (2,2) = std::cos (theta_r90);

    <rotation of y axis>
    transform_1 (0,0) = std::cos (theta_r90);float theta_r45 =  45* M_PI/ 180; // 라디안 각도로 회전 (45도 회전)
    transform_1 (0,2) = std::sin(theta_r90);
    transform_1 (2,0) = -sin (theta_r90);
    transform_1 (2,1) = std::cos (theta_r90);

    <rotation of z axis>
    transform_2 (0,0) = std::cos (theta_r90);
    transform_2 (0,1) = -sin(theta_r90);
    transform_2 (1,0) = std::sin (theta_r90);
    transform_2 (1,1) = std::cos (theta_r45) */

