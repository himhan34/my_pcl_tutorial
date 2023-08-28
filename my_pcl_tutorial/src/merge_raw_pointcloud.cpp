#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloudCallback(const PointCloud::ConstPtr& cloud1, const PointCloud::ConstPtr& cloud2, ros::Publisher& pub) {
    // PCL 포인트 클라우드 병합 작업 수행
    PointCloud::Ptr mergedCloud(new PointCloud);
    *mergedCloud = *cloud1 + *cloud2;

    // ROS 메시지로 변환
    sensor_msgs::PointCloud2 mergedMsg;
    pcl::toROSMsg(*mergedCloud, mergedMsg);

    // 퍼블리시
    pub.publish(mergedMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_merge_node");
    ros::NodeHandle nh;

    // 수정된 부분: output_topic 이름 설정
    ros::Publisher output_pub = nh.advertise<PointCloud>("output_topic", 1);

    ros::Subscriber sub1 = nh.subscribe<PointCloud>("input_topic1", 1,
        boost::bind(cloudCallback, _1, boost::shared_ptr<PointCloud const>(), boost::ref(output_pub)));
    ros::Subscriber sub2 = nh.subscribe<PointCloud>("input_topic2", 1,
        boost::bind(cloudCallback, boost::shared_ptr<PointCloud const>(), _1, boost::ref(output_pub)));

    ros::spin();

    return 0;
}
