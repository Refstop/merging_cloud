#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
void syncCallback(const sensor_msgs::PointCloud2ConstPtr& lidar1_msg,
                  const sensor_msgs::PointCloud2ConstPtr& lidar2_msg,
                  const Eigen::Matrix4f& lidar1_to_lidar2) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*lidar1_msg, *cloud1);
    pcl::fromROSMsg(*lidar2_msg, *cloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud2, *transformed_cloud2, lidar1_to_lidar2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *merged_cloud = *cloud1 + *transformed_cloud2;
    sensor_msgs::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    static ros::Publisher pub_merged = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("merged_cloud", 1);
    merged_msg.header.frame_id = "livox_frame";
    merged_msg.header.stamp = lidar1_msg->header.stamp;
    pub_merged.publish(merged_msg);
    // ROS_INFO("Merged point cloud with intensity published.");
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_merging_node");
    ros::NodeHandle nh;
    Eigen::Matrix4f lidar1_to_lidar2 = (Eigen::Matrix4f() <<
        0.9988877,   -0.00413159,  -0.04697848,   0.23684261,
        -0.00472748, -0.9999095, -0.01257858,  0.01228462,
        -0.04692231, 0.01278673, -0.99881654, -0.11065581,
        0., 0., 0., 1.).finished();
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar1_sub(nh, "/livox/lidar_192_168_1_189", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar2_sub(nh, "/livox/lidar_192_168_1_175", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), lidar1_sub, lidar2_sub);
    sync.registerCallback(boost::bind(&syncCallback, _1, _2, boost::ref(lidar1_to_lidar2)));
    ros::spin();
    return 0;
}