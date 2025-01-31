

// #include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode();

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;



    void convert_pointclouds_to_rgb(double x, double y, double z, int& u, int& v);
    
    void synchronized_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

    void load_parameters();


   //Parameters
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d R_lc_;
    Eigen::Vector3d t_;

    //Subscription Topics
    std::string pointcloud_topic_ = "/iv_points";
    std::string image_topic_ = "/camera/image_raw";
    //Publishing Topics
    std::string output_topic_ = "/colored_point";

};