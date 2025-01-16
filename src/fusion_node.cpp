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
    SensorFusionNode()
        : Node("sensor_fusion_node") {
        
        // Create publishers and subscribers
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_point", 5); //pointcloud2 output
        
        pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/simple_drone/scan"); //pointcloud2
        image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/simple_drone/front/image_raw"); //image

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *pc_sub_, *image_sub_);
        sync_->registerCallback(std::bind(&SensorFusionNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Sensor fusion node has been started.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

void synchronized_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    // Convert ROS image message to OpenCV
    cv::Mat latest_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

    // Convert PointCloud2 to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& point : cloud->points) {
        int u, v;
        pcl2pxl(point.x, point.y, point.z, u, v);

        if (u >= 0 && u < latest_image.cols && v >= 0 && v < latest_image.rows) {
            cv::Vec3b color_bgr = latest_image.at<cv::Vec3b>(v, u);

            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = color_bgr[2];
            colored_point.g = color_bgr[1];
            colored_point.b = color_bgr[0];

            colored_cloud->points.push_back(colored_point);
        }
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

    // Convert PCL back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*colored_cloud, output_msg);
    output_msg.header = pointcloud_msg->header;
    pointcloud_publisher_->publish(output_msg);
}


    void pcl2pxl(double x, double y, double z, int& u, int& v) {
        // Camera intrinsic parameters
        double fx = 71.82582983959233;
        double fy = 71.82582983959233;
        double cx = 320.5;
        double cy = 240.5;

        // Extrinsic parameters
        Eigen::Matrix3d R_lc;
        R_lc << 0, -1, 0,
                0, 0, -1,
                1, 0, 0;

        Eigen::Vector3d t(0.0, 0.0, -0.05);
        Eigen::Vector3d P_l(x, y, z);
        Eigen::Vector3d P_c = R_lc * (P_l + t);

        // Projection
        u = static_cast<int>((fx * P_c[0] / P_c[2]) + cx);
        v = static_cast<int>((fy * P_c[1] / P_c[2]) + cy);
    }
};

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create and spin node
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}