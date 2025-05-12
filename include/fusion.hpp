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

    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d R_lc_;
    Eigen::Vector3d t_;
    std::vector<double> R_vals, t_vals;

    std::string pointcloud_topic_ = "/iv_points";
    std::string image_topic_ = "/camera/image_raw";
    std::string output_topic_ = "/colored_point";

    void convert_pointclouds_to_rgb(double x, double y, double z, int& u, int& v);
    void synchronized_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&, 
                             const sensor_msgs::msg::Image::ConstSharedPtr&);
    void load_parameters();
};

SensorFusionNode::SensorFusionNode()
    : Node("sensor_fusion_node") 
{
    // Declare parameters with default values
    this->declare_parameter("topic.pointcloud_in", pointcloud_topic_);
    this->declare_parameter("topic.camera_image_in", image_topic_); 
    this->declare_parameter("topic.pointcloud_out", output_topic_);
    
    this->declare_parameter("camera.fx", 1527.58);
    this->declare_parameter("camera.fy", 1550.17);
    this->declare_parameter("camera.cx", 970.83);
    this->declare_parameter("camera.cy", 731.92);
    
    this->declare_parameter("transform.R_lc", std::vector<double>{0.0, 1.0, 0.0, 
                                                                  0.0, 0.0, -1.0, 
                                                                  1.0, 0.0, 0.0});
    this->declare_parameter("transform.t", std::vector<double>{0.0,0.0,0.05});

    // Get parameters
    this->get_parameter("topic.pointcloud_in", pointcloud_topic_);
    this->get_parameter("topic.camera_image_in", image_topic_);
    this->get_parameter("topic.pointcloud_out", output_topic_);
    
    this->get_parameter("camera.fx", fx_);
    this->get_parameter("camera.fy", fy_);
    this->get_parameter("camera.cx", cx_);
    this->get_parameter("camera.cy", cy_);
    
    this->get_parameter("transform.R_lc", R_vals);
    this->get_parameter("transform.t", t_vals);

    // Initialize transformation matrices
    R_lc_ << R_vals[0], R_vals[1], R_vals[2],
             R_vals[3], R_vals[4], R_vals[5],
             R_vals[6], R_vals[7], R_vals[8];
             
    t_ << t_vals[0], t_vals[1], t_vals[2];

    // Create publishers and subscribers
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    
    pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pointcloud_topic_);
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, image_topic_);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30), *pc_sub_, *image_sub_);
    sync_->registerCallback(std::bind(&SensorFusionNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Sensor fusion node has been started.");
}    

void SensorFusionNode::convert_pointclouds_to_rgb(double x, double y, double z, int& u, int& v) {
    // Camera intrinsic parameters
    double fx = 1527.58;  //focal_length in x
    double fy = 1550.17;  //focal_length in y;
    double cx = 970.83; //CX
    double cy = 731.92; //CY

    // Extrinsic parameters
    Eigen::Matrix3d R_lc;
    R_lc << 0, 1, 0,
            0, 0, -1,
            1, 0, 0;

    Eigen::Vector3d t(0.0, 0.0, 0.05);
    Eigen::Vector3d P_l(x, y, z);
    Eigen::Vector3d P_c = R_lc * (P_l + t);

    // Projection
    u = static_cast<int>((fx * P_c[0] / P_c[2]) + cx);
    v = static_cast<int>((fy * P_c[1] / P_c[2]) + cy);
}

void SensorFusionNode::synchronized_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    std::cout << "Received synchronized messages." << std::endl;
    // Convert ROS image message to OpenCV
    cv::Mat latest_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

    // Convert PointCloud2 to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& point : cloud->points) {
        int u, v;
        convert_pointclouds_to_rgb(point.z, point.y, point.x, u, v); //x and y axis are swapped for Seyond's lidar /////

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
    output_msg.header.frame_id = pointcloud_msg->header.frame_id;
    output_msg.header.stamp = this->get_clock()->now();
    pointcloud_publisher_->publish(output_msg);
}

void SensorFusionNode::load_parameters() {

/*
 * @brief declear and get parametes. will be loaded from launch file
*/

    this->declare_parameter("topic.pointcloud_in", "/iv_points");
    this->declare_parameter("topic.camera_image_in", "/camera/image_raw");
    this->declare_parameter("topic.pointcloud_out", "/colored_point");

    this->get_parameter("topic.pointcloud_in", pointcloud_topic_);
    this->get_parameter("topic.camera_image_in", image_topic_);
    this->get_parameter("topic.pointcloud_out", output_topic_);

    
    this->declare_parameter("camera.fx", 1527.58);
    this->declare_parameter("camera.fy", 1550.17);
    this->declare_parameter("camera.cx", 970.83);
    this->declare_parameter("camera.cy", 731.92);

    this->get_parameter("camera.fx", fx_);
    this->get_parameter("camera.fy", fy_);
    this->get_parameter("camera.cx", cx_);
    this->get_parameter("camera.cy", cy_);

    // Extrinsic Parameters
    std::vector<double> R_vals, t_vals;
    this->declare_parameter("transform.R_lc", std::vector<double>{0.0, 1.0, 0.0, 
                                                                  0.0, 0.0, -1.0, 
                                                                  1.0, 0.0, 0.0}); //will be transformed to matrix3d

    this->declare_parameter("transform.t", std::vector<double>{0.0, 0.0, 0.05}); //will be transformed to vector3d

    this->get_parameter("transform.R_lc", R_vals);
    this->get_parameter("transform.t", t_vals);


    R_lc_ << R_vals[0], R_vals[1], R_vals[2],
             R_vals[3], R_vals[4], R_vals[5],
             R_vals[6], R_vals[7], R_vals[8]; //defined as matrix3d

    t_ << t_vals[0], t_vals[1], t_vals[2]; //defined as vector3d
}