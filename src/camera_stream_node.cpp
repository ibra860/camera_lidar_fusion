#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CameraStreamNode : public rclcpp::Node
{
public:
    CameraStreamNode()
        : Node("camera_stream_node")
    {
        // Create an image publisher
        image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");

        /* It was used because Innovusion lidar counts time separately, so in 
        * the system, lidar time is different than system time. For each 
        * message published by lidar, we only take the header to get time 
        * and set camera time with that time. */
       
        // pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/iv_points", 10, std::bind(&CameraStreamNode::callback, this, std::placeholders::_1));

        // Open the camera (camera index 0 for default camera)
        cap_.open("/dev/video0"); 
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            rclcpp::shutdown();
            return;
        }
        //May I stand unshaken? هل انت 

        RCLCPP_INFO(this->get_logger(), "Camera stream node started.");

        // Set a timer to periodically capture and publish frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // Approximately 30 FPS
            std::bind(&CameraStreamNode::publishFrame, this));
    }

private:

    //save lidar time to set camera time

    // void callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_msg){

    //     lidar_time_ = point_msg->header.stamp;

    // }

    void publishFrame()
    {
        cv::Mat frame;
        cap_ >> frame; // Capture a frame

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame, skipping...");
            return;
        }

        // Convert the frame to a ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        // msg->header.stamp = now();

        // Publish the image
        image_publisher_.publish(msg);
    }

    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Time lidar_time_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStreamNode>());
    rclcpp::shutdown();
    return 0;
}
