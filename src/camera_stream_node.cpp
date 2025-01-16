#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraStreamNode : public rclcpp::Node
{
public:
    CameraStreamNode()
        : Node("camera_stream_node")
    {
        // Create an image publisher
        image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");

        // Open the camera (camera index 0 for default camera)
        cap_.open(2); 
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            rclcpp::shutdown();
            return;
        }


        RCLCPP_INFO(this->get_logger(), "Camera stream node started.");

        // Set a timer to periodically capture and publish frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Approximately 30 FPS
            std::bind(&CameraStreamNode::publishFrame, this));
    }

private:
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
        msg->header.stamp = this->now();

        // Publish the image
        image_publisher_.publish(msg);
    }

    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStreamNode>());
    rclcpp::shutdown();
    return 0;
}
