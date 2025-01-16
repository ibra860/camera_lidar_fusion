#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>

class CameraStreamNode : public rclcpp::Node {
public:
    CameraStreamNode()
        : Node("camera_stream_node") {
        
        // Create publishers and subscribers
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_stream", 5); //image output
        RCLCPP_INFO(this->get_logger(), "Camera stream node has been started.");
        

    }


private:

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    void publish_image() {
        cv::VideoCapture cap(2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }
        /*
        by : Ibrahim
        This comment must not be removed. 
        For some reason, if it was removed compilation fails. and produces an unknown error.
        
        */
        cv::Mat frame;
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>();
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) {
            cap >> frame;
            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame.");
                break;
            }

            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_link";
            msg->height = frame.rows;
            msg->width = frame.cols;
            msg->encoding = "rgb8";
            msg->is_bigendian = false;
            msg->step = frame.cols * frame.elemSize();
            size_t size = msg->step * frame.rows;
            msg->data.resize(size);
            memcpy(&msg->data[0], frame.data, size);

            image_publisher_->publish(std::move(msg));
            loop_rate.sleep();
        }
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStreamNode>());
    rclcpp::shutdown();
    return 0;
}
