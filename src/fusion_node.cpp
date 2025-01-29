#include "../include/fusion.hpp"


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