/*

@MEZO Check the void load_parameters() function in the SensorFusionNode class. 
This function is used to load parameters from the parameter server. 
The parameters are declared using the declare_parameter() function and then retrieved using the get_parameter() function. 
The parameters include the camera intrinsic parameters (fx, fy, cx, cy), 
the extrinsic parameters (R_lc and t), 
and the topic names for the point cloud input, 
camera image input, and colored point cloud output.



COMPLETE THE REST OF THE LOO

*/



#include "dev_fusion.hpp"

SensorFusionNode::SensorFusionNode()
    : Node("sensor_fusion_node") {
    
    // Create publishers and subscribers
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_point", 10); //pointcloud2 output
    
    pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pointcloud_topic_); //pointcloud2
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, image_topic_); //image

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30), *pointcloud_sub_, *image_sub_);
    sync_->registerCallback(std::bind(&SensorFusionNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Sensor fusion node has been initialized.");
}


//Load Parameters
void FusionNode::load_parameters() {

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
    this->declare_parameter("transform.R_lc", std::vector<double>{0,1,0,0,0,-1,1,0,0}); //will be transformed to matrix3d
    this->declare_parameter("transform.t", std::vector<double>{0.0, 0.0, 0.05}); //will be transformed to vector3d

    this->get_parameter("transform.R_lc", R_vals);
    this->get_parameter("transform.t", t_vals);

    R_lc_ << R_vals[0], R_vals[1], R_vals[2],
             R_vals[3], R_vals[4], R_vals[5],
             R_vals[6], R_vals[7], R_vals[8]; //defined as matrix3d

    t_ << t_vals[0], t_vals[1], t_vals[2]; //defined as vector3d
}


