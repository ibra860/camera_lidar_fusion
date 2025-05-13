# ros2_camera_lidar_fusion

## Overview

**ros2_camera_lidar_fusion** is a ROS 2 package for fusing LiDAR point cloud data with RGB camera images. It enables the projection of 3D LiDAR points onto the 2D camera image plane, assigning corresponding pixel colors to each point. This results in a colorized point cloud, which enhances perception in applications such as robotics and autonomous vehicles.

---

## Features

- Projects 3D LiDAR points to 2D image coordinates
- Retrieves pixel color from the camera image and assigns it to corresponding LiDAR points
- Publishes the resulting colorized point cloud as a `sensor_msgs::msg::PointCloud2` message

---

## Configuration

The configuration parameters are defined in `config/fusion_params.yaml`. An example configuration:

```yaml
/**:
  ros__parameters:
    topic:
      pointcloud_in: "/iv_points"       # Input point cloud topic
      camera_image_in: "/cam"           # Camera image topic
      pointcloud_out: "/colored_point"  # Output topic for colorized point cloud
    camera:
      fx: 1527.58
      fy: 1550.17
      cx: 970.83
      cy: 731.92
    transform:
      R_lc: [0.0, 1.0, 0.0, 
             0.0, 0.0, -1.0, 
             1.0, 0.0, 0.0]
      t: [0.0, 0.0, 0.05]
