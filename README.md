# ros2_camera_lidar_fusion

## 1. Overview

**ros2_camera_lidar_fusion** is a ROS 2 package for fusing LiDAR point cloud data with RGB camera images. It enables the projection of 3D LiDAR points onto the 2D camera image plane, assigning corresponding pixel colors to each point. This results in a colorized point cloud, which enhances perception in applications such as robotics and autonomous vehicles.

---

## 2. Features

- Projects 3D LiDAR points to 2D image coordinates
- Retrieves pixel color from the camera image and assigns it to corresponding LiDAR points
- Publishes the resulting colorized point cloud as a `sensor_msgs::msg::PointCloud2` message

---


## 3. Installtion

Clone the repository:

```
cd ~/ros2_ws/src
git clone https://github.com/ibra860/ros2_camera_lidar_fusion.git

```

## 4. Configuration

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
```

## 5. Transformation and Projection

### a. Transform LiDAR Point to Camera Frame

Each point in the LiDAR frame `P_l` is transformed to the camera frame `P_c` using a rotation matrix `R_lc` and translation vector `t`:

```cpp
P_c = R_lc * (P_l + t);
```
### b. Project to 2D Image Plane

Once in the camera frame, the 3D point is projected onto the 2D image plane using the camera intrinsics
```cpp
u = (fx * P_c.x / P_c.z) + cx;
v = (fy * P_c.y / P_c.z) + cy;
```

- fx, fy: Focal lengths

- cx, cy: Optical center (principal point)

- (u, v): Pixel coordinates

## 6. Color Mapping

If (u, v) falls within the image bounds, the corresponding pixel color is retrieved from the camera image. This color is then assigned to the LiDAR point.

```cpp
cv::Vec3b color_bgr = image.at<cv::Vec3b>(v, u);
```
