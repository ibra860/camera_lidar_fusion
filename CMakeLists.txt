cmake_minimum_required(VERSION 3.5)
project(pointcloud_rgb_transform)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# Add executable
add_executable(pointcloud_rgb_transform src/fusion_node.cpp)
add_executable(camera_stream_node src/camera_stream_node.cpp)

# Link libraries and dependencies
ament_target_dependencies(pointcloud_rgb_transform
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
)
ament_target_dependencies(camera_stream_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Include directories
target_include_directories(pointcloud_rgb_transform PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link PCL and OpenCV libraries
target_link_libraries(pointcloud_rgb_transform
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Include directories
target_include_directories(camera_stream_node PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link PCL and OpenCV libraries
target_link_libraries(camera_stream_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


# Install rules
install(TARGETS pointcloud_rgb_transform
  DESTINATION lib/${PROJECT_NAME}
)

# Install rules
install(TARGETS camera_stream_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Add test cases if applicable
endif()

ament_package()
