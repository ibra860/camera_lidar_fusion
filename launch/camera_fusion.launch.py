from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('camera_lidar_fusion'),
        'config',
        'fusion_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='camera_lidar_fusion',
            executable='camera_lidar_fusion',
            name='sensor_fusion_node',
            parameters=[config_path],
            output='screen'
        )
    ])