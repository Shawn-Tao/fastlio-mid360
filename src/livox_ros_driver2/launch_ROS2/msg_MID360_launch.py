"""Standalone Humble driver: CustomMsg is required by MID360 FAST-LIO."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('livox_ros_driver2')) / 'config' / 'MID360.json'
    return LaunchDescription([
        DeclareLaunchArgument('user_config_path', default_value=str(config),
                              description='Absolute path to the Livox MID360 JSON'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                          'publish_freq': 10.0, 'output_data_type': 0,
                          'frame_id': 'livox_frame', 'lvx_file_path': '',
                          'cmdline_input_bd_code': 'livox0000000001',
                          'user_config_path': LaunchConfiguration('user_config_path')}]),
    ])
