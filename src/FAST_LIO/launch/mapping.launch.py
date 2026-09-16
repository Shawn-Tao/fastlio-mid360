"""FAST-LIO only: delegates validation/map naming to the unified launcher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio'))
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=str(share / 'config')),
        DeclareLaunchArgument('config_file', default_value='mid360.yaml'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'mid360.launch.py')),
            launch_arguments={
                'mode': 'replay',
                'with_driver': 'false',
                'config_file': PathJoinSubstitution([
                    LaunchConfiguration('config_path'), LaunchConfiguration('config_file')]),
            }.items()),
    ])
