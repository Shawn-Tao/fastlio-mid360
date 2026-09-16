"""Compatibility name for the unified mapping launch (bag recording is opt-in)."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch = Path(get_package_share_directory('fast_lio')) / 'launch' / 'mid360.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch)), launch_arguments={'mode': 'mapping'}.items())])
