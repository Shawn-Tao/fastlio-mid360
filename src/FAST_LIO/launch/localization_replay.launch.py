"""Driver-free localization replay; pass config_file:=mid360.yaml for mapping."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch = Path(get_package_share_directory('fast_lio')) / 'launch' / 'mid360.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='mid360_localization.yaml'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),
                                 launch_arguments={'mode': 'replay'}.items()),
    ])
