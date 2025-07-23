"""Launch file for the IMU visualization tool."""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch configuration for the MTw visualization node of the xsens_mtw_driver package."""
    mtw_driver_path = get_package_share_path('xsens_mtw_driver')
    ros_config_params = Path(mtw_driver_path) / 'config/params.yaml'

    xsens_mtw_visualization_node = Node(
        package='xsens_mtw_driver',
        name='xsens_mtw_visualization',
        executable='xsens_mtw_visualization',
        output='screen',
        parameters=[ros_config_params],
    )

    return LaunchDescription(
        [
            xsens_mtw_visualization_node,
        ]
    )
