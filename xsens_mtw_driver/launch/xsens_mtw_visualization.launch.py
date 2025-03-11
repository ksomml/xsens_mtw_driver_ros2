"""Launch file for the IMU visualization tool."""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch configuration for the xsens_mtw_driver's IMU visualization Tool."""
    mtw_driver_path = get_package_share_path('xsens_mtw_driver')
    imu_mapping_config = Path(mtw_driver_path) / 'config/imu_mapping.yaml'

    xsens_mtw_visualization_node = Node(
        package='xsens_mtw_driver',
        name='xsens_mtw_visualization',
        executable='xsens_mtw_visualization',
        output='screen',
        parameters=[imu_mapping_config],
    )

    return LaunchDescription(
        [
            xsens_mtw_visualization_node,
        ]
    )
