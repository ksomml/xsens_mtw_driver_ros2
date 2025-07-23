"""Launch file for the xsens_mtw_driver."""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


# Keyboard inputs won't work when using this launch file.
# If you still need them, use the node services instead or
# manually add the parameters in the command line when launching the node.
# Example:
# ros2 run xsens_mtw_driver_ros2 xsens_mtw_manager --ros-args -p ros2_rate:=100

def generate_launch_description() -> LaunchDescription:
    """Launch configuration for the MTw Awinda manager of the xsens_mtw_driver package."""
    mtw_driver_path = get_package_share_path('xsens_mtw_driver')
    ros_config_params = Path(mtw_driver_path) / 'config/params.yaml'

    xsens_mtw_manager_node = Node(
        package='xsens_mtw_driver',
        name='xsens_mtw_manager',
        executable='xsens_mtw_manager',
        output='screen',
        parameters=[ros_config_params],
    )

    return LaunchDescription(
        [
            xsens_mtw_manager_node,
        ]
    )
