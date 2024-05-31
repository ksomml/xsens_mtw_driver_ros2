import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mtw_driver_path = get_package_share_path("xsens_mtw_driver_ros2")
    imu_mapping_config = os.path.join(mtw_driver_path,
                                      'config',
                                      'imu_mapping.yaml')


    xsens_mtw_visualization_node = Node(
        package="xsens_mtw_driver_ros2",
        executable="xsens_mtw_visualization",
        name="xsens_mtw_visualization",
        output="screen",
        parameters=[imu_mapping_config],
    )


    return LaunchDescription(
        [
            xsens_mtw_visualization_node,
        ]
    )
