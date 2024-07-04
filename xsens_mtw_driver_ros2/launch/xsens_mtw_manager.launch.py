import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


# Keyboard inputs won't work when using this launch file.
# Use the node services instead or manually add the parameters
# in the command line when launching the node. Example:
# ros2 run xsens_mtw_driver_ros2 xsens_mtw_manager --ros-args -p ros2_rate:=100

def generate_launch_description():
    mtw_driver_path = get_package_share_path("xsens_mtw_driver_ros2")
    imu_mapping_config = os.path.join(mtw_driver_path,
                                      'config',
                                      'params.yaml')


    xsens_mtw_manager_node = Node(
        package="xsens_mtw_driver_ros2",
        executable="xsens_mtw_manager",
        name="xsens_mtw_manager",
        output="screen",
        parameters=[imu_mapping_config],
    )


    return LaunchDescription(
        [
            xsens_mtw_manager_node,
        ]
    )
