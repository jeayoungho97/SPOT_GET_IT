import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory("global_path_manager"),
        "config", "global_path_manager.param.yaml",
    )
    return LaunchDescription([
        Node(
            package="global_path_manager",
            executable="global_path_manager_node",
            name="global_path_manager_node",
            parameters=[param_path],
            output="screen",
        )
    ])
