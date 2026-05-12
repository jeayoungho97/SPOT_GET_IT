import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("sim_executor")
    param_path = os.path.join(pkg_dir, "config", "sim_executor.param.yaml")

    spot_02 = Node(
        package="sim_executor",
        executable="sim_executor_node",
        name="sim_executor_spot_02",
        parameters=[
            param_path,
            {
                "robot_id": "spot_02",
                "start_x":  2.0,
                "start_y":  2.0,
            },
        ],
        output="screen",
    )

    spot_03 = Node(
        package="sim_executor",
        executable="sim_executor_node",
        name="sim_executor_spot_03",
        parameters=[
            param_path,
            {
                "robot_id": "spot_03",
                "start_x":  3.0,
                "start_y":  1.0,
            },
        ],
        output="screen",
    )

    return LaunchDescription([spot_02, spot_03])
