"""Launch classical_gait_node with default config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('classical_gait'),
        'config',
        'classical_gait.param.yaml',
    )

    classical_gait_node = Node(
        package='classical_gait',
        executable='classical_gait_node',
        name='classical_gait_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        classical_gait_node,
    ])
