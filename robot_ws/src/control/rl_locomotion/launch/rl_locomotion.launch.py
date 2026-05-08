from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rl_locomotion')

    policy_config = os.path.join(pkg_share, 'config', 'policy_config.yaml')
    command_limit = os.path.join(pkg_share, 'config', 'command_limit.yaml')
    joint_limit = os.path.join(pkg_share, 'config', 'joint_limit.yaml')

    return LaunchDescription([
        Node(
            package='rl_locomotion',
            executable='rl_locomotion_node',
            name='rl_locomotion_node',
            output='screen',
            parameters=[
                policy_config,
                command_limit,
                joint_limit,
            ],
        )
    ])
