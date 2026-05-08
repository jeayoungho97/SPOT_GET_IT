from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rl_share = get_package_share_directory('rl_locomotion')
    actuator_share = get_package_share_directory('actuator_bridge')

    rl_param = os.path.join(
        rl_share,
        'config',
        'policy_config.yaml'
    )

    actuator_param = os.path.join(
        actuator_share,
        'config',
        'spi_bridge.param.yaml'
    )

    actuator_bridge_node = Node(
        package='actuator_bridge',
        executable='actuator_bridge_node',
        name='actuator_bridge_node',
        output='screen',
        parameters=[actuator_param],
    )

    rl_locomotion_node = Node(
        package='rl_locomotion',
        executable='rl_locomotion_node',
        name='rl_locomotion_node',
        output='screen',
        parameters=[rl_param],
    )

    return LaunchDescription([
        actuator_bridge_node,
        rl_locomotion_node,
    ])
