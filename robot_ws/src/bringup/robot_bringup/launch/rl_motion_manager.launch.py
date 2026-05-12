from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rl_share = get_package_share_directory('rl_locomotion')
    actuator_share = get_package_share_directory('actuator_bridge')
    motion_share = get_package_share_directory('motion_manager')

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

    motion_param = os.path.join(
        motion_share,
        'config',
        'motion_manager.param.yaml'
    )

    rl_locomotion_node = Node(
        package='rl_locomotion',
        executable='rl_locomotion_node',
        name='rl_locomotion_node',
        output='screen',
        parameters=[rl_param],
    )

    stand_motion_node = Node(
        package='motion_manager',
        executable='stand_motion_node',
        name='stand_motion_node',
        output='screen',
        parameters=[motion_param],
    )

    joint_target_mux_node = Node(
        package='motion_manager',
        executable='joint_target_mux_node',
        name='joint_target_mux_node',
        output='screen',
        parameters=[motion_param],
    )

    actuator_bridge_node = Node(
        package='actuator_bridge',
        executable='actuator_bridge_node',
        name='actuator_bridge_node',
        output='screen',
        parameters=[actuator_param],
    )

    return LaunchDescription([
        rl_locomotion_node,
        stand_motion_node,
        joint_target_mux_node,
        actuator_bridge_node,
    ])