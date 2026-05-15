from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rl_share = get_package_share_directory('rl_locomotion')
    actuator_share = get_package_share_directory('actuator_bridge')
    motion_share = get_package_share_directory('motion_manager')

    common_config = os.path.join(
        rl_share,
        'config',
        'common_policy_config.yaml'
    )

    policy_config = os.path.join(
        rl_share,
        'config',
        'policy_exp043.yaml'
    )

    actuator_param = os.path.join(
        actuator_share,
        'config',
        'uart_bridge.param.yaml'
    )

    motion_param = os.path.join(
        motion_share,
        'config',
        'motion_manager.param.yaml'
    )

    common_config_arg = DeclareLaunchArgument(
        'common_config',
        default_value=common_config,
        description='Path to common RL locomotion YAML',
    )

    policy_config_arg = DeclareLaunchArgument(
        'policy_config',
        default_value=policy_config,
        description='Path to model/profile-specific RL policy YAML',
    )

    rl_locomotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rl_share, 'launch', 'rl_locomotion.launch.py')
        ),
        launch_arguments={
            'common_config': LaunchConfiguration('common_config'),
            'policy_config': LaunchConfiguration('policy_config'),
        }.items(),
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
        common_config_arg,
        policy_config_arg,
        rl_locomotion_launch,
        stand_motion_node,
        joint_target_mux_node,
        actuator_bridge_node,
    ])
