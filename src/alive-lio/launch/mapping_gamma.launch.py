from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    share = Path(get_package_share_directory('alive_lio'))
    config = Path(LaunchConfiguration('config').perform(context))
    if not config.is_absolute():
        config = share / 'config' / config
    if not config.is_file():
        raise RuntimeError(f'ALIVE-LIO configuration not found: {config}')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(package='alive_lio', executable='alive_lio_node', name='alive_lio_node',
             output='screen', parameters=[str(config), {'use_sim_time': sim_time}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(share / 'config/rviz_cfg/voxel_mapping.rviz')],
             parameters=[{'use_sim_time': sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='geode/avia_gamma.yaml'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
