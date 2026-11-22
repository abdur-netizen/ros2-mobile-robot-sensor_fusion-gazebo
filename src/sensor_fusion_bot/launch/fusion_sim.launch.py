import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion_bot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'fusion_bot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'fusion_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fusion_bot', '-topic', 'robot_description', '-z', '0.05'],
        output='screen',
    )
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )
    fusion_node = Node(
        package='sensor_fusion_bot',
        executable='fusion_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
        bridge,
        fusion_node,
    ])
