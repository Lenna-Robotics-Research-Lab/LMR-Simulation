"""Gazebo Harmonic + ROS 2 Jazzy; use `ros2 launch lmr2_description gazebo.launch.py`."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = Path(get_package_share_directory('lmr2_description'))
    gz_pkg = Path(get_package_share_directory('ros_gz_sim'))
    urdf = str(package / 'urdf' / 'robot.urdf.xacro')
    world = str(package / 'worlds' / 'lmr2_world.sdf')
    description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    rviz_cfg = str(package / 'rviz' / 'gazebo.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.045'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_pkg / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ' -r -v 3 ' + world}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='lmr2_robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_lmr2', output='screen',
             arguments=['-world', 'lmr2_world', '-name', 'lmr2',
                        '-topic', '/robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='lmr2_bridge',
             output='screen', parameters=[{'config_file': str(package / 'config' / 'bridge.yaml'),
                                           'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
