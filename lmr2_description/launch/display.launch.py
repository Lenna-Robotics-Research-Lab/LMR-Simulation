"""Kinematic-only RViz2 preview with two independent joint sliders (no Gazebo)."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = Path(get_package_share_directory('lmr2_description'))
    description = ParameterValue(Command(['xacro ', str(package / 'urdf' / 'robot.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': description, 'use_sim_time': False}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen', parameters=[{'robot_description': description, 'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', output='screen', name='rviz2',
             arguments=['-d', str(package / 'rviz' / 'display.rviz')],
             parameters=[{'use_sim_time': False}]),
    ])
