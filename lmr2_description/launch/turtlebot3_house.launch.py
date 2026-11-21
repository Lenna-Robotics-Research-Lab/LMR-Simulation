"""Gazebo Harmonic + ROS 2 Jazzy; use `ros2 launch lmr2_description gazebo.launch.py`.

Default world: turtlebot3_house (from turtlebot3_gazebo).
Use your own world again with:
    ros2 launch lmr2_description turtlebot3_house.launch.py world:=default x:=0.0 y:=0.0 yaw:=1.57
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    package = Path(get_package_share_directory('lmr2_description'))
    gz_pkg = Path(get_package_share_directory('ros_gz_sim'))
    tb3_pkg = Path(get_package_share_directory('turtlebot3_gazebo'))

    # world:=<key>  ->  (sdf file, <world name="..."> inside that file)
    worlds = {
        'turtlebot3_house': (tb3_pkg / 'worlds' / 'turtlebot3_house.world', 'default'),
        'turtlebot3_world': (tb3_pkg / 'worlds' / 'turtlebot3_world.world', 'default'),
        'lmr2':             (package / 'worlds' / 'lmr2_world.sdf', 'lmr2_world'),
    }
    key = LaunchConfiguration('world').perform(context)
    if key not in worlds:
        raise RuntimeError(f"Unknown world '{key}'. Choose one of: {list(worlds)}")
    world_file, world_name = worlds[key]

    urdf = str(package / 'urdf' / 'robot.urdf.xacro')
    description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    rviz_cfg = str(package / 'rviz' / 'gazebo.rviz')

    return [
        # model://turtlebot3_house etc. are resolved from turtlebot3_gazebo/models
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(tb3_pkg / 'models')),
        # parent of the lmr2_description share dir, for package:// mesh URIs
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(package.parent)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_pkg / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f' -r -v 3 {world_file}',
                              'on_exit_shutdown': 'true'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='lmr2_robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', name='spawn_lmr2', output='screen',
             arguments=['-world', world_name, '-name', 'lmr2',
                        '-topic', '/robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='lmr2_bridge',
             output='screen',
             parameters=[{'config_file': str(package / 'config' / 'bridge.yaml'),
                          'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),

        Node(package='ros_gz_image', executable='image_bridge',
             arguments=['/lmr2/stereo_camera/right/image_raw']),
        Node(package='ros_gz_image', executable='image_bridge',
             arguments=['/lmr2/stereo_camera/left/image_raw']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='turtlebot3_house',
                              description='turtlebot3_house | turtlebot3_world | lmr2'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        # Free floor spot inside the house (same spawn point TurtleBot3 uses)
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='-0.5'),
        DeclareLaunchArgument('z', default_value='0.045'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])