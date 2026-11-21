"""Nav2 bringup for LMR2 (ROS 2 Jazzy + Gazebo Harmonic).

Run AFTER gazebo.launch.py and slam.launch.py (slam_toolbox provides /map and map->odom).

Velocity routing is done entirely in nav2_params.yaml:
    controller_server  -> cmd_vel_nav
    velocity_smoother  -> cmd_vel_smoothed
    collision_monitor  -> /lmr2/cmd_vel   (-> ros_gz_bridge -> Gazebo DiffDrive)

Do NOT wrap this include in a SetRemap('cmd_vel' -> '/lmr2/cmd_vel'):
global remaps are applied before navigation_launch.py's own per-node remaps
(cmd_vel -> cmd_vel_nav), so the controller would publish straight to the robot,
bypassing the velocity smoother and collision monitor, and the smoother would
subscribe to its own downstream output.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_params = PathJoinSubstitution([
        FindPackageShare('lmr2_description'),
        'config',
        'nav2_params.yaml'
    ])

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'log_level': log_level,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo /clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure + activate the Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Nav2 log level'),
        navigation_launch,
    ])