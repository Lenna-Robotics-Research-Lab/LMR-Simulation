"""Stereo depth for an ALREADY RUNNING lmr2 Gazebo simulation.

Start the simulation first (separate terminal):
    ros2 launch lmr2_description gazebo.launch.py

Then:
    ros2 launch lmr2_stereo_depth lmr2_stereo_sim.launch.py

All arguments of stereo_depth.launch.py are forwarded (calibration_mode,
calibration_file, calibration_unit, spawn_targets, rviz, params_file).
Gazebo, robot spawning, robot_state_publisher and the ros_gz bridges are NOT
started here - they come from the already running gazebo.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mine = Path(get_package_share_directory("lmr2_stereo_depth"))

    forwarded = ["calibration_mode", "calibration_file", "calibration_unit",
                 "spawn_targets", "rviz", "params_file"]
    defaults = {"calibration_mode": "camera_info", "calibration_file": "",
                "calibration_unit": "mm", "spawn_targets": "false", "rviz": "false",
                "params_file": str(mine / "config" / "stereo_depth.yaml")}

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=defaults[n]) for n in forwarded]
        + [
            # Gazebo is already up, so no delay is needed.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(mine / "launch" / "stereo_depth.launch.py")),
                launch_arguments={n: LaunchConfiguration(n) for n in forwarded}.items(),
            ),
        ]
    )