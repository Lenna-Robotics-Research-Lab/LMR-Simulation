"""Stereo depth for an already running lmr2_description Gazebo simulation.

    ros2 launch lmr2_description gazebo.launch.py rviz:=false      # terminal A
    ros2 launch lmr2_stereo_depth stereo_depth.launch.py           # terminal B

Arguments:
    params_file       YAML with node parameters (default: config/stereo_depth.yaml)
    calibration_mode  camera_info | npz
    calibration_file  .npz path (npz mode)
    calibration_unit  m | cm | mm (npz mode)
    rviz              start RViz2 with the stereo view (default true)
    spawn_targets     spawn textured stereo targets in front of the robot (default true)
    world             Gazebo world name (default lmr2_world)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "lmr2_stereo_depth"

# (name, x, y, z, yaw, geometry, texture, has_collision)
# Placed in front of the default spawn pose (0, 0, yaw 0); free area of lmr2_world.
TARGETS = [
    # ("stereo_floor_mat", 1.85, 0.0, 0.0025, 0.0,
    #  "<box><size>3.0 2.4 0.005</size></box>", "stereo_texture_floor_2.jpg", False),
    ("stereo_box", 1.0, 0.45, 0.25, 0.4,
     "<box><size>0.4 0.4 0.5</size></box>", "stereo_texture_a.png", True),
    ("stereo_cylinder", 1.6, -0.45, 0.3, 0.0,
     "<cylinder><radius>0.15</radius><length>0.6</length></cylinder>", "stereo_texture_b.png", True),
    ("stereo_sphere", 2.2, 0.25, 0.2, 0.0,
     "<sphere><radius>0.2</radius></sphere>", "stereo_texture_a.png", True),
    # ("stereo_backdrop", 3.4, 0.0, 0.6, 0.0,
    #  "<box><size>0.05 2.4 1.2</size></box>", "stereo_texture_b.png", True),
]


def _target_sdf(name: str, geometry: str, texture_path: str, collision: bool) -> str:
    col = (f'<collision name="collision"><geometry>{geometry}</geometry></collision>'
           if collision else "")
    return (
        '<?xml version="1.0"?><sdf version="1.9">'
        f'<model name="{name}"><static>true</static><link name="link">{col}'
        f'<visual name="visual"><geometry>{geometry}</geometry>'
        '<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
        '<specular>0.05 0.05 0.05 1</specular>'
        f'<pbr><metal><albedo_map>{texture_path}</albedo_map>'
        '<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>'
        '</material></visual></link></model></sdf>'
    )


def generate_launch_description():
    share = Path(get_package_share_directory(PKG))

    spawners = [
        Node(
            package="ros_gz_sim", executable="create", name=f"spawn_{name}", output="screen",
            arguments=[
                "-world", LaunchConfiguration("world"), "-name", name,
                "-string", _target_sdf(name, geom, str(share / "textures" / tex), col),
                "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw),
            ],
        )
        for name, x, y, z, yaw, geom, tex, col in TARGETS
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "stereo_depth.yaml")),
        DeclareLaunchArgument("calibration_mode", default_value="camera_info",
                              description="camera_info (Gazebo CameraInfo + TF) or npz"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="Absolute path to a stereo .npz (npz mode)"),
        DeclareLaunchArgument("calibration_unit", default_value="mm",
                              description="Unit of T/Q in the .npz: m, cm or mm"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("spawn_targets", default_value="true"),
        DeclareLaunchArgument("world", default_value="lmr2_world"),

        Node(
            package=PKG, executable="stereo_depth_node", name="stereo_depth_node",
            output="screen", emulate_tty=True,
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": True,
                    "calibration_mode": ParameterValue(LaunchConfiguration("calibration_mode"), value_type=str),
                    "calibration_file": ParameterValue(LaunchConfiguration("calibration_file"), value_type=str),
                    "calibration_unit": ParameterValue(LaunchConfiguration("calibration_unit"), value_type=str),
                },
            ],
        ),
        Node(
            package="rviz2", executable="rviz2", name="stereo_rviz", output="screen",
            arguments=["-d", str(share / "rviz" / "stereo_depth.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        GroupAction(spawners, condition=IfCondition(LaunchConfiguration("spawn_targets"))),
    ])
