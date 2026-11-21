# LMR2 description — ROS 2 Jazzy / Gazebo Harmonic

**Port of the uploaded `lmr1_description` robot, not a newly designed robot.** This package is named `lmr2_description` but keeps the original LMR1 mechanical geometry. It is targeted to Ubuntu 24.04, ROS 2 **Jazzy**, and **Gazebo Harmonic** (`gz sim`, **not** Gazebo Classic). It provides a 2-wheel differential-drive simulated robot, IMU, LD19-like 2D LiDAR, ROS 2 odometry/joint states/TF, an RViz2 simulation view, and a second RViz2-only joint-slider demo.

**Test status:** All files were statically inspected / parsed in the artifact creation environment, but this environment does not have a ROS 2 / Gazebo installation, so an end-to-end runtime test on Jazzy + Harmonic was **not** possible. Run the validation steps below on your Ubuntu 24.04 machine.

## 1. Install dependencies

A working ROS 2 Jazzy desktop installation must already exist; ROS 2 Jazzy targets Ubuntu 24.04. Use **Harmonic**, not Gazebo Classic or Gazebo Jetty.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-rosdep
```

`ros-jazzy-ros-gz` provides `ros_gz_sim`, `ros_gz_bridge`, and the distro-compatible Gazebo packages. Verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
gz sim --version
```

If `rosdep` has not yet been initialized, initialize it **once** using `sudo rosdep init && rosdep update`.

## 2. Install in a workspace

Extract `lmr2_description.zip`, then place the **one package folder** at `~/lmr2_ws/src/lmr2_description` (not nested one level deeper):

```bash
mkdir -p ~/lmr2_ws/src
# Replace ~/Downloads with the location where you saved the ZIP.
unzip ~/Downloads/lmr2_description.zip -d ~/lmr2_ws/src
cd ~/lmr2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select lmr2_description
source install/setup.bash
ros2 pkg prefix lmr2_description
```

Open a **new terminal** whenever indicated below and source both setup files again, or add them to your shell startup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lmr2_ws/install/setup.bash
```

## 3. Validate the description BEFORE launching Gazebo

```bash
cd ~/lmr2_ws
python3 src/lmr2_description/scripts/check_description.py
xacro src/lmr2_description/urdf/robot.urdf.xacro > /tmp/lmr2.urdf
check_urdf /tmp/lmr2.urdf  # optional: install ros-jazzy-urdf if missing
```

Expected check output: `PASS: generated URDF, frame/joint contract, Gazebo plugins and meshes`.

## 4. Launch simulated robot in Gazebo AND RViz2

**Terminal A**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lmr2_ws/install/setup.bash
ros2 launch lmr2_description gazebo.launch.py
```

This automatically starts the included world (`lmr2_world`), Gazebo Harmonic **running/unpaused**, spawns `lmr2`, starts `robot_state_publisher`, the ROS ↔ Gazebo bridge, and RViz2. In Gazebo you should see the robot on a ground plane and a blue test box about 1.8 m in front of it. Robot is spawned at x=0, y=0, z=0.045 m, yaw=0 by default (adjustable: `x:=1.0 y:=0.0 z:=0.045 yaw:=1.57`). RViz's fixed frame is `odom`.

For Gazebo without RViz2:

```bash
ros2 launch lmr2_description gazebo.launch.py rviz:=false
```

**Terminal B — keyboard teleoperation** (keep this terminal in focus):

```bash
source /opt/ros/jazzy/setup.bash
source ~/lmr2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/lmr2/cmd_vel
```

Use the keys shown by teleop (typically `i` forward, `,` back, `j`/`l` turn; `k` stop). Start at linear speed ~0.2 m/s. The Gazebo DiffDrive plugin acts on the wheel joints; Gazebo's joint-state plugin publishes their measured joint angles; `robot_state_publisher` drives the wheel TFs for RViz2.

**Terminal C — verify topic flow**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lmr2_ws/install/setup.bash
ros2 topic list | grep -E '(clock|tf|joint_states|lmr2|robot_description)'
ros2 topic echo /lmr2/odom --once
ros2 topic echo /joint_states --once
ros2 topic echo /lmr2/scan --once
ros2 topic echo /lmr2/imu --once
ros2 run tf2_ros tf2_echo odom base_link
```

Or send a repeatable Twist command to test without teleop:

```bash
# Press Ctrl+C to stop after checking motion. Do not run this together with teleop.
ros2 topic pub --rate 10 /lmr2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.3}}'
```

Inspect the Gazebo-side topics if data is absent: `gz topic -l | grep lmr2`. The bridge transports `/lmr2/cmd_vel` **ROS → Gazebo**; `/lmr2/odom`, `/lmr2/scan`, `/lmr2/imu`, and `/joint_states` **Gazebo → ROS**. `ros_gz_bridge` also maps Gazebo's `/lmr2/tf` into the ROS `/tf` topic (odom→base_link). ROS `robot_state_publisher` handles base_link→wheels/IMU/LiDAR. `/clock` drives simulation time.

## 5. RViz2 wheel joint slider mode (independent demo)

**Stop the Gazebo launch first** (Ctrl+C in Terminal A) so there is only **one** source of `/joint_states` and one state publisher. Then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lmr2_ws/install/setup.bash
ros2 launch lmr2_description display.launch.py
```

This launches RViz2, `robot_state_publisher`, and the `joint_state_publisher_gui`. Adjust **left_wheel_joint** and **right_wheel_joint** sliders to rotate the virtual wheels in RViz2. This is **kinematic visualization only**: the sliders do **not** actuate the Gazebo physics engine; they only change the joint-state values displayed by RViz2. The robot has no other movable joints; the caster, IMU, and LiDAR joints are fixed.

Do not launch `display.launch.py` concurrently with `gazebo.launch.py` without isolating their ROS namespaces / domain IDs.

## 6. Folder structure

```text
lmr2_description/
├── CMakeLists.txt                 # ament_cmake, installs assets
├── package.xml                    # Jazzy dependencies + Gazebo model search path
├── README.md
├── urdf/
│   ├── robot.urdf.xacro           # main robot + Gazebo Harmonic systems and sensors
│   ├── common/
│   │   ├── properties.xacro       # original dimensions/mass/wheel separation
│   │   └── inertials.xacro        # original analytic inertia macros
│   └── components/
│       ├── base.xacro             # original base link, mesh and collision
│       ├── caster.xacro           # original caster
│       ├── wheel.xacro            # original wheel macro and revolute joints
│       └── sensors.xacro          # original fixed LiDAR/IMU joints
├── meshes/
│   ├── base_light.dae             # original base mesh, unmodified
│   └── wheel_light.dae            # original wheel mesh, unmodified
├── maps/                          # copied historical PGM/YAML maps (not used by launches)
├── legacy_ros1/                   # reference-only original Nav1/SLAM launch/config/RViz files
├── worlds/lmr2_world.sdf          # native Harmonic world + simulation systems
├── config/bridge.yaml            # one-way and bidirectional topic mapping
├── launch/
│   ├── gazebo.launch.py           # Gazebo simulation + RViz2 + bridge
│   └── display.launch.py          # independent joint slider RViz2 mode
├── rviz/
│   ├── gazebo.rviz                # simulation view (odom, robot model, laser, TF)
│   └── display.rviz               # base_link fixed frame and robot model
└── scripts/check_description.py   # source-level / expanded-Xacro smoke test
```

## 7. Migration methodology and evidence

| Element | Based on | Changes |
|---|---|---|
| Robot kinematics / CAD | Your uploaded `lmr1_description/urdf/{robot.urdf.xacro,common,components}` and `meshes/*.dae` | Replaced package URIs with `lmr2_description`, kept wheel radius `0.0325 m`, wheel separation `0.20 m`, base mass `2 kg`, joint / link names and inertias. |
| ROS packaging | ROS 2 `ament_cmake` conventions | Converted Catkin manifests and asset install paths to `ament_cmake` with ament package indexing. |
| Driving / odometry | Gazebo Harmonic's `gz::sim::systems::DiffDrive` and Gazebo's migration documentation | Replaced `libgazebo_ros_diff_drive.so` with Gazebo native DiffDrive. Retained two wheel joints, generated odometry and odom→base TF. |
| Joint states / TF | Gazebo `JointStatePublisher` + ROS 2 `robot_state_publisher` | Physical Gazebo joint states bridged into ROS; no concurrent GUI joint publisher in simulation. |
| LD19-like LiDAR | Original 450-sample, 10 Hz, 0.2–12 m scanner + Gazebo native `gpu_lidar` | Migrated `<ray>` to `<lidar>`, sensor emits `/lmr2/scan`; Gazebo world runs Sensors system. |
| IMU | Original `imu_link`, 100 Hz, ~0.002 noise model + Gazebo native Imu system | Migrated Classic sensor plugin to Gazebo IMU, ROS bridge sends `/lmr2/imu`. |
| World | Gazebo Harmonic SDF world convention | Self-contained test world; Gazebo physics, scene broadcaster, user commands, Sensors and Imu system plugins loaded explicitly. |
| ROS / Gazebo topics | `ros_gz_bridge` YAML schema | Explicit topic types / directions for Twist, odometry, TF, joint states, LaserScan, Imu and Clock. |
| Spawn & RViz2 | `ros_gz_sim create` and ROS 2 Python launch / `robot_state_publisher` | Xacro expands at launch; Gazebo spawns from published `/robot_description`; separate simulator vs. joint sliders. |

Reference documentation: [Gazebo Harmonic: Migrating Gazebo Classic ROS 2 packages](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/), [Gazebo Harmonic: ROS 2 interoperability](https://gazebosim.org/docs/harmonic/ros2_interop/), [Gazebo Harmonic: Sensors](https://gazebosim.org/docs/harmonic/sensors/), [Gazebo Harmonic: ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration/), [ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim), [ROS 2 robot_state_publisher](https://index.ros.org/p/robot_state_publisher/).

## 8. Troubleshooting

| Symptom | Check |
|---|---|
| `Package 'lmr2_description' not found` | `source /opt/ros/jazzy/setup.bash` then `source ~/lmr2_ws/install/setup.bash`; ensure folder is directly inside `src`. |
| `gz` missing or plugins fail to load | `ros-jazzy-ros-gz` and its Gazebo Harmonic dependencies installed; avoid using Gazebo Classic launch commands (`gazebo`, `spawn_model`). |
| Robot visible in RViz but not Gazebo | Look at `spawn_lmr2` output; make sure `lmr2_world` exists, that `robot_description` was published, and no other Gazebo server is running. |
| Robot is black / meshes missing in Gazebo | Verify `GZ_SIM_RESOURCE_PATH` contains workspace install's `share` parent after `source install/setup.bash`. Check URIs are `package://lmr2_description/meshes/...` and mesh files were installed. |
| `/lmr2/cmd_vel` exists, robot not moving | Gazebo must be playing; check `gz topic -l`, the bridge launch log and wheel joint names; run `ros2 topic hz /lmr2/cmd_vel` during teleop. |
| `/joint_states` empty, wheel TF missing | Inspect `gz topic -l | grep joint_states` and the `JointStatePublisher` system; ensure only one `/joint_states` source. |
| No laser in RViz | Check `gz topic -l | grep scan`; ensure `Sensors` loaded and graphics supports Ogre2; set LaserScan QoS `Best Effort`; check laser frame TF. For headless graphics use appropriate software rendering setup. |
| TF shows missing `odom` | `gz topic -l | grep /lmr2/tf` and ROS `/tf`; bridge routes DiffDrive Pose_V -> TFMessage; verify `/clock` and consistent `use_sim_time`. |
| RViz2 joint sliders don't move the robot in Gazebo | Expected. Joint sliders only alter RViz2 kinematics, not simulated effort / velocity controllers. Use teleop for physical Gazebo control. |

### Scope / non-goals

This is **not** a port of the ROS 1 `gmapping`, `hector`, `move_base`, `amcl` or old `costmap` / maps configuration. The original maps were retained under `maps/` and the original ROS 1 launch/config/RViz files are archived unchanged under `legacy_ros1/` for reference, but they **do not run on ROS 2**. ROS 2 Nav2 + slam_toolbox require their **own** navigation configs and maps and are intentionally excluded to avoid misleading ROS 1 compatibility claims. The LiDAR and IMU approximate the original simulated sensors, not hardware-calibrated LD19 / MPU6050 drivers. No hardware control is included.
