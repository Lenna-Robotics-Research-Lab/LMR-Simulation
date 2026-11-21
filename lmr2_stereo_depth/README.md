# lmr2_stereo_depth — stereo depth for the simulated LMR2 (ROS 2 Jazzy / Gazebo Harmonic)

Subscribes to the two Gazebo cameras of `lmr2_description`, computes disparity with the
**same algorithm** as [`Lenna-Stereo-Camera/ROS/stereo_depth_node`](https://github.com/Lenna-Robotics-Research-Lab/Lenna-Stereo-Camera/tree/main/ROS/stereo_depth_node)
(rectify → StereoSGBM 3WAY → right matcher + WLS → `reprojectImageTo3D(Q)`), and publishes a
metric depth image and a coloured point cloud that you can view in RViz2.

```
/lmr2/stereo_camera/left/image_raw  ─┐                         ┌─► /lmr2/stereo/depth/image_raw    (32FC1, m)
/lmr2/stereo_camera/right/image_raw ─┤   stereo_depth_node     ├─► /lmr2/stereo/depth/camera_info
/lmr2/stereo_camera/*/camera_info   ─┤  SGBM + WLS + Q-matrix  ├─► /lmr2/stereo/points            (PointCloud2 xyz+rgb)
/tf_static (left→right optical)     ─┘                         └─► /lmr2/stereo/disparity/color   (bgr8, debug)
```

All outputs use frame `left_camera_link_optical` and the Gazebo sim-time stamps of the left image.

---

## 1. Package layout

```text
lmr2_stereo_depth/
├── package.xml / setup.py / setup.cfg / resource/   # ament_python package
├── lmr2_stereo_depth/
│   ├── stereo_core.py           # algorithm + calibration (no ROS imports, unit-testable)
│   ├── stereo_depth_node.py     # ROS 2 node (subscribe, sync, publish depth + cloud)
│   └── make_sim_calibration.py  # writes an ideal .npz in the calibration.py format
├── config/stereo_depth.yaml     # all parameters, with the camera geometry explained
├── launch/
│   ├── stereo_depth.launch.py   # node + RViz2 + textured targets (Gazebo already running)
│   └── lmr2_stereo_sim.launch.py# one command: lmr2 Gazebo + everything above
├── rviz/stereo_depth.rviz       # robot, camera TFs, point cloud, depth/disparity/left images
├── textures/*.png               # procedural textures for the stereo targets
└── test/test_stereo_core.py     # offline tests with synthetic stereo pairs of known depth
```

## 2. What changed compared to the real-camera node

| Kept (unchanged algorithm) | Removed / simplified for simulation | Added |
|---|---|---|
| `cv2.remap` rectification with maps + `Q` | Mandatory `.npz` file (now optional — maps/Q are built from Gazebo CameraInfo + TF) | Point cloud `PointCloud2` (xyz + rgb) for RViz |
| Gray conversion + `downscale` + `INTER_AREA` | `/stereo/depth/center`, `/stereo/sync/delta_ms` (Gazebo renders both cameras at the same sim time) | `CameraInfo` for the depth image |
| `StereoSGBM` 3WAY, blockSize 7, P1=8·b², P2=32·b², uniqueness 10, speckle 100/2, preFilterCap 63 | Raw-disparity and rectified-left/right topics | RViz2 config + textured stereo targets |
| Right matcher + WLS (λ=8000, σ=1.5) | `example/depth_viewer.py` (RViz replaces it) | `use_wls` fallback if OpenCV contrib is missing |
| `reprojectImageTo3D(disparity, Q)` → Z | `calibration_unit` only matters in npz mode | `mask_left_band` (see below) |
| TURBO disparity colouring, image-size checks, ApproximateTime sync | | Offline tests |

Two small post-processing changes, both to keep the point cloud clean:

* **`mask_left_band`** — SGBM cannot match the first `num_disparities` columns; the WLS filter
  fills that band with smeared values that appear as fake surfaces in 3-D. They are set to invalid.
* **Disparity upscale uses `INTER_NEAREST`** (was `INTER_LINEAR`) — linear interpolation between a
  valid and an invalid (0) pixel creates half-disparity values = points at double distance.

`num_disparities` defaults to **64** (at half resolution), i.e. depths from ~0.2 m. The original
96 would cut 30 % off the left of a 640 px image for a range you don't need with a 6 cm baseline.

## 3. Camera calibration — how it works in simulation

The Gazebo cameras in `robot.urdf.xacro` are **ideal pinhole cameras**: 640×480, `horizontal_fov`
1.274 rad → fx = fy = 320 / tan(0.637) ≈ **432.8 px**, principal point at the centre, no distortion,
and parallel because both have `rpy="0 0.175 0"`. They are `2 × camera_offset_y` = **0.060 m** apart.
So there is nothing to *estimate*, but the node still runs the full calibration path
(K, D, R, T → `stereoRectify` → maps + Q) so that the same code works for real or distorted cameras.
Choose one of the three options:

### A. `calibration_mode: camera_info` (default — nothing to do)

* **Intrinsics K, D** come from `/lmr2/stereo_camera/{left,right}/camera_info` (published by Gazebo).
* **Extrinsics R, T** come from TF `left_camera_link_optical → right_camera_link_optical`
  (the URDF), so if you move/rotate a camera in the xacro the rectification follows automatically.
* Fallbacks after 5 s: `fallback_hfov` (1.274) if no CameraInfo, `baseline` (0.06) if no TF.

The log prints what was used, e.g.
`Calibration ready - CameraInfo + TF left_camera_link_optical->right_camera_link_optical: 640x480, f=432.8px, baseline=60.0mm`.

### B. `calibration_mode: npz` with an ideal file (same format as `Calibration/calibration.py`)

```bash
ros2 run lmr2_stereo_depth make_sim_calibration --out ~/lmr2_sim_stereo.npz --unit mm
ros2 launch lmr2_stereo_depth lmr2_stereo_sim.launch.py \
    calibration_mode:=npz calibration_file:=$HOME/lmr2_sim_stereo.npz calibration_unit:=mm
```

The file contains `left/right_map_x/y`, `Q`, `K_*`, `D_*`, `R`, `T`, `R_left`, `P_left`, so it is
also accepted by the real-camera `stereo_depth_node`.

### C. `calibration_mode: npz` with a real checkerboard calibration done in simulation

Use this to test your calibration pipeline end-to-end:

1. Spawn a checkerboard model in Gazebo (e.g. a thin box with a checkerboard `albedo_map`, same as
   the targets in `launch/stereo_depth.launch.py`) and move it / the robot to ~20 poses.
2. Save image pairs from `/lmr2/stereo_camera/{left,right}/image_raw`
   (e.g. `ros2 run image_view image_saver`, or adapt `stereo_calibration_capture.py` to subscribe).
3. Run `Lenna-Stereo-Camera/Calibration/calibration.py` on them (set `BOARD_SIZE`, `SQUARE_SIZE_MM`).
4. Launch with `calibration_mode:=npz calibration_file:=/abs/path/stereo_calibration.npz calibration_unit:=mm`.
   The result should be ≈ fx 432.8 px, T ≈ (−60, 0, 0) mm, D ≈ 0.

ROS's own `camera_calibration` (`ros2 run camera_calibration cameracalibrator --no-service-check --approximate 0.05 --size 8x6 --square 0.025 left:=/lmr2/stereo_camera/left/image_raw right:=/lmr2/stereo_camera/right/image_raw`) works too; its K/D/R/P can be turned into an npz with `StereoCalibration.from_intrinsics(...)`.

> **Your real IMX219 file cannot be used on the simulator images.** It is 960×540 (the node will
> refuse 640×480 frames) and its optics differ: fx ≈ 1097 px → hfov ≈ 0.825 rad, baseline 60.8 mm,
> noticeable radial distortion.

### Making the simulation match the real IMX219-83 rig (optional)

Edit `lmr2_description/urdf/robot.urdf.xacro` (both camera sensors) and `properties.xacro`:

```xml
<horizontal_fov>0.825</horizontal_fov>            <!-- 2*atan(960 / (2*1097.2)) -->
<image><format>R8G8B8</format><width>960</width><height>540</height></image>
<noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>   <!-- sensor noise -->
```
```xml
<xacro:property name="camera_offset_y" value="0.0304"/>   <!-- half of 60.8 mm -->
```

With option A nothing else changes. Gazebo also supports a `<distortion>` block (k1 k2 k3 p1 p2) if
you want to simulate lens distortion — option A then undistorts using the D in CameraInfo.

## 4. Install, build and run — step by step

Tested target: Ubuntu 24.04, ROS 2 **Jazzy**, Gazebo **Harmonic**, with your `lmr2_description`
already in the workspace (see its README).

### Step 1 — system dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-tf2-ros \
  ros-jazzy-rviz2 python3-numpy python3-opencv python3-pytest python3-colcon-common-extensions
```

Check that OpenCV has the contrib module `ximgproc` (needed for the WLS filter):

```bash
python3 -c "import cv2; print(cv2.__version__, 'ximgproc:', hasattr(cv2, 'ximgproc'))"
```

If it prints `False`, the node still runs (plain SGBM, with a warning). To get WLS:
`sudo apt install -y libopencv-contrib-dev` and re-check; as a last resort
`pip3 install --break-system-packages opencv-contrib-python==4.6.0.66` (keep it on 4.x to stay
compatible with the system NumPy that `cv_bridge` is built against).

### Step 2 — put the package in the workspace

```bash
cd ~/lmr2_ws/src
unzip ~/Downloads/lmr2_stereo_depth.zip      # -> ~/lmr2_ws/src/lmr2_stereo_depth
ls ~/lmr2_ws/src                             # lmr2_description  lmr2_stereo_depth
```

### Step 3 — (recommended) fix one key in lmr2_description/config/bridge.yaml

The two camera_info entries use `ros_gz_image_bridge:` where `ros_topic_name:` is meant. The
bridge usually still works (it reuses `gz_topic_name`), but make it explicit:

```yaml
- ros_topic_name: /lmr2/stereo_camera/right/camera_info
  gz_topic_name: /lmr2/stereo_camera/right/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
- ros_topic_name: /lmr2/stereo_camera/left/camera_info
  gz_topic_name: /lmr2/stereo_camera/left/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
```

(Without CameraInfo the node falls back to `fallback_hfov` after 5 s, so this is not blocking.)
Also add `<exec_depend>ros_gz_image</exec_depend>` to `lmr2_description/package.xml`, since its
launch file uses `ros_gz_image/image_bridge`.

### Step 4 — build

```bash
cd ~/lmr2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select lmr2_description lmr2_stereo_depth
source install/setup.bash
```

### Step 5 — (optional) offline test of the algorithm, no simulator needed

```bash
cd ~/lmr2_ws/src/lmr2_stereo_depth
python3 -m pytest test/ -v        # 12 tests: plane depths 0.3–4 m, npz round trip, cloud, TF convention
```

### Step 6 — run

**Option 1, one command** (Gazebo + robot + bridges + stereo node + stereo RViz + targets):

```bash
source /opt/ros/jazzy/setup.bash && source ~/lmr2_ws/install/setup.bash
ros2 launch lmr2_stereo_depth lmr2_stereo_sim.launch.py
```

**Option 2, two terminals** (if you start the simulation yourself):

```bash
# Terminal A
ros2 launch lmr2_description gazebo.launch.py rviz:=false
# Terminal B
ros2 launch lmr2_stereo_depth stereo_depth.launch.py
```

Useful launch arguments: `spawn_targets:=false`, `rviz:=false`,
`calibration_mode:=npz calibration_file:=/abs/path.npz calibration_unit:=mm`,
`params_file:=/abs/path/my_params.yaml`.

**Drive around** (Terminal C):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/lmr2/cmd_vel
```

### Step 7 — what you should see / verify

In RViz2 (fixed frame `odom`): the robot, coloured 3-D points on the floor mat, box, cylinder,
sphere and backdrop in front of it, plus three image panels (depth, colour disparity, left camera).

```bash
ros2 topic hz /lmr2/stereo_camera/left/image_raw       # ~30 Hz input
ros2 topic hz /lmr2/stereo/depth/image_raw              # output rate (CPU bound, ~15–30 Hz)
ros2 topic echo /lmr2/stereo_camera/left/camera_info --once   # k[0] ≈ 432.8
ros2 run tf2_ros tf2_echo left_camera_link_optical right_camera_link_optical   # x = 0.06
```

Expected accuracy (measured with the offline tests at the default settings): < 1 % up to 2 m,
≈ 5 % at 3–6 m. That is the limit of a 6 cm baseline (disparity = 26/Z px), not of the code.

## 5. Parameters worth tuning (`config/stereo_depth.yaml`)

| Parameter | Default | Effect |
|---|---|---|
| `downscale` | 0.5 | 1.0 = full-res matching, ~3× slower, little accuracy gain here |
| `num_disparities` | 64 | Max disparity at the downscaled size; closest depth = 26 / (num_disparities / downscale) m. Must be a multiple of 16 |
| `block_size` | 7 | Larger = smoother, fewer holes, blurrier edges |
| `use_wls`, `wls_lambda`, `wls_sigma` | true, 8000, 1.5 | WLS smoothing strength / edge sensitivity |
| `cloud_stride` | 2 | Point-cloud subsampling (1 = 307 k points per frame) |
| `min_depth`, `max_depth` | 0.15, 6.0 | Range kept in the point cloud |

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| `Waiting for left/right CameraInfo ...` | `ros2 topic list | grep camera_info`; see Step 3. Falls back to `fallback_hfov` after 5 s. |
| `Waiting for TF ...` | `robot_state_publisher` running? `ros2 run tf2_ros tf2_echo left_camera_link_optical right_camera_link_optical`. Falls back to `baseline` after 5 s. |
| No output at all | Are both images arriving (`ros2 topic hz`)? Is `/clock` published (the node uses sim time)? |
| `image is 640x480, but calibration maps are 960x540` | You passed the real-camera npz — see section 3. |
| Holes on the grey ground / walls | Expected: stereo needs texture. The default world is untextured; the spawned targets are textured. Add `albedo_map` textures to your world for full coverage. |
| Points in the air around object edges | WLS halo at depth discontinuities; lower `wls_lambda` or raise `wls_sigma`. |
| Point cloud not shown in RViz | Topic `/lmr2/stereo/points`, fixed frame `odom`, TF from `odom` to `left_camera_link_optical` must exist (needs the Gazebo bridge + `robot_state_publisher`). |
| Targets not spawned | Gazebo world name must be `lmr2_world` (`world:=` argument), and Gazebo must be running before `stereo_depth.launch.py`. |
| `cv2.ximgproc not found` warning | See Step 1. |
