#!/usr/bin/env python3
"""LMR2 simulated stereo depth node.

Subscribes to the two Gazebo cameras of lmr2_description, computes disparity
with StereoSGBM + WLS (algorithm from Lenna-Stereo-Camera/stereo_depth_node)
and publishes a metric depth image and a coloured point cloud for RViz2.

Calibration modes (parameter ``calibration_mode``):

* ``camera_info`` (default) - intrinsics from the Gazebo CameraInfo topics and
  left->right extrinsics from TF (URDF). Falls back to ``fallback_hfov`` and
  ``baseline`` if those are not available within the time-outs.
* ``npz`` - a calibration file in the original Calibration/calibration.py
  format (e.g. produced by a checkerboard calibration in simulation).
"""

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

import cv2

from lmr2_stereo_depth.stereo_core import (
    CLOUD_DTYPE,
    StereoCalibration,
    StereoDepth,
    disparity_to_color,
    ideal_camera_matrix,
    make_cloud,
    quaternion_to_matrix,
)


class StereoDepthNode(Node):

    def __init__(self) -> None:
        super().__init__("stereo_depth_node")

        p = self.declare_parameter
        # inputs (lmr2_description topics / frames)
        p("left_topic", "/lmr2/stereo_camera/left/image_raw")
        p("right_topic", "/lmr2/stereo_camera/right/image_raw")
        p("left_info_topic", "/lmr2/stereo_camera/left/camera_info")
        p("right_info_topic", "/lmr2/stereo_camera/right/camera_info")
        p("left_frame", "left_camera_link_optical")
        p("right_frame", "right_camera_link_optical")
        # calibration
        p("calibration_mode", "camera_info")       # camera_info | npz
        p("calibration_file", "")                  # npz mode
        p("calibration_unit", "mm")                # npz mode: unit of T / Q
        p("fallback_hfov", 1.274)                  # rad, from robot.urdf.xacro
        p("baseline", 0.06)                        # m, 2 * camera_offset_y
        p("camera_info_timeout", 5.0)              # s (wall clock)
        p("tf_timeout", 5.0)                       # s (wall clock)
        # algorithm
        p("downscale", 0.5)
        p("num_disparities", 64)
        p("block_size", 7)
        p("use_wls", True)
        p("wls_lambda", 8000.0)
        p("wls_sigma", 1.5)
        p("mask_left_band", True)
        p("sync_queue_size", 10)
        p("sync_slop", 0.02)
        # outputs
        p("depth_topic", "/lmr2/stereo/depth/image_raw")
        p("depth_info_topic", "/lmr2/stereo/depth/camera_info")
        p("points_topic", "/lmr2/stereo/points")
        p("disparity_color_topic", "/lmr2/stereo/disparity/color")
        p("cloud_stride", 2)
        p("min_depth", 0.15)
        p("max_depth", 6.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.left_frame = g("left_frame")
        self.right_frame = g("right_frame")
        self.mode = str(g("calibration_mode")).lower()
        if self.mode not in ("camera_info", "npz"):
            raise ValueError("calibration_mode must be 'camera_info' or 'npz'")

        self.bridge = CvBridge()
        self.estimator = None
        self.left_info = None
        self.right_info = None
        self.t_start = time.monotonic()

        if self.mode == "npz":
            if not g("calibration_file"):
                raise RuntimeError("calibration_mode=npz needs 'calibration_file'")
            self._create_estimator(
                StereoCalibration.from_npz(g("calibration_file"), g("calibration_unit")))
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(CameraInfo, g("left_info_topic"),
                                     self._left_info_cb, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g("right_info_topic"),
                                     self._right_info_cb, qos_profile_sensor_data)

        self.depth_pub = self.create_publisher(Image, g("depth_topic"), 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, g("depth_info_topic"), 5)
        self.points_pub = self.create_publisher(PointCloud2, g("points_topic"), 5)
        self.disp_color_pub = self.create_publisher(Image, g("disparity_color_topic"), 5)

        left_sub = Subscriber(self, Image, g("left_topic"), qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(self, Image, g("right_topic"), qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=int(g("sync_queue_size")),
            slop=float(g("sync_slop")))
        self.sync.registerCallback(self.stereo_callback)

        self.get_logger().info(
            f"Stereo depth node started (calibration_mode={self.mode}); "
            f"inputs: {g('left_topic')} | {g('right_topic')}")

    # ------------------------------------------------------------------ setup
    def _left_info_cb(self, msg: CameraInfo) -> None:
        self.left_info = msg

    def _right_info_cb(self, msg: CameraInfo) -> None:
        self.right_info = msg

    def _create_estimator(self, calib: StereoCalibration) -> None:
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.estimator = StereoDepth(
            calib,
            downscale=float(g("downscale")),
            num_disparities=int(g("num_disparities")),
            block_size=int(g("block_size")),
            use_wls=bool(g("use_wls")),
            wls_lambda=float(g("wls_lambda")),
            wls_sigma=float(g("wls_sigma")),
            mask_left_band=bool(g("mask_left_band")),
        )
        log = self.get_logger()
        log.info(f"Calibration ready - {calib.summary()}")
        log.info(f"Disparity range {self.estimator.num_disparities} px @ downscale "
                 f"{self.estimator.downscale} -> closest depth ~{self.estimator.min_depth_m:.2f} m")
        if self.estimator.wls_unavailable:
            log.warning("cv2.ximgproc not found (OpenCV contrib missing): running SGBM without "
                     "the WLS filter. Install opencv-contrib for the full algorithm.")

    def _try_calibrate_from_ros(self, width: int, height: int) -> bool:
        """camera_info mode: build calibration from CameraInfo + TF (with fallbacks)."""
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        log = self.get_logger()
        elapsed = time.monotonic() - self.t_start

        # --- intrinsics
        if self.left_info is not None and self.right_info is not None:
            K1 = np.array(self.left_info.k, np.float64).reshape(3, 3)
            K2 = np.array(self.right_info.k, np.float64).reshape(3, 3)
            D1, D2 = np.array(self.left_info.d), np.array(self.right_info.d)
            if (self.left_info.width, self.left_info.height) != (width, height):
                log.warning(f"CameraInfo size {self.left_info.width}x{self.left_info.height} "
                         f"differs from image size {width}x{height}")
            if K1[0, 0] <= 0.0 or K2[0, 0] <= 0.0:
                log.warning("CameraInfo K is empty - using fallback_hfov instead")
                K1 = K2 = None
            src = "CameraInfo"
        elif elapsed > float(g("camera_info_timeout")):
            K1 = K2 = None
            src = "fallback_hfov"
        else:
            log.info("Waiting for left/right CameraInfo ...", throttle_duration_sec=2.0)
            return False
        if K1 is None:
            hfov = float(g("fallback_hfov"))
            K1 = K2 = ideal_camera_matrix(width, height, hfov)
            D1 = D2 = np.zeros(5)
            log.warning(f"No usable CameraInfo - using ideal pinhole with hfov={hfov} rad")

        # --- extrinsics: p_right = R p_left + T
        try:
            tf = self.tf_buffer.lookup_transform(self.right_frame, self.left_frame, Time(),
                                                 timeout=Duration(seconds=0.0))
            q, t = tf.transform.rotation, tf.transform.translation
            R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
            T = np.array([t.x, t.y, t.z])
            ext = f"TF {self.left_frame}->{self.right_frame}"
        except TransformException as error:
            if elapsed <= float(g("tf_timeout")):
                log.info(f"Waiting for TF {self.left_frame}->{self.right_frame} ({error})",
                         throttle_duration_sec=2.0)
                return False
            R = np.eye(3)
            T = np.array([-abs(float(g("baseline"))), 0.0, 0.0])
            ext = "baseline parameter"
            log.warning(f"TF unavailable - using baseline={g('baseline')} m, parallel cameras")

        self._create_estimator(StereoCalibration.from_intrinsics(
            K1, D1, K2, D2, (width, height), R, T, source=f"{src} + {ext}"))
        return True

    # --------------------------------------------------------------- callback
    def stereo_callback(self, left_msg: Image, right_msg: Image) -> None:
        try:
            left_bgr = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_bgr = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")

            if self.estimator is None:
                h, w = left_bgr.shape[:2]
                if not self._try_calibrate_from_ros(w, h):
                    return

            result = self.estimator.process(left_bgr, right_bgr)
            header = left_msg.header
            header.frame_id = self.left_frame or header.frame_id

            depth_msg = self.bridge.cv2_to_imgmsg(result.depth_m, encoding="32FC1")
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)
            self.depth_info_pub.publish(self._depth_camera_info(header))

            if self.points_pub.get_subscription_count() > 0:
                self.points_pub.publish(self._cloud_msg(result, header))

            if self.disp_color_pub.get_subscription_count() > 0:
                color = self.bridge.cv2_to_imgmsg(disparity_to_color(result.disparity), "bgr8")
                color.header = header
                self.disp_color_pub.publish(color)

        except CvBridgeError as error:
            self.get_logger().error(f"cv_bridge conversion failed: {error}")
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f"Depth processing failed: {error}",
                                    throttle_duration_sec=2.0)
        except Exception as error:  # keep the node alive on bad frames
            self.get_logger().error(
                f"Unexpected stereo callback error: {type(error).__name__}: {error}",
                throttle_duration_sec=2.0)

    # ---------------------------------------------------------------- outputs
    def _cloud_msg(self, result, header) -> PointCloud2:
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        cloud = make_cloud(result.points_m, result.left_rectified, self.estimator.calib.R1,
                           stride=int(g("cloud_stride")), min_depth=float(g("min_depth")),
                           max_depth=float(g("max_depth")))
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(cloud.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = CLOUD_DTYPE.itemsize
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = cloud.tobytes()
        return msg

    def _depth_camera_info(self, header) -> CameraInfo:
        c = self.estimator.calib
        info = CameraInfo()
        info.header = header
        info.width, info.height = c.width, c.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(v) for v in c.P1[:3, :3].ravel()]
        info.r = [float(v) for v in c.R1.ravel()]
        info.p = [float(v) for v in c.P1.ravel()]
        return info


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StereoDepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
