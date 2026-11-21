"""ROS-independent stereo depth core for the LMR2 simulation.

The matching algorithm is the one from Lenna-Stereo-Camera/ROS/stereo_depth_node:

    remap (rectify) -> gray -> optional downscale -> StereoSGBM (3WAY)
    -> right matcher + WLS filter -> disparity / 16 -> upscale
    -> cv2.reprojectImageTo3D(disparity, Q) -> depth = Z

What changed for simulation is only *where the rectification maps and Q come
from* (see :class:`StereoCalibration`):

* ``from_npz``        - the original .npz format written by Calibration/calibration.py
* ``from_intrinsics`` - K/D of both cameras + R/T between them (Gazebo CameraInfo + TF)
* ``ideal``           - pinhole model from horizontal FOV + baseline (URDF values)

Nothing in this file imports ROS, so it can be unit tested on any machine
with numpy + OpenCV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

UNIT_TO_METERS = {"m": 1.0, "cm": 0.01, "mm": 0.001}


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #
@dataclass
class StereoCalibration:
    """Rectification maps + reprojection matrix for one stereo pair."""

    left_map_x: np.ndarray
    left_map_y: np.ndarray
    right_map_x: np.ndarray
    right_map_y: np.ndarray
    Q: np.ndarray
    # Rotation of the rectified left frame w.r.t. the raw left optical frame.
    R1: np.ndarray
    # Projection matrix of the rectified left image (3x4).
    P1: np.ndarray
    # Multiply Q-units by this to obtain meters.
    to_meters: float = 1.0
    source: str = ""

    REQUIRED_NPZ_KEYS = ("left_map_x", "left_map_y", "right_map_x", "right_map_y", "Q")

    # ----------------------------------------------------------------- props
    @property
    def width(self) -> int:
        return int(self.left_map_x.shape[1])

    @property
    def height(self) -> int:
        return int(self.left_map_x.shape[0])

    @property
    def focal_px(self) -> float:
        return float(self.Q[2, 3])

    @property
    def baseline_m(self) -> float:
        # Q[3,2] = -1/Tx  ->  |Tx| = 1/|Q[3,2]|
        return abs(1.0 / float(self.Q[3, 2])) * self.to_meters

    def summary(self) -> str:
        return (
            f"{self.source}: {self.width}x{self.height}, f={self.focal_px:.1f}px, "
            f"baseline={self.baseline_m * 1000.0:.1f}mm"
        )

    # ------------------------------------------------------------- builders
    @classmethod
    def from_npz(cls, path: str, unit: str = "mm") -> "StereoCalibration":
        """Load a calibration produced by Lenna-Stereo-Camera/Calibration/calibration.py."""
        unit = unit.lower()
        if unit not in UNIT_TO_METERS:
            raise ValueError("calibration_unit must be one of: m, cm, mm")
        calibration_path = Path(path).expanduser()
        if not calibration_path.is_file():
            raise FileNotFoundError(f"Calibration file does not exist: {calibration_path}")

        with np.load(str(calibration_path), allow_pickle=False) as data:
            missing = [k for k in cls.REQUIRED_NPZ_KEYS if k not in data]
            if missing:
                raise KeyError("Calibration file is missing keys: " + ", ".join(missing))
            maps = [data[k].copy() for k in cls.REQUIRED_NPZ_KEYS[:4]]
            Q = data["Q"].astype(np.float64)
            # Optional keys (present in files written by calibration.py).
            R1 = data["R_left"].astype(np.float64) if "R_left" in data else np.eye(3)
            if "P_left" in data:
                P1 = data["P_left"].astype(np.float64)
            else:
                P1 = np.array([[Q[2, 3], 0, -Q[0, 3], 0],
                               [0, Q[2, 3], -Q[1, 3], 0],
                               [0, 0, 1, 0]], dtype=np.float64)

        calib = cls(*maps, Q=Q, R1=R1, P1=P1, to_meters=UNIT_TO_METERS[unit],
                    source=f"npz '{calibration_path.name}'")
        calib._validate()
        return calib

    @classmethod
    def from_intrinsics(
        cls,
        K1: np.ndarray, D1: np.ndarray,
        K2: np.ndarray, D2: np.ndarray,
        image_size: Tuple[int, int],
        R: np.ndarray, T: np.ndarray,
        to_meters: float = 1.0,
        source: str = "intrinsics",
    ) -> "StereoCalibration":
        """Same steps as calibration.py after cv2.stereoCalibrate().

        ``R``/``T`` follow the OpenCV convention: p_right = R @ p_left + T.
        ``image_size`` is (width, height).
        """
        K1, K2 = np.asarray(K1, np.float64), np.asarray(K2, np.float64)
        D1 = _as_distortion(D1)
        D2 = _as_distortion(D2)
        R = np.asarray(R, np.float64).reshape(3, 3)
        T = np.asarray(T, np.float64).reshape(3, 1)
        if np.linalg.norm(T) < 1e-6:
            raise ValueError("Stereo baseline is zero - check the camera frames / TF.")

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, image_size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        lx, ly = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
        rx, ry = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)
        calib = cls(lx, ly, rx, ry, Q=Q, R1=R1, P1=P1, to_meters=to_meters, source=source)
        calib._validate()
        return calib

    @classmethod
    def ideal(cls, width: int, height: int, hfov: float, baseline_m: float) -> "StereoCalibration":
        """Pinhole model of a Gazebo camera (no distortion, parallel cameras)."""
        K = ideal_camera_matrix(width, height, hfov)
        return cls.from_intrinsics(
            K, np.zeros(5), K, np.zeros(5), (width, height),
            np.eye(3), np.array([-abs(baseline_m), 0.0, 0.0]),
            source=f"ideal pinhole (hfov={hfov:.3f} rad)",
        )

    def save_npz(self, path: str, **extra) -> None:
        """Write the original .npz layout (so the file also works with the real-camera node)."""
        np.savez_compressed(
            path,
            image_size=np.array([self.width, self.height]),
            left_map_x=self.left_map_x, left_map_y=self.left_map_y,
            right_map_x=self.right_map_x, right_map_y=self.right_map_y,
            Q=self.Q, R_left=self.R1, P_left=self.P1, **extra,
        )

    def _validate(self) -> None:
        if self.left_map_x.shape != self.left_map_y.shape:
            raise ValueError("Left calibration maps have different shapes")
        if self.right_map_x.shape != self.right_map_y.shape:
            raise ValueError("Right calibration maps have different shapes")
        if self.left_map_x.shape[:2] != self.right_map_x.shape[:2]:
            raise ValueError("Left and right calibration maps have different sizes")
        if self.Q.shape != (4, 4):
            raise ValueError(f"Q must have shape (4, 4), got {self.Q.shape}")


def ideal_camera_matrix(width: int, height: int, hfov: float) -> np.ndarray:
    """Gazebo's pinhole model: fx = fy = (W/2) / tan(hfov/2), principal point at the centre."""
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[f, 0.0, width / 2.0],
                     [0.0, f, height / 2.0],
                     [0.0, 0.0, 1.0]])


def _as_distortion(D) -> np.ndarray:
    D = np.asarray(D if D is not None else [], np.float64).ravel()
    if D.size == 0:
        D = np.zeros(5)
    return D.reshape(1, -1)


# --------------------------------------------------------------------------- #
# Depth estimator (algorithm unchanged from stereo_depth_node.py)
# --------------------------------------------------------------------------- #
@dataclass
class StereoResult:
    left_rectified: np.ndarray   # BGR, full resolution
    disparity: np.ndarray        # float32 px, full resolution, <=0 means invalid
    depth_m: np.ndarray          # float32 m, 0 means invalid
    points_m: np.ndarray         # HxWx3 float32 (rectified-left frame, meters)


class StereoDepth:
    """StereoSGBM + right matcher + WLS depth estimator."""

    def __init__(
        self,
        calibration: StereoCalibration,
        downscale: float = 0.5,
        num_disparities: int = 0,
        block_size: int = 7,
        use_wls: bool = True,
        wls_lambda: float = 8000.0,
        wls_sigma: float = 1.5,
        mask_left_band: bool = True,
    ) -> None:
        if not 0.0 < downscale <= 1.0:
            raise ValueError("downscale must be greater than 0 and at most 1.0")
        if num_disparities <= 0:  # original rule
            num_disparities = 128 if downscale >= 1.0 else 96
        if num_disparities % 16 != 0:
            raise ValueError("num_disparities must be a multiple of 16")
        if block_size % 2 == 0 or block_size < 1:
            raise ValueError("block_size must be odd")

        self.calib = calibration
        self.downscale = float(downscale)
        self.num_disparities = int(num_disparities)
        self.mask_left_band = mask_left_band

        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=block_size,
            P1=8 * block_size**2,
            P2=32 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self.use_wls = bool(use_wls) and hasattr(cv2, "ximgproc")
        self.wls_unavailable = bool(use_wls) and not self.use_wls
        if self.use_wls:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
            self.wls_filter.setLambda(float(wls_lambda))
            self.wls_filter.setSigmaColor(float(wls_sigma))

    @property
    def min_depth_m(self) -> float:
        """Closest measurable depth given the disparity search range."""
        max_disp_full = self.num_disparities / self.downscale
        return self.calib.focal_px * self.calib.baseline_m / max_disp_full

    def process(self, left_bgr: np.ndarray, right_bgr: np.ndarray) -> StereoResult:
        self._validate_input_image(left_bgr, "left")
        self._validate_input_image(right_bgr, "right")
        c = self.calib

        left_rectified = cv2.remap(left_bgr, c.left_map_x, c.left_map_y, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_bgr, c.right_map_x, c.right_map_y, cv2.INTER_LINEAR)

        left_gray = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)

        if self.downscale != 1.0:
            small_left = cv2.resize(left_gray, None, fx=self.downscale, fy=self.downscale,
                                    interpolation=cv2.INTER_AREA)
            small_right = cv2.resize(right_gray, None, fx=self.downscale, fy=self.downscale,
                                     interpolation=cv2.INTER_AREA)
        else:
            small_left, small_right = left_gray, right_gray

        disparity_left = self.left_matcher.compute(small_left, small_right)
        if self.use_wls:
            disparity_right = self.right_matcher.compute(small_right, small_left)
            filtered = self.wls_filter.filter(disparity_left, small_left,
                                              disparity_map_right=disparity_right)
        else:
            filtered = disparity_left
        disparity = filtered.astype(np.float32) / 16.0

        if self.mask_left_band:
            # SGBM cannot match the first numDisparities columns; WLS only smears
            # neighbouring values into that band, which shows up as fake surfaces.
            disparity[:, : self.num_disparities] = 0.0

        if self.downscale != 1.0:
            disparity = cv2.resize(disparity, (left_gray.shape[1], left_gray.shape[0]),
                                   interpolation=cv2.INTER_NEAREST) / self.downscale

        points = cv2.reprojectImageTo3D(disparity, c.Q).astype(np.float32, copy=False)
        if c.to_meters != 1.0:
            points *= np.float32(c.to_meters)
        depth = points[:, :, 2].copy()

        valid = np.isfinite(depth) & (disparity > 0.0) & (depth > 0.0)
        depth[~valid] = 0.0
        points[~valid] = np.nan

        return StereoResult(left_rectified, disparity, depth, points)

    def _validate_input_image(self, image: np.ndarray, side: str) -> None:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"{side} image must be a three-channel BGR image")
        h, w = image.shape[:2]
        if (h, w) != (self.calib.height, self.calib.width):
            raise ValueError(
                f"{side} image is {w}x{h}, but calibration maps are "
                f"{self.calib.width}x{self.calib.height}"
            )


# --------------------------------------------------------------------------- #
# Helpers for the ROS outputs
# --------------------------------------------------------------------------- #
def disparity_to_color(disparity: np.ndarray) -> np.ndarray:
    """Percentile-normalised TURBO colour map (unchanged from the original node)."""
    valid = np.isfinite(disparity) & (disparity > 0.0)
    display = np.zeros(disparity.shape, dtype=np.uint8)
    if np.any(valid):
        values = disparity[valid]
        lo, hi = float(np.percentile(values, 2)), float(np.percentile(values, 98))
        if hi > lo:
            display = (np.clip((disparity - lo) / (hi - lo), 0.0, 1.0) * 255.0).astype(np.uint8)
    display[~valid] = 0
    return cv2.applyColorMap(display, cv2.COLORMAP_TURBO)


CLOUD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])


def make_cloud(
    points_m: np.ndarray,
    color_bgr: np.ndarray,
    R1: Optional[np.ndarray] = None,
    stride: int = 2,
    min_depth: float = 0.1,
    max_depth: float = 6.0,
) -> np.ndarray:
    """Return a structured array (x, y, z, rgb) in the raw left optical frame.

    ``points_m`` is in the *rectified* left frame; rotating by R1^T brings it
    back to the raw left optical frame (identity for the parallel Gazebo cameras).
    """
    stride = max(1, int(stride))
    pts = points_m[::stride, ::stride].reshape(-1, 3)
    col = color_bgr[::stride, ::stride].reshape(-1, 3)

    z = pts[:, 2]
    keep = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    pts, col = pts[keep], col[keep]

    if R1 is not None and not np.allclose(R1, np.eye(3), atol=1e-9):
        pts = (pts @ np.asarray(R1, np.float32))  # == (R1.T @ pts.T).T

    rgb = ((col[:, 2].astype(np.uint32) << 16)
           | (col[:, 1].astype(np.uint32) << 8)
           | col[:, 0].astype(np.uint32))

    cloud = np.empty(pts.shape[0], dtype=CLOUD_DTYPE)
    cloud["x"], cloud["y"], cloud["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    cloud["rgb"] = rgb.view(np.float32)
    return cloud


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
