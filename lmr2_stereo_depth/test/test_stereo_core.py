"""Offline tests (no ROS needed):  python3 -m pytest test/ -v  (from the package root)."""

import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lmr2_stereo_depth import make_sim_calibration  # noqa: E402
from lmr2_stereo_depth.stereo_core import (  # noqa: E402
    StereoCalibration, StereoDepth, disparity_to_color, ideal_camera_matrix,
    make_cloud, quaternion_to_matrix,
)

W, H, HFOV, B = 640, 480, 1.274, 0.06
TEXTURE = os.path.join(os.path.dirname(__file__), "..", "textures", "stereo_texture_a.png")


def _texture():
    tex = cv2.imread(TEXTURE)
    assert tex is not None
    return cv2.resize(tex, (W + 200, H))


def _pair_for_plane(z):
    """Left/right images of a fronto-parallel textured plane at depth z."""
    f = ideal_camera_matrix(W, H, HFOV)[0, 0]
    d = f * B / z
    tex = _texture()
    left = tex[:, :W].copy()
    # right(x) = left(x + d)
    xs, ys = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    right = cv2.remap(tex, xs + np.float32(d), ys, cv2.INTER_LINEAR)
    return left, right, d


@pytest.mark.parametrize("z,tol", [(0.3, 0.03), (0.5, 0.03), (1.0, 0.03), (2.0, 0.03), (4.0, 0.08)])
def test_plane_depth(z, tol):
    # 6 cm baseline: d = 26/Z px, so beyond ~3 m a 0.2 px sub-pixel bias is ~5 %.
    est = StereoDepth(StereoCalibration.ideal(W, H, HFOV, B), downscale=0.5, num_disparities=64)
    left, right, _ = _pair_for_plane(z)
    res = est.process(left, right)
    roi = res.depth_m[60:-60, 200:-40]
    valid = roi[roi > 0]
    assert valid.size > 0.8 * roi.size
    assert abs(np.median(valid) - z) / z < tol


def test_left_band_masked_and_min_depth():
    est = StereoDepth(StereoCalibration.ideal(W, H, HFOV, B), downscale=0.5, num_disparities=64)
    left, right, _ = _pair_for_plane(1.0)
    res = est.process(left, right)
    assert np.all(res.depth_m[:, :128] == 0)
    assert est.min_depth_m == pytest.approx(ideal_camera_matrix(W, H, HFOV)[0, 0] * B / 128, rel=1e-3)


def test_npz_roundtrip_mm(tmp_path):
    out = tmp_path / "sim.npz"
    make_sim_calibration.main(["--out", str(out), "--unit", "mm"])
    calib = StereoCalibration.from_npz(str(out), "mm")
    assert calib.baseline_m == pytest.approx(B, rel=1e-6)
    est = StereoDepth(calib, downscale=0.5, num_disparities=64)
    left, right, _ = _pair_for_plane(1.5)
    d = est.process(left, right).depth_m
    assert abs(np.median(d[d > 0]) - 1.5) < 0.05


def test_tf_convention_gives_same_result_as_ideal():
    # lmr2 URDF: left optical origin expressed in right optical frame = (-0.06, 0, 0)
    K = ideal_camera_matrix(W, H, HFOV)
    calib = StereoCalibration.from_intrinsics(
        K, [], K, [0, 0, 0, 0, 0], (W, H), quaternion_to_matrix(0, 0, 0, 1), [-0.06, 0, 0])
    assert np.allclose(calib.R1, np.eye(3), atol=1e-9)
    assert calib.focal_px == pytest.approx(K[0, 0], rel=1e-6)
    assert calib.baseline_m == pytest.approx(0.06)


def test_cloud_and_color():
    est = StereoDepth(StereoCalibration.ideal(W, H, HFOV, B))
    left, right, _ = _pair_for_plane(2.0)
    res = est.process(left, right)
    cloud = make_cloud(res.points_m, res.left_rectified, est.calib.R1, stride=2,
                       min_depth=0.1, max_depth=6.0)
    assert cloud.size > 10000 and cloud.dtype.itemsize == 16
    assert np.all(np.abs(cloud["z"] - 2.0) < 0.15)
    # x spans the right part of the image (left band masked) in the optical frame (x right)
    assert cloud["x"].max() > 0.5
    rgb = cloud["rgb"].view(np.uint32)
    assert rgb.max() <= 0xFFFFFF and rgb.max() > 0
    assert disparity_to_color(res.disparity).shape == (H, W, 3)


def test_wrong_size_rejected():
    est = StereoDepth(StereoCalibration.ideal(W, H, HFOV, B))
    img = np.zeros((240, 320, 3), np.uint8)
    with pytest.raises(ValueError):
        est.process(img, img)


def test_quaternion():
    yaw = 0.7
    R = quaternion_to_matrix(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert np.allclose(R, [[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])


def test_without_wls_fallback():
    est = StereoDepth(StereoCalibration.ideal(W, H, HFOV, B), use_wls=False)
    left, right, _ = _pair_for_plane(1.0)
    d = est.process(left, right).depth_m
    assert abs(np.median(d[d > 0]) - 1.0) < 0.05
