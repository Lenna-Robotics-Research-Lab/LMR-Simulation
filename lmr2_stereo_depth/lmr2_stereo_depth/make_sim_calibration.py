#!/usr/bin/env python3
"""Write an ideal stereo calibration (.npz) for the simulated LMR2 cameras.

The file uses the same keys as Lenna-Stereo-Camera/Calibration/calibration.py
(left/right maps, Q, K_*, D_*, R, T, R_left, P_left, ...), so it can be used
by this package (calibration_mode:=npz) and by the real-camera node.

Defaults come from lmr2_description/urdf/robot.urdf.xacro:
  640x480, horizontal_fov 1.274 rad, camera_offset_y 0.030 -> baseline 0.060 m

Example:
  ros2 run lmr2_stereo_depth make_sim_calibration --out ~/lmr2_sim_stereo.npz --unit mm
"""

import argparse

import numpy as np

from lmr2_stereo_depth.stereo_core import StereoCalibration, UNIT_TO_METERS, ideal_camera_matrix


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--hfov", type=float, default=1.274, help="horizontal FOV [rad]")
    ap.add_argument("--baseline", type=float, default=0.06, help="baseline [m]")
    ap.add_argument("--unit", choices=sorted(UNIT_TO_METERS), default="mm",
                    help="unit for T and Q (calibration.py uses mm)")
    ap.add_argument("--out", default="stereo_calibration_sim.npz")
    a = ap.parse_args(argv)

    baseline = a.baseline / UNIT_TO_METERS[a.unit]          # in the chosen unit
    K = ideal_camera_matrix(a.width, a.height, a.hfov)
    R = np.eye(3)
    T = np.array([[-baseline], [0.0], [0.0]])
    calib = StereoCalibration.from_intrinsics(
        K, np.zeros(5), K, np.zeros(5), (a.width, a.height), R, T)

    calib.save_npz(
        a.out,
        K_left=K, D_left=np.zeros((1, 5)), K_right=K, D_right=np.zeros((1, 5)),
        R=R, T=T, R_right=np.eye(3),
        P_right=np.hstack([K, np.array([[-baseline * K[0, 0]], [0.0], [0.0]])]),
    )
    print(f"Saved {a.out}: {a.width}x{a.height}, f={K[0, 0]:.2f}px, "
          f"baseline={baseline:g}{a.unit}")
    print(f"Use: calibration_mode:=npz calibration_file:={a.out} calibration_unit:={a.unit}")


if __name__ == "__main__":
    main()
