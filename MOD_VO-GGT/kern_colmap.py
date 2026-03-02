"""
kern_colmap - COLMAP Text Format I/O
=====================================

Writes cameras.txt, images.txt, and points3D.txt in the COLMAP text
format so that results can be visualized in COLMAP's GUI or any
compatible viewer (e.g. meshlab, CloudCompare).

Adapted from MOD_Pi3X_VisualOdometry/kern_colmap.py.

Dependencies: numpy, scipy (no torch, no VGGT)
"""

from __future__ import annotations

import os
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_matrix_to_quaternion(R: np.ndarray) -> List[float]:
    """Convert 3x3 rotation matrix to quaternion [qw, qx, qy, qz]."""
    rot = Rotation.from_matrix(R)
    quat = rot.as_quat()  # scipy returns [qx, qy, qz, qw]
    return [float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2])]


def write_cameras_txt(
    output_path: str,
    intrinsics: np.ndarray,
    height: int,
    width: int,
) -> None:
    """
    Write cameras.txt.  One PINHOLE camera per image.

    Args:
        output_path: Directory to write into.
        intrinsics:  (N, 3, 3) per-image intrinsic matrices.
        height:      Image height in pixels.
        width:       Image width in pixels.
    """
    filepath = os.path.join(output_path, "cameras.txt")
    with open(filepath, "w") as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"# Number of cameras: {len(intrinsics)}\n")
        for i, K in enumerate(intrinsics):
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            f.write(f"{i} PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n")


def write_images_txt(
    output_path: str,
    poses_c2w: np.ndarray,
    image_names: List[str],
) -> None:
    """
    Write images.txt.

    Args:
        output_path: Directory to write into.
        poses_c2w:   (N, 4, 4) cam-to-world matrices.
        image_names: List of image filenames / paths.
    """
    filepath = os.path.join(output_path, "images.txt")
    with open(filepath, "w") as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        f.write(f"# Number of images: {len(poses_c2w)}\n")

        for i, (pose, name) in enumerate(zip(poses_c2w, image_names)):
            R_c2w = pose[:3, :3]
            t_c2w = pose[:3, 3]

            # COLMAP stores world-to-camera
            R_w2c = R_c2w.T
            t_w2c = -R_w2c @ t_c2w

            qw, qx, qy, qz = rotation_matrix_to_quaternion(R_w2c)
            tx, ty, tz = t_w2c

            basename = os.path.basename(name)
            f.write(f"{i} {qw} {qx} {qy} {qz} {tx} {ty} {tz} {i} {basename}\n")
            f.write("\n")  # empty POINTS2D line


def write_points3d_txt(
    output_path: str,
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
) -> None:
    """
    Write points3D.txt.

    Args:
        output_path: Directory to write into.
        points:      (N, 3) world-space 3D points.
        colors:      (N, 3) RGB in [0, 255]. Defaults to gray.
    """
    filepath = os.path.join(output_path, "points3D.txt")
    with open(filepath, "w") as f:
        f.write("# 3D point list with one line of data per point:\n")
        f.write("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n")
        f.write(f"# Number of points: {len(points)}\n")
        for i, pt in enumerate(points):
            x, y, z = pt
            if colors is not None:
                r, g, b = int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2])
            else:
                r, g, b = 128, 128, 128
            f.write(f"{i} {x} {y} {z} {r} {g} {b} -1\n")


def write_colmap_txt(
    output_path: str,
    poses_c2w: np.ndarray,
    image_names: List[str],
    intrinsics: np.ndarray,
    points: np.ndarray,
    colors: Optional[np.ndarray],
    height: int,
    width: int,
) -> str:
    """
    Write a complete COLMAP text-format sparse reconstruction.

    Returns the output directory path.
    """
    os.makedirs(output_path, exist_ok=True)
    write_cameras_txt(output_path, intrinsics, height, width)
    write_images_txt(output_path, poses_c2w, image_names)
    write_points3d_txt(output_path, points, colors)
    print(f"[kern_colmap] Wrote COLMAP txt to {output_path} "
          f"({len(poses_c2w)} images, {len(points)} points)")
    return output_path
