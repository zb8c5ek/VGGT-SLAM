"""
kern_keyframe - Multi-Orientation Optical Flow Keyframe Selection
=================================================================

Selects keyframes from a time-ordered sequence of multi-camera images
by running independent optical-flow trackers on each orientation of
a primary camera.  A timestamp becomes a keyframe when ANY orientation
exceeds a minimum mean-pixel disparity threshold.

See _Docs/kerndoc_keyframe.md for algorithm details.

Dependencies: cv2, numpy, vggt_slam.frame_overlap.FrameTracker
              (no torch, no VGGT)
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import cv2

from vggt_slam.frame_overlap import FrameTracker


def select_keyframes(
    sorted_timestamps: Sequence[int],
    ts_to_view_paths: Dict[int, Dict[str, str]],
    primary_views: List[str],
    min_disparity: float = 50.0,
    vis_flow: bool = False,
) -> Tuple[List[int], float]:
    """
    Walk timestamps in order, evaluate optical flow on every primary-cam
    orientation, and return the subset of timestamps selected as keyframes.

    Args:
        sorted_timestamps: Chronologically sorted list of integer timestamps.
        ts_to_view_paths:  {timestamp -> {view_id -> image_path}}.
        primary_views:     Sorted list of view_ids belonging to the primary
                           camera (e.g. ['cam0_p+0_y+0_r+0', 'cam0_p+0_y+30_r+0']).
        min_disparity:     Minimum mean-pixel displacement to trigger a keyframe.
        vis_flow:          If True, show an OpenCV window with tracked flow arrows.

    Returns:
        (keyframe_timestamps, elapsed_seconds)
    """
    import time

    flow_trackers: Dict[str, FrameTracker] = {
        view: FrameTracker() for view in primary_views
    }

    keyframe_timestamps: List[int] = []
    t_start = time.perf_counter()

    for ts in sorted_timestamps:
        view_paths = ts_to_view_paths.get(ts, {})

        any_disparity = False
        for view in primary_views:
            if view not in view_paths:
                continue
            img = cv2.imread(view_paths[view])
            if img is None:
                continue
            if flow_trackers[view].compute_disparity(img, min_disparity, vis_flow):
                any_disparity = True

        if any_disparity:
            keyframe_timestamps.append(ts)

    elapsed = time.perf_counter() - t_start
    return keyframe_timestamps, elapsed
