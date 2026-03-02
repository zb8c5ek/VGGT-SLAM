"""
MOD_VO-GGT — Multi-Camera Visual Odometry with VGGT-SLAM
=========================================================

Structure
---------
- kern_keyframe:        Per-orientation optical flow keyframe selection
- kern_colmap:          COLMAP text format I/O
- essn_multicam_slam:   High-level multi-camera SLAM orchestration

Usage::

    from MOD_VO_GGT import run_multicam_slam, MultiCamSLAMConfig, select_keyframes

    config = MultiCamSLAMConfig(
        ts_to_view_paths=loaded_data,
        unique_views=views,
        sorted_timestamps=timestamps,
    )
    results = run_multicam_slam(config)
"""

from .kern_keyframe import select_keyframes
from .kern_colmap import write_colmap_txt
from .essn_multicam_slam import run_multicam_slam, MultiCamSLAMConfig

__all__ = [
    # kern_keyframe
    "select_keyframes",
    # kern_colmap
    "write_colmap_txt",
    # essn_multicam_slam
    "run_multicam_slam",
    "MultiCamSLAMConfig",
]
