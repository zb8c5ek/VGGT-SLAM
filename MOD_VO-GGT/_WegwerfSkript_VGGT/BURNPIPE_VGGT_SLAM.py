#!/usr/bin/env python
"""
BURNPIPE_VGGT_SLAM - Multi-Camera VGGT-SLAM Entry Point
========================================================

Thin entry script that:
  1. Loads a YAML config
  2. Calls UTIL_IO_Discovery to discover and load images
  3. Builds a MultiCamSLAMConfig
  4. Runs the full pipeline via essn_multicam_slam

Usage:
    python BURNPIPE_VGGT_SLAM.py  <config.yaml>
    python BURNPIPE_VGGT_SLAM.py  _configs_vggt/config_template.yaml
"""

import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_MOD_DIR = os.path.dirname(_SCRIPT_DIR)       # MOD_VO-GGT/
_REPO_ROOT = os.path.dirname(_MOD_DIR)         # VGGT-SLAM/

sys.path.insert(0, _REPO_ROOT)   # so `import vggt_slam` works
sys.path.insert(0, _MOD_DIR)     # so `import essn_multicam_slam` etc. works
sys.path.insert(0, _SCRIPT_DIR)  # so `import UTIL4Burn_LoadYaml` etc. works

from UTIL4Burn_LoadYaml import load_vggt_config, print_config_summary
from UTIL_IO_Discovery import load_multicam_data
from essn_multicam_slam import run_multicam_slam, MultiCamSLAMConfig


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {os.path.basename(__file__)}  <config.yaml>")
        sys.exit(1)

    config_path = sys.argv[1]
    if not os.path.isabs(config_path):
        config_path = os.path.join(_SCRIPT_DIR, config_path)

    # 1. Load YAML config
    cfg = load_vggt_config(config_path)
    print_config_summary(cfg, config_path)

    # 2. Discover images via IO layer
    ts_to_view_paths, unique_views, sorted_timestamps = load_multicam_data(
        data_root=cfg["data_root"],
        cameras=cfg["cameras"],
        camera_angles=cfg["camera_angles"],
        primary_cam=cfg["primary_cam"],
    )

    # 3. Build pipeline config
    slam_config = MultiCamSLAMConfig(
        ts_to_view_paths=ts_to_view_paths,
        unique_views=unique_views,
        sorted_timestamps=sorted_timestamps,
        primary_cam=cfg["primary_cam"],
        submap_size=cfg["submap_size"],
        overlapping_window_size=cfg["overlapping_window_size"],
        max_loops=cfg["max_loops"],
        min_disparity=cfg["min_disparity"],
        conf_threshold=cfg["conf_threshold"],
        lc_thres=cfg["lc_thres"],
        vis_flow=cfg["vis_flow"],
        vis_map=cfg["vis_map"],
        vis_voxel_size=cfg["vis_voxel_size"],
        log_results=cfg["log_results"],
        skip_dense_log=cfg["skip_dense_log"],
        log_path=cfg["log_path"],
        colmap_output_path=cfg["colmap_output_path"],
    )

    # 4. Run pipeline
    results = run_multicam_slam(slam_config)

    # 5. Keep viser alive
    print(f"\n=== Viser viewer ready at http://localhost:8080 ===")
    print("Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Shutting down.")


if __name__ == "__main__":
    main()
