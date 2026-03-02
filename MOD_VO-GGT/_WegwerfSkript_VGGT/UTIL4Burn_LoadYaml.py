"""
UTIL4Burn_LoadYaml - YAML Configuration Loader for VGGT-SLAM
=============================================================

Loads and validates a YAML config file, returning a flat dict
ready for constructing ``MultiCamSLAMConfig``.

Usage::

    from UTIL4Burn_LoadYaml import load_vggt_config
    config = load_vggt_config("_configs_vggt/my_run.yaml")
"""

from pathlib import Path
from typing import Any, Dict

import yaml


def load_vggt_config(config_path: str) -> Dict[str, Any]:
    """
    Load VGGT-SLAM configuration from a YAML file.

    Args:
        config_path: Path to the YAML configuration file.

    Returns:
        Flat dict with all configuration values.
    """
    p = Path(config_path)
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")

    with open(p, "r", encoding="utf-8") as f:
        y = yaml.safe_load(f)

    config: Dict[str, Any] = {
        # Paths
        "data_root": y["paths"]["data_root"],

        # Input selection
        "primary_cam": y["input"].get("primary_cam", "cam0"),
        "cameras": y["input"].get("cameras", None),
        "camera_angles": y["input"].get("camera_angles", None),

        # SLAM parameters
        "submap_size": y["slam"].get("submap_size", 4),
        "overlapping_window_size": y["slam"].get("overlapping_window_size", 1),
        "max_loops": y["slam"].get("max_loops", 1),
        "min_disparity": y["slam"].get("min_disparity", 50),
        "conf_threshold": y["slam"].get("conf_threshold", 25.0),
        "lc_thres": y["slam"].get("lc_thres", 0.95),

        # Output
        "vis_map": y.get("output", {}).get("vis_map", True),
        "vis_voxel_size": y.get("output", {}).get("vis_voxel_size", None),
        "vis_flow": y.get("output", {}).get("vis_flow", False),
        "log_results": y.get("output", {}).get("log_results", False),
        "skip_dense_log": y.get("output", {}).get("skip_dense_log", False),
        "log_path": y.get("output", {}).get("log_path", "poses_multicam.txt"),
        "colmap_output_path": y.get("output", {}).get("colmap_output_path", None),
    }

    return config


def print_config_summary(config: Dict[str, Any], config_path: str = None) -> None:
    """Print a formatted summary of the loaded configuration."""
    header = f"Config: {config_path}" if config_path else "Config summary"
    print(f"\n{'═' * 60}")
    print(f"  {header}")
    print(f"{'═' * 60}")
    for key, value in config.items():
        print(f"  {key:30s}: {value}")
    print(f"{'═' * 60}\n")
