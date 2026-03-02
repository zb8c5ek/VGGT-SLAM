"""
essn_multicam_slam - High-Level Multi-Camera VGGT-SLAM Orchestration
=====================================================================

Provides ``MultiCamSLAMConfig`` (a plain dataclass that carries
pre-loaded data **and** tuning parameters) and ``run_multicam_slam()``
which executes the full pipeline:

    Phase 1  ->  kern_keyframe.select_keyframes()
    Phase 2  ->  VGGT submap loop  (Solver + VGGT model)

This module never touches the filesystem for discovery — it receives
already-loaded ``ts_to_view_paths`` from the caller (usually the
BURNPIPE script via UTIL_IO_Discovery).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import torch
from tqdm.auto import tqdm

import vggt_slam.slam_utils as utils
from vggt_slam.solver import Solver
from vggt.models.vggt import VGGT

try:
    from .kern_keyframe import select_keyframes
    from .kern_colmap import write_colmap_txt
except ImportError:
    from kern_keyframe import select_keyframes
    from kern_colmap import write_colmap_txt


# ── Configuration dataclass ──────────────────────────────────────────

@dataclass
class MultiCamSLAMConfig:
    """All parameters + pre-loaded data needed to run the multi-cam pipeline."""

    # --- Pre-loaded data (populated by the IO layer) ---
    ts_to_view_paths: Dict[int, Dict[str, str]] = field(default_factory=dict)
    unique_views: List[str] = field(default_factory=list)
    sorted_timestamps: List[int] = field(default_factory=list)

    # --- Camera selection ---
    primary_cam: str = "cam0"

    # --- SLAM parameters ---
    submap_size: int = 4
    overlapping_window_size: int = 1
    max_loops: int = 1
    min_disparity: float = 50.0
    conf_threshold: float = 25.0
    lc_thres: float = 0.95

    # --- Keyframe selection ---
    vis_flow: bool = False

    # --- Output ---
    vis_map: bool = True
    vis_voxel_size: Optional[float] = None
    log_results: bool = False
    skip_dense_log: bool = False
    log_path: str = "poses_multicam.txt"
    colmap_output_path: Optional[str] = None


# ── Pipeline entry point ─────────────────────────────────────────────

def run_multicam_slam(config: MultiCamSLAMConfig) -> Dict[str, Any]:
    """
    Execute the full multi-camera VGGT-SLAM pipeline.

    Returns a dict with timing stats, the solver, and selected keyframes.
    """
    ts_to_view_paths = config.ts_to_view_paths
    unique_views = config.unique_views
    sorted_timestamps = config.sorted_timestamps

    if not sorted_timestamps:
        raise ValueError("[essn] No timestamps in config — did the IO layer load data?")

    # Identify primary camera orientations
    primary_views = sorted(
        [v for v in unique_views if v.startswith(config.primary_cam)]
    )
    if not primary_views:
        raise ValueError(
            f"[essn] No views matching primary_cam='{config.primary_cam}' "
            f"found in unique_views: {unique_views}"
        )

    num_views = len(unique_views)
    frames_per_submap = config.submap_size * num_views + config.overlapping_window_size * num_views

    print(f"[essn] Strategy: keyframe selection on {len(primary_views)} "
          f"orientations of {config.primary_cam}")
    print(f"[essn] Primary views: {primary_views}")
    print(f"[essn] submap_size={config.submap_size} timestamps -> "
          f"~{frames_per_submap} frames per submap")

    # ── Phase 1: keyframe selection ──────────────────────────────────
    print(f"\n[Phase 1] Keyframe selection ({len(sorted_timestamps)} timestamps)...")

    keyframe_timestamps, kf_elapsed = select_keyframes(
        sorted_timestamps,
        ts_to_view_paths,
        primary_views,
        min_disparity=config.min_disparity,
        vis_flow=config.vis_flow,
    )

    print(f"[Phase 1] Selected {len(keyframe_timestamps)} keyframe timestamps "
          f"from {len(sorted_timestamps)} total ({kf_elapsed:.1f}s)")

    if not keyframe_timestamps:
        raise RuntimeError(
            "[essn] No keyframes selected. Try lowering min_disparity."
        )

    # ── Model + solver init ──────────────────────────────────────────
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[essn] Using device: {device}")

    solver = Solver(
        init_conf_threshold=config.conf_threshold,
        lc_thres=config.lc_thres,
        vis_voxel_size=config.vis_voxel_size,
    )

    print("[essn] Loading VGGT model...")
    model = VGGT()
    _URL = "https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt"
    model.load_state_dict(torch.hub.load_state_dict_from_url(_URL))
    model.eval()
    model = model.to(torch.bfloat16)
    model = model.to(device)

    clip_model, clip_preprocess = None, None

    # ── Phase 2: submap processing ───────────────────────────────────
    print(f"\n[Phase 2] Building submaps ({config.submap_size} timestamps x "
          f"{num_views} views = ~{config.submap_size * num_views} frames each)...")

    count = 0
    total_frames = 0
    total_time_start = time.time()
    backend_time = utils.Accumulator()

    kf_window: List[int] = []
    overlap = config.overlapping_window_size

    for i, ts in enumerate(tqdm(keyframe_timestamps, desc="Submap processing")):
        kf_window.append(ts)

        is_last = (i == len(keyframe_timestamps) - 1)
        window_full = len(kf_window) == config.submap_size + overlap

        if window_full or is_last:
            image_names_subset: List[str] = []
            for wts in kf_window:
                view_paths = ts_to_view_paths[wts]
                for view in unique_views:
                    if view in view_paths:
                        image_names_subset.append(view_paths[view])

            if len(image_names_subset) < 2:
                kf_window = kf_window[-overlap:] if overlap > 0 else []
                continue

            count += 1
            total_frames += len(image_names_subset)
            t1 = time.time()
            predictions = solver.run_predictions(
                image_names_subset, model, config.max_loops,
                clip_model, clip_preprocess,
            )
            print(f"  Submap {count}: {len(image_names_subset)} frames, "
                  f"{len(kf_window)} timestamps, "
                  f"solver {time.time() - t1:.2f}s")

            solver.add_points(predictions)

            with backend_time:
                solver.graph.optimize()

            loop_closure_detected = len(predictions["detected_loops"]) > 0
            if loop_closure_detected:
                solver.update_all_submap_vis()
            else:
                solver.update_latest_submap_vis()

            kf_window = kf_window[-overlap:] if overlap > 0 else []

    total_time = time.time() - total_time_start

    # ── Stats ────────────────────────────────────────────────────────
    stats: Dict[str, Any] = {
        "keyframe_timestamps": len(keyframe_timestamps),
        "total_frames": total_frames,
        "total_time": total_time,
        "vggt_time": solver.vggt_timer.total_time,
        "num_submaps": solver.map.get_num_submaps(),
        "num_loop_closures": solver.graph.get_num_loops(),
    }

    if total_frames > 0:
        print(f"\n{'=' * 60}")
        print(f"Keyframe timestamps:  {stats['keyframe_timestamps']}")
        print(f"Total frames fed:     {stats['total_frames']}")
        print(f"Total time:           {stats['total_time']:.2f}s")
        print(f"VGGT calls:           {stats['vggt_time']:.4f}s")
        print(f"Avg VGGT/frame:       {stats['vggt_time'] / total_frames:.4f}s")
        print(f"Avg total/frame:      {total_time / total_frames:.4f}s")
        print(f"Average FPS:          {total_frames / total_time:.2f}")
        print(f"Submaps:              {stats['num_submaps']}")
        print(f"Loop closures:        {stats['num_loop_closures']}")
    else:
        print("[essn] WARNING: no frames were processed")

    if not config.vis_map:
        solver.update_all_submap_vis()

    if config.log_results:
        solver.map.write_poses_to_file(config.log_path, solver.graph, kitti_format=False)
        if not config.skip_dense_log:
            solver.map.save_framewise_pointclouds(
                solver.graph, config.log_path.replace(".txt", "_logs")
            )

    if config.colmap_output_path:
        _export_colmap(solver, config.colmap_output_path)

    return {
        "solver": solver,
        "stats": stats,
        "keyframe_timestamps": keyframe_timestamps,
    }


# ── COLMAP export helper ─────────────────────────────────────────────

def _export_colmap(solver: Solver, output_path: str) -> None:
    """Collect all non-loop-closure submap data and write COLMAP txt."""
    import numpy as np
    from vggt_slam.slam_utils import decompose_camera

    all_poses_c2w = []
    all_image_names = []
    all_intrinsics = []
    all_points = []
    all_colors = []
    img_h, img_w = None, None

    for submap in solver.map.ordered_submaps_by_key():
        if submap.get_lc_status():
            continue

        n_frames = len(submap.poses)

        # Poses: world-to-cam projection mats -> decompose -> cam-to-world
        for idx in range(n_frames):
            node_id = submap.get_id() + idx
            H_world = solver.graph.get_homography(node_id)
            proj_mat = submap.proj_mats[idx] @ np.linalg.inv(H_world)
            proj_mat = proj_mat / proj_mat[-1, -1]

            K, R_w2c, t_w2c, _ = decompose_camera(proj_mat[:3, :])

            # cam-to-world
            c2w = np.eye(4)
            c2w[:3, :3] = R_w2c.T
            c2w[:3, 3] = -R_w2c.T @ t_w2c
            all_poses_c2w.append(c2w)

            K_33 = np.eye(3)
            K_33[0, 0] = abs(K[0, 0])
            K_33[1, 1] = abs(K[1, 1])
            K_33[0, 2] = abs(K[0, 2])
            K_33[1, 2] = abs(K[1, 2])
            all_intrinsics.append(K_33)

        # Image names
        if submap.img_names is not None:
            all_image_names.extend(submap.img_names[:n_frames])
        else:
            all_image_names.extend([f"submap{submap.get_id()}_f{i}" for i in range(n_frames)])

        # Points + colors (confidence-filtered, in world frame)
        pts = submap.get_points_in_world_frame(solver.graph)
        if pts is not None and len(pts) > 0:
            all_points.append(pts)
            cols = submap.get_points_colors()
            if cols is not None:
                cols_filtered = submap.filter_data_by_confidence(submap.colors)
                cols_flat = cols_filtered.reshape(-1, 3)
                all_colors.append(cols_flat[:len(pts)])

        # Grab image dimensions from frames tensor
        if img_h is None and submap.frames is not None:
            img_h = submap.frames.shape[2]
            img_w = submap.frames.shape[3]

    if not all_poses_c2w:
        print("[essn] No poses to export for COLMAP")
        return

    poses_c2w = np.stack(all_poses_c2w)
    intrinsics = np.stack(all_intrinsics)

    if all_points:
        points = np.vstack(all_points)
        colors = np.vstack(all_colors) if all_colors else None
        # Filter NaN/inf
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]
        if colors is not None:
            colors = colors[valid]
        # Subsample if huge
        max_pts = 500_000
        if len(points) > max_pts:
            idx = np.random.choice(len(points), max_pts, replace=False)
            points = points[idx]
            colors = colors[idx] if colors is not None else None
            print(f"[essn] Subsampled COLMAP points: {len(idx)} -> {max_pts}")
    else:
        points = np.zeros((0, 3))
        colors = None

    write_colmap_txt(
        output_path=output_path,
        poses_c2w=poses_c2w,
        image_names=all_image_names,
        intrinsics=intrinsics,
        points=points,
        colors=colors,
        height=img_h or 480,
        width=img_w or 640,
    )
