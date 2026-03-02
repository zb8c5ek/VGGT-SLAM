"""
UTIL_IO_Discovery - Multi-Camera Image Discovery Data Loader
=============================================================

Discovers, parses, deduplicates and organises multi-camera images
into a timestamp-keyed structure suitable for the VGGT-SLAM pipeline.

When the on-disk data format changes (new naming convention, different
directory layout, etc.) only this file needs updating — kern and essn
layers remain untouched.

Supported layouts
-----------------
Format A (flat cameras):
    <data_root>/ep*_group_*_front/images/cam0_.../*.jpg

Format B (grouped cameras with orientations):
    <data_root>/group_*/cam0/p+0_y+0_r+0/*.jpg
    <data_root>/group_*/cam1/p+0_y+30_r+0/*.jpg

Filename format:  SEQNUM_TS1_TS2_camN[_ORIENTATION].jpg
"""

from __future__ import annotations

import os
import re
import glob
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

# ── Filename regex ────────────────────────────────────────────────────

FNAME_RE = re.compile(
    r"^(\d+)_(\d+)_(\d+)_(cam\d+)(?:_([^.]+))?\.(?:jpg|jpeg|png)$",
    re.IGNORECASE,
)

# Parent directory pattern: camN_ORIENTATION (orientation encoded in dir name)
DIR_CAM_RE = re.compile(r"^(cam\d+)_(.+)$")


def parse_image_filename(path: str):
    """Return (seq_num, timestamp, cam_id, view_id) or None.

    ``view_id`` combines camera and orientation, e.g. ``cam0_p+0_y+30_r+0``.
    For Format A (no orientation suffix), view_id == cam_id.
    """
    name = os.path.basename(path)
    m = FNAME_RE.match(name)
    if m is None:
        return None
    seq = int(m.group(1))
    ts = int(m.group(3))          # TS2 = canonical timestamp
    cam = m.group(4)
    orientation = m.group(5)      # None for format A
    view_id = f"{cam}_{orientation}" if orientation else cam
    return seq, ts, cam, view_id


# ── Glob patterns per layout ─────────────────────────────────────────

def _glob_images(data_root: str) -> List[str]:
    """Try multiple layout patterns and return all matched paths."""
    patterns = [
        # Format B: group_*/cam*/orientation/*.jpg
        os.path.join(data_root, "group_*", "cam*", "*", "*.jpg"),
        os.path.join(data_root, "group_*", "cam*", "*", "*.png"),
        # Single group passed directly: cam*/orientation/*.jpg
        os.path.join(data_root, "cam*", "*", "*.jpg"),
        os.path.join(data_root, "cam*", "*", "*.png"),
        # Format A: ep*/images/cam*/*.jpg
        os.path.join(data_root, "ep*", "images", "cam*", "*.jpg"),
        os.path.join(data_root, "ep*", "images", "cam*", "*.png"),
    ]
    for pat in patterns:
        paths = glob.glob(pat)
        if paths:
            return paths
    return []


# ── Public entry point ────────────────────────────────────────────────

def load_multicam_data(
    data_root: str,
    cameras: Optional[List[str]] = None,
    camera_angles: Optional[Dict[str, List[str]]] = None,
    primary_cam: str = "cam0",
) -> Tuple[Dict[int, Dict[str, str]], List[str], List[int]]:
    """
    Discover images, deduplicate, optionally filter by camera/orientation,
    and return the pipeline-ready data structures.

    Args:
        data_root:      Root directory to search for images.
        cameras:        Whitelist of camera IDs (e.g. ['cam0', 'cam1']).
                        None = accept all cameras.
        camera_angles:  Per-camera angle whitelist, e.g.
                        {'cam0': ['p+0_y+0_r+0', 'p+0_y+30_r+0'],
                         'cam1': ['p+0_y+0_r+0']}.
                        None = accept all angles for every camera.
                        If a camera appears in ``cameras`` but not in
                        ``camera_angles``, all its angles are accepted.
        primary_cam:    Camera prefix used for keyframe selection.

    Returns:
        ts_to_view_paths:  {timestamp -> {view_id -> path}}
        unique_views:      Sorted list of all view_ids present.
        sorted_timestamps: Sorted list of all timestamps.

    Raises:
        FileNotFoundError: If no images are found under data_root.
    """
    raw_paths = _glob_images(data_root)
    if not raw_paths:
        raise FileNotFoundError(
            f"[UTIL_IO_Discovery] No images found under {data_root}"
        )

    seen: Dict[Tuple[int, str], str] = {}
    unparsed: List[str] = []

    for p in raw_paths:
        info = parse_image_filename(p)
        if info is None:
            unparsed.append(p)
            continue
        seq, ts, cam, view_id = info

        # Fallback: if filename has no orientation, check parent directory
        # e.g. parent dir "cam0_p+0_y+30_r+0" -> view_id = "cam0_p+0_y+30_r+0"
        if "_" not in view_id:
            parent_dir = os.path.basename(os.path.dirname(p))
            dir_match = DIR_CAM_RE.match(parent_dir)
            if dir_match and dir_match.group(1) == cam:
                view_id = parent_dir

        # Camera filter
        if cameras is not None and cam not in cameras:
            continue

        # Per-camera angle filter
        if camera_angles is not None and cam in camera_angles:
            parts = view_id.split("_", 1)
            orient = parts[1] if len(parts) > 1 else None
            if orient not in camera_angles[cam]:
                continue

        key = (ts, view_id)
        if key not in seen:
            seen[key] = p

    if unparsed:
        print(f"[UTIL_IO_Discovery] WARNING: {len(unparsed)} files could not be parsed")

    if not seen:
        raise FileNotFoundError(
            f"[UTIL_IO_Discovery] Images found but none matched the camera/orientation filters"
        )

    # Build timestamp -> {view_id -> path}
    ts_to_view_paths: Dict[int, Dict[str, str]] = defaultdict(dict)
    for (ts, view_id), path in seen.items():
        ts_to_view_paths[ts][view_id] = path

    unique_views = sorted({k[1] for k in seen.keys()})
    sorted_timestamps = sorted(ts_to_view_paths.keys())

    # Summary
    primary_views = sorted([v for v in unique_views if v.startswith(primary_cam)])
    print(f"[UTIL_IO_Discovery] Discovered {len(seen)} images "
          f"({len(sorted_timestamps)} timestamps x {len(unique_views)} views)")
    print(f"[UTIL_IO_Discovery] Views: {unique_views}")
    print(f"[UTIL_IO_Discovery] Primary camera views ({primary_cam}): {primary_views}")

    return dict(ts_to_view_paths), unique_views, sorted_timestamps
