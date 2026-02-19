# VGGT-SLAM vs MASt3R-SLAM — Detailed Comparison

Both projects are **dense RGB SLAM systems** powered by visual foundation models, but they differ fundamentally in backbone, inference pattern, optimization manifold, and engineering complexity.

---

## 1. Foundation Model Backbone

| | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| **Core Model** | **VGGT** (1B params, Meta) — `facebook/VGGT-1B` | **MASt3R** (ViT-Large, Naver Labs) — built on DUSt3R |
| **Inference Style** | **Batch**: feeds an entire submap of N frames (16–32) in **one** forward pass | **Pairwise**: processes **2 frames** at a time |
| **Outputs** | Pose encodings → (extrinsics, intrinsics), depth map, depth confidence | Per-pixel 3D pointmaps, confidence, dense descriptors |
| **Key Insight** | One VGGT call replaces $O(N^2)$ pairwise MASt3R calls | Must run encoder+decoder for every frame pair |

---

## 2. Overall SLAM Pipeline Architecture

### VGGT-SLAM (Submap-based Feed-forward)
1. **Keyframe selection** via optical flow (Lucas-Kanade, disparity threshold)
2. Accumulate $W$ keyframes into a **submap** (default: 16 new + 1 overlapping)
3. Run VGGT on the entire submap → get all poses + depths in one shot
4. **Scale estimation** between consecutive submaps (median of distance ratios)
5. Add submap nodes/edges to a **pose graph** on the **SL(4) manifold**
6. **Loop closure** via DINO-SALAD retrieval → VGGT verification → inter-submap constraints
7. **Backend optimization** via GTSAM Levenberg-Marquardt on SL(4) between-factors
8. Optional open-set 3D object detection (CLIP + SAM3)

### MASt3R-SLAM (Frame-by-frame Tracking)
1. **Tracking (frontend)**: For each new frame, **MASt3R inference** against last keyframe → pointmaps + correspondences
2. **Keyframe selection** based on match fraction and unique correspondence thresholds
3. **Backend**: For each new keyframe, **symmetric MASt3R inference** against retrieval-selected previous keyframes
4. Build a **factor graph** of pairwise 3D point correspondences
5. **Global optimization** via custom CUDA Gauss-Newton solver on **Sim(3)** (using `lietorch`)
6. **Relocalization** mode when tracking is lost

---

## 3. Tracking / Frontend

| Aspect | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| **Approach** | No per-frame model inference — just **optical flow** (OpenCV LK) for keyframe selection | Every frame → **MASt3R forward pass** + iterative Gauss-Newton pose solve |
| **Keyframe Criterion** | Mean pixel disparity > threshold (default ~50 px) | Match fraction drops below `match_frac_thresh` |
| **Pose Estimation** | Poses come directly from VGGT's feed-forward prediction (no iterative optimization for tracking) | Iterative Gauss-Newton: minimizes ray direction + distance residuals (uncalibrated) or reprojection + log-depth residuals (calibrated) with Huber robust loss |
| **Robustness** | Relies on VGGT's accuracy; confidence-based point filtering | Robust estimation with Huber loss, Cholesky solver, convergence checks. Falls back to **relocalization** mode on tracking failure |
| **Speed Characteristic** | Very fast per-frame (only optical flow); batched VGGT call amortized over entire submap | Every frame requires a MASt3R forward pass (encoder + decoder) |

---

## 4. Mapping / Backend Optimization

| Aspect | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| **Pose Manifold** | **SL(4)** — Special Linear group of 4×4 matrices (homographies with det=1). This captures full projective geometry including intrinsics. | **Sim(3)** — Similarity transformations (rotation + translation + scale), via `lietorch` |
| **Optimization Library** | **GTSAM** (Levenberg-Marquardt with custom SL(4) factors) | **Custom CUDA backend** (`mast3r_slam_backends`) implementing Gauss-Newton |
| **Factor Types** | `PriorFactorSL4` (anchor), `BetweenFactorSL4` (relative constraints) | Pairwise factors based on 3D point correspondences with confidence weighting |
| **Noise Model** | Diagonal noise σ=0.05 for 15-dim SL(4) tangent space; σ=1e-6 for anchor | Configurable σ_ray, σ_dist (uncalibrated) or σ_pixel, σ_depth (calibrated), weighted by match quality |
| **Scale Handling** | Explicit **pairwise scale estimation** between submaps using median of distance ratios | Handled implicitly via Sim(3) — scale is part of the pose |
| **Point Cloud** | Dense per-submap, transformed to world frame via optimized SL(4) homographies | Dense per-keyframe, updated via weighted averaging |

---

## 5. Loop Closure

| Aspect | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| **Retrieval Method** | **DINO-SALAD** — separate pretrained visual place recognition model | **ASMK** with MASt3R's own backbone features + codebook |
| **Matching Strategy** | L2 distance between SALAD descriptors, priority queue of best matches | Top-k retrieval from ASMK inverted file index |
| **Verification** | Re-runs **full VGGT inference** on the pair, checks confidence ≥ 0.85 | Runs **symmetric MASt3R inference**, checks `min_match_frac` |
| **Integration** | Creates a loop closure submap (2 frames), adds inter-submap between-factors, re-optimizes globally | Adds new edges directly between existing keyframes |
| **Relocalization** | Not explicit — loop closure is the recovery mechanism | Explicit **RELOC mode** with state machine |

---

## 6. Build & Dependencies (Critical Differences)

### VGGT-SLAM
| Dependency | Purpose |
|---|---|
| `torch`, `torchvision` | Deep learning framework |
| **`gtsam` (develop branch, with SL(4) support)** | Factor graph optimization — **the hardest dep to install** |
| `vggt` (thirdparty fork) | Meta's VGGT model |
| `salad` (thirdparty) | DINO-SALAD for image retrieval |
| `open3d` | Point cloud processing |
| `viser` | Web-based 3D visualization |
| `opencv-python` | Optical flow |
| Optional: `perception_models` (PE CLIP), `sam3` | Open-set 3D detection |
| **No CUDA extensions to compile** | Pure Python + PyTorch + GTSAM |
| **No `lietorch`** | Does not use Lie group operations via lietorch |

### MASt3R-SLAM
| Dependency | Purpose |
|---|---|
| `torch`, `torchvision`, `xformers` | Deep learning + efficient attention |
| **`lietorch`** | Lie group operations (SE3, Sim3) — **requires CUDA compilation** |
| **`mast3r`** (includes `dust3r`) | MASt3R model — **requires CUDA extension compilation** |
| **`curope`** | CUDA Rotary Position Embeddings — **requires CUDA compilation** |
| **`mast3r_slam_backends`** | Custom CUDA Gauss-Newton solver — **requires CUDA compilation** |
| `rerun-sdk` | 3D visualization |
| `asmk` | Retrieval codebook |

### Build Complexity
| | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| CUDA extensions to compile | **0** | **4** (lietorch, curope, mast3r/dust3r, slam_backends) |
| Build difficulty on Windows | Low–Medium (GTSAM is the only tricky part) | **Very High** (fragile CUDA builds, especially on Windows) |
| Setup script | `setup.sh` — clones repos + pip install | Manual multi-step process with C++/CUDA compilation |

---

## 7. Evaluation Datasets

| Dataset | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| TUM RGB-D | ✅ | ✅ |
| 7-Scenes | ✅ | ✅ |
| EuRoC MAV | ❌ | ✅ |
| ETH3D | ❌ | ✅ |
| Live camera (RealSense/Webcam) | Planned (Jetson Thor) | ✅ |
| Video files (MP4/MOV) | Via ffmpeg preprocessing | ✅ (native) |

VGGT-SLAM's eval scripts explicitly link to MASt3R-SLAM's dataset download instructions — confirming **direct head-to-head benchmarking**.

---

## 8. Code Structure

| Aspect | **VGGT-SLAM** | **MASt3R-SLAM** |
|---|---|---|
| **Package** | `vggt_slam/` — ~10 pure Python modules | `mast3r_slam/` — ~12 modules + CUDA backends |
| **Entry Point** | `main.py` — single-threaded sequential pipeline | `main.py` — supports multiprocessing (separate tracker/backend) |
| **Map** | `Map` containing `Submap` objects; `PoseGraphSolver` wraps GTSAM | `SharedKeyframes` (shared-memory tensors); `FactorGraph` wraps CUDA solver |
| **Frame** | No dedicated frame class; submap batches frames together | `Frame` dataclass with rich state (pose, pointmap, features, confidence) |
| **Config** | CLI `argparse` only | YAML config files with inheritance |
| **Visualization** | `viser` (web-based 3D viewer) | Rerun or OpenGL (`in3d`) |
| **Modularity** | Simpler, more monolithic (`Map` class orchestrates everything) | More modular with explicit state machines (INIT/TRACKING/RELOC/TERMINATED) |

---

## 9. Why VGGT-SLAM Claims to Be Better

1. **Feed-forward efficiency** — One VGGT call replaces $O(N^2)$ MASt3R pairwise inferences per submap. No iterative pose optimization during tracking.

2. **Novel SL(4) manifold optimization** — Optimization on the Special Linear group captures the full projective geometry (including intrinsics) in one variable. This was contributed to the official GTSAM repository (August 2025). MASt3R-SLAM uses the more conventional Sim(3).

3. **Zero CUDA compilation** — Purely Python + PyTorch + GTSAM. MASt3R-SLAM's 4 separate CUDA extension builds are fragile, especially on Windows.

4. **Open-set 3D understanding** — Optional integration with **Perception Encoder CLIP + SAM3** for open-set 3D object detection — a capability MASt3R-SLAM doesn't have.

5. **Simpler codebase** — ~10 pure Python modules, single-threaded pipeline, no shared-memory tensor management or CUDA kernel development.

6. **Publication trajectory** — VGGT-SLAM 1.0 accepted at **NeurIPS 2025**, VGGT-SLAM 2.0 is an **arXiv Jan 2026** preprint. MASt3R-SLAM was published at **CVPR 2025**.

---

## 10. Where MASt3R-SLAM Still Has Advantages

- **Broader dataset/hardware support** — live cameras, video files, EuRoC, ETH3D all ready to go
- **Relocalization** — explicit recovery mode when tracking is lost
- **Multiprocessing** — can run frontend/backend concurrently for real-time performance
- **More mature engineering** — detailed config system (YAML inheritance), robust state machine, shared-memory keyframe management
- **Calibrated + uncalibrated modes** — supports both, with explicit camera intrinsic handling

---

## Summary

| Dimension | VGGT-SLAM | MASt3R-SLAM |
|---|---|---|
| Foundation Model | VGGT (1B params, Meta) | MASt3R (ViT-Large, Naver) |
| Inference Pattern | Batch (submap of N frames) | Pairwise (2 frames) |
| Pose Manifold | SL(4) | Sim(3) |
| Optimization | GTSAM (Levenberg-Marquardt) | Custom CUDA Gauss-Newton |
| Tracking | Optical flow keyframe selection | Dense model-based tracking |
| Loop Closure | DINO-SALAD retrieval | ASMK with MASt3R features |
| CUDA Extensions | ❌ None required | ✅ 4 separate builds |
| Build Difficulty | Low–Medium | High (especially Windows) |
| Venue | NeurIPS 2025 + arXiv 2026 | CVPR 2025 |
| Open-Set Detection | ✅ (CLIP + SAM3) | ❌ |
| Live Camera | Planned (Jetson Thor) | ✅ (RealSense, Webcam) |
| Visualization | viser (web) | Rerun / OpenGL |
