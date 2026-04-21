# Rigorous Breakdown of Global Optimization in VGGT-SLAM 2.0

Global optimization in VGGT-SLAM 2.0 acts as the backend for bundle adjustment and scale-drift correction. Instead of typical rigid body $SE(3)$ or similarity $Sim(3)$ parameterizations, it performs Pose Graph Optimization (PGO) on the $SL(4)$ manifold (the Special Linear group of $4 \times 4$ matrices with unit determinant). This handles scale ambiguities inherent to purely monocular, dense feed-forward reconstruction intrinsically and gracefully.

Below is a breakdown of the mathematical intuition, expected data structures, code implementation, and algorithmic flow.

---

## 1. Expected Data Model

The optimization pipeline acts upon neural network predictions. The VGGT network outputs batched dense properties for a local _submap_ (a small window of frames $S$). These raw predictions look like this:

```python
# Expected dictionary structure produced by VGGT network:
pred_dict = {
    "images": (S, 3, H, W),              # Preprocessed source images
    "depth": (S, H, W, 1),               # Dense depth inferences
    "depth_conf": (S, H, W),             # Confidence maps
    "extrinsic": (S, 3, 4),              # Network-inferred SE(3) equivalents
    "intrinsic": (S, 3, 3),              # Respective camera intrinsics
    "detected_loops": [LoopMatch(...)]   # Predicted loop closure associations
}
```

In `solver.py` (`add_points` method), these outputs are decoupled. Depths are unprojected into `world_points` scaled by local estimations. Extrinsics are inverted into $4 \times 4$ `cam_to_world` matrices, setting up standard transformations that the factor graph will digest.

---

## 2. Global Graph Formulation (Nodes and Factors)

With dense point clouds formed, the solver organizes constraints. The solver populates a `gtsam.NonlinearFactorGraph` via the `PoseGraph` wrapper class located in `vggt_slam/graph.py`.

### **Nodes: $SL(4)$ Variables**
Each camera pose in the environment is added as a node parameterized by an `SL4` element. It acts as an affine transformation, accounting for both pose estimation and scale.

```python
def add_homography(self, key, global_h):
    """Add a new homography node to the graph."""
    key = X(key) # Create a GTSAM distinct symbol
    self.values.insert(key, SL4(global_h))
    self.initialized_nodes.add(key)
```

### **Edges: BetweenFactors and Scale Pairwise Alignment**
Edges specify the geometric transformations mapping one node to another. Because scale drifts locally, VGGT-SLAM aligns overlapping frames from consecutive submaps to compute a local correcting scale.

```python
# Found in solver.py -> add_edge()
# 1. Isolate high-confidence robust point matches between overlapping frames
good_mask = (prior_conf > prior_submap.get_conf_threshold()) & (current_conf > prior_submap.get_conf_threshold())

# 2. Extract corresponding 3D points
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]

# 3. Estimate scaling based on distance variances
scale_factor_est_output = estimate_scale_pairwise(t1, t2)
scale_factor = scale_factor_est_output[0]

# 4. Formulate the scaled homography constraint
H_scale = np.diag((scale_factor, scale_factor, scale_factor, 1.0))
H_overlap_prior_overlap_current = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0] @ H_scale
```

Once the corrected local transform (`H_overlap_prior_overlap_current`) is prepared, it is added to the graph with a static uniform diagonal noise model:

```python
def add_between_factor(self, key1, key2, relative_h, noise):
    """Add a relative SL4 constraint between two nodes."""
    self.graph.add(gtsam.BetweenFactorSL4(key1, key2, SL4(relative_h), noise))
```
*Note: In `PoseGraph`, the solver introduces `inner_submap_noise` for frame-by-frame edges, `intra_submap_noise` for overlapping submap connecting edges, and constraints for potential loop closures.*

---

## 3. The Global Optimization Invocation

Every time a submap is finalized inside the main tracking loop (`main.py`), the optimization is systematically triggered. 

```python
# main.py running sequentially over subsets of tracking images ...
solver.add_points(predictions) # Decouples dense geometries & populates graph
with backend_time:
    solver.graph.optimize()    # LM calculation executed
```

### **Algorithmic Routine (`graph.py -> optimize`)**
Under the hood, GTSAM configures bounds and solves for optimal variable states relative to the cumulative factor weights:

```python
def optimize(self, verbose=False):
    """Optimize the graph with Levenberg–Marquardt and print per-factor errors."""
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)

    # Optimizes Levenberg-Marquardt across the SL(4) sub-manifold constraint mappings
    result = optimizer.optimize()

    # Store optimized values back to graph values for subsequent rendering/querying
    self.values = result
```

This procedure absorbs discrepancies built up by visual odometry bias. Finally, the newly optimized global sequence is visually updated (`solver.update_all_submap_vis`), which un-projects the cached maps back into the viewer using final refined projection matrices directly decoded from updated $SL(4)$ values.

