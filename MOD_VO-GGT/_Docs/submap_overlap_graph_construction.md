# Rigorous Breakdown of Submap Overlap and Graph Construction

In VGGT-SLAM 2.0, to keep runtime and memory manageable, inference is done in batches of frames called **submaps**. However, because visual odometry networks output un-scaled (or arbitrarily scaled) local projections per batch, the system must carefully stitch these submaps together into a global geometry. 

This document explains the mechanism of how submaps are processed, how shared frames are utilized to solve geometric mismatches, and how these relationships construct the GTSAM pose graph.

---

## 1. Overlapping Sliding Window (Tracking Loop)

In `main.py`, frames are buffered into the model using a sliding window. 

The image batch is loaded until it matches `submap_size + overlapping_window_size`. Once processed, the exact same frames that formed the tail end of the *current* window are kept to form the beginning of the *next* window.

```python
# main.py
if len(image_names_subset) == args.submap_size + args.overlapping_window_size:
    # Run the model on the batch (e.g., 17 frames)
    predictions = solver.run_predictions(image_names_subset, model, ...)
    solver.add_points(predictions)
    
    # Store the last few frames to overlap with the next submap
    image_names_subset = image_names_subset[-args.overlapping_window_size:] 
```
By default, the `overlapping_window_size` is 1. This means **the last frame of Submap $N-1$ is physically identical to the first frame (frame 0) of Submap $N$**.

---

## 2. Resolving Inter-Submap Scale and Pose (`solver.py -> add_edge`)

Because Submap $N$ and Submap $N-1$ are computed independently by the neural network, their coordinate frames and scales *will not match*, despite sharing a physical frame. 

When `add_edge()` is triggered for a new submap, the system isolates the shared frame (i.e. `frame_id_prev = last frame of previous submap` and `frame_id_curr = 0 of current submap`). 

### Extracting corresponding geometries
The system unprojects the dense depth map of the shared frame from *both* submaps. Since they are the same image, dense point-to-point correspondence is guaranteed. To ensure robustness, the solver uses pixel confidence maps (`depth_conf`) output by VGGT to mask out poor geometry:
```python
# Filter points by highest confidence in both un-projections
current_conf = current_submap.get_conf_masks_frame(frame_id_curr)
prior_conf = prior_submap.get_conf_masks_frame(frame_id_prev)

good_mask = (prior_conf > prior_submap.get_conf_threshold()) * \
            (current_conf > prior_submap.get_conf_threshold())
```

### Estimating Pairwise Scale
With the highly confident 3D points (`t1` from the new submap, `t2` from the prior submap), numerical variance distributions are used to find the exact ratio of scale between them.
```python
# Extract and transform new submap's points to align orientations
P_temp = np.linalg.inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
t1 = (P_temp[0:3,0:3] @ current_submap.get_frame_pointcloud(frame_id_curr).reshape(-1, 3)[good_mask].T).T
t2 = prior_submap.get_frame_pointcloud(frame_id_prev).reshape(-1, 3)[good_mask]

# Compute metric scaling factor resolving the discrepancy 
scale_factor = estimate_scale_pairwise(t1, t2)[0]
H_scale = np.diag((scale_factor, scale_factor, scale_factor, 1.0))
```

### Mapping the Transformation (Inter-Submap Constraints)
The total relative transformation mapping the end of the previous submap to the beginning of the new submap combines the network-predicted relative transform `P_temp` and the recovered dense `H_scale`:

```python
# The scaled relative transform bridging submap boundaries
H_overlap_prior_overlap_current = P_temp @ H_scale

# Add constraint to GTSAM graph 
self.graph.add_between_factor(
    key1=overlapping_node_id_prev, 
    key2=submap_id_curr + frame_id_curr, 
    relative_h=H_overlap_prior_overlap_current, 
    noise=self.graph.intra_submap_noise
)
```

---

## 3. Populating Intra-Submap Graph Edges

Once the anchor frame connecting the two submaps is inserted, the remaining frames making up Submap $N$ must be rigidly added to the GTSAM graph.

Since frames *within* a single batch are guaranteed by the network to be geometrically and metrically consistent relative to each other, the solver loops over the remaining poses in Submap $N$ (from index 1 to `submap_size`) and extracts simple frame-by-frame relative transforms.

```python
world_to_cam = current_submap.get_all_poses()
for index, pose in enumerate(world_to_cam):
    if index == 0: continue # Handled by the overlap edge

    # Relative transformation purely inside the submap
    H_inner = world_to_cam[index-1] @ np.linalg.inv(pose) 
    
    current_node = self.graph.get_homography(submap_id_curr + index - 1) @ H_inner

    # Initialize node
    self.graph.add_homography(submap_id_curr + index, current_node)

    # Attach graph edge using native continuous bounds
    self.graph.add_between_factor(
        key1=submap_id_curr + index - 1, 
        key2=submap_id_curr + index, 
        relative_h=H_inner, 
        noise=self.graph.inner_submap_noise
    )
```

## Summary
The scene graph handles scale-drift intrinsically. It achieves this by structuring the node sequence into batches stitched together entirely out of single shared frames overlapping on the boundary. The shared frame provides a strict point-cloud intersection constraint that is solved globally by the `SL4` factors during graph execution!

