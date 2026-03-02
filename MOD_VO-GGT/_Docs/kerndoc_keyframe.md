# kern_keyframe — Multi-Orientation Optical Flow Keyframe Selection

## Problem

In a multi-camera rig each camera can have multiple orientations (pitch/yaw/roll
presets). A single forward-facing view may miss lateral or vertical motion,
causing the pipeline to skip timestamps where significant ego-motion occurs.

## Strategy

Instead of tracking optical flow on a single view, we create **one independent
`FrameTracker`** for each orientation of the primary camera. A timestamp becomes
a keyframe when **any** orientation exceeds the disparity threshold
(OR-fusion rule).

```
primary_views = [cam0_p+0_y+0_r+0,  cam0_p+0_y+30_r+0,  cam0_p+0_y-30_r+0]

For each timestamp:
    for view in primary_views:
        disparity = flow_tracker[view].compute_disparity(image)
    if ANY disparity > min_disparity  ->  KEYFRAME
```

### Why OR, not AND?

- OR is more sensitive — it captures motion visible to **any** camera angle.
- A rigid multi-camera rig moves as one body; if any orientation sees large
  disparity, the rig has moved enough to warrant a new keyframe.
- AND would under-select: an orientation looking along the motion axis may show
  near-zero disparity even during large forward translation.

## Optical Flow Details

Each `FrameTracker` maintains:

| State            | Description                                         |
|------------------|-----------------------------------------------------|
| `last_kf`        | The image at the last keyframe for this orientation  |
| `kf_gray`        | Greyscale of `last_kf`                               |
| `kf_pts`         | Shi-Tomasi corner points detected in `kf_gray`       |

At each new timestamp:

1. **Shi-Tomasi corners** are detected in the keyframe image
   (`cv2.goodFeaturesToTrack`, up to 1000 points).
2. Points are tracked forward to the current frame via **Lucas-Kanade pyramidal
   optical flow** (`cv2.calcOpticalFlowPyrLK`, 3 pyramid levels, 21×21 window).
3. **Mean pixel displacement** is computed over successfully tracked points.
4. If displacement > `min_disparity` → keyframe; re-initialise the tracker on
   the current image.

### Re-initialisation triggers

- First frame seen by the tracker (no prior keyframe).
- Fewer than 10 corners were tracked successfully (scene change, occlusion).
- Disparity threshold exceeded → accept keyframe and reset.

## Parameters

| Parameter       | Default | Effect                                              |
|-----------------|---------|-----------------------------------------------------|
| `min_disparity` | 50.0    | Higher = fewer keyframes, more baseline per submap   |
| `vis_flow`      | False   | Opens an OpenCV window showing tracked flow arrows   |

## Data Flow

```
sorted_timestamps ─┐
                    ├── select_keyframes() ──> keyframe_timestamps
ts_to_view_paths  ─┘
primary_views     ─┘
```

`keyframe_timestamps` is then consumed by `essn_multicam_slam.run_multicam_slam()`
to build submaps with ALL cameras and ALL orientations at the selected timestamps.
