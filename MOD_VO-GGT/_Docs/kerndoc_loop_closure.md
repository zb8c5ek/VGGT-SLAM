# Loop Closure in VGGT-SLAM — Detailed Technical Document

## Overview

Loop closure detects when the camera revisits a previously observed location
and injects a constraint into the pose graph to correct accumulated drift.
The pipeline has four stages:

```
1. Embedding   — SALAD encodes each frame into a retrieval vector
2. Retrieval   — find the best-matching frame from earlier submaps
3. Verification — VGGT re-infers the pair and checks image_match_ratio
4. Integration — a loop-closure submap + graph edges are created
```

---

## Stage 1: Image Embedding (SALAD)

**Where:** `ImageRetrieval.__init__()` and `get_all_submap_embeddings()` in
`vggt_slam/loop_closure.py`

**Source files:**
- `3rdParty/salad/salad/eval.py` — `load_model()`
- `3rdParty/salad/salad/vpr_model.py` — `VPRModel` (backbone + aggregator)
- `3rdParty/salad/salad/models_salad/backbones/dinov2.py` — DINOv2 backbone
- `3rdParty/salad/salad/models_salad/aggregators/salad.py` — SALAD aggregator

### 1.1 Model Architecture

SALAD = Sinkhorn Algorithm for Locally-Aggregated Descriptors. The full
model is a `VPRModel` with two parts:

```
Image (B, 3, 224, 224)
        │
        ▼
┌─────────────────────────────────┐
│  DINOv2-ViT-B/14 Backbone      │
│  ─────────────────────────      │
│  • Architecture: dinov2_vitb14  │
│  • Patch size: 14x14            │
│  • Feature dim: 768             │
│  • Input: (B, 3, 224, 224)     │
│  • First N-4 blocks: frozen     │
│  • Last 4 blocks: trainable     │
│  • Outputs:                     │
│    - f: patch features           │
│      (B, 768, 16, 16)           │
│      = 256 spatial patches       │
│    - t: CLS token                │
│      (B, 768)                    │
└──────────┬──────────────────────┘
           │
           ▼
┌─────────────────────────────────┐
│  SALAD Aggregator               │
│  ─────────────────              │
│  Config:                        │
│    num_channels = 768           │
│    num_clusters = 64 (m)        │
│    cluster_dim  = 128 (l)       │
│    token_dim    = 256 (g)       │
│                                 │
│  Three parallel MLPs:           │
│                                 │
│  1. token_features: t → g       │
│     768 → 512 → 256             │
│     Produces global scene token  │
│                                 │
│  2. cluster_features: f → l     │
│     768 → 512 → 128  (1x1 conv)│
│     Produces local descriptors   │
│     (B, 128, 256)                │
│                                 │
│  3. score: f → m                │
│     768 → 512 → 64   (1x1 conv)│
│     Produces assignment scores   │
│     (B, 64, 256)                 │
│                                 │
│  Sinkhorn OT (3 iterations):    │
│  ─────────────────────────       │
│  score matrix S (B, 64, 256)    │
│  augmented with dustbin row      │
│  → log-domain Sinkhorn solver    │
│  → soft assignment P (B,64,256) │
│    P[j,i] = prob that patch i    │
│    belongs to cluster j          │
│                                 │
│  VLAD-like aggregation:         │
│  ─────────────────────          │
│  For each cluster j:            │
│    c_j = Σ_i P[j,i] * f_i      │
│    (weighted sum of local desc.) │
│  → (B, 128, 64) → flatten       │
│  → L2-normalize                  │
│                                 │
│  Final descriptor:              │
│  [L2norm(scene_token) ‖          │
│   L2norm(VLAD_vector)]          │
│  = (B, 256 + 128*64)            │
│  = (B, 8448)                    │
│  → L2-normalize the whole thing  │
└──────────┬──────────────────────┘
           │
           ▼
     descriptor (B, 8448)
     unit-normalized
```

### 1.2 Preprocessing

Before feeding to SALAD, each frame is transformed:

```python
# In loop_closure.py
T.ToPILImage()           # tensor → PIL
T.Resize((224, 224))     # bilinear resize to fixed size
T.ToTensor()             # PIL → tensor [0,1]
T.Normalize(             # ImageNet normalization
    mean=[0.485, 0.456, 0.406],
    std =[0.229, 0.224, 0.225]
)
```

### 1.3 When Embeddings Are Computed

In `solver.run_predictions()`, SALAD embedding happens BEFORE VGGT
inference, immediately after the submap is created:

```
1. Load images → create Submap
2. SALAD: get_all_submap_embeddings(submap) → S descriptors (S, 8448)
3. Store as submap.retrieval_vectors
4. VGGT inference on the same images (separate step)
```

This means retrieval can begin as soon as the submap is created, without
waiting for VGGT's heavier inference.

### 1.4 Descriptor Properties

| Property           | Value                              |
|--------------------|------------------------------------|
| Backbone           | DINOv2 ViT-B/14                    |
| Input resolution   | 224 x 224                          |
| Patch grid         | 16 x 16 = 256 patches             |
| Feature dim        | 768 per patch                      |
| Num clusters       | 64                                 |
| Cluster dim        | 128                                |
| Scene token dim    | 256                                |
| Final descriptor   | 256 + 128*64 = **8448**            |
| Normalization      | L2-normalized (unit sphere)        |

---

## Stage 2: Retrieval — Finding Loop Closure Candidates

**Where:** `ImageRetrieval.find_loop_closures()` in `vggt_slam/loop_closure.py`
and `GraphMap.retrieve_best_score_frame()` in `vggt_slam/map.py`

### 2.1 Inputs

- **Query vectors:** Current submap's `retrieval_vectors` — one 8448-dim
  L2-normalized descriptor per frame in the submap.
- **Database:** All previous submaps' `retrieval_vectors`, stored in
  `GraphMap.submaps`.
- `max_similarity_thres` (config `lc_thres`, default 0.95) — L2 distance
  threshold.
- `max_loop_closures` (config `max_loops`, default 1) — max matches to
  return.

### 2.2 Distance Metric

The code uses **L2 distance** (Euclidean norm) between the 8448-dim
descriptors:

```python
score = torch.linalg.norm(embedding - query_vector)
```

Since both vectors are L2-normalized (unit vectors), L2 distance relates
to cosine similarity:

```
‖a - b‖² = ‖a‖² + ‖b‖² - 2·a·b = 2 - 2·cos(a,b)
‖a - b‖  = √(2 - 2·cos(a,b))
```

| L2 distance | Cosine similarity | Meaning              |
|-------------|-------------------|----------------------|
| 0.0         | 1.0               | Identical            |
| 0.5         | 0.875             | Very similar         |
| 1.0         | 0.5               | Moderate similarity  |
| 1.414       | 0.0               | Orthogonal           |
| `lc_thres=0.95` | ~0.549       | Threshold for match  |

So `lc_thres = 0.95` means "only consider loop closures when two frames'
descriptors have L2 distance < 0.95", which corresponds to cosine
similarity > ~0.55.

### 2.3 Search Algorithm (Brute-Force)

```
find_loop_closures(map, current_submap, max_similarity_thres, max_loop_closures):

    matches_queue = LoopMatchQueue(max_size=max_loop_closures)

    For each query_frame_idx, query_vector in current_submap.retrieval_vectors:

        overall_best_score = 1000  (infinity)
        overall_best_submap_id = 0
        overall_best_frame_id = 0

        For each historical_submap in map.submaps (sorted by key):

            SKIP if submap == current_submap
            SKIP if submap == immediately_previous_submap  ← avoids recent matches
            SKIP if submap.is_loop_closure_submap           ← only search real submaps

            scores = []
            For each embedding in historical_submap.retrieval_vectors:
                score = L2_norm(embedding - query_vector)
                scores.append(score)

            best_frame_id = argmin(scores)
            best_score = scores[best_frame_id]

            if best_score < overall_best_score:
                overall_best_score = best_score
                overall_best_submap_id = submap_key
                overall_best_frame_id = best_frame_id

        if overall_best_score < max_similarity_thres:
            matches_queue.add(LoopMatch(
                overall_best_score,
                current_submap_id,
                query_frame_idx,
                overall_best_submap_id,
                overall_best_frame_id
            ))

    return matches_queue.get_matches()  ← sorted by score (best first)
```

### 2.4 LoopMatchQueue — Priority Queue for Top-K

The `LoopMatchQueue` is a **max-heap** (using negated scores to simulate
with Python's min-heap). It keeps at most `max_loop_closures` entries:

- When a new match is added and the queue is not full, it's pushed.
- When the queue is full, `heappushpop` replaces the worst (highest
  distance) entry if the new one is better.

With `max_loops=1`, the queue keeps exactly the single best match across
all frames in the current submap.

### 2.5 Exclusion Rules

Two categories of submaps are excluded from the search:

1. **Current submap** — trivially, you don't match against yourself.
2. **Immediately previous submap** (`ignore_last_submap=True`) — the
   overlap window means the previous submap shares frames with the
   current one. Matching against it would give false loop closures
   from temporally adjacent frames, not genuine revisitations.
3. **Loop-closure submaps** (`get_lc_status() == True`) — these are
   synthetic 2-frame submaps created by earlier loop closures, not
   real sequential data.

### 2.6 Output

A list of `LoopMatch` named tuples, sorted by score (best first):

```python
LoopMatch(
    similarity_score,      # L2 distance (lower = better match)
    query_submap_id,       # current submap ID
    query_submap_frame,    # which frame in current submap is the query
    detected_submap_id,    # which historical submap was matched
    detected_submap_frame  # which frame in that submap was matched
)
```

### 2.7 Complexity Note

The retrieval is brute-force O(Q * D * F) where Q = frames in current
submap, D = total database submaps, F = frames per submap. For large maps
this could become slow, but since SALAD descriptors are compact (8448-dim)
and the number of submaps grows slowly, this is fast in practice (the
`loop_closure_timer` in the demo showed ~6ms per frame average).

---

## Stage 3: Verification — VGGT Geometric Check

**Where:** `solver.run_predictions()` lines 375–414 in `vggt_slam/solver.py`

Once a candidate loop closure is found, it must be geometrically verified:

```
1. Retrieve the query frame from the current submap
2. Retrieve the matched frame from the historical submap
3. Stack them as a 2-frame batch: [query_frame, matched_frame]
4. Run VGGT inference on this pair with compute_similarity=True
5. VGGT returns image_match_ratio from its internal attention layers
```

**image_match_ratio** is computed inside VGGT's aggregator
(`3rdParty/vggt/vggt/models/aggregator.py`). At a specific global
attention layer (`target_layer`), the key and query matrices are extracted
and their similarity is computed via `get_similarity(k, q)`. This measures
how well the two frames' internal representations agree geometrically.

**Acceptance gate:**
```python
if image_match_ratio < 0.85:
    # Reject — the pair doesn't have enough geometric agreement
    predictions_lc = None
    predictions["detected_loops"] = []
else:
    # Accept — proceed with loop closure integration
    graph.increment_loop_closure()
```

**If accepted**, VGGT also produces for the 2-frame pair:
- `extrinsic_lc`: relative camera poses (2, 3, 4)
- `intrinsic_lc`: camera intrinsics (2, 3, 3)
- `depth_lc`: depth maps (2, H, W, 1)
- `depth_conf_lc`: depth confidence (2, H, W)

---

## Stage 4: Integration — Loop Closure Submap + Graph Edges

**Where:** `solver.add_points()` lines 286–319 in `vggt_slam/solver.py`

For each accepted loop closure, the system creates a **dedicated 2-frame
loop-closure submap** and adds two edges to the pose graph:

### 4a. Create the loop closure submap

```python
lc_submap = Submap(lc_submap_num)
lc_submap.set_lc_status(True)           # mark as LC submap
lc_submap.add_all_frames(lc_frames)     # the 2-frame pair
lc_submap.add_all_poses(world_to_cam)   # VGGT's predicted poses for the pair
lc_submap.add_all_points(world_points)  # VGGT's predicted point clouds
lc_submap.set_last_non_loop_frame_index(1)  # frame 1 = the matched historical frame
```

The LC submap contains exactly 2 frames:
- **Frame 0:** the query frame from the current submap
- **Frame 1:** the matched frame from the historical submap

### 4b. Add two graph edges

```python
# Edge 1: Connect current submap's query frame to LC submap's frame 0
#   This is a NORMAL edge (is_loop_closure=False)
#   It computes scale via overlapping point clouds
add_edge(lc_submap_num, 0, query_submap_id, query_submap_frame, is_loop_closure=False)

# Edge 2: Connect historical submap's matched frame to LC submap's frame 1
#   This is a LOOP CLOSURE edge (is_loop_closure=True)
#   It only adds a between-factor, no new node initialization
add_edge(detected_submap_id, detected_submap_frame, lc_submap_num, 1, is_loop_closure=True)
```

The graph structure after a loop closure:

```
  [Submap A: frames 0..N]
         |
    normal edges (inner_submap_noise)
         |
  [Submap B: frames 0..M]     <-- current submap
         |
    scale edge (intra_submap_noise)
         |
  [LC Submap: frame 0 = query, frame 1 = match]
         |
    loop closure edge (intra_submap_noise)
         |
  [Submap C: frames 0..K]     <-- historical submap
```

### 4c. Scale estimation across the loop closure

The first edge (`is_loop_closure=False`) triggers scale estimation:

```python
# In add_edge(), when submap_id_prev is not None:
P_temp = inv(prior_submap.proj_mats[-1]) @ current_submap.proj_mats[0]
t1 = P_temp[:3,:3] @ current_points[good_mask]   # transform current points
t2 = prior_points[good_mask]                       # reference points
scale_factor = estimate_scale_pairwise(t1, t2)     # robust scale alignment
```

The relative transform `H_overlap` is then computed incorporating the scale,
and a `BetweenFactorSL4` is added to the graph.

The second edge (`is_loop_closure=True`) adds ONLY a between-factor
constraint (no new node initialization). This closes the loop in the
graph, creating a cycle that the optimizer can exploit.

---

## Stage 5: Optimization

**Where:** `graph.optimize()` in `vggt_slam/graph.py`

After adding points and edges (including any loop closure edges), the
full pose graph is optimized:

```python
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
result = optimizer.optimize()
values = result  # store optimized poses
```

The factor graph contains three types of factors:

| Factor type          | Noise (sigma) | Purpose                              |
|---------------------|---------------|--------------------------------------|
| PriorFactorSL4      | 1e-6          | Anchors the first node               |
| BetweenFactorSL4    | inner: 0.05   | Consecutive frames within a submap   |
| BetweenFactorSL4    | intra: 0.05   | Between submaps (overlap + LC edges) |

All factors operate in the **SL(4)** Lie group (4x4 matrices with unit
determinant), giving 15 degrees of freedom per node. The loop closure
between-factor creates a cycle in the graph that pulls the historical and
current poses into agreement, correcting drift.

---

## After optimization

If a loop closure was detected, the visualization updates ALL submaps
(not just the latest), because the optimizer may have shifted earlier poses:

```python
if loop_closure_detected:
    solver.update_all_submap_vis()   # re-render everything
else:
    solver.update_latest_submap_vis()  # only render the new submap
```

---

## Parameter Summary

| Parameter             | Default | Location      | Effect                                        |
|-----------------------|---------|---------------|-----------------------------------------------|
| `max_loops`           | 1       | config.yaml   | Max loop closures per submap (0 = disable)    |
| `lc_thres`            | 0.95    | config.yaml   | L2 distance threshold for SALAD retrieval     |
| `image_match_ratio`   | 0.85    | hardcoded     | VGGT geometric verification threshold         |
| inner_submap_noise    | 0.05    | graph.py      | Noise for within-submap between-factors       |
| intra_submap_noise    | 0.05    | graph.py      | Noise for cross-submap between-factors (+ LC) |
| anchor_noise          | 1e-6    | graph.py      | Very tight prior on the first node            |

---

## Sequence Diagram

```
  run_predictions()
       |
       ├── 1. Compute SALAD embeddings for all frames in new submap
       |
       ├── 2. find_loop_closures()
       |       ├── For each frame's embedding:
       |       │     └── retrieve_best_score_frame() across all earlier submaps
       |       └── Return top-K matches (LoopMatchQueue)
       |
       ├── 3. If matches found:
       |       ├── Stack query + matched frame
       |       ├── VGGT(pair, compute_similarity=True)
       |       ├── Check image_match_ratio >= 0.85
       |       └── If pass: extract LC poses, depth, intrinsics
       |
       ├── 4. VGGT inference on full submap (normal)
       |
       └── Return predictions dict (includes detected_loops + LC data)

  add_points()
       |
       ├── Create main submap, add to map + graph
       |
       └── For each accepted loop closure:
             ├── Create 2-frame LC submap
             ├── add_edge(LC_submap → current_submap)  [normal, with scale]
             └── add_edge(historical_submap → LC_submap) [loop closure]

  graph.optimize()   ← Levenberg-Marquardt on full factor graph
```
