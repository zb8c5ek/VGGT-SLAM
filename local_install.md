# VGGT-SLAM — Local Installation Guide (Windows & Linux)

> Tested on Windows 11 with VS 2022 Build Tools, Python 3.11, CUDA 12.8, RTX 5090 (SM 12.0)
> Pipeline verified: 473 images → 208 keyframes → 14 submaps → 1 loop closure @ 5.5 FPS

---

## Prerequisites

| Tool | Version Tested | Notes |
|------|---------------|-------|
| **Visual Studio 2022 Build Tools** | MSVC 14.44+ | Desktop C++ workload required |
| **CMake** | 4.x (≥ 3.24) | Must be on `PATH` |
| **Ninja** | any recent | Must be on `PATH` |
| **Python** | 3.11.x | conda / micromamba recommended |
| **CUDA Toolkit** | 12.x | For PyTorch GPU support |

---

## 1. Create a Python Environment

```bash
# Using micromamba (or conda / mamba)
micromamba create -n sfm3r python=3.11 -y
micromamba activate sfm3r
```

## 2. Install Conda-Forge Dependencies

These are easier to get from conda-forge than from pip on Windows:

```bash
micromamba install -c conda-forge boost pybind11 pyparsing -y
```

> **Important:** GTSAM needs **shared** Boost libraries (DLLs).
> conda-forge provides these by default.

## 3. Install pip Requirements

```bash
pip install -r requirements.txt
pip install open3d termcolor pytorch-lightning pytorch_metric_learning
```

This installs PyTorch, NumPy, Open3D, OpenCV, etc.  
`gtsam-develop` is commented out because it has no Windows wheels —
we build it from source below.

> **xformers:** Do NOT install xformers unless your GPU is SM ≤ 9.0.
> On newer GPUs (e.g. RTX 5090, SM 12.0) the xformers flash-attention kernels
> will crash. DINOv2 (used by salad) automatically falls back to PyTorch SDPA
> when xformers is absent — this works on all GPUs.
> If xformers is already installed: `pip uninstall xformers -y`

## 4. Install Third-Party Python Packages

`third_party/vggt` and `third_party/salad` are integrated directly into this
repo (no git submodules). They have been patched for NumPy 2.x compatibility.

```bash
pip install -e third_party/salad
pip install --no-deps -e third_party/vggt
```

> `--no-deps` for vggt avoids redundant dependency resolution; all deps
> are already satisfied by `requirements.txt`.

```bash
# (Optional) Perception Encoder + SAM3 — only needed for --run_os
# git clone https://github.com/facebookresearch/perception_models.git third_party/perception_models
# pip install -e third_party/perception_models
# git clone https://github.com/facebookresearch/sam3.git third_party/sam3
# pip install -e third_party/sam3
```

## 5. Install VGGT-SLAM itself

```bash
pip install -e .
```

## 6. Build GTSAM from Source (SL4 support)

The `gtsam-develop` branch is required for `SL4`, `PriorFactorSL4`, and
`BetweenFactorSL4`. No pre-built Windows wheels exist, so we build from source.

### 6.1 Source Location

The patched GTSAM develop source tree is already included at
`3rdParty/gtsam-develop/`. All Windows-specific patches listed below have
been pre-applied — you do **not** need to clone or patch anything.

If you need a fresh copy:

```bash
cd 3rdParty
git clone -b develop https://github.com/borglab/gtsam.git gtsam-develop
cd gtsam-develop
# Then apply the patches in 6.2 below
```

### 6.2 Apply Windows-specific patches

Four small changes are needed for MSVC + conda-forge Boost + NumPy 2.x:

#### a) Use shared Boost (conda-forge layout)

In `cmake/HandleBoost.cmake`, change line ~16:

```cmake
# BEFORE
set(Boost_USE_STATIC_LIBS ON)
# AFTER
set(Boost_USE_STATIC_LIBS OFF)  # use shared Boost from conda-forge
```

#### b) Suppress C4819 (code-page) warnings

In `cmake/GtsamBuildTypes.cmake`, find the MSVC flags line and:
- Remove `/WX` (warnings-as-errors) or keep it but add `/wd4819`
- Add `/wd4819` to suppress non-ASCII code-page warnings

```cmake
# Example result:
set(GTSAM_COMPILE_OPTIONS_PRIVATE_COMMON /W3 /GR /EHsc /MP /wd4819 ...)
```

#### c) Fix Boost DLL copy for conda-forge layout

In `python/CMakeLists.txt`, the default `$<TARGET_RUNTIME_DLLS:gtsam>` generator
expression resolves Boost DLLs to `Library/lib/` (import libs only).
conda-forge places actual DLLs in `Library/bin/`.

Replace the `TARGET_RUNTIME_DLLS` post-build commands for both `gtsam` and
`gtsam_unstable` with the `_gtsam_copy_boost_dlls()` function that probes both
`lib/` and `../bin/` paths. See the patched
[python/CMakeLists.txt](3rdParty/gtsam-develop/python/CMakeLists.txt) for the
full implementation.

#### d) Fix NumPy 2.x compatibility

In `python/gtsam/__init__.py`, replace `np.nan` with `float('nan')`:

```python
# BEFORE
def Point2(x=np.nan, y=np.nan):
def Point3(x=np.nan, y=np.nan, z=np.nan):

# AFTER
def Point2(x=float('nan'), y=float('nan')):
def Point3(x=float('nan'), y=float('nan'), z=float('nan')):
```

### 6.3 Configure & Build

Open a terminal that has both MSVC and your Python env on `PATH`:

```bat
@rem Activate VS2022 Build Tools
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

@rem Activate conda/micromamba env (ensure python, pybind11, boost are visible)
@rem e.g.  micromamba activate sfm3r

@rem Set UTF-8 code page (avoids C4819 on Chinese/CJK locales)
chcp 65001

cd 3rdParty\gtsam-develop
mkdir build && cd build

cmake .. -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DGTSAM_BUILD_PYTHON=ON ^
  -DGTSAM_BUILD_TESTS=OFF ^
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF ^
  -DGTSAM_USE_SYSTEM_PYBIND=ON ^
  -DGTSAM_WITH_TBB=OFF ^
  -DBoost_USE_STATIC_LIBS=OFF

cmake --build . --config Release -j8
```

Build takes ~5-10 minutes. All 256 targets should succeed.

A convenience batch script is also provided:

```bat
3rdParty\gtsam-develop\build_gtsam.bat
```

### 6.3b Configure & Build (Linux)

On Linux you can either `pip install gtsam-develop` directly (wheels exist)
or build from the same source tree:

```bash
cd 3rdParty/gtsam-develop && mkdir build && cd build
cmake .. -DGTSAM_BUILD_PYTHON=ON \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_USE_SYSTEM_PYBIND=ON \
         -DGTSAM_WITH_TBB=OFF
make -j$(nproc)
```

### 6.4 Install into Python environment

```bash
cd build/python
python setup.py install
```

### 6.5 Verify

```python
python -c "import gtsam; print('gtsam OK'); from gtsam import SL4, PriorFactorSL4, BetweenFactorSL4; print('SL4 OK!')"
```

Expected output:
```
gtsam OK
SL4 OK!
```

---

## 7. Download SALAD Checkpoint

The salad loop-closure model expects `dino_salad.ckpt` in the torch hub cache:

```bash
# PowerShell
Invoke-WebRequest -Uri "https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt" `
  -OutFile "$env:USERPROFILE\.cache\torch\hub\checkpoints\dino_salad.ckpt"

# Linux / macOS
curl -L -o ~/.cache/torch/hub/checkpoints/dino_salad.ckpt \
  https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt
```

---

## Quick-Start: Run VGGT-SLAM

```bash
# Full verify — all imports
python -c "
import numpy, torch, gtsam, vggt, salad
from gtsam import SL4, PriorFactorSL4, BetweenFactorSL4
print(f'numpy {numpy.__version__}, torch {torch.__version__}')
print('gtsam SL4 OK, vggt OK, salad OK — all good!')
"

# Run on example images (first run downloads VGGT 1B model ~4.7 GB)
python main.py --image_folder office_loop/ --vis_map
```

### Key Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--image_folder` | `examples/kitchen/images/` | Input image directory |
| `--submap_size` | 16 | Frames per submap |
| `--vis_map` | off | Live point-cloud visualisation (viser) |
| `--run_os` | off | Open-set semantic search (needs Perception Encoder + SAM3) |
| `--vis_flow` | off | Show optical flow for keyframe selection |
| `--log_results` | off | Save trajectory to `--log_path` |
| `--conf_threshold` | 25.0 | % low-confidence points to filter |
| `--lc_thres` | 0.95 | Loop-closure retrieval threshold |
| `--min_disparity` | 50 | Min disparity for new keyframe |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `pyparsing` not found during cmake | `pip install --force-reinstall pyparsing` |
| C4819 warnings become errors | Remove `/WX` from cmake flags, add `/wd4819` |
| pybind11 `stl_bind.h` error with MSVC 14.44 | Set `-DGTSAM_USE_SYSTEM_PYBIND=ON`, install pybind11 from conda-forge |
| Boost DLL not found at runtime | Ensure `Library/bin/` is on `PATH`, or apply the DLL copy patch above |
| `np.nan` AttributeError on import | Apply the NumPy 2.x patch (step 6.2d) |
| `gtsam-develop` pip install fails | Expected on Windows — build from source (step 6) |
| xformers `NotImplementedError` on SM 12.0+ GPU | `pip uninstall xformers -y` — DINOv2 uses PyTorch SDPA fallback |
| `dino_salad.ckpt` not found | Download checkpoint (step 7) |
| `FutureWarning: torch.cuda.amp.autocast` | Informational only — already patched in `third_party/vggt` |
