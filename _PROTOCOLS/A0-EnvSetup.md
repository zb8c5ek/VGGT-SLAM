# VGGT-SLAM — Local Installation Guide (Windows & Linux)

> Tested on Windows 11 with VS 2022/18 Build Tools, Python 3.11, CUDA 12.6+, RTX 5090 (SM 12.0)
> Pipeline verified: 473 images → 208 keyframes → 14 submaps → 1 loop closure @ 5.5 FPS

---

## Prerequisites

| Tool | Version Tested | Notes |
|------|---------------|-------|
| **Visual Studio Build Tools** | 2022 or 18 (MSVC 14.44+) | "Desktop development with C++" workload required |
| **CMake** | 4.x (≥ 3.24) | Installed via conda-forge in step 2 |
| **Ninja** | any recent | Installed via conda-forge in step 2 |
| **Python** | 3.11.x | micromamba recommended |
| **CUDA Toolkit** | 12.x | For PyTorch GPU support |

**VS Build Tools — check & install:**

```powershell
# Check if VS Build Tools are installed and which version
& "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -all -property installationPath 2>$null
# If nothing is printed, no VS instance is installed.
```

If **VS 2022 Build Tools** are not installed, download from:
<https://visualstudio.microsoft.com/visual-cpp-build-tools/>

During installation, select the **"Desktop development with C++"** workload.
This provides `cl.exe`, `link.exe`, `vcvars64.bat`, and the Windows SDK.

> The `build_gtsam.bat` script auto-detects VS via `vswhere.exe` and falls
> back to common paths for VS 2022/2019 (BuildTools, Community, Professional,
> Enterprise). Any edition works.

---

## 1. Create a Python Environment

```bash
# Using micromamba (or conda / mamba)
micromamba create -n sfm3rV2 python=3.11 -y
micromamba activate sfm3rV2
```

## 2. Install Conda-Forge Dependencies

These are easier to get from conda-forge than from pip on Windows:

```bash
micromamba install -c conda-forge boost cmake ninja eigen "pybind11>=2.13,<3" pybind11-global pyparsing -y
```

> **Important — pybind11 version:** GTSAM's develop branch is **incompatible
> with pybind11 3.x** (`stl_bind.h` compilation errors on MSVC). You **must**
> use pybind11 2.13.x. The constraint `"pybind11>=2.13,<3"` ensures this.
>
> **Important:** GTSAM needs **shared** Boost libraries (DLLs).
> conda-forge provides these by default.

## 3. Install PyTorch (GPU) and pip Requirements

> **Critical:** `requirements.txt` does **not** include `torch` / `torchvision`
> because `pip install -r requirements.txt` pulls the **CPU-only** build from
> PyPI. You must install PyTorch with CUDA support **first**, then install the
> remaining requirements.

```bash
# Step A — Check your CUDA toolkit version
nvcc --version
# Look for "release X.Y" in the output, e.g.:
#   Cuda compilation tools, release 12.6, V12.6.20  → use cu126
#   Cuda compilation tools, release 12.8, V12.8.x   → use cu128

# Step B — Install PyTorch 2.10 with CUDA support
# Replace cu126 below with cu128 if nvcc reported 12.8

# Option 1: Aliyun mirror (fast in China, falls back to official)
pip install torch torchvision --retries 10 --timeout 120 \
    --index-url https://mirrors.aliyun.com/pytorch-wheels/cu126/ \
    --extra-index-url https://download.pytorch.org/whl/cu126

# Option 2: Official PyTorch only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126

# Step B — Install the remaining dependencies
pip install -r requirements.txt
pip install open3d termcolor pytorch-lightning pytorch_metric_learning
```

If you accidentally installed CPU PyTorch, fix it by re-running Step A —
pip will upgrade in-place.

`gtsam-develop` is commented out in `requirements.txt` because it has no
Windows wheels — we build it from source below.

> **xformers:** Do NOT install xformers unless your GPU is SM ≤ 9.0.
> On newer GPUs (e.g. RTX 5090, SM 12.0) the xformers flash-attention kernels
> will crash. DINOv2 (used by salad) automatically falls back to PyTorch SDPA
> when xformers is absent — this works on all GPUs.
> If xformers is already installed: `pip uninstall xformers -y`

## 4. Install Third-Party Python Packages

`3rdParty/vggt` and `3rdParty/salad` are integrated directly into this
repo (no git submodules). They have been patched for NumPy 2.x compatibility.

```bash
pip install -e 3rdParty/salad
pip install --no-deps -e 3rdParty/vggt
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

The **complete** patched GTSAM develop source tree must be at
`3rdParty/gtsam-develop/`. The following directories are **required**:

| Directory | Contents |
|-----------|----------|
| `gtsam/` | Core C++ library (geometry, linear, nonlinear, slam, etc.) |
| `gtsam/3rdparty/` | Bundled Eigen, metis, cephes |
| `gtsam_unstable/` | Experimental features |
| `python/gtsam/` | Python wrapper templates (`gtsam.tpl`, `__init__.py`) |
| `python/gtsam_unstable/` | Unstable Python wrappers |
| `wrap/` | pybind11 wrapper generator |

If any of these are missing (e.g. after a partial ZIP download), get a fresh copy:

```bash
cd 3rdParty
git clone -b develop https://github.com/borglab/gtsam.git gtsam-develop
cd gtsam-develop
# Then apply the patches in 6.2 below
```

All Windows-specific patches listed below have been pre-applied in the
checked-in source — you do **not** need to patch anything if using the
included tree.

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
@rem Activate VS Build Tools (auto-detect location)
@rem The build script build_gtsam.bat does this automatically.
@rem If running manually, find vcvars64.bat:
@rem   vswhere -latest -products * -property installationPath
@rem Then call: <install_path>\VC\Auxiliary\Build\vcvars64.bat

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

Build takes ~5-10 minutes. All 345 targets should succeed.

A convenience batch script is also provided:

```bat
3rdParty\gtsam-develop\build_gtsam.bat
```

The script does the following automatically:
1. Auto-detects VS Build Tools via `vswhere.exe` (falls back to common paths)
2. Calls `vcvars64.bat` to set up the MSVC compiler environment
3. **Prepends** the micromamba `sfm3rV2` env to `PATH` so Python/Boost/pybind11 are found
4. Runs CMake configure + Ninja build

> **Note:** The `PATH` prepend in `build_gtsam.bat` is critical. `vcvars64.bat`
> overwrites `PATH`, removing the Python env. The script must set the sfm3rV2
> paths **after** calling `vcvars64.bat`. If you change env names, edit the
> `set "PATH=D:\MICROMAMBA\envs\sfm3rV2;..."` line in `build_gtsam.bat`.

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
| pybind11 `stl_bind.h` C2001/C2679 errors | **pybind11 3.x is incompatible.** Downgrade: `micromamba install -c conda-forge "pybind11>=2.13,<3" "pybind11-global>=2.13,<3" -y` |
| pybind11 `stl_bind.h` error with MSVC 14.44 | Set `-DGTSAM_USE_SYSTEM_PYBIND=ON`, install pybind11 2.13.x from conda-forge |
| CMake: "No CMAKE_CXX_COMPILER could be found" | `vcvars64.bat` not found or not called. Install VS Build Tools "Desktop development with C++". Check path with `vswhere` |
| CMake: "CMAKE_MAKE_PROGRAM is not set" (Ninja) | `micromamba install -c conda-forge ninja -y` |
| `Python was not found` during GTSAM build | `vcvars64.bat` overwrote PATH. Edit `build_gtsam.bat` to prepend sfm3rV2 env **after** vcvars call |
| GTSAM build: missing `gtsam/` or `python/gtsam/` | Source tree is incomplete. Re-clone from `borglab/gtsam` develop branch (see step 6.1) |
| Boost DLL not found at runtime | Ensure `Library/bin/` is on `PATH`, or apply the DLL copy patch above |
| `np.nan` AttributeError on import | Apply the NumPy 2.x patch (step 6.2d) |
| `gtsam-develop` pip install fails | Expected on Windows — build from source (step 6) |
| xformers `NotImplementedError` on SM 12.0+ GPU | `pip uninstall xformers -y` — DINOv2 uses PyTorch SDPA fallback |
| `dino_salad.ckpt` not found | Download checkpoint (step 7) |
| `FutureWarning: torch.cuda.amp.autocast` | Informational only — already patched in `3rdParty/vggt` |
| vggt `numpy<2` constraint | Edit `3rdParty/vggt/pyproject.toml`: change `"numpy<2"` to `"numpy>=2.0"` |
| `DLL load failed: An Application Control policy has blocked this file` (gtsam) | **Windows Smart App Control** blocks unsigned locally-compiled DLLs. Turn it off: Settings → Privacy & Security → Windows Security → App & Browser Control → Smart App Control settings → **Off**. Note: once off it cannot be re-enabled without a Windows reset. |
