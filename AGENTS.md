# AGENTS.md - Development Guide for Code Agents

## Project Overview
**vision_evaluation_tools** is a Python/C++ toolkit for extracting and evaluating SLAM system trajectories using Structure from Motion (SfM) data and KITTI format evaluation.

---

## Build, Lint & Test Commands

### Python Scripts
The project consists primarily of standalone Python scripts. No unified test framework is configured.

**Running individual scripts:**
```bash
python <script_name>.py
```

**Main Python entry points:**
- `sfm_evaluation_tools/transformations/align_slam_to_gt.py` - Align SLAM trajectory to ground truth
- `Extract images from bags/stereo/save_stereo.py` - Extract stereo images from ROS bags
- `qualitative.py` - Qualitative analysis
- `helpers.py` - Utility functions
- `utils.py` - Utility functions

### C++ Evaluation Tools
Located in `sfm_evaluation_tools/cpp/` and `kitti_tools/cpp/`

**Build:**
```bash
cd sfm_evaluation_tools/cpp
./build.sh
```

**Run evaluation:**
```bash
./evaluate_odometry [result_sha] [sequence_file_name]
./evaluate_odometry_avg [result_sha] [sequence_file_name]
```

### External Dependencies
**System packages required:**
```bash
sudo apt-get install gnuplot
sudo apt-get install texlive-extra-utils
```

**Python dependencies:**
```bash
pip install -r requirements.txt
```

---

## Code Style Guidelines

### Imports
- Use absolute imports from standard library first, then third-party, then local modules
- Group imports with blank lines between sections
- Common imports follow this pattern:
  ```python
  import numpy as np
  import json
  import sys
  import math
  import matplotlib.pyplot as plt
  from collections import Counter
  ```
- ROS-related imports (rosbag, cv_bridge, rosbags) should be grouped separately

### Formatting & Types
- **Type hints:** Use type annotations in function signatures (see `utils.py` examples)
  ```python
  def compare_trajectories(traj1: list, traj2: list, attribute: str, attribute2: str, tolerance: float) -> list:
  ```
- **Line length:** Keep lines reasonable, no strict enforced limit but aim for readability
- **Whitespace:** Standard 4-space indentation (not tabs)
- **Naming conventions:**
  - Snake_case for functions and variables: `compare_trajectories`, `get_values`
  - CamelCase for classes: `GlobalCounters`, `ContextVar`, `Timing`, `Profiling`
  - UPPER_CASE for constants: `DEBUG`

### Naming Conventions
- Descriptive names for functions: `align()`, `compare_trajectories()`, `merge_dicts()`
- Trajectory-related variables: use consistent naming like `traj`, `model`, `data`, `poses`
- Use `gt` for ground truth data
- Use `slam` for SLAM system output

### Error Handling
- Use assertions for invariant checks:
  ```python
  assert key not in ContextVar._cache, f"attempt to recreate ContextVar {key}"
  assert len(kvs) == len(set(kv[0] for kv in kvs)), f"cannot merge, {kvs} contains..."
  ```
- Use descriptive assertion messages with context
- For file operations, use context managers (`with` statements)
- Return tuples for multiple outputs: `return corresponding_points, corresponding_orientation`

### Documentation
- Add module-level docstrings for scripts:
  ```python
  """ Align Poses to Ground Truth
  
  This script allows to align the poses of a SLAM System to a GT extracted from AliceVision.
  """
  ```
- Add docstrings to functions with parameter and output descriptions
- Comments should explain the "why", not just the "what"

### Key Dependencies
- **numpy** - Numerical operations and trajectory data
- **opencv** - Image processing (opencv-python, opencv-python-headless)
- **matplotlib** - Visualization and plotting
- **rosbag/rosbags** - ROS data extraction
- **sophuspy** - Lie group operations for pose alignment (SO(3), SE(3))
- **cv_bridge** - ROS-OpenCV bridge
- **PyYAML** - Configuration file parsing

### Common Patterns
- Use `np.random.uniform()` for random sampling
- Use `np.linalg.svd()` for SVD decomposition in pose alignment
- Use `with open()` for file I/O
- Trajectory data stored as dictionaries with keys: `position`, `orientation`, `timestamp`, `nanosecond`
- KITTI format output: space-separated pose matrices in .txt files

### Directory Structure
```
.
├── sfm_evaluation_tools/
│   ├── cpp/              # C++ KITTI evaluation tools
│   ├── transformations/  # Pose alignment scripts
│   ├── sfm_to_csv.py     # SfM data conversion
│   └── gnss_to_csv.py    # GNSS data conversion
├── Extract images from bags/  # ROS bag image extraction
│   └── stereo/
├── kitti_tools/          # KITTI dataset tools
│   └── cpp/
├── helpers.py            # Utility functions
├── utils.py              # Utility functions
└── requirements.txt      # Python dependencies
```

---

## General Notes for Agents
- This is a research/evaluation toolkit, not production code
- Some scripts contain hardcoded paths (e.g., in `qualitative.py`) - update these when running in different environments
- The code focuses on pose/trajectory transformation and alignment accuracy
- ROS 1/2 compatibility: some scripts use ROS bags (rosbag) with ROS 1 format
- Tests are typically manual; verify outputs visually with matplotlib plots
