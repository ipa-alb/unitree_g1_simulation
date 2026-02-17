# LeRobot Unitree G1 MuJoCo Sim — Setup Guide

Standalone MuJoCo physics simulator for the Unitree G1 humanoid (29 DOF), from [lerobot/unitree-g1-mujoco](https://huggingface.co/lerobot/unitree-g1-mujoco). Adapted from NVIDIA's `gr00t_wbc`.

## Prerequisites

- Python 3.10+
- git
- cmake
- A GPU with OpenGL support (for the MuJoCo viewer)

## Quick Start (automated)

```bash
cd /home/alb/workspace/unitree_sim/hugging_face
./init.sh
```

The script handles everything below automatically. If you need to do it manually or on a fresh machine, follow the steps below.

## Manual Setup

### 1. Create a virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
```

### 2. Download the sim from HuggingFace

```bash
pip install huggingface_hub

python3 -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='lerobot/unitree-g1-mujoco', local_dir='./unitree-g1-mujoco')
"
```

This downloads ~60MB: MuJoCo XML scenes, STL meshes, and Python sim code.

### 3. Build CycloneDDS from source

The `unitree_sdk2py` package requires CycloneDDS 0.10.x. It is **not** on PyPI — you must build it:

```bash
git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git
mkdir -p cyclonedds/build
cmake -S cyclonedds -B cyclonedds/build -DCMAKE_INSTALL_PREFIX=cyclonedds/install
cmake --build cyclonedds/build --target install
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

### 4. Install unitree_sdk2_python

This is **not on PyPI** either — install from GitHub:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -e unitree_sdk2_python
```

`CYCLONEDDS_HOME` must be set before running `pip install`.

### 5. Install Python dependencies

From the repo's requirements.txt (skipping `unitree-sdk2py` since we installed it from source):

```bash
pip install "mujoco>=3.0.0" "numpy>=1.24.0" "pyyaml>=6.0" "loguru>=0.7.0"
pip install "opencv-python>=4.8.0" "pyzmq>=25.0.0" "msgpack>=1.0.0" "msgpack-numpy>=0.4.8" "matplotlib>=3.5.0"
```

Undeclared dependencies (used in code but missing from requirements.txt):

```bash
pip install gymnasium pygame scipy termcolor
```

## Running

### Environment variable

Always set this before running:

```bash
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

Or add it to your shell profile.

### Launch the simulator

```bash
source venv/bin/activate
cd unitree-g1-mujoco
python run_sim.py
```

This opens a MuJoCo viewer window with the G1 robot standing with an elastic band support.

### View live cameras (in a second terminal)

```bash
source venv/bin/activate
cd unitree-g1-mujoco
python view_cameras_live.py
```

## Controls

### With a gamepad (Xbox/Switch)

Set in `config.yaml`:
```yaml
USE_JOYSTICK: 1
JOYSTICK_TYPE: "xbox"   # or "switch"
```

### Without a gamepad (keyboard fallback)

If no gamepad is detected, the sim automatically falls back to keyboard control via the MuJoCo viewer window:

| Key | Action |
|-----|--------|
| W / S | Left stick Y (forward/back) |
| A / D | Left stick X (strafe) |
| Arrow Up / Down | Right stick Y |
| Arrow Left / Right | Right stick X |
| Enter | Start |
| Tab | Select |
| J / K / U / I | A / B / X / Y buttons |
| Q / E | L1 / R1 |
| Z / C | L2 / R2 |
| 7 / 8 | Lower / raise elastic band |
| 9 | Toggle elastic band on/off |
| Backspace | Reset simulation |
| V | Toggle camera tracking |

**Note:** Keys must be pressed while the MuJoCo viewer window has focus.

## Configuration

Edit `unitree-g1-mujoco/config.yaml`:

| Setting | Default | Description |
|---------|---------|-------------|
| `ROBOT_TYPE` | `g1_29dof` | Robot model |
| `ROBOT_SCENE` | `assets/scene_43dof.xml` | MuJoCo scene file |
| `INTERFACE` | `lo` | `lo` for sim, network interface for real robot |
| `USE_JOYSTICK` | `1` | Enable gamepad input |
| `ENABLE_ONSCREEN` | `true` | MuJoCo viewer window (set `false` for headless) |
| `ENABLE_OFFSCREEN` | `false` | Offscreen rendering for cameras |
| `ENABLE_ELASTIC_BAND` | `true` | Support band to keep robot upright |
| `SIMULATE_DT` | `0.004` | Physics timestep (250Hz) |
| `VIEWER_DT` | `0.02` | Viewer refresh (50Hz) |
| `IMAGE_DT` | `0.033333` | Camera publish rate (~30Hz) |

## Using as a Gymnasium Environment

```python
from env import make_env

env = make_env()
obs, info = env.reset()

# Simulation loop
while True:
    obs, reward, terminated, truncated, info = env.step(action=None)
```

- **Action space:** `Box(-pi, pi, shape=(29,))` — joint positions
- **Observation space:** `Box(-inf, inf, shape=(97,))` — 29x3 joint states + 10 base states

## Headless Mode

For running without a display (e.g. SSH, CI):

```yaml
ENABLE_ONSCREEN: false
USE_JOYSTICK: 0
```

## Project Structure

```
hugging_face/
├── init.sh                          # Automated setup script
├── SETUP.md                         # This file
├── venv/                            # Python virtual environment
├── cyclonedds/                      # CycloneDDS build (0.10.x)
│   └── install/                     # CYCLONEDDS_HOME points here
├── unitree_sdk2_python/             # Unitree SDK (editable install)
└── unitree-g1-mujoco/               # Downloaded from HuggingFace
    ├── run_sim.py                   # Main entry point
    ├── env.py                       # Gymnasium environment wrapper
    ├── config.yaml                  # Simulation configuration
    ├── view_cameras_live.py         # Live camera viewer
    ├── assets/                      # MuJoCo XMLs + STL meshes
    │   └── scene_43dof.xml
    └── sim/                         # Simulation internals
        ├── base_sim.py              # DefaultEnv + BaseSimulator
        ├── simulator_factory.py     # Factory for creating simulators
        ├── unitree_sdk2py_bridge.py # DDS bridge + keyboard/joystick input
        ├── image_publish_utils.py   # ZMQ camera publishing
        ├── metric_utils.py          # Collision detection
        └── sim_utils.py             # Helper utilities
```

## Troubleshooting

**`Could not locate cyclonedds`** — `CYCLONEDDS_HOME` not set. Run:
```bash
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

**`No module named 'unitree_sdk2py'`** — SDK not installed. Re-run step 4 with `CYCLONEDDS_HOME` set.

**Core dump on startup** — No display available. Either run from a desktop terminal or set `ENABLE_ONSCREEN: false` in config.yaml.

**`No gamepad detected`** — This is fine. The sim falls back to keyboard control automatically.

## Links

- [lerobot/unitree-g1-mujoco](https://huggingface.co/lerobot/unitree-g1-mujoco) — HuggingFace repo
- [LeRobot Unitree G1 docs](https://huggingface.co/docs/lerobot/en/unitree_g1) — Full robot setup guide (physical + sim)
- [LeRobot GitHub](https://github.com/huggingface/lerobot) — LeRobot framework
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) — Unitree SDK
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — DDS middleware
