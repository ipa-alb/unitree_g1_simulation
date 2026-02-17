# Unitree G1 MuJoCo Simulation

Standalone MuJoCo physics simulator for the Unitree G1 humanoid robot (29 DOF), adapted from [NVIDIA's gr00t_wbc](https://github.com/NVIDIA-AI-IOT/gr00t_wbc). Downloaded from [lerobot/unitree-g1-mujoco](https://huggingface.co/lerobot/unitree-g1-mujoco). Part of the [LeRobot](https://github.com/huggingface/lerobot) ecosystem.

Uses CycloneDDS for communication, identical to the real robot protocol — the same controller code works in sim and on hardware.

## Prerequisites

- Python 3.10+
- git
- cmake
- A C compiler (gcc/clang)

## Installation

Clone the repo and run the setup script:

```bash
git clone <your-repo-url> unitree_g1_simulation
cd unitree_g1_simulation
./init.sh
```

`init.sh` will automatically:
1. Create a Python virtual environment (`venv/`)
2. Download the MuJoCo model from HuggingFace
3. Build CycloneDDS 0.10.x from source
4. Clone and install `unitree_sdk2_python`
5. Install all remaining Python dependencies

## Usage

### Activate the environment

Always activate before running anything:

```bash
source venv/bin/activate
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

### Run the simulator

```bash
cd unitree-g1-mujoco && python run_sim.py
```

### View live cameras (separate terminal)

```bash
cd unitree-g1-mujoco && python view_cameras_live.py
```

### Control hands (separate terminal)

```bash
cd unitree-g1-mujoco && python hand_controller.py --action close   # or open, wave
```

### Use as a Gymnasium environment

```python
from env import make_env

env = make_env()
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

## Configuration

Edit `unitree-g1-mujoco/config.yaml`:

| Setting | Description | Default |
|---------|-------------|---------|
| `ROBOT_SCENE` | MuJoCo XML scene | `scene_29dof.xml` |
| `INTERFACE` | `lo` for sim, real NIC for hardware | `lo` |
| `SIMULATE_DT` | Physics timestep | `0.004` (250Hz) |
| `ENABLE_ONSCREEN` | Set `false` for headless | `true` |
| `ENABLE_ELASTIC_BAND` | Virtual support spring on torso | `true` |
| `USE_JOYSTICK` | `1` for gamepad, `0` for keyboard | `0` |
| `JOYSTICK_TYPE` | `xbox` or `switch` | `xbox` |
