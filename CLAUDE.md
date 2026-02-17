# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Standalone MuJoCo physics simulator for the Unitree G1 humanoid robot (29 DOF), downloaded from [lerobot/unitree-g1-mujoco](https://huggingface.co/lerobot/unitree-g1-mujoco). Adapted from NVIDIA's `gr00t_wbc`. Part of the LeRobot ecosystem (Hugging Face). Uses CycloneDDS for communication identical to the real robot protocol.

## Common Commands

```bash
# Activate environment (always do first)
source venv/bin/activate
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"

# Run simulator (opens MuJoCo viewer)
cd unitree-g1-mujoco && python run_sim.py

# View live cameras (separate terminal, while sim is running)
cd unitree-g1-mujoco && python view_cameras_live.py

# Control hands (separate terminal, while sim is running)
cd unitree-g1-mujoco && python hand_controller.py --action close   # or open, wave

# Full automated setup from scratch
./init.sh
```

There are no tests, linting, or CI configured in this project.

## Architecture

The sim follows a layered architecture with DDS-based communication:

```
run_sim.py / env.py
  └─ SimulatorFactory.create_simulator()  [sim/simulator_factory.py]
       └─ BaseSimulator                   [sim/base_sim.py]
            ├─ DefaultEnv                  [sim/base_sim.py]
            │    └─ MuJoCo model + data, PD torque control, elastic band
            ├─ UnitreeSdk2Bridge           [sim/unitree_sdk2py_bridge.py]
            │    └─ DDS pub/sub (LowState, LowCmd), keyboard/joystick input
            └─ ImagePublishProcess         [sim/image_publish_utils.py]
                 └─ Shared memory → ZMQ PUB (tcp://localhost:5555)
```

**Simulation step flow:**
1. `UnitreeSdk2Bridge` receives `LowCmd` (joint targets + PD gains) over DDS
2. `DefaultEnv.sim_step()` computes PD torques, clamps to limits, calls `mujoco.mj_step()`
3. State (joint pos/vel, IMU, torques) published back via DDS as `LowState`
4. Camera images rendered offscreen → shared memory → ZMQ subprocess

**Key design principle:** DDS-first — all control/state flows through CycloneDDS, so the same controller code works in sim and on the real robot by changing `INTERFACE` in `config.yaml` (`lo` for sim, real NIC for hardware).

## Key Files

| File | Role |
|------|------|
| `unitree-g1-mujoco/hand_controller.py` | DDS hand controller (open/close/wave) — run in separate terminal |
| `unitree-g1-mujoco/run_sim.py` | Main CLI entry point |
| `unitree-g1-mujoco/env.py` | Gymnasium wrapper (`make_env()`) — action: 29 joint positions, obs: 97-dim |
| `unitree-g1-mujoco/config.yaml` | All simulation settings (timesteps, robot type, interface, rendering) |
| `unitree-g1-mujoco/sim/base_sim.py` | Core: `DefaultEnv` (physics) + `BaseSimulator` (main loop + threading) |
| `unitree-g1-mujoco/sim/unitree_sdk2py_bridge.py` | DDS bridge + `ElasticBand` + keyboard/joystick input handling |
| `unitree-g1-mujoco/sim/simulator_factory.py` | Factory + `init_channel()` for DDS initialization |
| `unitree-g1-mujoco/sim/image_publish_utils.py` | Multiprocess camera pipeline (shared memory + ZMQ) |
| `unitree-g1-mujoco/sim/sensor_utils.py` | ZMQ PUB/SUB utilities + image codec helpers |

## Configuration

Edit `unitree-g1-mujoco/config.yaml`. Key settings:
- `ROBOT_SCENE`: MuJoCo XML scene (`scene_29dof.xml` or `scene_43dof.xml`)
- `INTERFACE`: `lo` for sim, real network interface for hardware
- `SIMULATE_DT`: physics timestep (default 0.004 = 250Hz)
- `ENABLE_ONSCREEN`: set `false` for headless operation
- `ENABLE_ELASTIC_BAND`: virtual support spring on the torso
- `USE_JOYSTICK`: `1` for gamepad, `0` falls back to keyboard via GLFW callbacks

## Dependencies

- Python 3.10+ with venv at `./venv/`
- CycloneDDS 0.10.x built from source at `./cyclonedds/install/` (requires `CYCLONEDDS_HOME` env var)
- `unitree_sdk2_python` installed as editable from `./unitree_sdk2_python/`
- MuJoCo >= 3.0, numpy, pygame, gymnasium, scipy, opencv, pyzmq, msgpack
