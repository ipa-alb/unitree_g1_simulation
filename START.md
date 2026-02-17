# Start the Unitree G1 Sim

```bash
cd /path/to/unitree_g1_simulation
source venv/bin/activate
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
cd unitree-g1-mujoco
python run_sim.py
```

Stop with `Ctrl+C`.

## Keyboard Controls

Focus the MuJoCo viewer window, then:

| Key | Action |
|-----|--------|
| W/S | Forward / backward |
| A/D | Strafe left / right |
| Arrows | Right stick (look/turn) |
| Enter | Start |
| 7/8 | Lower / raise elastic band |
| 9 | Toggle elastic band on/off |
| Backspace | Reset sim |

## Hand Control

In a second terminal:

```bash
source venv/bin/activate && cd unitree-g1-mujoco

python hand_controller.py --action close   # make fists
python hand_controller.py --action open    # open hands
python hand_controller.py --action wave    # wave fingers open/close
```

## Camera Feed

In a second terminal:

```bash
source venv/bin/activate && cd unitree-g1-mujoco
python view_cameras_live.py
```

## Config Tweaks

Edit `unitree-g1-mujoco/config.yaml`:

```yaml
ENABLE_ONSCREEN: false       # headless (no viewer)
ENABLE_ELASTIC_BAND: False   # robot must self-balance
USE_JOYSTICK: 0              # skip gamepad detection
```

## Run a Locomotion Policy (GR00T WBC)

Make sure `ENABLE_ELASTIC_BAND: False` in `config.yaml`, then in a second terminal:

```bash
source venv/bin/activate && cd unitree-g1-mujoco

# Downloads Balance + Walk ONNX models from Hugging Face on first run
python groot_controller.py
```

Use keyboard (W/A/S/D) or joystick in the MuJoCo viewer to send walk commands.
The controller switches between Balance (standing) and Walk policies automatically.

For installation see [SETUP.md](SETUP.md).
