#!/usr/bin/env python3
"""
GR00T Whole-Body-Control locomotion controller for the Unitree G1 sim.

Downloads two ONNX policies from Hugging Face (Balance + Walk), reads robot
state over DDS (rt/lowstate), runs inference at 50 Hz, and publishes joint
commands back (rt/lowcmd).  Works with run_sim.py running in another terminal.

Usage:
    # Balance only (stand in place):
    python groot_controller.py

    # With a specific HF repo (default shown):
    python groot_controller.py --repo-id nepyope/GR00T-WholeBodyControl_g1

    # Use keyboard/joystick in the MuJoCo viewer to send velocity commands —
    # the controller automatically switches between Balance and Walk policies.

Before running, disable the elastic band in config.yaml:
    ENABLE_ELASTIC_BAND: False
"""
import argparse
import struct
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import onnxruntime as ort
import yaml
from huggingface_hub import hf_hub_download
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_ as LowCmd_default,
    unitree_hg_msg_dds__LowState_ as LowState_default,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ---------------------------------------------------------------------------
# Constants (matched to LeRobot gr00t_locomotion.py)
# ---------------------------------------------------------------------------
NUM_MOTORS = 29
NUM_OBS = 86          # observation dimension per frame
NUM_FRAMES = 6        # stacked history
NUM_ACTIONS = 15      # left leg(6) + right leg(6) + waist(3)
CONTROL_DT = 0.02     # 50 Hz

# Default standing pose — slightly crouched for stability
# (matches GROOT_DEFAULT_ANGLES from LeRobot)
DEFAULT_JOINT_POS = np.zeros(NUM_MOTORS, dtype=np.float32)
DEFAULT_JOINT_POS[[0, 6]] = -0.1    # Hip pitch (left, right)
DEFAULT_JOINT_POS[[3, 9]] = 0.3     # Knee (left, right)
DEFAULT_JOINT_POS[[4, 10]] = -0.2   # Ankle pitch (left, right)

# Joints to zero for g1_23 model variant (the policy was trained with these locked)
# WaistYaw(12), WaistPitch(14), LeftWristPitch(20), LeftWristYaw(21),
# RightWristPitch(27), RightWristYaw(28)
MISSING_JOINTS = [12, 14, 20, 21, 27, 28]

# PD gains (matched to LeRobot UnitreeG1Config)
KP = np.array([
    150, 150, 150, 300, 40, 40,      # left leg
    150, 150, 150, 300, 40, 40,      # right leg
    250, 250, 250,                    # waist
    80, 80, 80, 80,                   # left arm (shoulder_p/r/y, elbow)
    40, 40, 40,                       # left wrist (roll, pitch, yaw)
    80, 80, 80, 80,                   # right arm
    40, 40, 40,                       # right wrist
], dtype=np.float32)

KD = np.array([
    2, 2, 2, 4, 2, 2,                # left leg
    2, 2, 2, 4, 2, 2,                # right leg
    5, 5, 5,                          # waist
    3, 3, 3, 3,                       # left arm
    1.5, 1.5, 1.5,                    # left wrist
    3, 3, 3, 3,                       # right arm
    1.5, 1.5, 1.5,                    # right wrist
], dtype=np.float32)

# Observation scaling (matches GR00T WBC training)
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
ANG_VEL_SCALE = 0.25
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
ACTION_SCALE = 0.25


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def get_gravity_orientation(quaternion):
    """Compute projected gravity in body frame from IMU quaternion (w,x,y,z).

    This is the exact formula from LeRobot's UnitreeG1.get_gravity_orientation().
    """
    qw, qx, qy, qz = quaternion
    gravity = np.zeros(3, dtype=np.float32)
    gravity[0] = 2.0 * (-qz * qx + qw * qy)
    gravity[1] = -2.0 * (qz * qy + qw * qx)
    gravity[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return gravity


def parse_wireless_remote(raw_bytes):
    """Extract joystick axes and buttons from the wireless_remote field.

    The sim packs: struct.pack('<HHfffff', 0, keys, lx, rx, ry, 0.0, ly)
    Returns (commands, buttons) where commands = [vx, vy, vyaw].
    Signs are flipped to match LeRobot convention.
    """
    buttons = [0] * 16
    if len(raw_bytes) < 24:
        return np.zeros(3, dtype=np.float32), buttons

    data = bytes(raw_bytes[:24])
    keys = struct.unpack_from('H', data, 2)[0]
    for i in range(16):
        buttons[i] = (keys >> i) & 1

    lx = struct.unpack_from('f', data, 4)[0]
    rx = struct.unpack_from('f', data, 8)[0]
    ly = struct.unpack_from('f', data, 20)[0]

    # Match LeRobot sign convention:
    #   cmd[0] = ly        (forward/backward)
    #   cmd[1] = lx * -1   (left/right, sign-flipped)
    #   cmd[2] = rx * -1   (rotation, sign-flipped)
    commands = np.array([ly, -lx, -rx], dtype=np.float32)
    return commands, buttons


def download_models(repo_id):
    """Download Balance and Walk ONNX models from Hugging Face."""
    print(f"Downloading models from {repo_id}...")
    balance_path = hf_hub_download(
        repo_id=repo_id,
        filename="GR00T-WholeBodyControl-Balance.onnx",
    )
    walk_path = hf_hub_download(
        repo_id=repo_id,
        filename="GR00T-WholeBodyControl-Walk.onnx",
    )
    print(f"  Balance: {balance_path}")
    print(f"  Walk:    {walk_path}")
    return balance_path, walk_path


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
class GrootLocomotionController:
    def __init__(self, balance_model_path, walk_model_path):
        self.balance_session = ort.InferenceSession(
            balance_model_path, providers=["CPUExecutionProvider"]
        )
        self.walk_session = ort.InferenceSession(
            walk_model_path, providers=["CPUExecutionProvider"]
        )
        # Observation history (deque of 6 frames, matching LeRobot)
        self.obs_history = deque(maxlen=NUM_FRAMES)
        for _ in range(NUM_FRAMES):
            self.obs_history.append(np.zeros(NUM_OBS, dtype=np.float32))
        self.obs_stacked = np.zeros(NUM_FRAMES * NUM_OBS, dtype=np.float32)
        self.prev_actions = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self.height_cmd = 0.74  # default base height

    def step(self, joint_pos, joint_vel, ang_vel, gravity, commands, buttons):
        """Run one inference step. Returns 15-D joint position targets for
        legs (12) + waist (3)."""

        # Height adjustment via R1/R2 buttons
        if buttons[0]:  # R1 — raise
            self.height_cmd = min(self.height_cmd + 0.001, 1.00)
        if buttons[4]:  # R2 — lower
            self.height_cmd = max(self.height_cmd - 0.001, 0.50)

        # Zero out missing joints (g1_23 model variant)
        qj = joint_pos.copy().astype(np.float32)
        dqj = joint_vel.copy().astype(np.float32)
        for idx in MISSING_JOINTS:
            qj[idx] = 0.0
            dqj[idx] = 0.0

        # Scale
        qj_obs = (qj - DEFAULT_JOINT_POS) * DOF_POS_SCALE
        dqj_obs = dqj * DOF_VEL_SCALE
        ang_vel_scaled = ang_vel * ANG_VEL_SCALE

        # Build single 86-D observation frame
        obs = np.zeros(NUM_OBS, dtype=np.float32)
        obs[0:3] = commands * CMD_SCALE
        obs[3] = self.height_cmd
        obs[4:7] = 0.0                     # orientation command (unused)
        obs[7:10] = ang_vel_scaled
        obs[10:13] = gravity
        obs[13:42] = qj_obs
        obs[42:71] = dqj_obs
        obs[71:86] = self.prev_actions

        # Append to history
        self.obs_history.append(obs.copy())

        # Stack all 6 frames into 516-D vector
        for i, frame in enumerate(self.obs_history):
            start = i * NUM_OBS
            self.obs_stacked[start:start + NUM_OBS] = frame

        obs_input = self.obs_stacked[np.newaxis, :].astype(np.float32)

        # Select policy: balance when nearly stationary, walk otherwise
        cmd_magnitude = np.linalg.norm(commands)
        if cmd_magnitude < 0.05:
            session = self.balance_session
        else:
            session = self.walk_session

        input_name = session.get_inputs()[0].name
        actions = session.run(None, {input_name: obs_input})[0].squeeze()

        self.prev_actions = actions.copy()

        # Convert deltas to absolute joint positions
        target_pos = DEFAULT_JOINT_POS[:NUM_ACTIONS].copy() + actions * ACTION_SCALE

        # Zero out missing joints in the action (WaistYaw=12, WaistPitch=14)
        for idx in MISSING_JOINTS:
            if idx < NUM_ACTIONS:
                target_pos[idx] = 0.0

        return target_pos


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="GR00T WBC locomotion controller")
    parser.add_argument(
        "--repo-id",
        default="nepyope/GR00T-WholeBodyControl_g1",
        help="Hugging Face model repo ID",
    )
    args = parser.parse_args()

    # Load config
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)

    if config.get("ENABLE_ELASTIC_BAND", True):
        print("WARNING: ENABLE_ELASTIC_BAND is True in config.yaml.")
        print("  The controller works best with it disabled.")
        print("  Set ENABLE_ELASTIC_BAND: False in config.yaml,")
        print("  or press 9 in the MuJoCo viewer to toggle it off.\n")

    # Download ONNX models
    balance_path, walk_path = download_models(args.repo_id)

    # Initialize DDS
    ChannelFactoryInitialize(config["DOMAIN_ID"], config["INTERFACE"])

    # Publisher for body joint commands
    lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    lowcmd_pub.Init()

    # Subscriber for robot state
    low_state = [None]

    def state_handler(msg: LowState_):
        low_state[0] = msg

    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init(state_handler, 10)

    crc = CRC()

    # Create controller
    controller = GrootLocomotionController(balance_path, walk_path)

    print("=" * 60)
    print("GR00T WBC Locomotion Controller")
    print("=" * 60)
    print(f"  Control rate:  {1.0 / CONTROL_DT:.0f} Hz")
    print(f"  Model:         {args.repo_id}")
    print(f"  DDS interface: {config['INTERFACE']}")
    print()
    print("Waiting for sim state (start run_sim.py if not running)...")

    # Wait for first state message
    while low_state[0] is None:
        time.sleep(0.1)

    print("Received state. Controller active!")
    print("Use keyboard/joystick in MuJoCo viewer to send walk commands.")
    print("R1 = raise waist, R2 = lower waist")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            t_start = time.time()
            state = low_state[0]
            if state is None:
                time.sleep(CONTROL_DT)
                continue

            # Read joint state
            joint_pos = np.array([state.motor_state[i].q for i in range(NUM_MOTORS)])
            joint_vel = np.array([state.motor_state[i].dq for i in range(NUM_MOTORS)])

            # IMU: quaternion (w,x,y,z) and angular velocity
            quat = np.array(state.imu_state.quaternion[:4])
            ang_vel = np.array(state.imu_state.gyroscope[:3])
            gravity = get_gravity_orientation(quat)

            # Velocity commands from wireless controller
            commands, buttons = parse_wireless_remote(state.wireless_remote)

            # Run policy
            target_pos_15 = controller.step(
                joint_pos, joint_vel, ang_vel, gravity, commands, buttons
            )

            # Build LowCmd
            cmd = LowCmd_default()
            cmd.mode_pr = 0  # PR mode
            cmd.mode_machine = 0

            # Policy-controlled joints (legs + waist, indices 0-14)
            for i in range(NUM_ACTIONS):
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q = float(target_pos_15[i])
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = float(KP[i])
                cmd.motor_cmd[i].kd = float(KD[i])
                cmd.motor_cmd[i].tau = 0.0

            # Arms hold default position with PD control
            for i in range(NUM_ACTIONS, NUM_MOTORS):
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q = float(DEFAULT_JOINT_POS[i])
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = float(KP[i])
                cmd.motor_cmd[i].kd = float(KD[i])
                cmd.motor_cmd[i].tau = 0.0

            cmd.crc = crc.Crc(cmd)
            lowcmd_pub.Write(cmd)

            # Maintain control rate
            elapsed = time.time() - t_start
            sleep_time = CONTROL_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        # Send zero-torque command to let robot go limp
        cmd = LowCmd_default()
        for i in range(NUM_MOTORS):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0
        cmd.crc = crc.Crc(cmd)
        lowcmd_pub.Write(cmd)
        print("\nController stopped.")


if __name__ == "__main__":
    main()
