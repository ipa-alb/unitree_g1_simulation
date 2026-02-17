#!/usr/bin/env python3
"""
Hand controller for the Unitree G1 sim.
Run this in a separate terminal while run_sim.py is running.

Usage:
    python hand_controller.py                  # close both hands
    python hand_controller.py --action open    # open both hands
    python hand_controller.py --action wave    # wave fingers open/close
"""
import argparse
import math
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_ as HandCmd_default
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_

# Load config for DDS settings
config_path = Path(__file__).parent / "config.yaml"
with open(config_path) as f:
    config = yaml.safe_load(f)

# Initialize DDS
ChannelFactoryInitialize(config["DOMAIN_ID"], config["INTERFACE"])

# Create publishers for left and right hand
left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
left_pub.Init()
right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
right_pub.Init()

NUM_HAND_MOTORS = 7  # thumb(3) + middle(2) + index(2)

# Joint order: thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1
# The left and right hands are mirrored in the MuJoCo model — the close
# direction for fingers (middle/index) is negative on the left hand and
# positive on the right hand, and vice-versa for the thumb.
# PD gains — moderate stiffness for smooth motion
KP = 5.0
KD = 0.3

# Close direction per joint: +1 or -1 indicating which end of the joint
# range corresponds to a closed fist.
#                        thumb_0  thumb_1  thumb_2  middle_0  middle_1  index_0  index_1
LEFT_CLOSE_SIGN  = [ 1.0,  1.0,  1.0, -1.0, -1.0, -1.0, -1.0]
RIGHT_CLOSE_SIGN = [-1.0, -1.0, -1.0,  1.0,  1.0,  1.0,  1.0]


def send_hand_cmd(pub, positions):
    """Send a hand command with target positions using PD control."""
    cmd = HandCmd_default()
    for i in range(NUM_HAND_MOTORS):
        cmd.motor_cmd[i].q = positions[i]
        cmd.motor_cmd[i].kp = KP
        cmd.motor_cmd[i].kd = KD
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
    pub.Write(cmd)


def close_hand(close_sign):
    """All fingers curled inward (using per-joint close direction)."""
    return [s * 1.0 for s in close_sign]


def open_hand():
    """All fingers extended (zero position)."""
    return [0.0] * NUM_HAND_MOTORS


def main():
    parser = argparse.ArgumentParser(description="G1 hand controller")
    parser.add_argument(
        "--action", choices=["open", "close", "wave"], default="close",
        help="Hand action to perform (default: close)"
    )
    args = parser.parse_args()

    print(f"Action: {args.action}")
    print("Publishing hand commands. Ctrl+C to stop.")
    time.sleep(0.5)  # let DDS discover

    try:
        if args.action == "open":
            while True:
                send_hand_cmd(left_pub, open_hand())
                send_hand_cmd(right_pub, open_hand())
                time.sleep(0.02)

        elif args.action == "close":
            while True:
                send_hand_cmd(left_pub, close_hand(LEFT_CLOSE_SIGN))
                send_hand_cmd(right_pub, close_hand(RIGHT_CLOSE_SIGN))
                time.sleep(0.02)

        elif args.action == "wave":
            t = 0.0
            while True:
                # Sinusoidal open/close at ~0.5 Hz
                val = (math.sin(t * math.pi) + 1.0) / 2.0  # 0..1
                left_pos = [val * s for s in LEFT_CLOSE_SIGN]
                right_pos = [val * s for s in RIGHT_CLOSE_SIGN]
                send_hand_cmd(left_pub, left_pos)
                send_hand_cmd(right_pub, right_pos)
                t += 0.02
                time.sleep(0.02)

    except KeyboardInterrupt:
        # Send zero position on exit so hands relax
        send_hand_cmd(left_pub, open_hand())
        send_hand_cmd(right_pub, open_hand())
        print("\nStopped.")


if __name__ == "__main__":
    main()
