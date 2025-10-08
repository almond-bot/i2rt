"""
Helper utilities for LeRobot integration with I2RT robots.

This module provides shared normalization functions and constants used by both
follower robots and leader teleoperators.
"""

import numpy as np
import time

from i2rt.robots.motor_chain_robot import MotorChainRobot

# YAM arm motor names (6 joints) + gripper
YAM_ARM_MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "wrist_rotate",
    "gripper",
]


class YAMLeaderRobot:
    def __init__(self, robot: MotorChainRobot):
        self._robot = robot
        self._motor_chain = robot.motor_chain

    def get_info(self) -> np.ndarray:
        qpos = self._robot.get_observations()["joint_pos"]
        encoder_obs = self._motor_chain.get_same_bus_device_states()
        time.sleep(0.01)
        gripper_cmd = 1 - encoder_obs[0].position
        qpos_with_gripper = np.concatenate([qpos, [gripper_cmd]])
        return qpos_with_gripper, encoder_obs[0].io_inputs

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        assert joint_pos.shape[0] == 6
        self._robot.command_joint_pos(joint_pos)

    def update_kp_kd(self, kp: np.ndarray, kd: np.ndarray) -> None:
        self._robot.update_kp_kd(kp, kd)


def normalize_arm_position(val_rad: float, min_rad: float, max_rad: float, use_degrees: bool) -> float:
    """Normalize arm joint position from radians to degrees or -100 to 100 range.

    Args:
        val_rad: Joint position in radians
        min_rad: Minimum joint limit in radians
        max_rad: Maximum joint limit in radians
        use_degrees: If True, normalize to degrees; if False, normalize to -100 to 100 range

    Returns:
        Normalized position (degrees or -100 to 100 range)
    """
    if use_degrees:
        return val_rad * 180.0 / np.pi
    else:
        bounded_val = min(max_rad, max(min_rad, val_rad))
        return (((bounded_val - min_rad) / (max_rad - min_rad)) * 200.0) - 100.0


def denormalize_arm_position(normalized_val: float, min_rad: float, max_rad: float, use_degrees: bool) -> float:
    """Denormalize arm joint position from degrees or -100 to 100 range to radians.

    Args:
        normalized_val: Normalized position (degrees or -100 to 100 range)
        min_rad: Minimum joint limit in radians
        max_rad: Maximum joint limit in radians
        use_degrees: If True, denormalize from degrees; if False, denormalize from -100 to 100 range

    Returns:
        Position in radians
    """
    if use_degrees:
        return normalized_val * np.pi / 180.0
    else:
        return ((normalized_val + 100.0) / 200.0) * (max_rad - min_rad) + min_rad


def normalize_gripper_position(val_0_1: float) -> float:
    """Normalize gripper position from 0-1 range to 0-100 range.

    Args:
        val_0_1: Gripper position where 0=closed, 1=open

    Returns:
        Normalized position where 0=closed, 100=open
    """
    return val_0_1 * 100.0


def denormalize_gripper_position(normalized_val: float) -> float:
    """Denormalize gripper position from 0-100 range to 0-1 range.

    Args:
        normalized_val: Normalized position (0=closed, 100=open)

    Returns:
        Position in 0-1 range (0=closed, 1=open)
    """
    return normalized_val / 100.0
