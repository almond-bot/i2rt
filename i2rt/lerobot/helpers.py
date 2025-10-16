"""
Helper utilities for LeRobot integration with I2RT robots.

This module provides shared normalization functions and constants used by both
follower robots and leader teleoperators.
"""

import time
import logging
import time
from functools import partial

import numpy as np

import numpy as np

from i2rt.motor_drivers.dm_driver import (
    DMChainCanInterface,
    EncoderChain,
    PassiveEncoderReader,
    ReceiveMode,
    CanInterface,
)
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.utils import GripperType


YAM_ARM_MOTOR_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
]
YAM_EE_LINK = "link_6"


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

def get_encoder_chain(can_interface: CanInterface) -> EncoderChain:
    passive_encoder_reader = PassiveEncoderReader(can_interface)
    return EncoderChain([0x50E], passive_encoder_reader)

def get_yam_robot(
    channel: str = "can0",
    gripper_type: GripperType = GripperType.CRANK_4310,
    gripper_max_force: float = 50.0,
    zero_gravity_mode:bool = True,
) -> MotorChainRobot:
    with_gripper = True
    with_teaching_handle = False
    if gripper_type == GripperType.YAM_TEACHING_HANDLE:
        with_gripper = False
        with_teaching_handle = True
    if gripper_type == GripperType.NO_GRIPPER:
        with_gripper = False
        with_teaching_handle = False

    model_path = gripper_type.get_xml_path()
    motor_list = [
        [0x01, "DM4340"],
        [0x02, "DM4340"],
        [0x03, "DM4340"],
        [0x04, "DM4310"],
        [0x05, "DM4310"],
        [0x06, "DM4310"],
    ]
    motor_offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_limits = np.array([[-2.617, 3.13], [0, 3.65], [0.0, 3.13], [-1.57, 1.57], [-1.57, 1.57], [-2.09, 2.09]])
    joint_limits[:,0] += -0.15 # add some buffer to the joint limits
    joint_limits[:,1] += 0.15

    motor_directions = [1, 1, 1, 1, 1, 1]
    kp = np.array([80, 80, 80, 40, 10, 10])
    kd = np.array([5, 5, 5, 1.5, 1.5, 1.5])
    if with_gripper:
        motor_type = gripper_type.get_motor_type()
        gripper_kp, gripper_kd = gripper_type.get_motor_kp_kd()
        assert motor_type != ""
        logging.info(
            f"adding gripper motor with type: {motor_type}, gripper_kp: {gripper_kp}, gripper_kd: {gripper_kd}"
        )
        motor_list.append([0x07, motor_type])
        motor_offsets.append(0.0)
        motor_directions.append(1)
        kp = np.concatenate([kp, np.array([gripper_kp])])
        kd = np.concatenate([kd, np.array([gripper_kd])])

    motor_chain = DMChainCanInterface(
        motor_list,
        motor_offsets,
        motor_directions,
        channel,
        motor_chain_name="yam_real",
        receive_mode=ReceiveMode.p16,
        start_thread=False,
    )
    motor_states = motor_chain.read_states()
    print(f"motor_states: {motor_states}")
    motor_chain.close()

    current_pos = [m.pos for m in motor_states]
    logging.info(f"current_pos: {current_pos}")

    for idx, motor_state in enumerate(motor_states):
        motor_position = motor_state.pos
        # if not within -pi to pi, set to the nearest equivalent position
        if motor_position < -np.pi:
            logging.info(f"motor {idx} is at {motor_position}, adding {2 * np.pi}")
            extra_offset = -2 * np.pi
        elif motor_position > np.pi:
            logging.info(f"motor {idx} is at {motor_position}, subtracting {2 * np.pi}")
            extra_offset = +2 * np.pi
        else:
            extra_offset = 0.0
        motor_offsets[idx] += extra_offset

    time.sleep(0.5)
    logging.info(f"adjusted motor_offsets: {motor_offsets}")

    motor_chain = DMChainCanInterface(
        motor_list,
        motor_offsets,
        motor_directions,
        channel,
        motor_chain_name="yam_real",
        receive_mode=ReceiveMode.p16,
        get_same_bus_device_driver=get_encoder_chain if with_teaching_handle else None,
        use_buffered_reader=False,
    )
    motor_states = motor_chain.read_states()
    logging.info(f"YAM initial motor_states: {motor_states}")
    get_robot = partial(
        MotorChainRobot,
        motor_chain=motor_chain,
        xml_path=model_path,
        use_gravity_comp=True,
        gravity_comp_factor=1.3,
        joint_limits=joint_limits,
        kp=kp,
        kd=kd,
        zero_gravity_mode=zero_gravity_mode,
    )

    if with_gripper:
        return get_robot(
            gripper_index=6,
            gripper_limits=gripper_type.get_gripper_limits(),
            enable_gripper_calibration=gripper_type.get_gripper_needs_calibration(),
            gripper_type=gripper_type,
            limit_gripper_force=gripper_max_force,
        )
    else:
        return get_robot()
