import time

from i2rt.robots.get_robot import get_yam_robot
from scripts.minimum_gello import YAMLeaderRobot
from i2rt.robots.utils import GripperType

FREQUENCY = 100 # Hz

leader_l_robot = YAMLeaderRobot(get_yam_robot(channel="can_leader_l", gripper_type=GripperType.YAM_TEACHING_HANDLE, zero_gravity_mode=True))
leader_r_robot = YAMLeaderRobot(get_yam_robot(channel="can_leader_r", gripper_type=GripperType.YAM_TEACHING_HANDLE, zero_gravity_mode=True))
follower_l_robot = get_yam_robot(channel="can_follower_l", gripper_type=GripperType.LINEAR_4310, zero_gravity_mode=False)
follower_r_robot = get_yam_robot(channel="can_follower_r", gripper_type=GripperType.LINEAR_4310, zero_gravity_mode=False)

try:
    i = 0
    while True:
        start = time.time()
        leader_l_jp = leader_l_robot.get_info()[0]
        follower_l_robot.command_joint_pos(leader_l_jp)

        leader_r_jp = leader_r_robot.get_info()[0]
        follower_r_robot.command_joint_pos(leader_r_jp)

        i += 1

        end = time.time()
        if end - start < 1 / FREQUENCY:
            time.sleep(1 / FREQUENCY - (end - start))
except KeyboardInterrupt:
    pass
finally:
    leader_l_robot._robot.close()
    leader_r_robot._robot.close()
    follower_l_robot.close()
    follower_r_robot.close()
