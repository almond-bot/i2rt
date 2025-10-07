import time

import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from scripts.minimum_gello import YAMLeaderRobot
from i2rt.robots.utils import GripperType

FREQUENCY = 100  # Hz
BILATERAL_KP = 0.2  # Bilateral force feedback strength (0.0 = no feedback, 1.0 = full feedback)

leader_l_robot = YAMLeaderRobot(get_yam_robot(channel="can_leader_l", gripper_type=GripperType.YAM_TEACHING_HANDLE, zero_gravity_mode=True))
leader_r_robot = YAMLeaderRobot(get_yam_robot(channel="can_leader_r", gripper_type=GripperType.YAM_TEACHING_HANDLE, zero_gravity_mode=True))
follower_l_robot = get_yam_robot(channel="can_follower_l", gripper_type=GripperType.LINEAR_4310, zero_gravity_mode=False)
follower_r_robot = get_yam_robot(channel="can_follower_r", gripper_type=GripperType.LINEAR_4310, zero_gravity_mode=False)

# Store initial kp/kd values for bilateral control
leader_l_kp = leader_l_robot._robot._kp.copy()
leader_r_kp = leader_r_robot._robot._kp.copy()

print("Initializing force feedback...")

# Get initial positions
leader_l_jp, _ = leader_l_robot.get_info()
leader_r_jp, _ = leader_r_robot.get_info()
follower_l_jp = follower_l_robot.get_joint_pos()
follower_r_jp = follower_r_robot.get_joint_pos()

print(f"Initial leader left pos: {leader_l_jp[:6]}")
print(f"Initial follower left pos: {follower_l_jp[:6]}")
print(f"Initial leader right pos: {leader_r_jp[:6]}")
print(f"Initial follower right pos: {follower_r_jp[:6]}")

# Enable bilateral control on both arms
print("Enabling force feedback on both arms...")
leader_l_robot.update_kp_kd(kp=leader_l_kp * BILATERAL_KP, kd=np.ones(6) * 0.0)
leader_r_robot.update_kp_kd(kp=leader_r_kp * BILATERAL_KP, kd=np.ones(6) * 0.0)

# Synchronize follower positions to leader positions
print("Synchronizing follower positions...")
steps = int(1.0 * FREQUENCY)  # 1 second duration
for i in range(steps):
    t = (i + 1) / steps
    interpolated_l = leader_l_jp * t + follower_l_jp * (1 - t)
    interpolated_r = leader_r_jp * t + follower_r_jp * (1 - t)
    follower_l_robot.command_joint_pos(interpolated_l)
    follower_r_robot.command_joint_pos(interpolated_r)
    time.sleep(1 / FREQUENCY)

print("Force feedback enabled! Starting teleoperation...")

try:
    i = 0
    
    while True:
        start = time.time()
        
        # Get leader states
        leader_l_jp, _ = leader_l_robot.get_info()
        leader_r_jp, _ = leader_r_robot.get_info()
        
        # Get follower states
        follower_l_jp = follower_l_robot.get_joint_pos()
        follower_r_jp = follower_r_robot.get_joint_pos()
        
        # Command followers with leader positions
        follower_l_robot.command_joint_pos(leader_l_jp)
        follower_r_robot.command_joint_pos(leader_r_jp)
        
        # Apply bilateral force feedback (command leader with follower positions)
        leader_l_robot.command_joint_pos(follower_l_jp[:6])
        leader_r_robot.command_joint_pos(follower_r_jp[:6])
        
        i += 1
        
        # Print status periodically
        if i % (FREQUENCY * 5) == 0:  # Every 5 seconds
            print(f"Running... (iteration {i})")
        
        end = time.time()
        if end - start < 1 / FREQUENCY:
            time.sleep(1 / FREQUENCY - (end - start))
            
except KeyboardInterrupt:
    print("\nStopping teleoperation...")
finally:
    # Reset leader robots to zero gravity mode before closing
    leader_l_robot.update_kp_kd(kp=np.ones(6) * 0.0, kd=np.ones(6) * 0.0)
    leader_r_robot.update_kp_kd(kp=np.ones(6) * 0.0, kd=np.ones(6) * 0.0)
    
    leader_l_robot._robot.close()
    leader_r_robot._robot.close()
    follower_l_robot.close()
    follower_r_robot.close()
    print("All robots closed.")
