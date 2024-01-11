import pybullet as p
import time
import pybullet_data

def print_joint_info(robot):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        # print(f"Joint {i}: {joint_info[1]} {joint_info[2]}")
        print(f"Joint {joint_info[0]}: \n \t Name: {joint_info[1]} \n \t Type: {joint_info[2]}")

"""
Joint 0: 
         Name: b'arm_joint_0'
         Type: 4 - FIXED
Joint 1:
         Name: b'arm_joint_1'
         Type: 0
Joint 2:
         Name: b'arm_joint_2'
         Type: 0
Joint 3:
         Name: b'arm_joint_3'
         Type: 0
Joint 4:
         Name: b'arm_joint_4'
         Type: 0
Joint 5:
         Name: b'arm_joint_5'
         Type: 4 - FIXED
"""

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0.4]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("arm.urdf", startPos, startOrientation)
print_joint_info(boxId)
