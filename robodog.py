import pybullet as p
import time
import pybullet_data
import kinpy as kp

"""
JointType:
I suppose the following:
0 - JOINT_REVOLUTE
1 - JOINT_PRISMATIC
2 - JOINT_SPHERICAL
3 - JOINT_PLANAR
4 - JOINT_FIXED

"""

"""
Joint 2: b'FR_hip_joint' 0
Joint 3: b'FR_thigh_joint' 0
Joint 4: b'FR_calf_joint' 0
Joint 5: b'FR_foot_fixed' 4
Joint 6: b'FR_calf_rotor_joint' 4
Joint 7: b'FR_thigh_rotor_joint' 4
Joint 8: b'FR_hip_rotor_joint' 4
Joint 9: b'FL_hip_joint' 0
Joint 10: b'FL_thigh_joint' 0
Joint 11: b'FL_calf_joint' 0
Joint 12: b'FL_foot_fixed' 4
Joint 13: b'FL_calf_rotor_joint' 4
Joint 14: b'FL_thigh_rotor_joint' 4
Joint 15: b'FL_hip_rotor_joint' 4
Joint 16: b'RR_hip_joint' 0
Joint 17: b'RR_thigh_joint' 0
Joint 18: b'RR_calf_joint' 0
Joint 19: b'RR_foot_fixed' 4
Joint 20: b'RR_calf_rotor_joint' 4
Joint 21: b'RR_thigh_rotor_joint' 4
Joint 22: b'RR_hip_rotor_joint' 4
Joint 23: b'RL_hip_joint' 0
Joint 24: b'RL_thigh_joint' 0
Joint 25: b'RL_calf_joint' 0
Joint 26: b'RL_foot_fixed' 4
Joint 27: b'RL_calf_rotor_joint' 4
Joint 28: b'RL_thigh_rotor_joint' 4
Joint 29: b'RL_hip_rotor_joint' 4


Joint 2: b'FR_hip_joint' 0
Joint 3: b'FR_thigh_joint' 0
Joint 4: b'FR_calf_joint' 0
Joint 9: b'FL_hip_joint' 0
Joint 10: b'FL_thigh_joint' 0
Joint 11: b'FL_calf_joint' 0
Joint 16: b'RR_hip_joint' 0
Joint 17: b'RR_thigh_joint' 0
Joint 18: b'RR_calf_joint' 0
Joint 23: b'RL_hip_joint' 0
Joint 24: b'RL_thigh_joint' 0
Joint 25: b'RL_calf_joint' 0
"""


def print_joint_info(robot):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        # print(f"Joint {i}: {joint_info[1]} {joint_info[2]}")

        print("----------------------------")
        print(p.getBasePositionAndOrientation(robot))
        print(p.getLinkState(robot, i)[0])
        print(p.getJointInfo(robot, i))
        print(i)
        print("----------------------------")


# Starting from the torso down to the ground:
# FR: 2, 3, 4
# FL: 9, 10, 11
# RR: 16, 17, 18
# RL: 23, 24, 25
def set_joint_angles(fr, fl, rr, rl, maxForces=[500] * 12):
    p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=[2, 3, 4,  # FR
                      9, 10, 11,  # FL
                      16, 17, 18,  # RR
                      23, 24, 25],  # RL
        controlMode=p.POSITION_CONTROL,
        targetPositions=[fr[0], fr[1], fr[2],
                         fl[0], fl[1], fl[2],
                         rr[0], rr[1], rr[2],
                         rl[0], rl[1], rl[2]],
        forces=maxForces
    )

    return


physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.4]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("go1_description/urdf/go1.urdf", startPos, startOrientation)
# print_joint_info(boxId)
p.changeVisualShape(boxId, 0, rgbaColor=[0, 0, 0, 0.5])
num_joints = p.getNumJoints(boxId)

# Changing the joint angles
fr = [0, 0.8, -1.2]
fl = [0, 0.8, -1.2]
rr = [0, 0.8, -1.2]
rl = [0, 0.8, -1.2]
x = [p.getJointState(boxId, jointIndex)[0] for jointIndex in range(num_joints)]
zer_vec = [0.0] * num_joints
com = p.getLinkState(boxId, 0)[0]
print(len(com))
print(num_joints)
print(len(x))

print(p.calculateJacobian(boxId, 0, com, x, zer_vec, zer_vec))
# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range(100000):
    p.stepSimulation()
    time.sleep(1. / 240.)
    p.addUserDebugPoints([p.getBasePositionAndOrientation(boxId)[0]], [(255, 0, 0)], 10, 0.2)

    print(p.getBaseVelocity(boxId))
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

p.set
    # print(cubePos,cubeOrn)
p.disconnect()



