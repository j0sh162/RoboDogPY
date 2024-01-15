import pybullet as p
import time
import pybullet_data
import numpy as np
from spatialmath.base import angvec2r

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
        joint_info = p.getJointInfo(robot, i)
        # print(f"Joint {joint_info[0]}: \n \t Name: {joint_info[1]} \n \t Type: {joint_info[2]}")
        print(f"\n{joint_info}")


def print_joint_states(robot):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        joint_info = p.getJointState(robot, i)
        # print(f"Joint {joint_info[0]}: \n \t Name: {joint_info[1]} \n \t Type: {joint_info[2]}")
        print(f"\n{joint_info}")


def print_link_info(robot):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        link_info = p.getLinkState(robot, i)
        print(link_info)


def get_joint_axis(robot, joint_id):
    return p.getJointInfo(robot, joint_id)[13]


def get_joint_rotation(robot, joint_id):
    return p.getJointState(robot, joint_id)[0]


# joints = [2,3,4,9,10,11,16,17,18,23,24,25]
def get_joint_rotation_and_axis(robot, joint_id):
    return [get_joint_rotation(robot, joint_id), get_joint_axis(robot, joint_id)]


def get_joint_rotation_matrix(robot, joint_id):
    rot_axis = get_joint_rotation_and_axis(robot, joint_id)
    return angvec2r(rot_axis[0], rot_axis[1])


def get_adjoint(X, X_d):
    X = np.asmatrix(X)
    X_d = np.asmatrix(X_d)
    X_inv = np.linalg.inv(X)
    product = np.matmul(X_inv, X_d)
    return product.getH()


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


# physicsClient = p.connect(p.GUI)
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.4]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("go1_description/urdf/go1.urdf", startPos, startOrientation)

print_joint_info(boxId)
print("===============================================================================================================")
print("===============================================================================================================")
print("===============================================================================================================")
print(get_joint_rotation_matrix(boxId, 3))

# num_joints = p.getNumJoints(boxId)
# jac = p.calculateJacobian(boxId, 0, p.getBasePositionAndOrientation(boxId)[0], [0.0]*num_joints, [0.0]*num_joints, [0.0]*num_joints)
# print(jac)
# p.changeVisualShape(boxId, 0, rgbaColor=[0, 0, 0, 0.5])

# Changing the joint angles
fr = [0, 0.8, -1.2]
fl = [0, 0.8, -1.2]
rr = [0, 0.8, -1.2]
rl = [0, 0.8, -1.2]

# #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# for i in range (100000):
#     p.stepSimulation()
#     time.sleep(1./240.)
#     # p.addUserDebugPoints([p.getBasePositionAndOrientation(boxId)[0]], [(255,0,0)], 10, 0.2)
#     if i > 100:
#         set_joint_angles(fr, fl, rr, rl)

for i in range(500):
    p.stepSimulation()
    set_joint_angles(fr, fl, rr, rl)

print("===============================================================================================================")
print("===============================================================================================================")
print("===============================================================================================================")
print(get_joint_rotation_matrix(boxId, 3))

# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
# p.disconnect()
