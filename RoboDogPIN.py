import numpy as np
import pybullet as p
import time
import pybullet_data
import kinpy as kp
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

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

import time
import sys
import pybullet as p
import pybullet_data
import numpy as np
import kinpy as kp
from pinocchio.visualize import GepettoVisualizer
import os
from os.path import dirname, join, abspath

class TaskSpaceManipulator:
    def __init__(self, robot_file_path, kp, kd):
        self.desired_vel = None
       

        self.time_step = 1 / 240.0
        self.kp = kp
        self.kd = kd
        self.model = pin.buildModelFromUrdf(robot_file_path,pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, self.q)

    def calc_com_jac(self):
        x = pin.jacobianCenterOfMass(self.model, self.data, self.q)
        return x[:, 6:]

    def calc_com(self):
        x = pin.centerOfMass(self.model, self.data, self.q)
        return x

    def task_space_iterate(self):
        Jacobian = self.calc_com_jac()

        error = self.desired_pos - self.calc_com()
        u = self.kp * error
        error[0] = 0
        error[1] = 0

        joint_velocities = np.dot(np.linalg.pinv(Jacobian), u)
        joint_velocities = np.concatenate(
            [[0, 0, 0, 0, 0, 0, 0], np.array(joint_velocities)]
        )
        #print(self.q)

        self.q = self.q + joint_velocities * 0.0001

        

        # Update joint positions based on joint velocities

    def set_target(self, desired_pos):
        current = np.array(list(self.calc_com()))

        self.desired_pos = np.array(desired_pos)


if __name__ == "__main__":
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", 0.05, 1)
    print(task_space.calc_com())
    task_space.set_target([0, 0, 0])
    viz = GepettoVisualizer(task_space.model)
    try:
        viz.initViewer()
    except ImportError as err:
        print("Error while initializing the viewer. It seems you should install gepetto-viewer")
        print(err)
        sys.exit(0)
    
    try:
        viz.loadViewerModel("pinocchio")
    except AttributeError as err:
        print("Error while loading the viewer model. It seems you should start gepetto-viewer")
        print(err)
        sys.exit(0)
    
    viz.display(task_space.q)
    for i in range(10000000):
        viz.display(task_space.q)
        if i > 1000:
            task_space.task_space_iterate()

        print(task_space.calc_com())
