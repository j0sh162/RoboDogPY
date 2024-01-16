import numpy as np
import pybullet as p
import time
import pybullet_data
import kinpy as kp
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

import pybullet as p
import pybullet_data
import numpy as np
import kinpy as kp


class TaskSpaceManipulator:
    def __init__(self, robot_file_path, kp, kd):
        self.desired_vel = None
        self.gravity_constant = -9.81
        self.time_step = 0.001
        self.p = p
        self.kp = kp
        self.kd = kd

        clid = self.p.connect(p.SHARED_MEMORY)
        if clid < 0:
            self.p.connect(p.GUI)

        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.p.resetSimulation()
        self.p.setTimeStep(self.time_step)
        self.p.setGravity(0.0, 0.0, self.gravity_constant)

        self.p.loadURDF("plane.urdf", [0, 0, 0])
        self.robot_id = self.p.loadURDF(robot_file_path)
        self.p.resetBasePositionAndOrientation(self.robot_id, [0,0,0.4], [0, 0, 0, 1])
        self.num_joints = p.getNumJoints(self.robot_id)


    def getMotorJointStates(self,robot):
        joint_states = self.p.getJointStates(robot, range(self.p.getNumJoints(robot)))
        joint_infos = [self.p.getJointInfo(robot, i) for i in range(self.p.getNumJoints(robot))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def calc_com_jac(self):
        mpos, mvel, mtorq = self.getMotorJointStates(self.robot_id)
        print("Velocity:")
        print(len(mvel))
        print("Pos:")
        print(len(mpos))
        result = p.getLinkState(self.robot_id,
                                0,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        zero_vec = [0.0] * len(mpos)
        jac_t, jac_r = p.calculateJacobian(self.robot_id, 0, com_trn, mpos, mvel, zero_vec)

        com_jac = np.concatenate([jac_t, jac_r])

        return jac_t

    def calc_com(self):
        result = p.getLinkState(self.robot_id,
                                0,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        return link_trn

    def get_joint_positions(self):
        return [p.getJointState(self.robot_id, jointIndex)[0] for jointIndex in range(self.num_joints)]



    def get_joint_velocities(self):
        return [p.getJointState(self.robot_id, jointIndex)[1] for jointIndex in range(self.num_joints)]

    def task_space_iterate(self):
        Jacobian = self.calc_com_jac()

        # Compute the current end-effector position (replace with actual calculation)
        X_current = self.calc_com()

        # Desired task space position (replace with your desired position)

        # Task space error

        # Compute joint velocities using PD control

        x_curr = np.linalg.inv(get_joint_rotation_matrix(self.robot_id, 0))

        # Error 1 setting the correct joints
        # Error 2 setting the joints
        # Error 3 matrix might not be in correct shape
        # Error 4 I might have interpreted formula wrong
        # Should also try with manipulator

        error = self.desired_pos - self.calc_com()
        u = self.kp*error

        print(Jacobian)
        joint_velocities = (u*np.dot(np.linalg.pinv(Jacobian), u))


        print(joint_velocities)

        # print(self.get_joint_positions()[1:5])
        print(len(joint_velocities))

        # Update joint positions based on joint velocities
        joint_positions = self.getMotorJointStates(self.robot_id)[0] + joint_velocities * self.time_step

        zero_vec = [0.0] * 12
        self.p.setJointMotorControlArray(self.robot_id,
                                         [2, 3, 4,9, 10, 11,16, 17, 18,23, 24, 25],
                                         p.POSITION_CONTROL,
                                         targetPositions=joint_positions,
                                         targetVelocities=zero_vec,)
        time.sleep(1. / 240.)
        self.p.addUserDebugPoints([X_current, self.calc_com()], [(255, 0, 0), (0, 255, 0)], 10, 0.3)
        self.p.stepSimulation()



    def set_target(self, desired_pos):

        current = np.array(list(self.calc_com()))
        desired_pos = np.array(desired_pos)

        self.desired_vel = desired_pos - current
        # self.desired_pos = desired_pos
        # zero_vec = [0.0] * 3
        # self.desired_vel = np.concatenate([desired_vel, zero_vec])
        # self.desired_vel_se3 = angvec2r(0, zero_vec)

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm


if __name__ == '__main__':

    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", 1.4, 1)
    print(task_space.calc_com())
    print_joint_info(task_space.robot_id)
    task_space.set_target([0,0,0])
    for i in range(1000):
        task_space.task_space_iterate()

    # print(p.getLinkStates(task_space.robot_id))
    print(task_space.calc_com())
