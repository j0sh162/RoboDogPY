import numpy as np
import pybullet as p
import time
import pybullet_data
import kinpy as kp
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


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
        self.model = pin.buildModelFromUrdf(robot_file_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.qv = [0.0] * 12
        pin.forwardKinematics(self.model, self.data, self.q)

    def calc_com_jac(self):
        x = pin.jacobianCenterOfMass(self.model, self.data, self.q)
        return x[:, 6:]

    def calc_com_jac_q(self,q):
        q = np.concatenate([[0, 0, 0, 0, 0, 0, 0], np.array(q)])
        x = pin.jacobianCenterOfMass(self.model, self.data, q)
        return x[:, 6:]

    def calc_com(self):
        x = pin.centerOfMass(self.model, self.data, self.q)
        return x

    def p_control_iterate(self):
        Jacobian = self.calc_com_jac()

        error = self.desired_pos - self.calc_com()
        u = self.kp * error

        joint_velocities = np.dot(np.linalg.pinv(Jacobian), u)
        joint_velocities = np.concatenate(
            [[0, 0, 0, 0, 0, 0, 0], np.array(joint_velocities)]
        )
        # print(self.q)

        self.q = self.q + joint_velocities * 0.001

    def damped_least_sqaures_iterate(self, damping_factor):
        jac = self.calc_com_jac()
        x_d = self.kp*(self.desired_pos - self.calc_com())

        tmp = np.matmul(np.transpose(jac), jac) + (damping_factor**2) * np.identity(
            12
        )
        self.qv = np.dot(np.matmul(np.linalg.inv(tmp), np.transpose(jac)), x_d)
        joint_velocities = np.concatenate([[0, 0, 0, 0, 0, 0, 0], self.qv])
        self.q = self.q + joint_velocities * 0.001

        # Update joint positions based on joint velocities

    def null_space_iterate(self, alpha):
        jac = self.calc_com_jac()
        x_d = self.kp*self.desired_pos - self.calc_com()
        jac_inv = np.linalg.pinv(jac)
        q0_d = self.get_joint_velocity_vec(alpha, jac)
        
        tmp = (np.identity(12)-np.matmul(jac_inv,jac))
        
        self.qv = np.dot(jac_inv,x_d) + np.shape(np.dot(tmp,q0_d))

        joint_velocities = np.concatenate([[0, 0, 0, 0, 0, 0, 0], self.qv])
        self.q = self.q + joint_velocities * 0.001

        # Update joint positions based on joint velocities

    def get_joint_velocity_vec(self, alpha, jac):
        q0_d = [0.0] * 12

        for i in range(len(q0_d)):
            current_q = self.q[7:]
            current_q[i] = current_q[i] + 0.001
            jac_step = self.calc_com_jac_q(current_q)

            q0_d[i] = (
                self.manipubilty_measure(jac_step) - self.manipubilty_measure(jac)
            ) / 0.001
        #print(q0_d)
        q0_d = np.dot(np.transpose(q0_d),alpha)
        
        return q0_d

    def manipubilty_measure(self, jac):
        return np.sqrt(np.linalg.det(np.matmul(jac,np.transpose(jac))))

    def set_target(self, desired_pos):
        self.desired_pos = np.array(desired_pos)

    def pybullet_viz_init(self, robot_file_path):
        self.time_step = 1
        self.gravity_constant = 0
        self.p = p
        clid = self.p.connect(p.SHARED_MEMORY)
        if clid < 0:
            self.p.connect(p.GUI)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.resetSimulation()
        self.p.setTimeStep(self.time_step)
        self.p.setGravity(0.0, 0.0, self.gravity_constant)

        plane = self.p.loadURDF("plane.urdf", [0, 0, 0])
        self.robot_id = self.p.loadURDF(robot_file_path, [0, 0, 0.45])
        self.num_joints = p.getNumJoints(self.robot_id)

    def pybullet_viz_step(self):
        maxForces = [500] * 12

        self.p.setJointMotorControlArray(
            self.robot_id,
            [2, 3, 4, 9, 10, 11, 16, 17, 18, 23, 24, 25],
            p.POSITION_CONTROL,
            targetPositions=self.q[7:],
        )
        self.p.addUserDebugPoints(
            [self.desired_pos, self.calc_com()], [(255, 0, 0), (0, 255, 0)], 10, 0.3
        )
        self.p.stepSimulation()


if __name__ == "__main__":
    task_space = TaskSpaceManipulator("go1_description/urdf/go1.urdf", 5, 1)
    print(task_space.calc_com())
    task_space.set_target([0, 0.01, -0.036])
    task_space.pybullet_viz_init("go1_description/urdf/go1.urdf")

    for i in range(10000000):
        task_space.pybullet_viz_step()

        # task_space.task_space_iterate()
        # task_space.damped_least_sqaures_iterate(0.1)
        task_space.p_control_iterate()
        print(task_space.calc_com())
