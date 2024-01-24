import time

import pybullet as p
import pybullet_data
import numpy as np
import kinpy as kp


class TaskSpaceManipulator:
    def __init__(self, robot_file_path, kp, kd):
        self.desired_vel = None
        self.gravity_constant = -9.81
        self.time_step = 1 / 240.0
        self.p = p
        self.kp = kp
        self.kd = kd
        self.prev_error = [0.0]*3

      
        self.p.connect(p.DIRECT)

        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.p.resetSimulation()
        self.p.setTimeStep(self.time_step)
        self.p.setGravity(0.0, 0.0, self.gravity_constant)

        self.p.loadURDF("plane.urdf", [0, 0, 0])
        self.robot_id = self.p.loadURDF(robot_file_path, useFixedBase=True)
        self.p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        self.num_joints = p.getNumJoints(self.robot_id)

        self.mass = []

        for i in range(self.num_joints):
            self.mass.append(p.getDynamicsInfo(self.robot_id, i)[0])

        self.total_mass = sum(self.mass)

    def calc_com_jac(self):
        model = kp.build_serial_chain_from_urdf(open("model.urdf"), "lbr_iiwa_link_7")
        jacobians = model.jacobian(self.get_joint_positions(), end_only=False)

        com_jac = np.zeros(jacobians["lbr_iiwa_link_7"].shape)
        counter = 0
        keys = jacobians.keys()
        for i in keys:
            com_jac += jacobians[i] * self.mass[counter - 1] / self.total_mass
            counter += 1

        return com_jac[:3]
    
    def calc_com_jac_q(self,q):
        model = kp.build_serial_chain_from_urdf(open("model.urdf"), "lbr_iiwa_link_7")
        jacobians = model.jacobian(q, end_only=False)

        com_jac = np.zeros(jacobians["lbr_iiwa_link_7"].shape)
        counter = 0
        keys = jacobians.keys()
        for i in keys:
            com_jac += jacobians[i] * self.mass[counter - 1] / self.total_mass
            counter += 1

        return com_jac[:3]

    def calc_com(self):
        weighted_positions = np.zeros(3)

        # Loop through each joint to calculate the total mass and weighted positions
        for joint_index in range(self.num_joints):
            # Get dynamics information for the joint

            result = self.p.getLinkState(
                self.robot_id,
                joint_index,
                computeLinkVelocity=1,
                computeForwardKinematics=1,
            )
            # Extract mass and position information

            # Update total mass and weighted positions
            weighted_positions += self.mass[joint_index] * np.array(result[0])

        com = weighted_positions / self.total_mass

        return com

    def get_joint_positions(self):
        return [
            p.getJointState(self.robot_id, jointIndex)[0]
            for jointIndex in range(self.num_joints)
        ]

    def get_joint_velocities(self):
        return [
            p.getJointState(self.robot_id, jointIndex)[1]
            for jointIndex in range(self.num_joints)
        ]

    def p_control_iterate(self):
        Jacobian = self.calc_com_jac()
        # Compute the current end-effector position (replace with actual calculation)
        X_current = self.calc_com()
        # Desired task space position (replace with your desired position)
        # Task space error
        error = self.desired_pos - X_current
        # print(error)
        # Compute joint velocities using PD control
        ut = self.kp * error
        # print(error)
        joint_velocities = np.dot(np.linalg.pinv(Jacobian), ut)

        # Update joint positions based on joint velocities
        joint_positions = self.get_joint_positions() + joint_velocities * 0.1
        
        
        self.p.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            p.POSITION_CONTROL,
            targetPositions=joint_positions,
        )

        self.p.addUserDebugPoints(
            [self.desired_pos, self.calc_com()], [(255, 0, 0), (0, 255, 0)], 10, 0.3
        )
        self.p.stepSimulation()

    def null_space_iterate(self, alpha):
        jac = self.calc_com_jac()
        error = self.desired_pos - self.calc_com()
        kd_value = error - self.prev_error
        self.prev_error = error
        x_d = self.kp * error + self.kd*kd_value
        jac_inv = np.linalg.pinv(jac)
      
        q0_d = self.get_joint_velocity_vec(alpha, jac)
        
        tmp = (np.identity(7)-np.matmul(jac_inv,jac))
        
        joint_velocities = np.dot(jac_inv,x_d) + np.dot(tmp,q0_d)

        
        joint_positions = self.get_joint_positions() + joint_velocities * 0.001
        self.p.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            p.POSITION_CONTROL,
            targetPositions=joint_positions,
        )

        self.p.addUserDebugPoints(
            [self.desired_pos, self.calc_com()], [(255, 0, 0), (0, 255, 0)], 10, 0.3
        )
        self.p.stepSimulation()
        # Update joint positions based on joint velocities

    def get_joint_velocity_vec(self, alpha, jac):
        q0_d = [0.0] * self.num_joints
        
        for i in range(len(q0_d)):
            current_q = self.get_joint_positions()
            current_q[i] = current_q[i] + 0.001
            jac_step = self.calc_com_jac_q(current_q)
       
            q0_d[i] = (
                self.manipubilty_measure(jac_step) - self.manipubilty_measure(jac)
            ) / 0.001
        #print(q0_d)
        q0_d = np.dot(np.transpose(q0_d),alpha)
        
        return q0_d

    def manipubilty_measure(self, jac):
        if np.abs(np.linalg.det(np.matmul(jac,np.transpose(jac)))) > 10**-20:
            return np.sqrt(np.linalg.det(np.matmul(jac,np.transpose(jac))))
        else:
            return np.sqrt(0)

    def damped_least_sqaures_iterate(self, damping_factor):
        jac = self.calc_com_jac()
        error = self.desired_pos - self.calc_com()
        kd_value = error - self.prev_error
        self.prev_error = error
        x_d = self.kp * error + self.kd*kd_value

        tmp = np.matmul(np.transpose(jac), jac) + (damping_factor**2) * np.identity(
            7
        )
        joint_velocities = np.dot(np.matmul(np.linalg.inv(tmp), np.transpose(jac)), x_d)
        
        joint_positions = self.get_joint_positions() + joint_velocities * 0.001
        self.p.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            p.POSITION_CONTROL,
            targetPositions=joint_positions,
        )

        self.p.addUserDebugPoints(
            [self.desired_pos, self.calc_com()], [(255, 0, 0), (0, 255, 0)], 10, 0.3
        )
        self.p.stepSimulation()

    def set_target(self, desired_pos):
        self.desired_pos = desired_pos


if __name__ == "__main__":
    task_space = TaskSpaceManipulator("kuka_iiwa/model.urdf", 100, 1)
    print(task_space.calc_com())
    task_space.set_target([0.4, 0.1, 0.3])
    for i in range(100000):
        # task_space.task_space_iterate()
        # print(task_space.calc_com())
        task_space.damped_least_sqaures_iterate(0.1)

    # print(p.getLinkStates(task_space.robot_id))
