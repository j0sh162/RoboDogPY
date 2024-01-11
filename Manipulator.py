import pybullet as p
import pybullet_data
import numpy as np
import kinpy as kp


class TaskSpaceManipulator:
    def __init__(self,robot_file_path,kp,kd):
        self.desired_vel = None
        self.gravity_constant = -9.81
        self.time_step = 0.001
        self.p = p
        self.kp = kp
        self.kd = kd

        clid = self.p.connect(p.SHARED_MEMORY)
        if clid < 0:
            self.p.connect(p.DIRECT)

        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.p.resetSimulation()
        self.p.setTimeStep(self.time_step)
        self.p.setGravity(0.0, 0.0, self.gravity_constant)

        self.p.loadURDF("plane.urdf", [0, 0, -0.3])
        self.robot_id = self.p.loadURDF(robot_file_path, useFixedBase=True)
        self.p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        self.num_joints = p.getNumJoints(self.robot_id)

        self.mass = []

        for i in range(self.num_joints):
            self.mass.append(p.getDynamicsInfo(self.robot_id, i)[0])

        self.total_mass = sum(self.mass)

    def calc_com_jac(self):
        jacobians = []
        for i in range(self.num_joints):
            result = self.p.getLinkState(self.robot_id, i, computeLinkVelocity=1, computeForwardKinematics=1)
            link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
            j_lin,j_ang = self.p.calculateJacobian(self.robot_id, i, com_trn, self.get_joint_positions()[1:5],
                                     self.get_joint_velocities()[1:5], [0, 0, 0, 0])
            jacobians.append(np.concatenate([j_lin,j_ang]))

        com_jac = np.zeros(jacobians[0].shape)

        for i in range(len(jacobians)):
            com_jac += jacobians[i] * self.mass[i] / self.total_mass

        return com_jac

    def calc_com(self):
        weighted_positions = np.zeros(3)

        # Loop through each joint to calculate the total mass and weighted positions
        for joint_index in range(self.num_joints):
            # Get dynamics information for the joint
            dynamics_info = self.p.getDynamicsInfo(self.robot_id, joint_index)

            # Extract mass and position information
            link_mass = dynamics_info[0]
            link_position = dynamics_info[3]

            # Update total mass and weighted positions
            weighted_positions += link_mass * np.array(link_position)

        com = weighted_positions / self.total_mass
        return com

    def get_joint_positions(self):
        return [p.getJointState(self.robot_id, jointIndex)[0] for jointIndex in range(self.num_joints)]

    def get_joint_velocities(self):
        return [p.getJointState(self.robot_id, jointIndex)[1] for jointIndex in range(self.num_joints)]

    def task_space_iterate(self):
        Jacobian = self.calc_com_jac()

        # Compute the current end-effector position (replace with actual calculation)
        X_current = self.calc_com()

        # Desired task space position (replace with your desired position)
        X_desired = np.array([0.1, 0.2, 0.3])

        # Task space error
        error = np.linalg.norm(X_desired-X_current)

        # Compute joint velocities using PD control



        joint_velocities = self.kp* error * np.dot(np.linalg.pinv(Jacobian),  self.desired_vel)
        print(self.get_joint_positions()[1:5])


        # Update joint positions based on joint velocities
        joint_positions = self.get_joint_positions()[1:5] + joint_velocities * self.time_step
        joint_positions = np.concatenate([joint_positions,[0,0]])
        zero_vec = [0.0] * self.num_joints
        self.p.setJointMotorControlArray(self.robot_id,
                                    range(self.num_joints),
                                    p.POSITION_CONTROL,
                                    targetPositions=joint_positions,
                                    targetVelocities=zero_vec,
                                    positionGains=[1.0] * self.num_joints,
                                    velocityGains=[0.3] * self.num_joints)
        self.p.stepSimulation()
    def set_target(self,desired_pos):
        current = self.calc_com()
        desired_vel = desired_pos - current

        zero_vec = [1.0] * 3
        self.desired_vel = np.concatenate([desired_vel,zero_vec])



if __name__ == '__main__':

    task_space = TaskSpaceManipulator("arm.urdf",0.5,1)
    print(task_space.calc_com())

    task_space.set_target([0.0,0.0,0.08])
    for i in range(100):
        task_space.task_space_iterate()
    print(task_space.calc_com())

