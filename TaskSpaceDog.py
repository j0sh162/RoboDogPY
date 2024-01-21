import pybullet as p
import time
import pybullet_data
import kinpy as kp
import pinocchio as pin
import numpy as np



class TaskSpaceDog:
    
    def __init__(self, filePath, target):
        self.p = p
        self.pin = pin
        self.timestep = 0.001
        self.physicsClient = p.connect(p.GUI)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setGravity(0,0,0)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0])       
        self.boxId = p.loadURDF(filePath, [0,0,0.4], self.p.getQuaternionFromEuler([0, 0, 0]))
        self.p.changeVisualShape(self.boxId, 0, rgbaColor=[0, 0, 0, 0.5])
        self.model = self.pin.buildModelFromUrdf(filePath, self.pin.JointModelFreeFlyer())
        self.modelData = self.model.createData()
        self.target = target
        self.dampingFactor = 0.1



    def getMotorJointStates(self):
        joint_states = self.p.getJointStates(self.boxId, range(self.p.getNumJoints(self.boxId)))
        joint_infos = [self.p.getJointInfo(self.boxId, i) for i in range(self.p.getNumJoints(self.boxId))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def calc_com(self, mpos, mvel, mtorq):
        q = np.concatenate([self.p.getBasePositionAndOrientation(self.boxId)[0], [0,0,0,0],np.array(mpos)])
        self.pin.forwardKinematics(self.model, self.modelData, q)
        com = self.pin.centerOfMass(self.model, self.modelData, q)
        return com

    def calc_com_jac(self, mpos, mvel, mtorq):
        q = np.concatenate([self.p.getBasePositionAndOrientation(self.boxId)[0], [0,0,0,0],np.array(mpos)])
        self.pin.forwardKinematics(self.model, self.modelData, q)
        comJac = self.pin.jacobianCenterOfMass(self.model,self.modelData,q)
        return comJac[:,6:]
    
    def iterate(self):
        mpos, mvel, mtorq = self.getMotorJointStates()
        com = self.calc_com(mpos, mvel, mtorq)
        comJac = self.calc_com_jac(mpos, mvel, mtorq)
        error = self.target - com
        tmp = (np.matmul(np.transpose(comJac),comJac)+(self.dampingFactor**2)*np.identity(12))
        jointVelocities = np.dot(np.matmul(np.linalg.inv(tmp),np.transpose(comJac)), error)
        mpos += jointVelocities * self.timestep
        self.p.setJointMotorControlArray(bodyUniqueId=self.boxId,
                                        jointIndices=[2, 3, 4,9, 10, 11, 16, 17, 18, 23, 24, 25],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=mpos,
                                        forces=[500] * 12)
        self.p.addUserDebugPoints([com, self.target], [(255,0,0), (0,255,0)], 10, 0.2)
        self.p.stepSimulation()

    def setJointAngles(self, q):
        self.p.setJointMotorControlArray(bodyUniqueId=self.boxId,
                                        jointIndices=[2, 3, 4,9, 10, 11, 16, 17, 18, 23, 24, 25],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=q,
                                        forces=[500] * 12)

if __name__ == '__main__':
    target = np.array([-0.03, 0, 0.31])
    doggo = TaskSpaceDog("go1_description/urdf/go1.urdf", target)
    for i in range(1000):
        if i == 500:
            doggo.setJointAngles([0, 0.8, -1.3, 0, 0.8, -1.3, 0, 0.8, -1.3, 0, 0.8, -1.3])
        doggo.p.stepSimulation()
    for i in range(1000000):
        doggo.iterate()

    doggo.p.disconnect()
