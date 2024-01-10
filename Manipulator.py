import pybullet as p
import pybullet_data
import numpy as np
import kinpy as kp


def get_joint_positions(p,robot_id,num_joints):
    return [p.getJointState(kukaId, jointIndex)[0] for jointIndex in range(numJoints)]

def get_joint_velocities(p,robot_id,num_joints):
    return [p.getJointState(kukaId, jointIndex)[1] for jointIndex in range(numJoints)]


def calc_com_jac(p,robot_id, end_effector_id,numJoints):
    jacobians = []
    for i in range(end_effector_id):
        result = p.getLinkState(kukaId, i, computeLinkVelocity=1, computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        jacobians.append(p.calculateJacobian(robot_id,i,com_trn,get_joint_positions(p,robot_id,numJoints)[1:5],get_joint_velocities(p,robot_id,numJoints)[1:5],[0,0,0,0]))


    mass = []

    for i in range(numJoints):
        mass.append(p.getDynamicsInfo(kukaId,i)[0])

    total_mass = sum(mass)

    com_jac = np.zeros(jacobians[0].shape)

    for i in range(len(jacobians)):
        com_jac += jacobians[i] * mass[i]/total_mass

    return com_jac

def calc_com(p,robot_id):
    return None




clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

time_step = 0.001
gravity_constant = -9.81
p.resetSimulation()
p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, gravity_constant)

p.loadURDF("plane.urdf", [0, 0, -0.3])

# kukaId = p.loadURDF("TwoJointRobot_w_fixedJoints.urdf", useFixedBase=True)
kukaId = p.loadURDF("arm.urdf", useFixedBase=True)
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
numJoints = p.getNumJoints(kukaId)

model = kp.build_serial_chain_from_urdf(open("arm.urdf"),'arm_link_5')
jacobians = model.jacobian([0,0,0,0],end_only=False)
i = 0
mass = []
for i in range(7):
    mass.append( p.getDynamicsInfo(kukaId,i-1)[0])
total_mass = sum(mass)
keys = jacobians.keys()


com_jac = np.zeros(jacobians['base_link'].shape)
counter = 0
for i in keys:
    com_jac += jacobians[i] * mass[counter]/total_mass
    counter += 1


com_jac_inv = np.linalg.pinv(com_jac)


V_task = np.array([2,1,1,0,0,0])

joint_velocities = np.dot(com_jac_inv, V_task)
print(joint_velocities)

