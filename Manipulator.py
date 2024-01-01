# import pybullet as p
# import time
# import pybullet_data
# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# startPos = [0,0,1]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
# p.disconnect()


import pybullet as p

# Initialize PyBullet and load your robot
p.connect(p.GUI)
robot_id = p.loadURDF("path/to/your/robot.urdf", [0, 0, 0])

# Get the number of joints in the robot
num_joints = p.getNumJoints(robot_id)


# Set the joint positions to some arbitrary values
joint_positions = [0.0] * num_joints
for joint_index in range(num_joints):
    p.resetJointState(robot_id, joint_index, joint_positions[joint_index])

# Get the robot's current state
state = p.getJointStates(robot_id, range(num_joints))

# Calculate the Jacobian of the center of mass
link_indices = range(num_joints)  # You may need to adjust these indices based on your robot
com_jacobian, com_velocity, com_reaction_forces, com_torques = p.calculateJacobian(
    bodyUniqueId=robot_id,
    linkIndex=-1,  # -1 refers to the center of mass
    localPosition=[0, 0, 0],  # Center of mass position in the link's frame
    objPositions=state[0],  # Joint positions
    objVelocities=state[1],  # Joint velocities
    objAccelerations=state[2],  # Joint accelerations
    physicsClientId=0
)

# Print or use the center of mass Jacobian matrix as needed
print("Center of Mass Jacobian:")
print(com_jacobian)
