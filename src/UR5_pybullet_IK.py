import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())  
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)

ROBOT_model = "/home/kt/Academics/RBE501/RBE501-Dynamics/UR5/urdf/UR5.urdf" # Krutarth
# ROBOT_model = r"E:\GitHub\RBE501-Dynamics\UR5\urdf\ur5.urdf"  # Kyle
# ROBOT_model = "../UR5/urdf/UR5.urdf"    #Karter

CUBE_model = "/home/kt/Academics/RBE501/Latest/RBE501-Dynamics/src/meshes/small_block.urdf" #Krutarth
# CUBE_model = "meshes/small_block.urdf"      #Karter

UR5_pos = [0, 0, 0.5]
UR5 = p.loadURDF(ROBOT_model, UR5_pos)
cube_pos = [0, -0.55, 0]
cubeId = p.loadURDF(CUBE_model, cube_pos)

#Go to Home and fetch the pose.
p.setJointMotorControlArray(
    UR5, range(10), p.POSITION_CONTROL,
    targetPositions=np.zeros(10), positionGains=[0,0,0,0.1,0,0,0,0,0,0])

position = []
orientation = []
joints = p.getNumJoints(UR5) 
for joint_index in range(joints):
    world_position, world_orientation = p.getLinkState(UR5, joint_index)[:2]
    position.append(world_position)
    orientation.append(world_orientation)
print(position[6], p.getMatrixFromQuaternion(orientation[6]))

#Go to the other location.
# p.setJointMotorControlArray(
#     UR5, range(10), p.POSITION_CONTROL,
#     targetPositions=[0,-1.57,0,0,0,0,0,0,0,0], positionGains=[0,0.1,0,0.1,0,0,0,0,0,0])
# time.sleep(3)

#Let's call IK to bring it back to Home.
jointPoses = p.calculateInverseKinematics(UR5,
                                            6,
                                            position[6],
                                            orientation[6])
Q = np.zeros(joints)
Q[1:len(jointPoses)+1] = jointPoses
idx = np.where(jointPoses!=0)
damping = np.ones(joints)
damping[idx] = 0.1

p.setJointMotorControlArray(
    UR5, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)

#Pick n Place simulation
# startPos = [0.45,0,0.3]
# startOri = p.getQuaternionFromEuler([-1.57,0,1.57])

startPos = [0,-0.5,0.3]
startOri = p.getQuaternionFromEuler([-1.57,0,0])

endPos = [0,0.5,0.3]
endOri = p.getQuaternionFromEuler([-1.57,0,0])


#Go to Start
jointPoses = p.calculateInverseKinematics(UR5,
                                            6,
                                            startPos,
                                            startOri)
Q = np.zeros(joints)
Q[1:len(jointPoses)+1] = jointPoses
idx = np.where(jointPoses!=0)
damping = np.ones(joints)
damping[idx] = 0.1
damping[3] =0.1
p.setJointMotorControlArray(
    UR5, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)
time.sleep(3)


#Grasp
Q[8] = 0.025
Q[9] = -0.025
idx = np.where(jointPoses!=0)
damping = np.ones(joints)
damping[idx] = 0.1
damping[3] =0.1
p.setJointMotorControlArray(
    UR5, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)
time.sleep(10)
robot_grasp = p.createConstraint(parentBodyUniqueId = UR5, parentLinkIndex = 9, childBodyUniqueId = cubeId, childLinkIndex = -1, jointType = p.JOINT_FIXED, jointAxis = [0,0,0], parentFramePosition = [0,0,0], childFramePosition = [0,0,0], physicsClientId = physicsClient)

#Move to Home - To avoid singularity, I guess so!!!!
Q = np.zeros(10)
Q[8] = 0.040
Q[9] = -0.040
idx = np.where(Q!=0)
damping = np.ones(10)
damping[idx] = 0.1
damping[3] =0.1
p.setJointMotorControlArray(
    robot_grasp, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)


#Move to Goal
jointPoses = p.calculateInverseKinematics(UR5,
                                            6,
                                            endPos,
                                            endOri)
Q = np.zeros(joints)
Q[1:len(jointPoses)+1] = jointPoses
Q[8] = 0.040
Q[9] = -0.040
idx = np.where(Q!=0)
damping = np.ones(joints)
damping[idx] = 0.1
damping[3] =0.1
p.setJointMotorControlArray(
    UR5, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)
time.sleep(3)

#Release
Q[8] = 0
Q[9] = 0
idx = np.where(jointPoses!=0)
damping = np.ones(joints)
damping[idx] = 0.1
damping[3] =0.1
p.setJointMotorControlArray(
    robot_grasp, range(len(Q)), p.POSITION_CONTROL,
    targetPositions=Q, positionGains=damping)
time.sleep(0.5)
p.removeConstraint(robot_grasp, physicsClient)

while True:
    p.stepSimulation()

p.disconnect()
