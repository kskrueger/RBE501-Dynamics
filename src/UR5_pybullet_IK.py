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

UR5_1_pos = [0, 0, 0.5]
UR5_1 = p.loadURDF(ROBOT_model, UR5_1_pos)

UR5_2_pos = [0, 0.5, 0.5]
UR5_2 = p.loadURDF(ROBOT_model, UR5_2_pos)

cube_pos = [0.45,0.5,0]             # [0, -0.55, 0]
cubeId = p.loadURDF(CUBE_model, cube_pos)

#Go to Home and fetch the pose.
p.setJointMotorControlArray(
    UR5_1, range(10), p.POSITION_CONTROL,
    targetPositions=np.zeros(10), positionGains=[0,0,0,0.1,0,0,0,0,0,0])

p.setJointMotorControlArray(
    UR5_2, range(10), p.POSITION_CONTROL,
    targetPositions=np.zeros(10), positionGains=[0,0,0,0.1,0,0,0,0,0,0])

# position = []
# orientation = []
# joints = p.getNumJoints(UR5) 
# for joint_index in range(joints):
#     world_position, world_orientation = p.getLinkState(UR5, joint_index)[:2]
#     position.append(world_position)
#     orientation.append(world_orientation)
# print(position[6], p.getMatrixFromQuaternion(orientation[6]))

#We can also call IK to bring it back to Home.
# jointPoses = p.calculateInverseKinematics(UR5,
#                                             6,
#                                             position[6],
#                                             orientation[6])
# Q = np.zeros(joints)
# Q[1:len(jointPoses)+1] = jointPoses
# idx = np.where(jointPoses!=0)
# damping = np.ones(joints)
# damping[idx] = 0.1

# p.setJointMotorControlArray(
#     UR5, range(len(Q)), p.POSITION_CONTROL,
#     targetPositions=Q, positionGains=damping)

#----------------- Pick n Place simulation ----------------------#

# startPos = [0.45,0,0.3]
# startOri = p.getQuaternionFromEuler([-1.57,0,1.57])

startPos_UR5_1 = [0.45, 0.25,0.5]           
startOri_UR5_1 = p.getQuaternionFromEuler([-1.57,0,0])

endPos_UR5_1 = [0.3,-0.25,0.5]  
endOri_UR5_1 = p.getQuaternionFromEuler([-1.57,0,0])

startPos_UR5_2 = [0.45, 0.75,0.5]       
startOri_UR5_2 = p.getQuaternionFromEuler([-1.57,0,0])

endPos_UR5_2 = [0.3,0.25,0.5]
endOri_UR5_2 = p.getQuaternionFromEuler([-1.57,0,0])

#Achieving required poses
jointPoses_1 = p.calculateInverseKinematics(UR5_1,
                                            6,
                                            startPos_UR5_1,
                                            startOri_UR5_1)
Q_1 = np.zeros(10)
Q_1[1:len(jointPoses_1)+1] = jointPoses_1
idx = np.where(jointPoses_1!=0)
damping_1 = np.ones(10)
damping_1[idx] = 0.1
damping_1[3] =0.1
p.setJointMotorControlArray(
    UR5_1, range(len(Q_1)), p.POSITION_CONTROL,
    targetPositions=Q_1, positionGains=damping_1)

jointPoses_2 = p.calculateInverseKinematics(UR5_2,
                                            6,
                                            startPos_UR5_2,
                                            startOri_UR5_2)
Q_2 = np.zeros(10)
Q_2[1:len(jointPoses_2)+1] = jointPoses_2
idx = np.where(jointPoses_2!=0)
damping_2 = np.ones(10)
damping_2[idx] = 0.1
damping_2[3] =0.1
p.setJointMotorControlArray(
    UR5_2, range(len(Q_2)), p.POSITION_CONTROL,
    targetPositions=Q_2, positionGains=damping_2)

time.sleep(5)

#Grasp synthesis
startPos_UR5_1[2] = startPos_UR5_1[2]-0.23
jointPoses_1 = p.calculateInverseKinematics(UR5_1,
                                            6,
                                            startPos_UR5_1,
                                            startOri_UR5_1)
Q_1 = np.zeros(10)
Q_1[1:len(jointPoses_1)+1] = jointPoses_1
idx = np.where(jointPoses_1!=0)
damping_1 = np.ones(10)
damping_1[idx] = 0.1
damping_1[3] =0.1
p.setJointMotorControlArray(
    UR5_1, range(len(Q_1)), p.POSITION_CONTROL,
    targetPositions=Q_1, positionGains=damping_1)

startPos_UR5_2[2] = startPos_UR5_2[2]-0.23
jointPoses_2 = p.calculateInverseKinematics(UR5_2,
                                            6,
                                            startPos_UR5_2,
                                            startOri_UR5_2)
Q_2 = np.zeros(10)
Q_2[1:len(jointPoses_2)+1] = jointPoses_2
idx = np.where(jointPoses_2!=0)
damping_2 = np.ones(10)
damping_2[idx] = 0.1
damping_2[3] =0.1
p.setJointMotorControlArray(
    UR5_2, range(len(Q_2)), p.POSITION_CONTROL,
    targetPositions=Q_2, positionGains=damping_2)
time.sleep(3)


#Grasp
Q_1[8] = 0.040
Q_1[9] = -0.040
idx = np.where(Q_1!=0)
damping_1 = np.ones(10)
damping_1[idx] = 0.1
damping_1[3] =0.1
p.setJointMotorControlArray(
    UR5_1, range(len(Q_1)), p.POSITION_CONTROL,
    targetPositions=Q_1, positionGains=damping_1)
time.sleep(1)

Q_2[8] = 0.025
Q_2[9] = -0.025
idx = np.where(Q_2!=0)
damping_2 = np.ones(10)
damping_2[idx] = 0.1
damping_2[3] =0.1
p.setJointMotorControlArray(
    UR5_2, range(len(Q_2)), p.POSITION_CONTROL,
    targetPositions=Q_2, positionGains=damping_2)
time.sleep(5)

# UR5_1_grasp = p.createConstraint(parentBodyUniqueId = UR5_1, parentLinkIndex = 9, childBodyUniqueId = cubeId, childLinkIndex = -1, jointType = p.JOINT_FIXED, jointAxis = [0,0,0], parentFramePosition = [0,0,0], childFramePosition = [0,0,0], physicsClientId = physicsClient)
# UR5_2_grasp = p.createConstraint(parentBodyUniqueId = UR5_2, parentLinkIndex = 9, childBodyUniqueId = cubeId, childLinkIndex = -1, jointType = p.JOINT_FIXED, jointAxis = [0,0,0], parentFramePosition = [0,0,0], childFramePosition = [0,0,0], physicsClientId = physicsClient)

# while True:
#     p.stepSimulation()

#Move to Home - To avoid singularity, I guess so!!!!
# Q_1 = np.zeros(10)
# Q_1[8] = 0.040
# Q_1[9] = -0.040
# Q_1[7] = 1.57
# idx = np.where(Q_1!=0)
# damping_1 = np.ones(10)
# damping_1[idx] = 0.1
# damping_1[3] =0.1
# p.setJointMotorControlArray(
#     UR5_1, range(len(Q_1)), p.POSITION_CONTROL,
#     targetPositions=Q_1, positionGains=damping_1)


#Move to Goal
jointPoses_1 = p.calculateInverseKinematics(UR5_1,
                                            6,
                                            endPos_UR5_1,
                                            endOri_UR5_1)
Q_1 = np.zeros(10)
Q_1[1:len(jointPoses_1)+1] = jointPoses_1
Q_1[8] = 0.040
Q_1[9] = -0.040
idx = np.where(Q_1!=0)
damping_1 = np.ones(10)
damping_1[idx] = 0.1
damping_1[3] =0.1
p.setJointMotorControlArray(
    UR5_1, range(len(Q_1)), p.POSITION_CONTROL,
    targetPositions=Q_1, positionGains=damping_1)
time.sleep(1)

jointPoses_2 = p.calculateInverseKinematics(UR5_2,
                                            6,
                                            endPos_UR5_2,
                                            endOri_UR5_2)
Q_2 = np.zeros(10)
Q_2[1:len(jointPoses_2)+1] = jointPoses_2
Q_2[8] = 0.040
Q_2[9] = -0.040
idx = np.where(Q_2!=0)
damping_2 = np.ones(10)
damping_2[idx] = 0.1
damping_2[3] =0.1
p.setJointMotorControlArray(
    UR5_2, range(len(Q_2)), p.POSITION_CONTROL,
    targetPositions=Q_2, positionGains=damping_2)
time.sleep(3)

#Release
# Q[8] = 0
# Q[9] = 0
# idx = np.where(jointPoses!=0)
# damping = np.ones(joints)
# damping[idx] = 0.1
# damping[3] =0.1
# p.setJointMotorControlArray(
#     robot_grasp, range(len(Q)), p.POSITION_CONTROL,
#     targetPositions=Q, positionGains=damping)
# time.sleep(0.5)
# p.removeConstraint(robot_grasp, physicsClient)

while True:
    p.stepSimulation()

p.disconnect()
