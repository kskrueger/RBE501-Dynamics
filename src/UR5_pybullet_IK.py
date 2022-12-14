import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())  
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
#UR5 = p.loadURDF("/home/kt/Academics/RBE501/RBE501-Dynamics/UR5/urdf/ur5.urdf") # Krutarth
# path = r"E:\GitHub\RBE501-Dynamics\UR5\urdf\ur5.urdf"  # Kyle
path = "../UR5/urdf/ur5.urdf"
UR5 = p.loadURDF(path)
CUBE_model = "meshes/small_block.urdf"
planeId = p.loadURDF("plane.urdf")
cube_pos = [.5, 0, 0]
cubeId = p.loadURDF(CUBE_model, cube_pos)

#Go to Home and fetch the pose.
p.setJointMotorControlArray(
    UR5, range(7), p.POSITION_CONTROL,
    targetPositions=[0,0,0,0,0,0,0], positionGains=[0,0,0,0,0,0,0])

position = []
orientation = []
joints = p.getNumJoints(UR5) 
for joint_index in range(joints):
    world_position, world_orientation = p.getLinkState(UR5, joint_index)[:2]
    position.append(world_position)
    orientation.append(world_orientation)
print(position[6], p.getMatrixFromQuaternion(orientation[6]))

# time.sleep(3)
# M = np.array([-1,0,0,0.805659],[0,-1,0,-0.110433],[0,0,1,0.354950])
while True:
    p.stepSimulation()

#Go to the other location.
p.setJointMotorControlArray(
    UR5, range(7), p.POSITION_CONTROL,
    targetPositions=[0,-1.57,0,0,0,0,0], positionGains=[0,0.1,0,0,0,0,0])
time.sleep(3)

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

time.sleep(100)

p.disconnect()
