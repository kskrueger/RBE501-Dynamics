import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from Kinematics import calc_fkine_space

# home_directory = os.getcwd()
# KR16_model =r"C:\Users\malte\Desktop\Project_Dynamics\RBE501-Dynamics\kuka_models\kuka_experimental-melodic-devel\kuka_kr16_support\urdf\kr16_2.urdf"
UR5_model = r"C:\Users\malte\Desktop\Project_Dynamics\RBE501-Dynamics\UR5\urdf\ur5.urdf"
# UR5_model = "../UR5/urdf/ur5.urdf"


def get_joint_info(robot):
    joints = p.getNumJoints(robot)

    name = []
    joint_type = []
    lower_limit = []
    upper_limit = []

    for joint_index in range(joints):
        joint_info = p.getJointInfo(robot, joint_index)
        name.append(joint_info[1])
        joint_type.append(joint_info[2])
        lower_limit.append(joint_info[8])
        upper_limit.append(joint_info[9])

    return name, joint_type, lower_limit, upper_limit

def get_joint_variables(robot):
    print(p.getNumJoints(robot))
    joints = p.getNumJoints(robot) -4
    return np.array([j[0] for j in p.getJointStates(robot, range(joints))])

def get_joint_velocities(robot):
    joints = p.getNumJoints(robot) - 2
    return [j[1] for j in p.getJointStates(robot, range(joints))]

def get_poses(robot):
    position = []
    orientation = []
    joints = p.getNumJoints(robot) - 2
    for joint_index in range(joints):
        world_position, world_orientation = p.getLinkState(robot, joint_index)[:2]
        position.append(world_position)
        orientation.append(world_orientation)
    return position, orientation

def path_line(start, goal):
    # start [x, y, z]
    # goal [x, y, z]
    N = 100
    path=[]
    distance_x=(goal[0]-start[0])
    distance_y=(goal[1]-start[1])
    distance_z=(goal[2]-start[2])
    for i in range(N+1):
        dX=start[0]+i/N*distance_x
        dY=start[1]+i/N*distance_y
        dZ=start[2]+i/N*distance_z
        path.append([start[0]+dX, start[1]+dY, start[2]+dZ])

    return path


# theta = .57
# s = np.array([[1, 2, 3, 4, 5, 6]])
# b = twist2ht(s, theta)


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  

planeId = p.loadURDF("plane.urdf")
robot_left = p.loadURDF(UR5_model, [0, 0, 0], useFixedBase=1)
#robot_left = p.loadURDF(KR16_model, [0, 0, 0], useFixedBase=1)
#robot_right = p.loadURDF(KR16_model, [0, 3, 0], useFixedBase=1)
info,_,_,_ = get_joint_info(robot_left)
# print(p.getNumJoints(robot_left))
print(info)


#simulation parameters
p.setGravity(0, 0, -9.81) 
p.setTimeStep(0.0001) 
p.setRealTimeSimulation(0)

num_joints = p.getNumJoints(robot_left)      

#Spawning at initial configurations
p.setJointMotorControlArray(
robot_left, range(num_joints), p.POSITION_CONTROL,
targetPositions=[0,0,0,0,1.57,0,0,0,0,0], positionGains = [0,0,0,0,1,0,0,0,0,0])

# p.setJointMotorControlArray(
# robot_right, range(num_joints), p.POSITION_CONTROL,
# targetPositions=[0,0,0,0,0,0], positionGains = [0.01,0,0,0,0,0])
# time.sleep(1. / 240.)

# start = np.array([0, 0, 0])
# goal = np.array([3, 2, 1])
# path = path_line(start, goal)

# arm lengths
d1 = 0.089159 # [m]
d4 = 0.10915 #[m]
d5 = 0.09465 #[m]
d6 = 0.0823 #[m]
a2 = 0.425 #[m]
a3 = 0.39225 #[m]
a4 = 0
a6 = 0

# Twists and Home config
S = np.transpose( np.array([[0, 0, 1, 0, 0, 0],
                            [0, -1, 0, d1, 0, 0],
                            [0, -1, 0, d1, 0, -a2],
                            [0, -1, 0, d1, 0, -(a2+a3)],
                            [0, 0, -1, 0, a2+a3+a4, 0],
                            [0, -1, 0, d1-d5, 0, -(a2+a3+a4)] ]))
    
M=np.array([[1, 0, 0, a2+a3+a4+a6],[0,0,-1,0],[0, 1, 0, d1-d5-d6],[0,0,0,1]])


for i in range(10000):
    p.stepSimulation()

    # Forward Kinematics

    q = get_joint_variables(robot_left)
    T = calc_fkine_space(S,M,q)
    print(T.round(2))

    #To-Do:
    '''
    IK
    '''

    #hard-coded joint variables
    # q_left = [0,0,0,0,0,0]
    # q_right = [0,0,0,0,0,0]

    #Implementing position controllers
    # p.setJointMotorControlArray(
    # robot_left, range(num_joints), p.POSITION_CONTROL,
    # targetPositions= q_left, positionGains = [0.01,0,0,0,0.01,0])

    # p.setJointMotorControlArray(
    # robot_right, range(num_joints), p.POSITION_CONTROL,
    # targetPositions= q_right, positionGains = [0.01,0,0,0,0.01,0])
    time.sleep(1. / 240.)

p.disconnect()