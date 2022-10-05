import pybullet as p
import pybullet_data
import time

def get_joint_info(robot):
    joints = p.getNumJoints(robot) - 2

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
    joints = p.getNumJoints(robot) - 2
    return [j[0] for j in p.getJointStates(robot, range(joints))]

def get_joint_velocities(robot):
    joints = p.getNumJoints(robot) - 2
    return [j[1] for j in p.getJointStates(robot, range(joint))]

def get_poses(robot):
    position = []
    orientation = []
    joints = p.getNumJoints(robot) - 2
    for joint_index in range(joints):
        world_position, world_orientation = p.getLinkState(robot, joint_index)[:2]
        position.append(world_position)
        orientation.append(world_orientation)
    return position, orientation

physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())  

planeId = p.loadURDF("plane.urdf")
robot_left = p.loadURDF("/home/kt/Academics/RBE501/Project/RBE501-Dynamics-main/kuka_models/kuka_experimental-melodic-devel/kuka_kr16_support/urdf/kr16_2.urdf", [0, 0, 0], useFixedBase=1)
robot_right = p.loadURDF("/home/kt/Academics/RBE501/Project/RBE501-Dynamics-main/kuka_models/kuka_experimental-melodic-devel/kuka_kr16_support/urdf/kr16_2.urdf", [0, 3, 0], useFixedBase=1)

#simulation parameters
p.setGravity(0, 0, -9.81) 
p.setTimeStep(0.0001) 
p.setRealTimeSimulation(0)

num_joints = p.getNumJoints(robot_left) - 2      #two fixed joints are usedd in urdf

#Spawning at initial configurations
p.setJointMotorControlArray(
robot_left, range(num_joints), p.POSITION_CONTROL,
targetPositions=[-0.5,0,0,0,0,0], positionGains = [0.01,0,0,0,0,0])

p.setJointMotorControlArray(
robot_right, range(num_joints), p.POSITION_CONTROL,
targetPositions=[0.5,0,0,0,0,0], positionGains = [0.01,0,0,0,0,0])
time.sleep(1. / 240.)

for i in range(10000):
    p.stepSimulation()

    #To-Do:
    '''
    IK
    '''

    #hard-coded joint variables
    q_left = [-0.5,0,0,0,2,0]
    q_right = [0.5,0,0,0,2,0]

    #Implementing position controllers
    p.setJointMotorControlArray(
    robot_left, range(num_joints), p.POSITION_CONTROL,
    targetPositions= q_left, positionGains = [0.01,0,0,0,0.01,0])

    p.setJointMotorControlArray(
    robot_right, range(num_joints), p.POSITION_CONTROL,
    targetPositions= q_right, positionGains = [0.01,0,0,0,0.01,0])
    time.sleep(1. / 240.)

p.disconnect()