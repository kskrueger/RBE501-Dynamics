import pybullet as p
import pybullet_data
import numpy as np
import time
import os

home_directory = os.getcwd()
KR16_model = os.path.join(home_directory, "../kuka_models/kuka_experimental-melodic-devel/kuka_kr16_support/urdf", "kr16_2.urdf")

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

def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

def twist2ht(S, theta):
    w = S[:, :3].T
    w_ss = skew(w)

    R = np.eye(3) + np.sin(theta) * w_ss + (1 - np.cos(theta)) * (w_ss @ w_ss)

    TopRight = (np.eye(3) * theta + (1 - np.cos(theta)) * w_ss + (theta - np.sin(theta)) * (w_ss @ w_ss))

    v = S[:, 3:].T

    T = np.vstack((np.hstack((R, TopRight @ v)), [0, 0, 0, 1]))

    return T

def axisangle2rot(omega, theta):
    omega_ss = skew(omega)
    R = np.eye(3) + np.sin(theta) * omega_ss + (1 - np.cos(theta)) * (omega_ss @ omega_ss)
    return R

def calc_fkine_space(S, M, q):

    S_ss = np.zeros(4,4*S.shape[1])
    for i in range(S.shape[1]):
        Si_ss = twist2ht(S[:, i], q[i])
        S_ss[:, 4*i-3:4*i] = Si_ss

    e_Stheta = np.zeros(4,4*S.shape[1])
    for i in range(S.shape[1]):
        w = S[:, :3]
        v = S[:, 3:]
        e_Sitheta = np.vstack([axisangle2rot(w, q[i]), (((np.eye(3) * q[i]) + ((1 - np.cos(q[i])) * skew(w)) + ((q[i] - np.sin(q[i]))) @ (skew(w) @ skew(w))) @ v)], [0, 0, 0, 1])
        e_Stheta[:, 4*i-3:4*i] = e_Sitheta

    T = np.eye(4)

    for i in range(S.shape[1]):
        T = T @ e_Stheta[1:4,4*i-3:4*i]

    T = T @ M

    return T

def adjoint(vector, T):
    R = T[:3, :3]
    P = T[:3, 3]

    AdjM = np.zeros((6,6))
    AdjM[:3, :3] = R
    AdjM[3:, :3] = skew(P) @ R
    AdjM[4:, 4:] = R

    return AdjM @ vector

def jacob0(S,q):

    Js = np.zeros((6,S.shape[1]))
    T = np.eye(4)

    for i in range(S.shape[1]):
        T = T @ twist2ht(S[:,i],q[i])

        Js[:,i] = adjoint(S[:,i], T)

    return Js

def calc_ik(robot, targetPose):
    currentPose = get_poses(robot)
    currentQ = get_joint_variables(robot)

    while np.linalg.norm(targetPose - currentPose) > 0.001:
        J = jacob0(robot.S, currentQ)
        deltaQ = np.invert(J) @ (targetPose - currentPose)
        currentQ = currentQ + deltaQ.T

        T = calc_fkine_space(robot.S, robot.M, currentQ)
        currentPose = MatrixLog6(T)
        currentPose = np.vstack((currentPose[2,1], currentPose[0,2], currentPose[1,0], currentPose[0,3], currentPose[1,3], currentPose[2,3]))

    return currentPose

def MatrixLog3(R):
    acosinput = (np.trace(R) - 1)/2
    if acosinput >= 1:
        so3mat = np.zeros((3,3))
    elif acosinput <= -1:
        if ~np.isclose(1 + R[3,3]):
            omg = (1 / np.sqrt(2 * (1 + R[3,3]))) @ (np.vstack([R[1,3], R[2,3], (1 + R[3,3])]))
        elif ~np.isclose(1 + R[2,2]):
            omg = (1 / np.sqrt(2 * (1 + R[2, 2]))) @ (np.vstack([R[1, 2], (1 + R[2, 2]), R[3, 2]]))
        else:
            omg = (1 / np.sqrt(2 * (1 + R[1, 1]))) @ (np.vstack([(1 + R[1, 1]), R[2, 1], R[3, 1]]))
        so3mat = skew(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        so3mat = theta * (1 / (2 * np.sin(theta))) * (R - R.T)

    return so3mat

physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())  

planeId = p.loadURDF("plane.urdf")
robot_left = p.loadURDF(KR16_model, [0, 0, 0], useFixedBase=1)
robot_right = p.loadURDF(KR16_model, [0, 3, 0], useFixedBase=1)

#simulation parameters
p.setGravity(0, 0, -9.81) 
p.setTimeStep(0.0001) 
p.setRealTimeSimulation(0)

num_joints = p.getNumJoints(robot_left) - 2      #two fixed joints are used in urdf

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