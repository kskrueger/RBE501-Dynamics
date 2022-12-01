import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from Kinematics import calc_fkine_space, convertPoseTo6, calc_ik

# home_directory = os.getcwd()
# KR16_model =r"C:\Users\malte\Desktop\Project_Dynamics\RBE501-Dynamics\kuka_models\kuka_experimental-melodic-devel\kuka_kr16_support\urdf\kr16_2.urdf"
UR5_model = r"C:\Users\malte\Desktop\Project_Dynamics\RBE501-Dynamics\UR5\urdf\ur5.urdf"
# UR5_model = "../UR5/urdf/ur5.urdf"

class Robot:
    def __init__(self, urdf_path:str, S, M, base_pos=(0, 0, 0), fixed_base=True):
        self.robot = p.loadURDF(urdf_path, base_pos, useFixedBase=fixed_base)
        self.S = S
        self.M = M


    def get_joint_info(self):
        joints = p.getNumJoints(self.robot)

        name = []
        joint_type = []
        lower_limit = []
        upper_limit = []

        for joint_index in range(joints):
            joint_info = p.getJointInfo(self.robot, joint_index)
            name.append(joint_info[1])
            joint_type.append(joint_info[2])
            lower_limit.append(joint_info[8])
            upper_limit.append(joint_info[9])

        return name, joint_type, lower_limit, upper_limit

    def getJointQs(self):
        print(p.getNumJoints(self.robot))
        joints = p.getNumJoints(self.robot)
        return np.array([j[0] for j in p.getJointStates(self.robot, range(joints))])[1:7]

    def get_joint_velocities(self):
        joints = p.getNumJoints(self.robot) - 2
        return [j[1] for j in p.getJointStates(self.robot, range(joints))]

    def get_poses(self):
        position = []
        orientation = []
        joints = p.getNumJoints(self.robot) - 2
        for joint_index in range(joints):
            world_position, world_orientation = p.getLinkState(self.robot, joint_index)[:2]
            position.append(world_position)
            orientation.append(world_orientation)
        return position, orientation

    def setJointsTargetQ(self, q, posGain=0.1, velGain=0.1):
        targets = np.zeros(10)
        targets[1:len(q)+1] = q
        p.setJointMotorControlArray(
            self.robot, range(num_joints), p.POSITION_CONTROL,
            targetPositions=targets, positionGains=np.ones(10)*posGain)#, velocityGains=np.ones(10)*velGain)

    def num_joints(self):
        return p.getNumJoints(self.robot)

    def moveTargetQ_noWait(self, q, max_time=5, dt=.01):
        self.setJointsTargetQ(q)
        targetT = calc_fkine_space(self.S, self.M, q)
        targetT6 = convertPoseTo6(targetT)
        for i in range(int(max_time/dt)):
            p.stepSimulation()

            q = robot_left.getJointQs()
            T = calc_fkine_space(S, M, q)

            # if np.linalg.norm(targetT[:3, 3:] - T[:3, 3:]) < .01 and
            error = np.linalg.norm(convertPoseTo6(T) - targetT6)
            if error > .87 and error < .89:
                print("h")
            print("error", error)
            if error < .1:
                return

            time.sleep(dt)
        print("DONE")

    def moveTargetPose_noWait(self, pose_T, max_time=5, dt=.01):
        start_q = self.getJointQs()
        start_pose = calc_fkine_space(self.S, self.M, start_q)
        target_q = calc_ik(start_q, start_pose, self.S, self.M, pose_T)[0]

        self.moveTargetQ_noWait(target_q, max_time, dt)


def path_line(start, goal):
    # start [x, y, z]
    # goal [x, y, z]
    N = 100
    path = []
    distance_x = (goal[0] - start[0])
    distance_y = (goal[1] - start[1])
    distance_z = (goal[2] - start[2])
    for i in range(N + 1):
        dX = start[0] + i / N * distance_x
        dY = start[1] + i / N * distance_y
        dZ = start[2] + i / N * distance_z
        path.append([start[0] + dX, start[1] + dY, start[2] + dZ])

    return path


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
S = np.array([[0, 0, 1, 0, 0, 0],
              [0, -1, 0, d1, 0, 0],
              [0, -1, 0, d1, 0, -a2],
              [0, -1, 0, d1, 0, -(a2 + a3)],
              [0, 0, -1, 0, a2 + a3 + a4, 0],
              [0, -1, 0, d1 - d5, 0, -(a2 + a3 + a4)]]).T

M = np.array([[1, 0, 0, a2 + a3 + a4 + a6], [0, 0, -1, 0], [0, 1, 0, d1 - d5 - d6], [0, 0, 0, 1]])

# Build Simulation Environment and setup robot arm(s)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  

planeId = p.loadURDF("plane.urdf")
robot_left = Robot(UR5_model, S, M, [0, 0, 0])
#robot_left = Robot(KR16_model, [0, 0, 0])
#robot_right = Robot(KR16_model, [0, 3, 0])
info,_,_,_ = robot_left.get_joint_info()
print(info)

#simulation parameters
# p.setGravity(0, 0, -9.81)
dt = .01
p.setGravity(0, 0, 0)
p.setTimeStep(dt)
p.setRealTimeSimulation(0)

num_joints = robot_left.num_joints()

time.sleep(5)


#Spawning at initial configurations
# q = [0, .5, 0, 0, 1.57, 0]
q = [0, 0, 0, 0, 0, 0]
# robot_left.setJointsTargetQ(q)
robot_left.moveTargetQ_noWait(q)

q = [0, .5, 0, 0, 1.57, 0]
robot_left.moveTargetQ_noWait(q)

q = [0, .5, .5, 0, 1.57, 0]
robot_left.moveTargetQ_noWait(q)

for i in range(10):
    q = np.random.randint(-75, 75, (6)) / 100.0
    target_T = calc_fkine_space(robot_left.S, robot_left.M, q)

    robot_left.moveTargetPose_noWait(target_T)

    print("DONE")
    time.sleep(1)

p.disconnect()