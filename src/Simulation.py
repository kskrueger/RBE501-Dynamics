import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
# startPos = [0, 0, 1]
# startOrientation = p.getQuaternionFromEuler([0, 0, 0])

kukaId = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")

# Get joint info from i=4
jointNum = 4
jointInfo = p.getJointInfo(kukaId[0], jointNum)

# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1. / 240.)


p.disconnect()
