import pybullet as p
import time
import pybullet_data
import numpy as np

g = 9.8
sim_cycle = 0.01

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.5]
cubeStartOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])
RobotId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(RobotId, [1, 0, 0])
mode = p.VELOCITY_CONTROL

# for i in range(600):
#     # 控制程序
#     p.stepSimulation()
#     time.sleep(sim_cycle)
#
# p.disconnect()