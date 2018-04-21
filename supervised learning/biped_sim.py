import pybullet as p
import time
import pybullet_data

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
RobotId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(RobotId, [0, 0, 0])
mode = p.VELOCITY_CONTROL
