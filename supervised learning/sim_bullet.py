import pybullet as p
import time
import pybullet_data

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(boxId, [1, 0, 0])

# 仿真循环
for i in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
