import pybullet as p
import time
import pybullet_data
import numpy as np

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.3]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
RobotId = p.loadURDF("../../../supervised learning/bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(RobotId, [0, 0, 0])
mode = p.VELOCITY_CONTROL
p.resetJointState(RobotId, 0, 0.1, targetVelocity=0.0)
p.resetJointState(RobotId, 1, 0.2, targetVelocity=0.0)  # -np.pi/8
p.resetJointState(RobotId, 2, 0.3, targetVelocity=0.0)  # np.pi/4
p.resetJointState(RobotId, 4, 0.4, targetVelocity=0.0)
p.resetJointState(RobotId, 5, 0.5, targetVelocity=0.0)  # np.pi/8
p.resetJointState(RobotId, 6, 0.6, targetVelocity=0.0)  # np.pi/4
# p.setJointMotorControl2(RobotId, 0, mode, targetVelocity=1, force=100)
# p.setJointMotorControl2(RobotId, 1, mode, targetVelocity=1, force=100)
# p.setJointMotorControl2(RobotId, 4, mode, targetVelocity=1, force=100)
zero_x = [0, 0, 0, 0, 0, 0]
jac = p.calculateJacobian(RobotId, 2, (0., 0., -0.5), zero_x, zero_x, zero_x)      # 计算左脚雅可比矩阵
# 计算结果6，7，8列是我们需要的

# 仿真循环
# for i in range(6000):
#     p.stepSimulation()
#     time.sleep(1./240.0)

p.disconnect()
