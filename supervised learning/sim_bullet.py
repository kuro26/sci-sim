import pybullet as p
import time
import pybullet_data

# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 创建模型

# 仿真循环
for i in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
