# 碰撞检测实验
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt


# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")
cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1., 1., 1.])
cuid = p.createCollisionShape(p.GEOM_CAPSULE)
mass = 1.0
cube = p.createMultiBody(mass, cuid, basePosition=[0., 0., 2.])

for i in range(600):
    p.stepSimulation()
    contact_list = p.getContactPoints(cube)
    if len(contact_list):
        for item in contact_list:
            print(item[5], item[9])
        break
    time.sleep(1./100.)

p.disconnect()
