import numpy as np
import rbdl


# 定义双足机器人基本参数

model = rbdl.Model()
model.gravity = np.array([0.0, 0.0, -9.81])
# 创建body
mass = 20.0                         # 躯干
com = np.array([0., 0., 0.])
inertia = np.diag([0.37, 0.29, 0.128])
b_torso = rbdl.Body.fromMassComInertia(mass, com, inertia)
mass = 0.5
com = 
