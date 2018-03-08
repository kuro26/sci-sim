import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)

