import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
from . import slip3D_ex


# 奔跑控制总程序
class BipedController:
    def __init__(self, robot_id):
        self.status = 'air'
        self.contact_status = [0, 0]
        self.robot_id = robot_id
        self.dic_vel_jac = {}         # 由速度索引的控制雅可比矩阵
        self.dic_vel_time = {}        # 速度索引半周期
        self.pair_table = []

    # 导入表格，生成控制雅可比矩阵
    def load_table(self, pair_path):
        pair_table = pd.read_csv(pair_path)
        table_len = pair_table.shape[0]
        for idx in range(table_len):
            pair = pair_table[i]
            jac = slip3D_ex.control_jac_calculation(pair)
            self.dic_vel_jac[pair_table[i, 1]] = jac
            self.dic_vel_time[pair_table[i, 1]] = pair_table[i, 7]
        self.pair_table = pair_table

    def control_para_calculation(self, des_vel, x_now):
        vel_list = self.pair_table[:, 1]
        if des_vel > vel_list.max() or des_vel < vel_list.min():
            print("Error, input wrong velocity!")
            

    # 输出[tau1 ……tau5]的控制量
    def robot_control(self):
        contact_list = p.getContactPoints(self.robot_id, planeId)
        if len(contact_list) == 1:
            body_pos = p.getBasePositionAndOrientation(self.robot_id)
            self.status = 'ground'
        # if self.status == 'air':



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
# 控制器
bc = BipedController(RobotId)

for i in range(600):
    # 控制程序
    bc.robot_control()
    if bc.status == 'ground':
        break
    p.stepSimulation()
    time.sleep(1./100.0)

p.disconnect()





