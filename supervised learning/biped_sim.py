import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
from . import slip3D_ex
from geomdl import BSpline
from geomdl import utilities

g = 9.8
sim_cycle = 0.01

# -------------------------------------------------------
# 类：奔跑运动控制器
#                     x方向与地面夹角
# pairs <h0, vx0, vy0, alpha, beta, ks1, ks2>
# -------------------------------------------------------
class BipedController:
    def __init__(self, robot_id):
        self.l0 = 1.0                 # 腿长
        self.status = 'air'
        self.contact_status = [0, 0]
        self.robot_id = robot_id
        self.dic_vel_jac = {}         # 由速度索引的控制雅可比矩阵
        self.dic_vel_time = {}        # 速度索引半周期
        self.pair_table = np.array([])
        # 本周期相关变量
        self.this_x = np.array([])     # 起始顶点状态
        self.this_rv = 0               # 参考速度
        self.this_pair = []            # 控制对
        self.this_du = np.array([])    # 无系数du
        self.this_T_half = 0           # 本半周期时间间隔
        self.this_t = 0                # 本周期时间

    # ----------导入表格，生成控制雅可比矩阵----------
    def load_table(self, pair_path):
        pair_table = pd.read_csv(pair_path, header=None).values
        table_len = pair_table.shape[0]
        for idx in range(table_len):
            pair = pair_table[i]
            jac = slip3D_ex.control_jac_calculation(pair)
            self.dic_vel_jac[pair_table[i, 1]] = jac
            self.dic_vel_time[pair_table[i, 1]] = pair_table[i, 7]
        self.pair_table = pair_table

    # -----------------计算控制量-------------------
    def control_para_calculation(self):
        des_vel = self.this_rv
        x_now = self.this_x
        vel_list = self.pair_table[:, 1]
        if des_vel > vel_list.max() or des_vel < vel_list.min():
            print("Error, input wrong velocity!")
        m_idx = np.fabs(vel_list - des_vel).argmin()
        m_pair = self.pair_table[m_idx]
        # 计算该状态下的delta控制量
        delta_x = x_now - m_pair[0: 3]
        jac = self.dic_vel_jac[m_pair[1]]
        delta_u = np.dot(jac, delta_x.transpose())
        delta_u_out = np.array((4,))
        delta_u_out[0:3] = delta_u
        delta_u_out[-1] += -delta_u[-1]      # 冗余量约束设计得到
        self.this_pair = m_pair
        self.this_du = delta_u_out

    # t为本周期时间
    # 输出(q, dq)
    def swing_planning(self, t):
        curve = BSpline.Curve()
        # 触地竖直速度估计，只在local用到的变量就放在local
        h0, vx0, vy0, alpha, beta, ks1, ks2 = self.this_pair[0:7]
        delta_h = h0 - self.l0 * np.sin(alpha)
        vz_contact = np.sqrt(2 * delta_h * g)
        vx_contact = vx0
        a_begin = np.array([vx_contact, vz_contact])/




    # -----------输出[tau1 ……tau5]的控制量-----------
    def robot_control(self):
        # 获取机器人状态
        vel = p.getBaseVelocity(self.robot_id)
        pos = p.getBasePositionAndOrientation(self.robot_id)
        contact_list = p.getContactPoints(self.robot_id, planeId)

        if len(contact_list):
            # body_pos = p.getBasePositionAndOrientation(self.robot_id)
            self.status = 'ground'
        else:
            self.status = 'air'

        if self.status == 'air':
            if len(self.this_x) == 0:   # 如果没有初始高度数据
                h0 = pos[0][2] + 0.5*vel[0][2]*vel[0][2]/g
                self.this_x = [h0, vel[0][0], vel[0][1]]
                self.control_para_calculation()         # 计算控制参数
            # 计算当前腿部规划点




# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
RobotId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(RobotId, [1, 0, 0])
mode = p.VELOCITY_CONTROL
# 控制器
bc = BipedController(RobotId)
bc.load_table('./data/stable_pair.csv')

for i in range(600):
    # 控制程序
    bc.robot_control()
    if bc.status == 'ground':
        break
    p.stepSimulation()
    time.sleep(sim_cycle)

p.disconnect()





