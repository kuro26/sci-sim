import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
from geomdl import BSpline
from geomdl import utilities

from .tools.utils import *
from . import slip3D_ex

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

        # -----------本周期相关变量------------------
        self.this_x = np.array([])     # 起始顶点状态
        self.this_rv = 0               # 参考速度
        self.this_pair = []            # 控制对
        self.this_du = np.array([])    # 无系数du
        self.this_T_half = 0           # 本半周期时间间隔
        self.this_t = 0                # 本周期时间
        self.this_curve = BSpline.Curve()

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

    # -----------------------------------------
    # 逆运动学：
    # 伪世界坐标系(位置固定在body上，)转化到关节坐标系
    # -----------------------------------------
    def coord_fake_world2joint(self, p_world, leg):
        assert p_world.shape == (3, 1)
        tmp = p_world.tolist()
        tmp.append([1])
        pw_ext = np.array(tmp)
        b_po = p.getBasePositionAndOrientation(self.robot_id)
        alpha, beta, gamma = p.getEulerFromQuaternion(b_po[1])        # 机体欧拉角表示
        if leg == 'left':
            dy = 0.12
        else:
            dy = -0.12
        t1 = rotate_x(alpha).dot(rotate_y(beta).dot(trans_xyz(0, dy, -0.2)))
        p_in1 = np.linalg.inv(t1).dot(pw_ext)
        ang_a = np.arctan2(p_in1[1], abs(p_in1[2]))
        t2 = rotate_x(ang_a)
        p_in2 = np.linalg.inv(t2).dot(p_in1)
        x2, z2 = p_in2[0], p_in2[2]
        # print(p_in2)
        tmp1 = x2 * x2 + z2 * z2
        tmp2 = np.sqrt(-tmp1 * (tmp1 - 1))
        ang_b = -2 * np.arctan((x2 + (x2 * x2 * tmp2) / tmp1 + (z2 * z2 * tmp2) / tmp1) / (tmp1 - z2))
        ang_c = 2 * np.arctan(tmp2 / tmp1)
        # ang_b = 2 * np.arctan((-x2 + (x2*x2*tmp2)/tmp1 + (z2*z2*tmp2)/tmp1)/(tmp1 - z2))
        # ang_c = -2 * np.arctan(tmp2/tmp1)
        return [ang_a, ang_b, ang_c]

    # -----------------------------------------
    # 足端位置控制
    # 基于三关节角度的足端位置控制
    # -----------------------------------------
    def position_control_leg(self, angle, leg):
        mode = p.POSITION_CONTROL
        id = self.robot_id
        j_id = [4, 5, 6]
        if leg == 'left':
            j_id = [0, 1, 2]
        for i in range(3):
            p.setJointMotorControl2(id, j_id[i], mode, targetPosition=angle[i])

    # -----------------------------------------
    # 函数： 根据控制pair，创建标准曲线表达
    # 将前置条件作为参数传入，保证理解
    # -----------------------------------------
    def create_swing_curve(self, pair):
        # 触地竖直速度估计，只在local用到的变量就放在local
        h0, vx0, vy0, alpha, beta, ks1, ks2 = pair[0:7]
        delta_h = h0 - self.l0 * np.sin(alpha)
        vz_contact = np.sqrt(2 * delta_h * g)
        vx_contact = vx0
        tmp = np.sqrt(vx_contact*vx_contact + vz_contact * vz_contact)
        a_begin = np.array([-vx_contact, -vz_contact])/tmp        # 初始速度向量
        a_end = np.array([-vx_contact, vz_contact])/
        p1 = (-self.l0 * np.cos(alpha), 0.0)                      # 初始点，以原点为坐标系
        p2 = (p1[0] + a_begin[0]*0.1, p1[1] + a_begin[1]*0.1)
        p3 = (0, 0.3)
        p5 = (self.l0 * np.cos(alpha), 0.0)
        p4 = (p5[0] + a_end[0]*0.1, p5[1] + a_end[1]*0.1)
        self.this_curve.ctrlpts = (p1, p2, p3, p4, p5)
        self.this_curve.delta = 0.01
        self.this_curve.degree = 4
        self.this_curve.knotvector = utilities.generate_knot_vector(4, len(self.this_curve.ctrlpts))
        self.this_curve.evaluate()


    # -----------------------------------------
    # 函数：获取标准规划下的此时状态
    # 初始状态和中间时刻
    # 此时状态 <足端位置， 足端速度>
    # -----------------------------------------
    def get_swing_planning(self):



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





