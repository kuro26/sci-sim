import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
from geomdl import BSpline
from geomdl import utilities

import tools.utils as utl
import slip3D_ex

g = 9.8
sim_cycle = 0.001                             # 仿真粒度


def limit_in01(p_in):
    if p_in < 0:
        return 0
    if p_in > 1:
        return 1
    return p_in


# -------------------------------------------------------
# 类：奔跑运动控制器
#                     x方向与地面夹角
# pairs <h0, vx0, vy0, alpha, beta, ks1, ks2>
# -------------------------------------------------------
class BipedController:
    def __init__(self, robot_id):
        self.sys_t = 0                 # 系统时间
        self.l0 = 1.0                  # 腿长
        self.para = [20.0, -9.8, 1.0]  # [m, g, l0]机器人参数
        self.status = 'air'
        self.contact_status = [0, 0]
        self.robot_id = robot_id
        self.dic_vel_jac = {}         # 由速度索引的控制雅可比矩阵
        # self.dic_air_time = {}        # 速度索引半周期
        # self.dic_sup_time = {}        # 速度索引的支撑时间
        self.pair_table = np.array([])
        # -----------本周期相关变量------------------
        self.start_time = 0             # 本周起的起始时间
        self.this_x = np.array([])     # 起始顶点状态
        self.des_v = 0                 # 参考速度
        self.des_pair = []             # 理想pair
        self.des_air_time = 0
        self.des_sup_time = 0
        self.this_pair = []            # 本周期的仿真pair
        self.this_du = np.array([])    # 无系数du
        self.this_u = np.array([])     # 本周期控制量
        self.this_T_half = 0           # 本半周期时间间隔
        self.this_t = 0                # 本周期时间
        self.this_curve_l = BSpline.Curve()
        self.this_curve_r = BSpline.Curve()
        self.a_begin = np.array([])    # 离地点速度方向
        self.p_begin = np.array([])
        self.a_end = np.array([])      # 着地点速度方向
        self.p_end = np.array([])
        self.status_change = True      # 初始默认有一个状态转换
        self.cycle_cnt = 1             # 半周期计数

    # -----------------------------------------
    # 导入表格
    # 生成控制雅可比矩阵
    # -----------------------------------------
    def load_table(self, pair_path):
        # 1. 读取表格
        pair_table = pd.read_csv(pair_path, header=None).values
        self.pair_table = pair_table
        # 2. 生成控制雅可比矩阵
        table_len = pair_table.shape[0]
        for idx in range(table_len):
            pair = pair_table[idx]
            jac = slip3D_ex.control_jac_calculation(pair, self.para)
            self.dic_vel_jac[pair_table[idx, 1]] = jac
            # self.dic_air_time[pair_table[i, 1]] = pair_table[i, 7]
            # self.dic_sup_time[pair_table[i, 1]] = pair_table[i, 8]

    # -----------------------------------------
    # 计算控制量：
    # 期望速度-->【pair】-->【控制量】-->落地点
    # 控制量： 相对于pair中u的偏移方向
    # 【在每次进入floating状态时调用-即离地】
    # -----------------------------------------
    # 设置控制器的【期望速度】
    def set_target_vel(self, target_vel):
        self.des_v = target_vel

    # 设置系统时间
    def set_system_time(self, t):
        self.sys_t = t

    # 选择控制器本周期的控制【pair】以及其他在table中的参数
    def choose_pair_from_speed(self):
        # 0. 前提：table和期望速度
        des_vel = self.des_v  # 获取期望速度
        vel_list = self.pair_table[:, 1]
        # 1. 根据速度从table中获取pair
        if des_vel > vel_list.max() or des_vel < vel_list.min():
            print("Error, input wrong velocity!")
        m_idx = np.fabs(vel_list - des_vel).argmin()
        m_pair = self.pair_table[m_idx]
        # 2. 更新本周期控制参数
        self.des_pair = m_pair
        self.des_air_time = m_pair[7]           # 半周期空中时间
        self.des_sup_time = m_pair[8]           # 半周期支撑时间

    # 计算控制器本周期内的【控制参数】
    def calculate_control_param(self):
        # 0. 前提：本次apex状态和期望pair
        m_pair = self.des_pair
        x_now = self.this_x                 # 获取当前顶点状态
        # 1. 计算在标准pair下的控制增量 delta u
        delta_x = x_now - m_pair[0: 3]
        jac = self.dic_vel_jac[m_pair[1]]
        delta_u = np.dot(jac, delta_x.transpose())
        delta_u_out = np.array([0, 0, 0, 0])
        delta_u_out[0:3] = delta_u
        delta_u_out[-1] += -delta_u[-1]      # 冗余量约束设计得到
        self.this_du = delta_u_out
        # 2. 计算本周期应用的pair
        #                                    增益系数
        self.this_u = np.array(m_pair[3:7]) + 0.1 * self.this_du
        self.this_pair[0:3] = x_now.tolist()
        self.this_pair[3:7] = self.this_u.tolist()
        # 3. 仿真获取数据
        sol1, sol2, sol3, sol4, foot_point = slip3D_ex.sim_cycle(self.this_pair, self.para)

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
        t1 = utl.rotate_x(alpha).dot(utl.rotate_y(beta).dot(utl.trans_xyz(0, dy, -0.2)))
        p_in1 = np.linalg.inv(t1).dot(pw_ext)
        ang_a = np.arctan2(p_in1[1], abs(p_in1[2]))
        t2 = utl.rotate_x(ang_a)
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
        md = p.POSITION_CONTROL
        rid = self.robot_id
        j_id = [4, 5, 6]
        if leg == 'left':
            j_id = [0, 1, 2]
        for idx in range(3):
            p.setJointMotorControl2(rid, j_id[idx], md, targetPosition=angle[idx])

    # -----------------------------------------
    # 支撑相关计算
    # 1. 计算此时腿部坐标系
    # 2. 计算此时腿长
    # -----------------------------------------
    def calc_forward_length(self, leg):
        # 获取必要信息
        rid = self.robot_id
        if leg == 'left':       # 左腿角度及偏移
            dy = 0.12
            ang_a = p.getJointState(rid, 0)[0]
            ang_b = p.getJointState(rid, 1)[0]
            ang_c = p.getJointState(rid, 2)[0]
        else:
            dy = -0.12
            ang_a = p.getJointState(rid, 4)[0]
            ang_b = p.getJointState(rid, 5)[0]
            ang_c = p.getJointState(rid, 6)[0]
        # 计算腿端点在机体坐标系下(注意不是世界坐标系)的位置
        t1 = utl.trans_xyz(0, dy, -0.2)
        t2 = utl.rotate_x(ang_a).dot(utl.rotate_y(ang_b).dot(utl.trans_xyz(0, 0, -0.5)))
        t3 = utl.rotate_y(ang_c)                       # 膝关节转动
        pos_ed = np.array([[0.], [0.], [-0.5], [1]])   # 末坐标系下的位置扩展
        p_w = t1.dot(t2.dot(t3.dot(pos_ed)))           # 机体坐标系下的位置
        leg_len = np.sqrt(p_w[0]*p_w[0] + p_w[1]*p_w[1] + p_w[2]*p_w[2])   # 腿长度
        return p_w, leg_len




    # -----------------------------------------
    # 在【本apex状态】【控制量】条件下
    # 1. 终点位置和速度
    # -----------------------------------------
    def swing_related_calculation(self):
        # 0. 获取预备信息
        h0, vx0, vy0 = self.this_x
        alpha, beta, ks1, ks2 = self.this_u
        l0 = self.para[2]
        # 1. 计算腿部着地时状态
        dx = l0 * np.cos(beta) * np.cos(alpha)
        dy = l0 * np.sin(beta)
        dz = -l0 * np.cos(beta)*np.sin(alpha)
        p_end = np.array([dx, dy, dz])
        # 1. 计算着地速度
        delta_h = h0 + dz
        vz_contact = np.sqrt(2 * delta_h * g)
        # 2. 计算向量长度
        tmp = np.sqrt(vx0 * vx0 + vy0 * vy0 + vz_contact * vz_contact)
        # 3. 计算速度单位向量
        # a_begin = np.array([-vx_contact, -vz_contact]) / tmp  # 2D-离地时速度单位向量
        a_end = np.array([-vx0, -vy0, vz_contact])/tmp  # 3D-着地时速度单位向量
        # 4. 写入信息
        self.p_end = p_end
        self.a_end = a_end

    # -----------------------------------------
    # 已知：
    #  1. 处在悬空状态，并且获知当前顶点
    #  2. 已经计算完成本周期控制量
    # 参数：
    #  leg--当前周期中正要落下的腿
    # 说明：
    # 1. 对两条腿均进行重规划
    # -----------------------------------------
    def swing_curve_planning(self, leg_down):
        # 1. 现在是哪条腿落下
        if leg_down is 'left':
            coe = -1
        else:
            coe = 1
        # 2. 首先处理当前准备触地的腿
        if self.cycle_cnt == 1:
            # 如果仿真刚开始
            p_end = self.p_end + coe * np.array([0., 0.12, 0])
            a_end = self.a_end
        else:
            p_end, a_end = self.p_end, self.a_end
        a_begin = np.array([a_end[0], a_end[1], -a_end[2]])
        p_begin = np.array([-p_end[0], p_end[1], p_end[2]])
        # 2.1 直角坐标空间下的关键点位置
        p1 = tuple(p_begin)
        p2 = tuple(p_begin + 0.1 * a_begin)
        p4 = tuple(p_end - 0.05 * a_end)
        p5 = tuple(p_end)
        p3 = (0., (p1[1]+p5[1])/2, p_begin[2] + 0.3)
        # 2.2 关节坐标空间下的关键点位置转化
        jp1 = self.coord_fake_world2joint(np.array(p1).reshape((3, 1)), leg_down)
        jp2 = self.coord_fake_world2joint(np.array(p2).reshape((3, 1)), leg_down)
        jp3 = self.coord_fake_world2joint(np.array(p3).reshape((3, 1)), leg_down)
        jp4 = self.coord_fake_world2joint(np.array(p4).reshape((3, 1)), leg_down)
        jp5 = self.coord_fake_world2joint(np.array(p5).reshape((3, 1)), leg_down)
        # 2.3 构造曲线对象
        tmp_curve = BSpline.Curve()
        tmp_curve.ctrlpts = (jp1, jp2, jp3, jp4, jp5)
        tmp_curve.delta = 0.01
        tmp_curve.degree = 4
        tmp_curve.knotvector = utilities.generate_knot_vector(4, len(tmp_curve.ctrlpts))
        tmp_curve.evaluate()
        if leg_down is 'left':
            self.this_curve_l = tmp_curve
        else:
            self.this_curve_r = tmp_curve
        # 3. 让我们来处理另一条腿吧
        p_end = np.array([p_end[0], coe*0.12, p_end[2]])  #
        a_end = np.array([a_end[0], 0.0, a_end[2]])       # 方向向前
        if self.cycle_cnt == 1:
            p_begin = np.array([-p_end[0], p_end[1], p_end[2]])    # x位置反向
            a_begin = np.array([a_end[0], a_end[1], -a_end[2]])    # z方向相反
        else:
            p_begin = self.p_begin
            a_begin = self.a_begin
        # 3.1 直角坐标系下的关键点位置
        p1 = tuple(p_begin)
        p2 = tuple(p_begin + 0.1 * a_begin)
        p4 = tuple(p_end - 0.05 * a_end)
        p5 = tuple(p_end)
        p3 = (0., (p1[1] + p5[1]) / 2, p_begin[2] + 0.3)
        # 3.2 关节坐标空间下的关键点位置转化
        jp1 = self.coord_fake_world2joint(np.array(p1).reshape((3, 1)), leg_down)
        jp2 = self.coord_fake_world2joint(np.array(p2).reshape((3, 1)), leg_down)
        jp3 = self.coord_fake_world2joint(np.array(p3).reshape((3, 1)), leg_down)
        jp4 = self.coord_fake_world2joint(np.array(p4).reshape((3, 1)), leg_down)
        jp5 = self.coord_fake_world2joint(np.array(p5).reshape((3, 1)), leg_down)
        # 3.3 构造曲线对象
        tmp_curve = BSpline.Curve()
        tmp_curve.ctrlpts = (jp1, jp2, jp3, jp4, jp5)
        tmp_curve.delta = 0.01
        tmp_curve.degree = 4
        tmp_curve.knotvector = utilities.generate_knot_vector(4, len(tmp_curve.ctrlpts))
        tmp_curve.evaluate()
        if leg_down is 'left':
            self.this_curve_r = tmp_curve
        else:
            self.this_curve_l = tmp_curve

    # 返回该周期开始后dt时间，左右足端的位置和速度
    def swing_get_planning(self, dt, leg_down):
        # 1. 获取信息
        t_air = self.des_air_time
        t_sup = self.des_sup_time
        # 2. 计算
        t_tot = t_air * 2 + t_sup          # 空中的总时间
        # print(dt, t_tot)
        if leg_down is 'left':            # 左右腿的不同进度
            p_c_l = (dt + t_air + t_sup)/t_tot
            p_c_r = dt/t_tot
        else:
            p_c_r = (dt + t_air + t_sup) / t_tot
            p_c_l = dt / t_tot
        left = self.this_curve_l.derivatives(limit_in01(p_c_l), order=1)
        right = self.this_curve_r.derivatives(limit_in01(p_c_r), order=1)
        return left, right

    # -----------------------------------------
    # 机器人控制：
    # 输出[tau1 ……tau6]的控制量（no）-->直接控制
    # -----------------------------------------
    def robot_control(self):
        # 获取一些必要信息
        rid = self.robot_id
        g = self.para[1]
        # 获取机器人状态
        vel = p.getBaseVelocity(rid)
        pos = p.getBasePositionAndOrientation(rid)
        contact_list = p.getContactPoints(rid, planeId)

        if self.status == 'air':
            # 如果刚从其他状态进入air状态-只需要在进入阶段执行一次
            if self.cycle_cnt % 2:
                leg_down = 'left'
            else:
                leg_down = 'right'
            if self.status_change:
                self.start_time = self.sys_t  # 获取本周期的起始时间
                # 计算本次能达到的顶点高度
                h0 = pos[0][2] + 0.5 * vel[0][2] * vel[0][2] / g
                self.this_x = np.array([h0, vel[0][0], vel[0][1]])
                # 更新本周期的控制pair，计算jac矩阵，以及控制参数
                self.choose_pair_from_speed()
                self.calculate_control_param()
                self.swing_related_calculation()
                self.swing_curve_planning(leg_down)
                if self.cycle_cnt == 1:    # 第一圈，的初始化
                    self.start_time = -self.des_air_time/2                 # 第一圈的起始时间在之前
                    # 系统初始，设置关节位置
                    lj, rj = self.swing_get_planning(self.sys_t-self.start_time, 'left')
                    # 左腿初始位置和速度
                    p.resetJointState(rid, 0, lj[0][0], targetVelocity=lj[1][0])
                    p.resetJointState(rid, 1, lj[0][1], targetVelocity=lj[1][1])
                    p.resetJointState(rid, 2, lj[0][2], targetVelocity=lj[1][2])
                    # 右腿初始位置和速度
                    p.resetJointState(rid, 4, rj[0][0], targetVelocity=rj[1][0])
                    p.resetJointState(rid, 5, rj[0][1], targetVelocity=rj[1][1])
                    p.resetJointState(rid, 6, rj[0][2], targetVelocity=rj[1][2])
                self.status_change = False
            else:
                # 如果不是刚进入腾空状态，直接获取状态并控制即可
                lj, rj = self.swing_get_planning(self.sys_t-self.start_time, leg_down)
                self.position_control_leg(lj[0], 'left')
                self.position_control_leg(rj[0], 'right')
                # 判定是否进行状态转化（即是否触地）
                if len(contact_list):
                    self.status = 'ground'
        # 触地部分控制
        elif self.status is 'ground':
            print('into ground')


# 准备环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)

# 创建模型
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
RobotId = p.loadURDF("bipedRobotOne.urdf", cubeStartPos, cubeStartOrientation)
p.resetBaseVelocity(RobotId, [2.0, 0, 0])
mode = p.VELOCITY_CONTROL
# 控制器
bc = BipedController(RobotId)
bc.load_table('./data/stable_pair.csv')
bc.set_target_vel(3.0)

for i in range(600):
    # 控制程序
    bc.set_system_time(i*sim_cycle)
    bc.robot_control()
    if bc.status == 'ground':
        break
    p.stepSimulation()
    time.sleep(sim_cycle*2)

p.disconnect()





