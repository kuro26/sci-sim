# -----------------------------------------------
# 模型说明：3D-SLIP模型
# 特征：点质量机体，无质量腿
# 控制输入：着地角(x)    着地角(y)   足端控制输入
#          alpha        beta         F
# 系统状态：y = [x, y, z, vx, vy, vz]
# 初始条件：y0 = [h0, vx0, vy0]
# 添加说明1：（2018/3/21）
#   该文件使用的是scipy中较早的积分器，实际上有新的可用积分器
# -----------------------------------------------

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# ------------------------------------------------
#                  系统方程
# para = [m, g, l0, x_f, y_f, z_f, f]
#               腿长     支撑点   控制足力
# g = -9.8
# ------------------------------------------------
def sys_support(yin, _, para):
    m, g, l0, x_f, y_f, z_f, f = para
    x, y, z, vx, vy, vz = yin

    c_rel_pos = np.array([x-x_f, y-y_f, z-z_f])                # 质心相对足端位置
    f_direction = c_rel_pos/np.linalg.norm(c_rel_pos)       # 质心相对足端方向
    f_vec = f * f_direction

    ddx = f_vec[0] / m
    ddy = f_vec[1] / m
    ddz = f_vec[2] / m + g

    return [vx, vy, vz, ddx, ddy, ddz]


def sys_air(yin, _, g):
    x, y, z, vx, vy, vz = yin
    return [vx, vy, vz, 0, 0, g]


# ------------------------------------------------
#              一个周期的仿真
# input: 控制变量 = [alpha, beta, ks1, ks2, vx0, vy0, h0]
# output: 仿真轨迹
# 后续使用仿真轨迹进行实际机器人的规划设计或者绘图都OK
# ------------------------------------------------
class SlipData:
    def __init__(self, h0, vx0, vy0):
        self.t = [0.0]                 # 系统状态
        self.x = [0.0]
        self.y = [0.0]
        self.z = [h0]
        self.vx = [vx0]
        self.vy = [vy0]
        self.vz = [0.0]
        self.te1, self.te1_idx = 0.0, 0  # 结束点1
        self.te2, self.te2_idx = 0.0, 0  # 结束点2
        self.te3, self.te3_idx = 0.0, 0  # 结束点3
        self.x_f, self.y_f, self.z_f = 0.0, 0.0, 0.0     # 触地点

    # 添加一组仿真更新的数据
    def status_update(self, ts, tmp):
        self.t.append(ts[1])
        self.x.append(tmp[-1][0])
        self.y.append(tmp[-1][1])
        self.z.append(tmp[-1][2])
        self.vx.append(tmp[-1][3])
        self.vy.append(tmp[-1][4])
        self.vz.append(tmp[-1][5])

    def get_recent_status(self):
        return [self.x[-1], self.y[-1], self.z[-1], self.vx[-1], self.vy[-1], self.vz[-1]]

    def plot_trajectory(self):
        # fig = plt.figure()
        ax = plt.axes(projection='3d')
        b, e = 0, self.te1_idx
        ax.plot3D(self.x[b:e], self.y[b:e], self.z[b:e], 'r')
        b, e = self.te1_idx, self.te2_idx
        ax.plot3D(self.x[b:e], self.y[b:e], self.z[b:e], 'b')
        b, e = self.te2_idx, self.te3_idx
        ax.plot3D(self.x[b:e], self.y[b:e], self.z[b:e], 'g')
        b, e = self.te3_idx, len(self.t)-1
        ax.plot3D(self.x[b:e], self.y[b:e], self.z[b:e], 'b')
        ax.scatter3D([0.0], [0.0], [0.0], '*')
        ax.scatter3D([self.x_f], [self.y_f], [self.z_f], 'r*')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        # ax.view_init(0, 10)

        plt.show()


def sim_cycle(alpha, beta, ks1, ks2, vx0, vy0, h0):
    m, g, l0 = [30.0, -9.8, 1.0]
    t_cycle = 0.005

    # 初始化数据存储变量
    data_sim = SlipData(h0, vx0, vy0)
    # ------------1.空中下落阶段------------
    loop_counter = int(5 / t_cycle)
    while loop_counter:
        loop_counter = loop_counter - 1
        ts = [data_sim.t[-1], data_sim.t[-1] + t_cycle]
        init_s = data_sim.get_recent_status()
        tmp = integrate.odeint(sys_air, init_s, ts, args=(g,))
        # 更新存储数据
        data_sim.status_update(ts, tmp)
        # 触地判定
        delta_h = l0 * np.cos(beta) * np.sin(np.pi - alpha)
        if data_sim.z[-1] < delta_h:
            break
    if loop_counter == 0:
        print('Error: loop 1 too long')
        return
    x_f = data_sim.x[-1] + l0*np.cos(beta)*np.cos(alpha)                  # 计算落足点(考虑vector与x轴的关系)
    y_f = data_sim.y[-1] + l0*np.sin(beta)
    z_f = data_sim.z[-1] - l0 * np.cos(beta) * np.sin(np.pi - alpha)
    data_sim.x_f, data_sim.y_f, data_sim.z_f = x_f, y_f, z_f              # 记录--着地点足位置
    data_sim.te1, data_sim.te1_idx = data_sim.t[-1], len(data_sim.t)-1    # 记录--落地时间

    # ------------2.支撑压缩阶段------------
    loop_counter = int(5 / t_cycle)
    while loop_counter:
        loop_counter = loop_counter - 1
        ts = [data_sim.t[-1], data_sim.t[-1] + t_cycle]
        init_s = data_sim.get_recent_status()
        tmp = [data_sim.x[-1]-x_f, data_sim.y[-1]-y_f, data_sim.z[-1]-z_f]
        f = ks1 * (l0 - np.linalg.norm(tmp))                              # 控制力计算
        para = [m, g, l0, x_f, y_f, z_f, f]
        tmp = integrate.odeint(sys_support, init_s, ts, args=(para,))     # 仿真
        # 更新存储数据
        data_sim.status_update(ts, tmp)
        # 判定是否达到最大压缩量（计算速度方向和腿向量的点积）
        tmp1 = [data_sim.x[-1] - x_f, data_sim.y[-1] - y_f, data_sim.z[-1] - z_f]
        tmp2 = [data_sim.vx[-1], data_sim.vy[-1], data_sim.vz[-1]]
        if np.dot(tmp1, tmp2) > 0:
            break
    if loop_counter == 0:
        print('Error: loop 2 too long')
        return
    data_sim.te2, data_sim.te2_idx = data_sim.t[-1], len(data_sim.t) - 1  # 记录--落地时间点

    # ------------3.支撑弹射阶段------------
    loop_counter = int(5 / t_cycle)
    while loop_counter:
        loop_counter = loop_counter - 1
        ts = [data_sim.t[-1], data_sim.t[-1] + t_cycle]
        init_s = data_sim.get_recent_status()
        tmp = [data_sim.x[-1] - x_f, data_sim.y[-1] - y_f, data_sim.z[-1] - z_f]
        f = ks2 * (l0 - np.linalg.norm(tmp))                              # 控制力计算
        para = [m, g, l0, x_f, y_f, z_f, f]
        tmp = integrate.odeint(sys_support, init_s, ts, args=(para,))     # 仿真
        # 更新存储数据
        data_sim.status_update(ts, tmp)
        # 判定是否离开地面
        leg_len = np.linalg.norm([data_sim.x[-1] - x_f, data_sim.y[-1] - y_f, data_sim.z[-1] - z_f])
        if leg_len > l0:
            break
    if loop_counter == 0:
        print('Error: loop 3 too long')
        return
    data_sim.te3, data_sim.te3_idx = data_sim.t[-1], len(data_sim.t) - 1  # 记录--落地时间点

    # ------------3.飞升阶段------------
    while True:
        ts = [data_sim.t[-1], data_sim.t[-1] + t_cycle]
        init_s = data_sim.get_recent_status()
        tmp = integrate.odeint(sys_air, init_s, ts, args=(g,))
        # 更新存储数据
        data_sim.status_update(ts, tmp)
        # 达到顶点判定
        if data_sim.vz[-1] < 0:
            break
    print('simulation finished!')
    return data_sim


#                触地角1 触地角2  刚度1    刚度2  x速度  y速度 初始高度
data = sim_cycle(1.1577,   0,   6.05e3, 6.05e3, 3.5,   0,   0.94)
data.plot_trajectory()
