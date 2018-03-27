# -----------------------------------------------
# 模型说明：3D-SLIP模型
# 特征：点质量机体，无质量腿
# 控制输入：着地角(x)    着地角(y)   足端控制输入
#          alpha        beta         F
# 系统状态：y = [x, y, z, vx, vy, vz]
# 初始条件：y0 = [h0, vx0, vy0]
# 添加说明1：（2018/3/21）
#   在slip3d的基础上使用新的scipy积分器，这个有事件判定
# -----------------------------------------------

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pandas as pd


# ------------------------------------------------
#                  系统方程
# para = [m, g, l0, x_f, y_f, z_f, f]
#               腿长     支撑点   控制足力
# g = -9.8
# ------------------------------------------------
def sys_support(_, yin, para):
    m, g, l0, x_f, y_f, z_f, f = para
    x, y, z, vx, vy, vz = yin

    c_rel_pos = np.array([x-x_f, y-y_f, z-z_f])                # 质心相对足端位置
    f_direction = c_rel_pos/np.linalg.norm(c_rel_pos)       # 质心相对足端方向
    f_vec = f * f_direction

    ddx = f_vec[0] / m
    ddy = f_vec[1] / m
    ddz = f_vec[2] / m + g

    return [vx, vy, vz, ddx, ddy, ddz]


def sys_air(_, yin, g):
    x, y, z, vx, vy, vz = yin
    return [vx, vy, vz, 0, 0, g]


# 所有事件均从正向负数
def event_touchdown(_, yin, alpha, beta, l0):
    x, y, z, vx, vy, vz = yin
    delta_h = l0 * np.cos(beta) * np.sin(np.pi - alpha)
    return z - delta_h                      # 从正到负


def event_shortest(_, yin, x_f, y_f, z_f):
    x, y, z, vx, vy, vz = yin
    tmp1 = [x - x_f, y - y_f, z - z_f]
    tmp2 = [vx, vy, vz]
    return -np.dot(tmp1, tmp2)               # 从正到负


def event_thrust(_, yin, x_f, y_f, z_f, l0):
    x, y, z, vx, vy, vz = yin
    leg_len = np.linalg.norm([x - x_f, y - y_f, z - z_f])
    return l0 - leg_len                     # 从正到负


def event_top(_, yin):
    x, y, z, vx, vy, vz = yin
    return vz                               # 从正到负


# ------------------------------------------------
#              一个周期的仿真
# input: 控制变量 = [alpha, beta, ks1, ks2, vx0, vy0, h0]
#        改为初始-控制对：pairs = [h0, vx0, vy0, alpha, beta, ks1, ks2]
# output: 仿真轨迹
# 后续使用仿真轨迹进行实际机器人的规划设计或者绘图都OK
# ------------------------------------------------
def sim_cycle(pairs):
    h0, vx0, vy0, alpha, beta, ks1, ks2 = pairs
    m, g, l0 = [20.0, -9.8, 1.0]
    t_span = (0, 2)
    t_eval = np.linspace(0, 2, 500)
    options = {'rtol': 1e-9, 'atol': 1e-12}

    # 初始化数据存储变量
    # ------------1.空中下落阶段------------
    # sys_fun = lambda t, y: sys_air(t, y, g)
    def sys_fun(t, y): return sys_air(t, y, g)

    def event_fun(t, y): return event_touchdown(t, y, alpha, beta, l0)
    event_fun.direction = -1
    event_fun.terminal = True
    init_s = [0.0, 0.0, h0, vx0, vy0, 0.0]
    in_sol1 = integrate.solve_ivp(sys_fun, t_span, init_s, t_eval=t_eval, events=event_fun, **options)

    last_y = in_sol1.y[:, -1]
    x_f = last_y[0] + l0 * np.cos(beta) * np.cos(alpha)                  # 计算落足点(考虑vector与x轴的关系)
    y_f = last_y[1] + l0 * np.sin(beta)
    z_f = last_y[2] - l0 * np.cos(beta) * np.sin(np.pi - alpha)

    # ------------2.支撑压缩阶段------------
    def sys_fun(t, yin):
        x, y, z, vx, vy, vz = yin
        inner_tmp = [x - x_f, y - y_f, z - z_f]
        inner_force = ks1 * (l0 - np.linalg.norm(inner_tmp))
        inner_para = [m, g, l0, x_f, y_f, z_f, inner_force]
        return sys_support(t, yin, inner_para)

    def event_fun(t, yin): return event_shortest(t, yin, x_f, y_f, z_f)
    event_fun.direction = -1
    event_fun.terminal = True
    init_s = in_sol1.y[:, -1]
    in_sol2 = integrate.solve_ivp(sys_fun, t_span, init_s, t_eval=t_eval, events=event_fun, **options)

    # ------------3.支撑弹射阶段------------
    def sys_fun(t, yin):
        x, y, z, vx, vy, vz = yin
        inner_tmp = [x - x_f, y - y_f, z - z_f]
        inner_force = ks2 * (l0 - np.linalg.norm(inner_tmp))
        inner_para = [m, g, l0, x_f, y_f, z_f, inner_force]
        return sys_support(t, yin, inner_para)

    def event_fun(t, yin): return event_thrust(t, yin, x_f, y_f, z_f, l0)
    event_fun.direction = -1
    event_fun.terminal = True
    init_s = in_sol2.y[:, -1]
    in_sol3 = integrate.solve_ivp(sys_fun, t_span, init_s, t_eval=t_eval, events=event_fun, **options)

    # ------------3.飞升阶段------------
    def sys_fun(t, yin): return sys_air(t, yin, g)

    def event_fun(t, yin): return event_top(t, yin)
    event_fun.direction = -1
    event_fun.terminal = True
    init_s = in_sol3.y[:, -1]
    in_sol4 = integrate.solve_ivp(sys_fun, t_span, init_s, t_eval=t_eval, events=event_fun, **options)

    print('simulation finished!')
    in_foot_point = [x_f, y_f, z_f]
    return [in_sol1, in_sol2, in_sol3, in_sol4, in_foot_point]


# 测试：sim_cycle_test([.94, 4.5, 0, 1.1577, 0, 6.05e3, 6.05e3])
def sim_cycle_test(pairs):
    sol1, sol2, sol3, sol4, foot_point = sim_cycle(pairs)
    ax = plt.axes(projection='3d')
    ax.plot(sol1.y[0, :], sol1.y[1, :], sol1.y[2, :], 'r')
    ax.plot(sol2.y[0, :], sol2.y[1, :], sol2.y[2, :], 'g')
    ax.plot(sol3.y[0, :], sol3.y[1, :], sol3.y[2, :], 'b')
    ax.plot(sol4.y[0, :], sol4.y[1, :], sol4.y[2, :], 'r')
    ax.plot([0], [0], [0], '*r')
    ax.plot([foot_point[0]], [foot_point[1]], [foot_point[2]], '*b')
    plt.show()
    # 存储数据为csv文件
    # pd.DataFrame(sol1.y).to_csv('data/sol1.csv')
    # pd.DataFrame(sol2.y).to_csv('data/sol2.csv')
    # pd.DataFrame(sol3.y).to_csv('data/sol3.csv')
    # pd.DataFrame(sol4.y).to_csv('data/sol4.csv')
