# 2D-SLIP模型
# ---point mass body
# ---mass-less leg

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


def robot_control():
    return 0


# 支撑过程状态微分计算函数
# (xi, yi)--支撑点坐标
# m--质心质量，f--足端出力， g--重力加速度
def fun_support(yin, _, m, xi, yi, f, g):
    x, vx, y, vy = yin
    delta_x = x - xi
    delta_y = y - yi

    # 当前腿长，质心到支撑点的长度
    leg_len = np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))

    # 输出
    dd_x = delta_x*f/(m*leg_len)
    dd_y = delta_y*f/(m*leg_len) - g
    return [vx, dd_x, vy, dd_y]


# 空中状态微分计算函数
def fun_air(yin, t, g):
    return [0, -g]


# ------------------------------------
# h0, v0, 初始条件，高度和速度
# alpha, beta, 触地角度和离地角度
# l1, l2, l3, l4，曲线中间的控制点
# ------------------------------------
def sim_model(h0, v0, alpha, l1, l2, l3, l4, l5, beta):
    m = 20
    t_cycle = 0.005            # 对应5ms的控制周期
    g = 9.8                    # 重力加速度
    leg_len_ori = 1            # 腿原长5米
    # 初始条件
    t = [0]
    x = [0]
    vx = [v0]
    y = [h0]
    vy = [0]

    # 第一段仿真
    flag_live = True
    while flag_live:
        ts = [t[-1], t[-1] + t_cycle]
        y0 = [x[-1], vx[-1], y[-1], vy[-1]]
        tmp = integrate.odeint(fun_air, y0, ts, args=(g,))
        if tmp[]




