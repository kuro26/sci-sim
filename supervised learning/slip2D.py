# 2D-SLIP模型
# ---point mass body
# ---mass-less leg

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


def robot_control():
    return 0


# ------------------------------------
# 支撑过程状态微分计算函数
# (xi, yi)--支撑点坐标
# m--质心质量，f--足端出力， g--重力加速度
# ------------------------------------
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


# ------------------------------------
# 空中状态微分计算函数
# ------------------------------------
def fun_air(yin, _, g):
    x, vx, y, vy = yin
    return [0, 0, vy, -g]


# ------------------------------------
# h0, v0, 初始条件，高度和速度
# alpha, beta, 触地角度和离地角度
# l1, l2, l3, l4，曲线中间的控制点
# ------------------------------------
def sim_model(h0, v0, alpha, theta, l1, l2, l3, l4, l5, beta):
    m = 20
    t_cycle = 0.005            # 对应5ms的控制周期
    g = 9.8                    # 重力加速度
    leg_len_ori = 1            # 腿原长5米
    # 初始条件
    t, x, vx, y, vy = [0.0], [0.0], [v0], [h0], [0.0]

    # 空中第一段仿真
    while True:
        ts = [t[-1], t[-1] + t_cycle]
        y0 = [x[-1], vx[-1], y[-1], vy[-1]]
        tmp = integrate.odeint(fun_air, y0, ts, args=(g,))
        # 更新需要存储的数据
        t.append(t[-1] + t_cycle)
        x.append(tmp[-1][0])
        vx.append(tmp[-1][1])
        y.append(tmp[-1][2])
        vy.append(tmp[-1][3])
        # 触地判定，微小的
        if y[-1]-leg_len_ori*np.sin(alpha) < 0:
            break
    t1_end = t[-1]                  # 记录第一阶段结束时间
    # 支撑阶段
    xs = x[-1] + leg_len_ori * np.cos(alpha)
    ys = y[-1] - leg_len_ori * np.sin(alpha)
    while True:
        # 计算控制量
        f = 0
        ts = [t[-1], t[-1] + t_cycle]
        y0 = [x[-1], vx[-1], y[-1], vy[-1]]              # 状态是统一的
        tmp = integrate.odeint(fun_support, y0, ts, arg=(m, xs, ys, f, g))




