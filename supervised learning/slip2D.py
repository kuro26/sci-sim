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
def fun_support(yin, t, m, xi, yi, f, g):
    x, y = yin
    delta_x = x - xi
    delta_y = y - yi

    # 当前腿长，质心到支撑点的长度
    l = np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))

    # 输出
    diff_x = delta_x*f/(m*l)
    diff_y = delta_y*f/(m*l) -g
    return [diff_x, diff_y]


# 空中状态微分计算函数
def fun_air(yin, t, g):
    return [0, -g]


