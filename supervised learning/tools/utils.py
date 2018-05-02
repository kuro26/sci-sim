# 一些编程过程中可能会用到的工具
import numpy as np
import pandas as pd


def box_inertia(shape, m):
    x, y, z = shape
    ixx = (y*y + z*z) * m / 12
    iyy = (x*x + z*z) * m / 12
    izz = (x*x + y*y) * m / 12
    return [ixx, iyy, izz]


# 这种写法不是很有效率，暂时用一下
def rotate_x(alpha):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[1.0, 0.0, 0.0, 0.0],
                     [0.0, ca,  -sa, 0.0],
                     [0.0, sa,   ca, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def rotate_y(beta):
    cb = np.cos(beta)
    sb = np.sin(beta)
    return np.array([[cb,  0.0,  sb, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [-sb, 0.0,  cb, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def rotate_z(gamma):
    cg = np.cos(gamma)
    sg = np.sin(gamma)
    return np.array([[cg,  -sg, 0.0, 0.0],
                     [sg,   cg, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def trans_xyz(x, y, z):
    return np.array([[1.0, 0.0, 0.0, x],
                     [0.0, 1.0, 0.0, y],
                     [0.0, 0.0, 1.0, z],
                     [0.0, 0.0, 0.0, 1.0]])
