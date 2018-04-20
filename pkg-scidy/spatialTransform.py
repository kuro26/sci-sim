# 处理空间转移矩阵相关运算
import numpy as np
import math


def rotate_x(alpha):
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, ca,  -sa],
                     [0.0, sa,  ca]])


def rotate_y(beta):
    cb = math.cos(beta)
    sb = math.sin(beta)
    return np.array([[cb,  0.0, sb],
                     [0.0, 1.0, 0.0],
                     [-sb, 0.0, cb]])


def rotate_z(gamma):
    cg = math.cos(gamma)
    sg = math.sin(gamma)
    return np.array([[cg,  -sg, 0.0],
                     [sg,   cg, 0.0],
                     [0.0, 0.0, 1.0]])


class Transform:
    def __init__(self, alpha, beta, gamma, r):
        ra = rotate_x(alpha)
        rb = rotate_y(beta)
        rc = rotate_z(gamma)
        self.E = np.dot(np.dot(ra, rb), rc)
        self.r = r.reshape((3, 1))
