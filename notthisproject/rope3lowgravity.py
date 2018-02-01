import numpy as np
from scipy.optimize import leastsq

# 给定真实参数
p1 = np.array([-5, 0, 5])
p2 = np.array([3, 4, 6])
p3 = np.array([4, -3, 5.5])


# 绳长方程组满足的函数
def func(para, x):
    pos1 = para[[0, 1, 2]]
    pos2 = para[[3, 4, 5]]
    pos3 = para[[6, 7, 8]]
    xc = x[[0, 1, 2]]
    l1 = x[3]
    l2 = x[4]
    l3 = x[5]
    y1 = ((xc - pos1) * (xc - pos1)).sum() - l1 * l1
    y2 = ((xc - pos2) * (xc - pos2)).sum() - l2 * l2
    y3 = ((xc - pos3) * (xc - pos3)).sum() - l3 * l3
    return [y1, y2, y3]


for i in np.arange(-3, 2, 0.1):
    for j in np.arange(-2, 3, 0.1):
        for k in np.arange(1, 4, 0.1):
            p = np.array([i, j, k])
