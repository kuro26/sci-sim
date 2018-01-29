# 使用leastsq函数求参数，拟合的函数形式是
# y = A*sin(2*pi*k*x + theta)

import numpy as np
from scipy.optimize import leastsq
import matplotlib.pyplot as plt


def func(x, p):
    A, k, theta = p
    return A*np.sin(2*np.pi*k*x + theta)


def residuals(p, y, x):
    # 函数需要计算等于零的情况，因此对上式重新组织
    return y - func(x, p)


# 生成数据
sample_x = np.linspace(-2*np.pi, 2*np.pi, 200)            # 采样点
real_A, real_k, real_theta = 10, 0.5, np.pi/5             # 真实参数
sample_y = func(sample_x, [real_A, real_k, real_theta]) + np.random.randn(len(sample_x))
plt.plot(sample_x, sample_y, '-b')

# 参数初值
p0 = [9, 0.6, 0]

# 调用函数进行求解
result = leastsq(residuals, p0, args=(sample_y, sample_x))


# 输出结果
print("拟合参数：", result[0])
get_y = func(sample_x, result[0])
plt.plot(sample_x, get_y,'-r')
plt.show()
