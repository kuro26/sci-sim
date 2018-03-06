import numpy as np
from scipy.optimize import leastsq
import matplotlib.pyplot as plt


def func(x, p):
    """ 数据拟合所用的函数: A*sin(2*pi*k*x + theta) """
    A, k, theta = p
    return A*np.sin(2*np.pi*k*x+theta)

def residuals(p, y, x):
    """ 实验数据x, y和拟合函数之间的差，p为拟合需要找到的系数 """
    return y - func(x, p)


# 生成输入数据X和输出输出Y
x = np.linspace(-2*np.pi, 0, 100)       # x
A, k, theta = 10, 0.34, np.pi/6         # 真实数据的函数参数
y0 = func(x, [A, k, theta])             # 真实数据
y1 = y0 + 2 * np.random.randn(len(x))   # 加入噪声之后的实验数据
plt.plot(x, y1)

# 需要求解的参数的初值，注意参数的顺序，要和func保持一致
p0 = [7, 0.8, 0]

# 调用leastsq进行数据拟合, residuals为计算误差的函数
# p0为拟合参数的初始值
# args为需要拟合的实验数据，也就是，residuals误差函数
# 除了P之外的其他参数都打包到args中

plsq = leastsq(residuals, p0, args=(y1, x))

# 除了初始值之外，还调用了args参数，用于指定residuals中使用到的其他参数
# （直线拟合时直接使用了X,Y的全局变量）,同样也返回一个元组，第一个元素为拟合后的参数数组；

# 这里将 (y1, x)传递给args参数。Leastsq()会将这两个额外的参数传递给residuals()。
# 因此residuals()有三个参数，p是正弦函数的参数，y和x是表示实验数据的数组。

# 拟合的参数
print("拟合参数", plsq[0])
get_y = func(x, plsq[0])
plt.plot(x, get_y, '-r')
plt.show()
# 拟合参数 [ 10.6359371    0.3397994    0.50520845]
# 发现跟实际值有差距，但是从下面的拟合图形来看还不错，说明结果还是 堪用 的。