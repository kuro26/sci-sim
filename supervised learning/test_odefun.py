import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


# 定义微分方程
def diff_fun(y, t):
    return [y[1], -2*y[0] - y[1]]


# 作为脚本执行的文件
def main():
    m_t = np.arange(0, 25.0, 0.01)
    m_solution = integrate.odeint(diff_fun, [1, 0], m_t)
    print('finish')
    plt.plot(m_t, m_solution[:, 0], 'b-')
    plt.show()


if __name__ == '__main__':
    main()

