import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


# 定义微分方程
def diff_fun(y, t):
    print(t)
    return [y[1], -2*y[0] - y[1]]


# 作为脚本执行的文件
def main():
    m_t = np.arange(0, 25.0, 0.01)
    m_solution = integrate.odeint(diff_fun, [1, 0], m_t)
    print('finish')
    plt.plot(m_t, m_solution[:, 0], 'b-')
    plt.show()


def pend(y, t, b, c):
    theta, omega = y
    print(t)
    dy_dt = [omega, -b*omega - c*np.sin(theta)]
    return dy_dt


def pend_sim():
    b = 0.25
    c = 5.0
    y0 = [np.pi - 0.1, 0.0]
    t = np.linspace(0, 10, 101)
    sol = integrate.odeint(pend, y0, t, args=(b, c))
    plt.plot(t, sol[:, 0], 'b', label='theta(t)')
    plt.plot(t, sol[:, 1], 'g', label='omega(t)')
    plt.legend(loc='best')
    plt.xlabel('t')
    plt.grid()
