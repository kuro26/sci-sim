import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from PIDdis import DisPid


# 模型，一个方向上的质量点与驱动力
# 状态：x, v
def fun_mass_point(yin, t, f, m):
    x, v = yin
    return [v, f/m]


def mass_point_control_test():
    sim_time = 8                         # 仿真时间
    t_cycle = 0.01                       # 仿真时间间隔
    step_cnt = int(sim_time/t_cycle)     # 仿真步数
    y0 = [0, 0]
    pid1 = DisPid(30, 1, 3, gap=t_cycle) # PID控制器
    mass = 1                             # 质量，1Kg

    y_out = np.zeros((2, step_cnt))      # 输出数据
    t = np.zeros(step_cnt)

    for i in range(0, step_cnt-1):
        t[i] = t_cycle * i
        ts = [0, t_cycle]
        ref = np.sin(t[i])               # 期望位置，正弦曲线
        err = ref - y0[0]
        pid1.update(err)
        f = pid1.out                     # 本周期PID控制量计算
        tmp = integrate.odeint(fun_mass_point, y0, ts, args=(f, mass))
        y_out[0, i+1] = tmp[-1][0]       # 记录数据
        y_out[1, i+1] = tmp[-1][1]
        y0 = [tmp[-1][0], tmp[-1][1]]    # 新的初始化
    # 绘图
    plt.figure()
    plt.plot(t[0:-1], y_out[0][0:-1], 'b-', linewidth=2, label='real position')
    ref_line = np.sin(t)
    plt.plot(t[0:-1], ref_line[0:-1], 'r--', label='ref position')
    plt.show()


mass_point_control_test()