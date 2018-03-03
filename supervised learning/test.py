import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from PIDdis import DisPid
from linebezier import BezierD1
from scipy.integrate import ode


# 模型，一个方向上的质量点与驱动力
# 状态：x, v
def fun_mass_point(yin, t, f, m):
    x, v = yin
    return [v, f/m]


# 质量点正弦运动驱动测试-PID没问题
def mass_point_control_test():
    sim_time = 8                          # 仿真时间
    t_cycle = 0.01                        # 仿真时间间隔
    step_cnt = int(sim_time/t_cycle)      # 仿真步数
    y0 = [0, 0]
    pid1 = DisPid(20, 1, 9, gap=t_cycle)  # PID控制器
    mass = 1                              # 质量，1Kg

    y_out = np.zeros((2, step_cnt))       # 输出数据
    t = np.zeros(step_cnt)

    for i in range(0, step_cnt-1):
        t[i] = t_cycle * i
        ts = [0, t_cycle]
        ref = np.sin(t[i])                # 期望位置，正弦曲线
        err = ref - y0[0]
        pid1.update(err)
        f = pid1.out                      # 本周期PID控制量计算
        tmp = integrate.odeint(fun_mass_point, y0, ts, args=(f, mass))
        y_out[0, i+1] = tmp[-1][0]        # 记录数据
        y_out[1, i+1] = tmp[-1][1]
        y0 = [tmp[-1][0], tmp[-1][1]]     # 新的初始化
    # 绘图
    plt.figure()
    plt.plot(t[0:-1], y_out[0][0:-1], 'b-', linewidth=2, label='real position')
    ref_line = np.sin(t)
    plt.plot(t[0:-1], ref_line[0:-1], 'r--', label='ref position')
    plt.show()


# 贝塞尔函数测试
def bezier_test():
    cp_cnt = 6
    bzr = BezierD1(cp_cnt-1, 2)            # 6个控制点，2条曲线
    coe = np.array([[0, 0.2, 0.7, 0.4, 0.9, 0.5],
                    [0, 0.6, 0.7, 0.4, 0.8, 0.6]])
    x_begin = 0
    x_end = 1
    bzr.setControlPoint(coe)
    bzr.setBezierBeginEnd(x_begin, x_end)
    cp_x = np.linspace(x_begin, x_end, cp_cnt)
    x_cnt = 100
    x = np.linspace(x_begin, x_end, x_cnt+1)
    res, y = bzr.value(x, diff=0)
    if res is False:
        return
    # 绘图
    plt.figure()
    plt.plot(cp_x, bzr.coeff[0, :], 'r*')
    plt.plot(cp_x, bzr.coeff[1, :], 'b+')
    plt.plot(x, y[0, :], 'r-')
    plt.plot(x, y[1, :], 'b-')
    plt.title('function value')
    plt.show()

    res, dy = bzr.value(x, diff=1)
    if res is False:
        return
    dis_dy = (y[:, 1:] - y[:, 0:-1])/((x_end - x_begin) / x_cnt)
    dis_x = x[0: -1] + ((x_end - x_begin) / x_cnt)/2
    # 绘图
    plt.plot(x, dy[0, :], 'r-')
    plt.plot(x, dy[1, :], 'b-')
    plt.plot(dis_x, dis_dy[0, :], 'r+')
    plt.plot(dis_x, dis_dy[1, :], 'b+')
    plt.show()

    res, ddy = bzr.value(x, diff=2)
    if res is False:
        return
    dis_ddy = (dy[:, 1:] - dy[:, 0:-1]) / ((x_end - x_begin) / x_cnt)
    # 绘图
    plt.plot(x, ddy[0, :], 'r-')
    plt.plot(x, ddy[1, :], 'b-')
    plt.plot(dis_x, dis_ddy[0, :], 'r+')
    plt.plot(dis_x, dis_ddy[1, :], 'b+')
    plt.show()


# 模型，一个方向上的质量点与驱动力
# 状态：x, v
def fun_mass_point2(t, y, arg1, arg2):
    # print(t)
    x, v = y
    f, m = arg1, arg2
    return [v, f/m]


def fun_mass_point_end(t, yin):
    x, v = yin
    if x > 2:
        # 出现终止条件
        return -1
    else:
        # 一切正常
        return 0


# 带终止条件的ODE测试
# 测试结果，这个终止条件并没有很好用，计算也不会返回终止状态
# 并且也是需要自己规定时间一步步积分的，意义不大
# 还是回到odeint，过零精度自己编辑来掌握
def ode_test():
    mass = 1
    t_cycle = 0.01
    solver = ode(fun_mass_point2)
    solver.set_integrator('dopri5')
    solver.set_solout(fun_mass_point_end)
    flag_live = True
    # 初始条件
    t = [0]
    x = [0]
    v = [0]
    # 仿真循环
    solver.set_initial_value([0, 0], 0)   # 设置初始条件--只需要设置一次
    while flag_live:
        f = np.sin(t[-1])
        solver.set_f_params(f, mass)      # 设置当前周期的控制量
        solver.integrate(solver.t+t_cycle)

        if solver.get_return_code() is 2 or solver.t > 4:
            break
        t.append(solver.t)
        x.append(solver.y[0])
        v.append(solver.y[1])
    print('solver t',solver.t)
    print('get t', t[-1])
    print('solver solution', solver.y)
    print('get solution', [x[-1], v[-1]])
    plt.figure()
    plt.plot(t, x, 'b-')
    plt.show()



ode_test()
