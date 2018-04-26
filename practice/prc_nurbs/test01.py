# 作为Nurbs曲线的第一个测试文件
from geomdl import BSpline
from geomdl import utilities
import matplotlib.pyplot as plt
import numpy as np


# nurbs-python测试
# 一阶导数，二阶导数测试
def nurbs_test():
    curve = BSpline.Curve()
    curve.ctrlpts = ((3.0,), (1.0, ), (3.0, ), (2.0, ), (5.0, ))      # 控制点
    curve.delta = 0.01                                                # 数据间隔
    curve.degree = 4                                                  # degree应该小于控制点数量
    # 自动计算knot point
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))
    curve.evaluate()                                                  # 计算曲线
    pt_y = []                       # y值
    pt_x = np.linspace(0, 1, 101)
    dpt_y = []                      # y一阶导数
    ddpt_y = []                     # y二阶导数
    for pt in pt_x:
        tmp = curve.derivatives(pt, order=2)   # 0，1，2阶导数全部计算
        pt_y.append(tmp[0][0])
        dpt_y.append(tmp[1][0])
        ddpt_y.append(tmp[2][0])
    ctrl_x = np.linspace(0, 1, 5)              # 控制点
    ctrl_y = []
    for item in curve.ctrlpts:                 #
        ctrl_y.append(item[0])
    plt.plot(pt_x, pt_y, 'b-')
    plt.plot(ctrl_x, ctrl_y, 'r*')
    plt.grid()
    plt.show()
    # 验证一阶求导的正确性
    pt_y = np.array(pt_y)
    dpt_y = np.array(dpt_y)
    dpt_y_dis = (pt_y[1:] - pt_y[:-1])/(pt_x[1]-pt_x[0])
    ddpt_y_dis = (dpt_y[1:] - dpt_y[:-1])/(pt_x[1]-pt_x[0])
    pt_x_dis = pt_x[:-1]+(pt_x[1]-pt_x[0])/2
    # 绘制一阶的
    plt.plot(pt_x_dis, dpt_y_dis, '+r')
    plt.plot(pt_x, dpt_y, '-b')
    plt.show()
    # 绘制二阶的
    plt.plot(pt_x_dis, ddpt_y_dis, '+r')
    plt.plot(pt_x, ddpt_y, '-b')
    plt.show()


# 该系统的构造不包含边界上的速度，所以应用受限，因此，后面还是需要发展自己的nurbs曲线，带边界条件的
def swing_leg_planning():
    curve = BSpline.Curve()
    curve.ctrlpts = ((-0.2, 0), (-0.3, -0.03), (0., 0.3), (0.3, 0.), (0.2, 0.))  # 控制点
    curve.delta = 0.01  # 数据间隔
    curve.degree = 4  # degree应该小于控制点数量
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))
    curve.evaluate()
    x = []
    dx = []
    y = []
    dy = []
    t_list = np.linspace(0, 1, 101)
    for t in t_list:
        tmp = curve.derivatives(t, order=1)
        x.append(tmp[0][0])
        dx.append(tmp[1][0])
        y.append(tmp[0][1])
        dy.append(tmp[1][1])
    plt.plot(x, y)
    plt.xlim((-0.3, 0.3))
    plt.ylim((-0.1, 0.5))
    plt.grid()
    plt.show()



swing_leg_planning()
