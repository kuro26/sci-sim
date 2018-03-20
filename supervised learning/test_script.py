from geomdl import BSpline
from geomdl import utilities
import numpy as np
import matplotlib.pyplot as plt

curve = BSpline.Curve()
curve.ctrlpts = ((3.0,), (1.0,), (3.0,), (2.0,), (5.0,))
curve.delta = 0.01
curve.degree = 4  # degree应该小于控制点数量
# 自动计算knot point
curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))
curve.evaluate()
pt_y = []
pt_x = np.linspace(0, 1, 101)
dpt_y = []
ddpt_y = []
for pt in pt_x:
    tmp = curve.derivatives(pt, order=2)
    pt_y.append(tmp[0][0])
    dpt_y.append(tmp[1][0])
    ddpt_y.append(tmp[2][0])
ctrl_x = np.linspace(0, 1, 5)
ctrl_y = []
for item in curve.ctrlpts:
    ctrl_y.append(item[0])
plt.plot(pt_x, pt_y, 'b-')
plt.plot(ctrl_x, ctrl_y, 'r*')
plt.grid()
plt.show()
# 验证一阶求导的正确性
pt_y = np.array(pt_y)
dpt_y = np.array(dpt_y)
dpt_y_dis = (pt_y[1:] - pt_y[:-1]) / (pt_x[1] - pt_x[0])
ddpt_y_dis = (dpt_y[1:] - dpt_y[:-1]) / (pt_x[1] - pt_x[0])
pt_x_dis = pt_x[:-1] + (pt_x[1] - pt_x[0]) / 2
# 绘制一阶的
plt.plot(pt_x_dis, dpt_y_dis, '+r')
plt.plot(pt_x, dpt_y, '-b')
plt.show()
# 绘制二阶的
plt.plot(pt_x_dis, ddpt_y_dis, '+r')
plt.plot(pt_x, ddpt_y, '-b')
plt.show()
