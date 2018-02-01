from linebezier import BezierD1
import matplotlib.pyplot as plt
import numpy as np

traj = BezierD1(4, 2)
coeff = np.array([[0, 0.2, 0.7, 0.4, 0.9],
                  [0, 0.6, 0.7, 0.4, 0.8]])
traj.setControlPoint(coeff)
control_x = np.linspace(0, 1, 5)
plt.plot(control_x, traj.coeff[0, :], 'r*')
plt.plot(control_x, traj.coeff[1, :], 'b+')

x = np.linspace(0, 1, 100)
res, y = traj.value(x, diff=0)
plt.plot(x, y[0,:], 'r-')
plt.plot(x, y[1,:], 'b-')
plt.title('原函数')
plt.grid(b='on')
plt.show()
plt.figure()
res, y = traj.value(x, diff=1)
plt.plot(x, y[0,:], 'r-')
plt.plot(x, y[1,:], 'b-')
plt.title('一阶导数')
plt.grid(b='on')
plt.show()
plt.figure()
res, y = traj.value(x, diff=2)
plt.plot(x, y[0,:], 'r-')
plt.plot(x, y[1,:], 'b-')
plt.title('二阶导数')
plt.grid(b='on')
plt.show()

