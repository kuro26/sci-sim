# 用三连杆测试多点接触一次建模
from sympy import symbols
from sympy.physics.mechanics import *
# 定义变量
q1, q2, alpha = dynamicsymbols('q1 q2 alp')
dq1, dq2, d_alpha = dynamicsymbols('q1 q2 alp', 1)
u = dynamicsymbols('u:3')
du = dynamicsymbols('u:3', 1)
l1, l2, l3, g, pi = symbols('l1, l2, l3, g, pi')
# 定义坐标系
ground = ReferenceFrame('ground')
fb = ground.orientnew('fb', 'Axis', [alpha, ground.x])
A = fb.orientnew('A', 'Axis', [q1-pi, fb.x])
C = fb.orientnew('C', 'Axis', [q2, fb.x])
# 定义关键点
OA = Point('OA')
OB = OA.locatenew('OB', l1 * OA.y)
OC = OB.locatenew('OC', l2 * OB.y)
EC = OC.locatenew('EC', l3 * OC.y)
# 设置点的相对速度以及位置表达式
OA.set_vel(ground, 0)
OB.v2pt_theory(OA, ground, A)
OC.v2pt_theory(OB, ground, fb)
EC.v2pt_theory(OC, ground, C)






