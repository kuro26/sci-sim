from sympy import symbols
from sympy.physics.mechanics import *

q1, q2 = dynamicsymbols('q1 q2')
q1d, q2d = dynamicsymbols('q1 q2', 1)
u1, u2 = dynamicsymbols('u1 u2')
u1d, u2d = dynamicsymbols('u1 u2', 1)
l, m, g = symbols('l m g')

N = ReferenceFrame('N')                          # 基坐标系
A = N.orientnew('A', 'Axis', [q1, N.z])         # 绑定q1到关节坐标
B = N.orientnew('B', 'Axis', [q2, N.z])

A.set_ang_vel(N, u1 * N.z)               # 设u1为绕z轴角速度,A相对于N
B.set_ang_vel(N, u2 * N.z)

O = Point('O')                           # 设置O点为原点
P = O.locatenew('P', l * A.x)            # 设置P点为从O点开始，沿A坐标系x轴l长度
R = P.locatenew('R', l * B.x)

O.set_vel(N, 0)
P.v2pt_theory(O, N, A)
R.v2pt_theory(P, N, B)

ParP = Particle('ParP', P, m)
ParR = Particle('ParR', R, m)

kd = [q1d - u1, q2d - u2]
FL = [(P, m * g * N.x), (R, m * g * N.x)]
BL = [ParP, ParR]

KM = KanesMethod(N, q_ind=[q1, q2], u_ind=[u1, u2], kd_eqs=kd)

(fr, frstar) = KM.kanes_equations(FL, BL)
kdd = KM.kindiffdict()
mm = KM.mass_matrix_full
fo = KM.forcing_full
qudots = mm.inv() * fo
qudots = qudots.subs(kdd)
qudots.simplify()
mechanics_printing()
mprint(qudots)